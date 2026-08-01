#include <Clients/JoltWaterVolume.h>

#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/std/parallel/lock.h>

#include <AzFramework/Physics/CollisionBus.h>

#include <Clients/JoltBuoyancyOverrideRegistry.h>
#include <Clients/JoltWaterVolumeRegistry.h>
#include <JoltPhysics/JoltPhysicsBus.h>

#include <cmath>

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyInterface.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltBuoyancy
{
    namespace
    {
        JPH::Vec3 ToJolt(const AZ::Vector3& v)
        {
            return JPH::Vec3(v.GetX(), v.GetY(), v.GetZ());
        }

        JPH::RVec3 ToJoltR(const AZ::Vector3& v)
        {
            return JPH::RVec3(v.GetX(), v.GetY(), v.GetZ());
        }

        AZ::Vector3 FromJolt(const JPH::Vec3& v)
        {
            return AZ::Vector3(v.GetX(), v.GetY(), v.GetZ());
        }

        //! Whether a body on this object layer is visible to a query using this mask.
        //! Only the physics gem can answer, so answers are cached for the step.
        bool ObjectLayerPassesFilter(AZ::u32 objectLayer, AZ::u64 collidesWithMask, AZStd::unordered_map<AZ::u32, bool>& cache)
        {
            const auto cached = cache.find(objectLayer);
            if (cached != cache.end())
            {
                return cached->second;
            }

            bool matches = true;
            JoltPhysics::JoltPhysicsSystemRequestBus::BroadcastResult(
                matches, &JoltPhysics::JoltPhysicsSystemRequests::ObjectLayerMatchesQueryMask, objectLayer, collidesWithMask);
            cache.emplace(objectLayer, matches);
            return matches;
        }
    }

    bool JoltWaterVolumeSnapshot::Contains(const AZ::Vector3& worldPoint) const
    {
        if (!m_worldBounds.IsValid())
        {
            return false;
        }

        // Into the volume's own space, so a rotated volume is tested against its real box
        // rather than against the axis-aligned bounds of its corners.
        const AZ::Vector3 localPoint = m_worldTransform.GetInverse().TransformPoint(worldPoint);
        return localPoint.GetAbs().IsLessEqualThan(m_dimensions * 0.5f);
    }

    float JoltWaterVolumeSnapshot::SubmersionDepth(const AZ::Vector3& worldPoint) const
    {
        const AZ::Vector3 surfaceNormal = m_worldTransform.TransformVector(AZ::Vector3::CreateAxisZ()).GetNormalizedSafe();
        const AZ::Vector3 surfacePosition =
            m_worldTransform.TransformPoint(AZ::Vector3(0.0f, 0.0f, m_dimensions.GetZ() * 0.5f));
        return (surfacePosition - worldPoint).Dot(surfaceNormal);
    }

    JoltWaterSurfaceSample JoltWaterVolume::EvaluateBuiltInSurface(
        const JoltWaterVolumeSnapshot& snapshot,
        const JoltWaterVolumeSettings& settings,
        float elapsedTime,
        const AZ::Vector3& worldPoint)
    {
        const AZ::Transform& transform = snapshot.m_worldTransform;
        const float halfHeight = snapshot.m_dimensions.GetZ() * 0.5f;

        if (!settings.m_wavesEnabled || settings.m_waveAmplitude <= 0.0f || settings.m_waveLength <= 0.0f)
        {
            JoltWaterSurfaceSample flat;
            flat.m_normal = transform.TransformVector(AZ::Vector3::CreateAxisZ()).GetNormalizedSafe();
            flat.m_position = transform.TransformPoint(AZ::Vector3(0.0f, 0.0f, halfHeight));
            return flat;
        }

        // Everything happens in the volume's own space, so the wave rides a tilted volume
        // instead of ignoring the tilt and staying world-flat.
        const AZ::Transform inverseTransform = transform.GetInverse();
        const AZ::Vector3 localPoint = inverseTransform.TransformPoint(worldPoint);

        const AZ::Vector2 direction = settings.m_waveDirection.GetLengthSq() > 0.0f
            ? settings.m_waveDirection.GetNormalized()
            : AZ::Vector2(1.0f, 0.0f);
        const float waveNumber = AZ::Constants::TwoPi / settings.m_waveLength;
        const float phase = waveNumber * settings.m_waveSpeed * elapsedTime;

        const auto heightAt = [&](float x, float y)
        {
            return halfHeight + settings.m_waveAmplitude * std::sin(waveNumber * (direction.GetX() * x + direction.GetY() * y) - phase);
        };

        // Three taps around the point give the local slope; the analytic derivative would
        // do as well, but finite differences keep working if this is ever swapped for a
        // sum of waves or a texture.
        const float epsilon = 0.05f * settings.m_waveLength;
        const float here = heightAt(localPoint.GetX(), localPoint.GetY());
        const float alongX = heightAt(localPoint.GetX() + epsilon, localPoint.GetY());
        const float alongY = heightAt(localPoint.GetX(), localPoint.GetY() + epsilon);

        const AZ::Vector3 tangentX(epsilon, 0.0f, alongX - here);
        const AZ::Vector3 tangentY(0.0f, epsilon, alongY - here);
        const AZ::Vector3 localNormal = tangentX.Cross(tangentY).GetNormalizedSafe();

        JoltWaterSurfaceSample sample;
        sample.m_position = transform.TransformPoint(AZ::Vector3(localPoint.GetX(), localPoint.GetY(), here));
        sample.m_normal = transform.TransformVector(localNormal).GetNormalizedSafe();
        return sample;
    }

    JoltWaterSurfaceSample JoltWaterVolume::EvaluateSurface(const AZ::Vector3& worldPoint) const
    {
        {
            AZStd::lock_guard lock(m_surfaceFunctionMutex);
            if (m_surfaceFunction)
            {
                return m_surfaceFunction(worldPoint);
            }
        }

        const JoltWaterVolumeSnapshot snapshot = GetSnapshot();
        return EvaluateBuiltInSurface(
            snapshot, GetSettings(), m_elapsedTime.load(AZStd::memory_order_relaxed), worldPoint);
    }

    void JoltWaterVolume::SetSurfaceFunction(SurfaceFunction surfaceFunction)
    {
        AZStd::lock_guard lock(m_surfaceFunctionMutex);
        m_surfaceFunction = AZStd::move(surfaceFunction);
    }

    float JoltWaterVolume::GetSubmergedFraction(AZ::EntityId bodyEntityId) const
    {
        AZStd::lock_guard lock(m_submergedMutex);
        const auto found = m_submerged.find(bodyEntityId);
        return found != m_submerged.end() ? found->second : 0.0f;
    }

    void JoltWaterVolume::TakePendingEvents(AZStd::vector<JoltWaterVolumeEvent>& outEvents)
    {
        outEvents.clear();
        AZStd::lock_guard lock(m_pendingEventMutex);
        outEvents.swap(m_pendingEvents);
    }

    JoltWaterVolume::~JoltWaterVolume()
    {
        Detach();
    }

    void JoltWaterVolume::BumpGeneration()
    {
        m_generation.fetch_add(1, AZStd::memory_order_relaxed);
    }

    JoltWaterVolumeSnapshot JoltWaterVolume::GetSnapshot() const
    {
        JoltWaterVolumeSnapshot snapshot;
        {
            AZStd::lock_guard lock(m_settingsMutex);
            snapshot.m_worldTransform = m_worldTransform;
            snapshot.m_dimensions = m_dimensions;
            snapshot.m_worldBounds = m_worldBounds;
        }
        snapshot.m_enabled = IsEnabled();
        snapshot.m_owner = this;
        return snapshot;
    }

    bool JoltWaterVolume::Attach(AzPhysics::SceneHandle sceneHandle)
    {
        Detach();

        // Asking the Jolt gem rather than casting the scene's native pointer: it answers
        // with null when the scene belongs to a different physics backend, so this gem is
        // inert rather than dangerous in a project that is not running Jolt.
        JPH::PhysicsSystem* physicsSystem = nullptr;
        JoltPhysics::JoltPhysicsSystemRequestBus::BroadcastResult(
            physicsSystem, &JoltPhysics::JoltPhysicsSystemRequests::GetNativePhysicsSystem, sceneHandle);

        AZ_WarningOnce("JoltBuoyancy", physicsSystem != nullptr,
            "No Jolt physics scene is available, so water volumes do nothing. This gem requires the JoltPhysics "
            "gem to be the active physics backend.");

        return AttachToPhysicsSystem(physicsSystem);
    }

    bool JoltWaterVolume::AttachToPhysicsSystem(JPH::PhysicsSystem* physicsSystem)
    {
        Detach();

        if (!physicsSystem)
        {
            return false;
        }

        m_physicsSystem = physicsSystem;
        m_physicsSystem->AddStepListener(this);
        JoltWaterVolumeRegistry::Get().Register(m_physicsSystem, this);

        // A volume that has just appeared counts as a change, so anything already asleep
        // inside it gets woken rather than ignored.
        BumpGeneration();
        return true;
    }

    void JoltWaterVolume::Detach()
    {
        if (m_physicsSystem)
        {
            JoltWaterVolumeRegistry::Get().Unregister(m_physicsSystem, this);
            m_physicsSystem->RemoveStepListener(this);
            m_physicsSystem = nullptr;
        }
        m_submergedBodyCount.store(0, AZStd::memory_order_relaxed);

        AZStd::lock_guard lock(m_pendingWakeMutex);
        m_pendingWake.clear();
    }

    void JoltWaterVolume::SetEnabled(bool enabled)
    {
        const bool changed = m_enabled.exchange(enabled, AZStd::memory_order_relaxed) != enabled;
        if (changed)
        {
            // Re-enabling has to reach bodies that fell asleep while the volume was off.
            BumpGeneration();
        }
    }

    void JoltWaterVolume::WakePendingBodies()
    {
        AZStd::vector<JPH::BodyID> toWake;
        {
            AZStd::lock_guard lock(m_pendingWakeMutex);
            toWake.swap(m_pendingWake);
        }

        if (toWake.empty() || !m_physicsSystem)
        {
            return;
        }

        // Takes each body's mutex, so this is only safe outside the step - see the header.
        m_physicsSystem->GetBodyInterface().ActivateBodies(toWake.data(), static_cast<int>(toWake.size()));
    }

    void JoltWaterVolume::SetVolume(const AZ::Transform& worldTransform, const AZ::Vector3& dimensions)
    {
        // A moved or resized volume is exactly the case sleeping bodies must be woken for.
        BumpGeneration();

        AZStd::lock_guard lock(m_settingsMutex);
        m_worldTransform = worldTransform;
        m_dimensions = dimensions.GetMax(AZ::Vector3(0.001f));

        // The broadphase is queried with an axis-aligned box, so a rotated volume needs
        // the bounds of its corners rather than of its dimensions.
        const AZ::Vector3 halfExtents = m_dimensions * 0.5f;
        m_worldBounds = AZ::Aabb::CreateNull();
        for (int corner = 0; corner < 8; ++corner)
        {
            const AZ::Vector3 localCorner(
                (corner & 1) ? halfExtents.GetX() : -halfExtents.GetX(),
                (corner & 2) ? halfExtents.GetY() : -halfExtents.GetY(),
                (corner & 4) ? halfExtents.GetZ() : -halfExtents.GetZ());
            m_worldBounds.AddPoint(m_worldTransform.TransformPoint(localCorner));
        }
    }

    void JoltWaterVolume::SetSettings(const JoltWaterVolumeSettings& settings)
    {
        // Denser fluid or a new current changes what the water does to a body that has
        // already settled, so this counts as a change too.
        BumpGeneration();

        // Resolved here, on whichever thread called the setter, because it is a bus call
        // and OnStep runs on Jolt's job threads where gameplay buses are off limits.
        AZ::u64 mask = ~0ull;
        if (!settings.m_collisionGroupId.m_id.IsNull())
        {
            AzPhysics::CollisionGroup group = AzPhysics::CollisionGroup::All;
            Physics::CollisionRequestBus::BroadcastResult(
                group, &Physics::CollisionRequests::GetCollisionGroupById, settings.m_collisionGroupId);
            mask = group.GetMask();
        }
        m_collidesWithMask.store(mask, AZStd::memory_order_relaxed);

        AZStd::lock_guard lock(m_settingsMutex);
        m_settings = settings;
    }

    JoltWaterVolumeSettings JoltWaterVolume::GetSettings() const
    {
        AZStd::lock_guard lock(m_settingsMutex);
        return m_settings;
    }

    void JoltWaterVolume::OnStep(const JPH::PhysicsStepListenerContext& inContext)
    {
        if (!IsEnabled() || inContext.mDeltaTime <= 0.0f)
        {
            m_submergedBodyCount.store(0, AZStd::memory_order_relaxed);
            return;
        }

        // Copy what gameplay may be writing, so the rest of the step works from a
        // consistent snapshot.
        JoltWaterVolumeSettings settings;
        AZ::Transform worldTransform;
        AZ::Vector3 dimensions;
        AZ::Aabb worldBounds;
        {
            AZStd::lock_guard lock(m_settingsMutex);
            settings = m_settings;
            worldTransform = m_worldTransform;
            dimensions = m_dimensions;
            worldBounds = m_worldBounds;
        }

        if (!worldBounds.IsValid())
        {
            return;
        }

        // The wave phase advances with the simulation rather than with a wall clock, so a
        // paused game has a still surface.
        const float elapsedTime = m_elapsedTime.fetch_add(inContext.mDeltaTime, AZStd::memory_order_relaxed) +
            inContext.mDeltaTime;

        const JPH::AABox queryBox(ToJolt(worldBounds.GetMin()), ToJolt(worldBounds.GetMax()));
        JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
        m_physicsSystem->GetBroadPhaseQuery().CollideAABox(queryBox, collector);

        const JPH::Vec3 gravity = m_physicsSystem->GetGravity();

        // Overlapping volumes used to both apply an impulse to the same body, doubling its
        // buoyancy. Peer placements are copied once per step rather than per body, and the
        // list is empty in the ordinary single-volume case, so this costs nothing then.
        AZStd::vector<JoltWaterVolume*> peers;
        AZStd::vector<JoltWaterVolumeSnapshot> peerSnapshots;
        JoltWaterVolumeRegistry::Get().CollectPeers(m_physicsSystem, this, peers);
        if (!peers.empty())
        {
            peerSnapshots.reserve(peers.size());
            for (JoltWaterVolume* peer : peers)
            {
                JoltWaterVolumeSnapshot peerSnapshot = peer->GetSnapshot();
                if (peerSnapshot.m_enabled && peerSnapshot.m_worldBounds.IsValid())
                {
                    peerSnapshots.push_back(peerSnapshot);
                }
            }
        }

        JoltWaterVolumeSnapshot self;
        self.m_worldTransform = worldTransform;
        self.m_dimensions = dimensions;
        self.m_worldBounds = worldBounds;
        self.m_enabled = true;
        self.m_owner = this;

        // Only wake sleeping bodies when the water itself has changed since the last pass.
        // Waking them every step would keep every floating body permanently awake.
        const AZ::u32 generation = m_generation.load(AZStd::memory_order_relaxed);
        const bool wakeSleepers = generation != m_wakeGeneration;
        m_wakeGeneration = generation;
        AZStd::vector<JPH::BodyID> toWake;

        // Copied once for the whole step rather than consulted per body: the surface
        // function is swappable from gameplay, and holding its mutex across the loop would
        // block a setter for the whole step.
        SurfaceFunction customSurface;
        {
            AZStd::lock_guard lock(m_surfaceFunctionMutex);
            customSurface = m_surfaceFunction;
        }

        const AZ::u64 collidesWithMask = m_collidesWithMask.load(AZStd::memory_order_relaxed);
        const bool hasOverrides = !JoltBuoyancyOverrideRegistry::Get().IsEmpty();

        // Object layers repeat heavily across bodies, so each distinct one is resolved once
        // per step instead of asking the physics gem per body.
        AZStd::unordered_map<AZ::u32, bool> layerFilterCache;

        AZStd::unordered_map<AZ::EntityId, float> nowSubmerged;
        AZStd::unordered_map<AZ::EntityId, float> entrySpeeds;

        // Who was in the water last step. A sleeping body is not processed, but it has not
        // left the water either, so it is carried across rather than reported as an exit.
        AZStd::unordered_map<AZ::EntityId, float> previouslySubmerged;
        {
            AZStd::lock_guard lock(m_submergedMutex);
            previouslySubmerged = m_submerged;
        }

        // Step listeners run with every body mutex already held, so bodies are read
        // through the no-lock interface; taking a lock here would deadlock.
        const JPH::BodyLockInterface& bodyLockInterface = m_physicsSystem->GetBodyLockInterfaceNoLock();

        int submergedCount = 0;
        for (const JPH::BodyID& bodyId : collector.mHits)
        {
            JPH::Body* body = bodyLockInterface.TryGetBody(bodyId);
            if (!body || !body->IsDynamic())
            {
                continue;
            }

            if (!body->IsActive())
            {
                // Asleep. ApplyBuoyancyImpulse would not wake it, so without this a body
                // that settled before the water arrived is never touched again. Waking
                // takes the body mutex, which is held right now, so only the id is queued.
                if (wakeSleepers)
                {
                    toWake.push_back(bodyId);
                }

                // Falling asleep is not leaving the water. Carrying it across keeps it out
                // of the exit events and keeps its last submerged fraction readable.
                const AZ::EntityId sleepingEntityId(body->GetUserData());
                const auto wasSubmerged = previouslySubmerged.find(sleepingEntityId);
                if (wasSubmerged != previouslySubmerged.end())
                {
                    nowSubmerged.emplace(sleepingEntityId, wasSubmerged->second);
                }
                continue;
            }

            // With overlapping volumes, the one holding the body deepest below its surface
            // owns it. Every volume computes this from the same data and reaches the same
            // answer, so it does not matter which order Jolt runs the listener jobs in.
            if (!peerSnapshots.empty())
            {
                const AZ::Vector3 bodyPosition = FromJolt(JPH::Vec3(body->GetCenterOfMassPosition()));
                if (self.Contains(bodyPosition))
                {
                    const float ownDepth = self.SubmersionDepth(bodyPosition);
                    bool ownedByPeer = false;
                    for (const JoltWaterVolumeSnapshot& peerSnapshot : peerSnapshots)
                    {
                        if (!peerSnapshot.Contains(bodyPosition))
                        {
                            continue;
                        }
                        const float peerDepth = peerSnapshot.SubmersionDepth(bodyPosition);
                        // Pointer order breaks an exact tie, so two identical volumes still
                        // settle on one owner instead of both claiming or both skipping.
                        if (peerDepth > ownDepth || (peerDepth == ownDepth && peerSnapshot.m_owner < self.m_owner))
                        {
                            ownedByPeer = true;
                            break;
                        }
                    }
                    if (ownedByPeer)
                    {
                        continue;
                    }
                }
            }

            // The physics gem stamps every body it creates with its entity id, which is
            // what makes per-body overrides and enter/exit notifications possible at all.
            const AZ::EntityId bodyEntityId(body->GetUserData());

            // Bodies the volume's collision group excludes never reach the solver, so a
            // volume can be made to float debris while ignoring the player.
            if (collidesWithMask != ~0ull &&
                !ObjectLayerPassesFilter(static_cast<AZ::u32>(body->GetObjectLayer()), collidesWithMask, layerFilterCache))
            {
                continue;
            }

            JoltBuoyancyOverride bodyOverride;
            if (hasOverrides)
            {
                bodyOverride = JoltBuoyancyOverrideRegistry::Get().Find(bodyEntityId);
                if (bodyOverride.m_excluded)
                {
                    continue;
                }
            }

            // Jolt's buoyancy factor is relative: 1 is neutral, above 1 floats. Deriving
            // it from the body's own density is what makes a dense body sink and a light
            // one bob, instead of every body behaving the same way. An explicit override is
            // how a sealed hull floats regardless of what its collider volume implies.
            float buoyancy = 1.0f;
            const JPH::Shape* shape = body->GetShape();
            if (bodyOverride.m_mode == JoltBuoyancyMode::Explicit)
            {
                buoyancy = bodyOverride.m_factor;
            }
            else
            {
                const float shapeVolume = shape ? shape->GetVolume() : 0.0f;
                const float inverseMass = body->GetMotionProperties()->GetInverseMass();
                if (shapeVolume > 0.0f && inverseMass > 0.0f)
                {
                    const float bodyDensity = (1.0f / inverseMass) / shapeVolume;
                    buoyancy = settings.m_fluidDensity / AZStd::max(bodyDensity, 0.001f);
                }
            }

            // Sampled per body rather than once for the volume, which is what lets the
            // surface be a wave instead of a plane: two boats on the same swell sit at
            // different heights and tilt with their own bit of it.
            const AZ::Vector3 bodyPosition = FromJolt(JPH::Vec3(body->GetCenterOfMassPosition()));
            JoltWaterSurfaceSample surface;
            if (customSurface)
            {
                surface = customSurface(bodyPosition);
            }
            else
            {
                surface = EvaluateBuiltInSurface(self, settings, elapsedTime, bodyPosition);
            }

            if (body->ApplyBuoyancyImpulse(
                    ToJoltR(surface.m_position), ToJolt(surface.m_normal), buoyancy, settings.m_linearDrag,
                    settings.m_angularDrag, ToJolt(settings.m_fluidVelocity), gravity, inContext.mDeltaTime))
            {
                ++submergedCount;

                float submergedFraction = 0.0f;
                if (settings.m_reportSubmergedFraction && shape)
                {
                    // Jolt works this out inside ApplyBuoyancyImpulse but keeps it, so
                    // getting at it means walking the shape again - hence the opt-in.
                    const JPH::Plane surfacePlane =
                        JPH::Plane::sFromPointAndNormal(ToJolt(surface.m_position), ToJolt(surface.m_normal));
                    float totalVolume = 0.0f;
                    float submergedVolume = 0.0f;
                    JPH::Vec3 centerOfBuoyancy = JPH::Vec3::sZero();
                    shape->GetSubmergedVolume(
                        body->GetCenterOfMassTransform().ToMat44(), JPH::Vec3::sReplicate(1.0f), surfacePlane, totalVolume,
                        submergedVolume, centerOfBuoyancy JPH_IF_DEBUG_RENDERER(, JPH::RVec3::sZero()));
                    submergedFraction = totalVolume > 0.0f ? submergedVolume / totalVolume : 0.0f;
                }

                if (bodyEntityId.IsValid())
                {
                    const float entrySpeed = AZStd::abs(FromJolt(body->GetLinearVelocity()).Dot(surface.m_normal));
                    nowSubmerged.emplace(bodyEntityId, submergedFraction);
                    entrySpeeds.emplace(bodyEntityId, entrySpeed);
                }
            }
        }

        m_submergedBodyCount.store(submergedCount, AZStd::memory_order_relaxed);

        if (!toWake.empty())
        {
            AZStd::lock_guard lock(m_pendingWakeMutex);
            m_pendingWake.insert(m_pendingWake.end(), toWake.begin(), toWake.end());
        }

        PublishSubmergedSet(nowSubmerged, entrySpeeds);
    }

    void JoltWaterVolume::PublishSubmergedSet(
        const AZStd::unordered_map<AZ::EntityId, float>& nowSubmerged,
        const AZStd::unordered_map<AZ::EntityId, float>& entrySpeeds)
    {
        AZStd::vector<JoltWaterVolumeEvent> events;
        {
            AZStd::lock_guard lock(m_submergedMutex);
            for (const auto& [entityId, fraction] : nowSubmerged)
            {
                if (m_submerged.find(entityId) == m_submerged.end())
                {
                    JoltWaterVolumeEvent entered;
                    entered.m_bodyEntityId = entityId;
                    entered.m_entered = true;
                    const auto speed = entrySpeeds.find(entityId);
                    entered.m_speed = speed != entrySpeeds.end() ? speed->second : 0.0f;
                    events.push_back(entered);
                }
            }
            for (const auto& [entityId, fraction] : m_submerged)
            {
                if (nowSubmerged.find(entityId) == nowSubmerged.end())
                {
                    JoltWaterVolumeEvent exited;
                    exited.m_bodyEntityId = entityId;
                    exited.m_entered = false;
                    events.push_back(exited);
                }
            }
            m_submerged = nowSubmerged;

            // Released as soon as it empties: this map lives on a volume that may outlive
            // the allocator, and an empty map that still owns buckets reads as a leak.
            if (m_submerged.empty())
            {
                AZStd::unordered_map<AZ::EntityId, float>().swap(m_submerged);
            }
        }

        if (!events.empty())
        {
            AZStd::lock_guard lock(m_pendingEventMutex);
            m_pendingEvents.insert(m_pendingEvents.end(), events.begin(), events.end());
        }
    }

} // namespace JoltBuoyancy
