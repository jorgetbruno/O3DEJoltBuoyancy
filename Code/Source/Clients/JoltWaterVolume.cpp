#include <Clients/JoltWaterVolume.h>

#include <AzCore/Interface/Interface.h>
#include <AzCore/std/parallel/lock.h>

#include <Clients/JoltWaterVolumeRegistry.h>
#include <JoltPhysics/JoltPhysicsBus.h>

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

        // The surface is the volume's local +Z face, so a tilted volume gives a tilted
        // water surface (a sloped river, say) rather than always a horizontal one.
        const AZ::Vector3 surfaceNormal = worldTransform.TransformVector(AZ::Vector3::CreateAxisZ()).GetNormalizedSafe();
        const AZ::Vector3 surfacePosition =
            worldTransform.TransformPoint(AZ::Vector3(0.0f, 0.0f, dimensions.GetZ() * 0.5f));

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

            // Jolt's buoyancy factor is relative: 1 is neutral, above 1 floats. Deriving
            // it from the body's own density is what makes a dense body sink and a light
            // one bob, instead of every body behaving the same way.
            float buoyancy = 1.0f;
            const JPH::Shape* shape = body->GetShape();
            const float shapeVolume = shape ? shape->GetVolume() : 0.0f;
            const float inverseMass = body->GetMotionProperties()->GetInverseMass();
            if (shapeVolume > 0.0f && inverseMass > 0.0f)
            {
                const float bodyDensity = (1.0f / inverseMass) / shapeVolume;
                buoyancy = settings.m_fluidDensity / AZStd::max(bodyDensity, 0.001f);
            }

            if (body->ApplyBuoyancyImpulse(
                    ToJoltR(surfacePosition), ToJolt(surfaceNormal), buoyancy, settings.m_linearDrag,
                    settings.m_angularDrag, ToJolt(settings.m_fluidVelocity), gravity, inContext.mDeltaTime))
            {
                ++submergedCount;
            }
        }

        m_submergedBodyCount.store(submergedCount, AZStd::memory_order_relaxed);

        if (!toWake.empty())
        {
            AZStd::lock_guard lock(m_pendingWakeMutex);
            m_pendingWake.insert(m_pendingWake.end(), toWake.begin(), toWake.end());
        }
    }

} // namespace JoltBuoyancy
