#include <Clients/JoltWaterVolume.h>

#include <AzCore/Console/IConsole.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/MathUtils.h>
#include <AzCore/std/limits.h>
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
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/PhysicsSystem.h>

namespace JoltBuoyancy
{
    namespace
    {
        //! Jolt's own diagnostic for buoyancy: it draws the slice of each shape that is
        //! under the surface, the centre of buoyancy, and the submerged volume. Far more
        //! informative than the translucent box when a body floats at the wrong height,
        //! because it shows what the solver thinks is wet rather than where the water is.
        //!
        //! A static inside Jolt, so it needs the debug renderer compiled in - profile and
        //! debug builds have it, release does not.
        //! Out of line because a preprocessor conditional cannot sit inside a macro
        //! invocation, and AZ_CVAR takes the handler as an argument.
        void SetDrawSubmergedVolumes([[maybe_unused]] bool enabled)
        {
#ifdef JPH_DEBUG_RENDERER
            JPH::Shape::sDrawSubmergedVolumes = enabled;
#else
            AZ_Warning("JoltBuoyancy", !enabled,
                "jolt_DebugSubmergedVolumes needs a build with Jolt's debug renderer, which release does not have.");
#endif
        }

        AZ_CVAR(bool, jolt_DebugSubmergedVolumes, false,
            [](const bool& enabled)
            {
                SetDrawSubmergedVolumes(enabled);
            },
            AZ::ConsoleFunctorFlags::Null,
            "Draw the submerged part of each shape, its centre of buoyancy and its submerged volume.");

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

        // Into the volume's own space, so a rotated volume is tested against its real
        // region rather than against the axis-aligned bounds of its corners.
        const AZ::Vector3 localPoint = m_worldTransform.GetInverse().TransformPoint(worldPoint);
        if (m_shape == JoltWaterVolumeShape::Sphere)
        {
            const float radius = m_dimensions.GetX() * 0.5f;
            return localPoint.GetLengthSq() <= radius * radius;
        }

        const AZ::Vector3 halfExtents = m_dimensions * 0.5f;
        if (m_shape == JoltWaterVolumeShape::Plane)
        {
            // No floor at the authored Z dimension: anything below the surface inside the
            // horizontal extent is in the water, however deep. A body sinking past the
            // bottom of a box stops being wet, which is wrong for open sea.
            //
            // Bounded by MaxDepth all the same, and deliberately by the same number
            // RebuildBoundsUnlocked gives the broadphase. The two used to disagree - this
            // said bottomless while the query box stopped at the authored dimension - so a
            // body sinking past it was reported as underwater and silently got nothing.
            const float floorHeight = halfExtents.GetZ() - AZ::GetMax(m_maxDepth, 0.0f);
            return AZStd::abs(localPoint.GetX()) <= halfExtents.GetX() &&
                AZStd::abs(localPoint.GetY()) <= halfExtents.GetY() && localPoint.GetZ() <= halfExtents.GetZ() &&
                localPoint.GetZ() >= floorHeight;
        }
        return localPoint.GetAbs().IsLessEqualThan(halfExtents);
    }

    float JoltWaterVolumeSnapshot::SubmersionDepth(const AZ::Vector3& worldPoint) const
    {
        const AZ::Vector3 surfaceNormal = m_worldTransform.TransformVector(AZ::Vector3::CreateAxisZ()).GetNormalizedSafe();
        // A sphere is filled to its brim, so its surface is at the top of the sphere.
        const float surfaceHeight = m_shape == JoltWaterVolumeShape::Sphere
            ? m_dimensions.GetX() * 0.5f
            : m_dimensions.GetZ() * 0.5f;
        const AZ::Vector3 surfacePosition = m_worldTransform.TransformPoint(AZ::Vector3(0.0f, 0.0f, surfaceHeight));
        return (surfacePosition - worldPoint).Dot(surfaceNormal);
    }

    bool JoltWaterVolume::HasWaves(const JoltWaterVolumeSettings& settings)
    {
        return settings.m_wavesEnabled && settings.m_spectrum.m_beaufort > 0.0f &&
            settings.m_spectrum.m_amplitudeScale > 0.0f;
    }

    JoltWaterSurfaceSample JoltWaterVolume::EvaluateBuiltInSurface(
        const JoltWaterVolumeSnapshot& snapshot,
        const JoltWaterVolumeSettings& settings,
        const JoltGerstnerWaves& waves,
        const AZ::Vector3& worldPoint)
    {
        const AZ::Transform& transform = snapshot.m_worldTransform;
        // A sphere is filled to its brim, so its surface is the top of the sphere.
        const float meanHeight = snapshot.m_shape == JoltWaterVolumeShape::Sphere
            ? snapshot.m_dimensions.GetX() * 0.5f
            : snapshot.m_dimensions.GetZ() * 0.5f;

        if (!HasWaves(settings) || waves.IsEmpty())
        {
            JoltWaterSurfaceSample flat;
            flat.m_normal = transform.TransformVector(AZ::Vector3::CreateAxisZ()).GetNormalizedSafe();
            flat.m_position = transform.TransformPoint(AZ::Vector3(0.0f, 0.0f, meanHeight));
            return flat;
        }

        // Everything happens in the volume's own space, so the sea rides a tilted volume
        // instead of ignoring the tilt and staying world-flat.
        const AZ::Vector3 localPoint = transform.GetInverse().TransformPoint(worldPoint);
        const JoltGerstnerSample wave =
            waves.SampleAbove(AZ::Vector2(localPoint.GetX(), localPoint.GetY()), meanHeight);

        JoltWaterSurfaceSample sample;
        sample.m_position = transform.TransformPoint(wave.m_position);
        sample.m_normal = transform.TransformVector(wave.m_normal).GetNormalizedSafe();
        sample.m_velocity = transform.TransformVector(wave.m_velocity);
        sample.m_jacobian = wave.m_jacobian;
        return sample;
    }

    JoltWaterSurfaceSample JoltWaterVolume::SampleAcrossFootprint(
        const JoltWaterVolumeSnapshot& snapshot,
        const JoltWaterVolumeSettings& settings,
        const JoltGerstnerWaves& waves,
        const SurfaceFunction& customSurface,
        const AZ::Vector3& bodyPosition,
        float footprintRadius,
        AZ::u32 sampleCount)
    {
        const auto sampleAt = [&](const AZ::Vector3& point)
        {
            return customSurface ? customSurface(point) : EvaluateBuiltInSurface(snapshot, settings, waves, point);
        };

        // A flat surface returns the same plane wherever it is sampled, so spreading the
        // samples would cost four evaluations to learn nothing.
        const bool flat = !customSurface && (!HasWaves(settings) || waves.IsEmpty());
        if (flat || sampleCount <= 1 || footprintRadius <= 0.01f)
        {
            return sampleAt(bodyPosition);
        }

        // Nor can a body much smaller than the waves straddle a crest - every sample lands
        // on effectively the same patch of slope. Multi-sampling exists for hulls long
        // enough to span a wave, and debris is usually the bulk of what is in the water,
        // so skipping it there is most of the saving for none of the fidelity.
        if (!customSurface)
        {
            float shortestWavelength = AZStd::numeric_limits<float>::max();
            for (const JoltGerstnerComponent& component : waves.GetComponents())
            {
                shortestWavelength =
                    AZ::GetMin(shortestWavelength, AZ::Constants::TwoPi / AZ::GetMax(component.m_waveNumber, 1.0e-4f));
            }
            if (footprintRadius * 2.0f < shortestWavelength * 0.25f)
            {
                return sampleAt(bodyPosition);
            }
        }

        // Spread around the body's footprint in the volume's own horizontal plane, so the
        // samples straddle a crest the way the hull does.
        const AZ::Transform& transform = snapshot.m_worldTransform;
        const AZ::Vector3 acrossX = transform.TransformVector(AZ::Vector3::CreateAxisX()).GetNormalizedSafe();
        const AZ::Vector3 acrossY = transform.TransformVector(AZ::Vector3::CreateAxisY()).GetNormalizedSafe();

        const AZ::u32 clampedCount = AZ::GetClamp(sampleCount, 2u, 16u);
        AZ::Vector3 averagePosition = AZ::Vector3::CreateZero();
        AZ::Vector3 averageNormal = AZ::Vector3::CreateZero();
        AZ::Vector3 averageVelocity = AZ::Vector3::CreateZero();
        float averageJacobian = 0.0f;

        for (AZ::u32 index = 0; index < clampedCount; ++index)
        {
            const float angle = AZ::Constants::TwoPi * static_cast<float>(index) / static_cast<float>(clampedCount);
            const AZ::Vector3 offset =
                acrossX * (footprintRadius * std::cos(angle)) + acrossY * (footprintRadius * std::sin(angle));
            const JoltWaterSurfaceSample sample = sampleAt(bodyPosition + offset);

            // Only the height of each sample matters for the plane; keeping the offset
            // would just re-describe the ring the samples were taken on.
            averagePosition += sample.m_position - offset;
            averageNormal += sample.m_normal;
            averageVelocity += sample.m_velocity;
            averageJacobian += sample.m_jacobian;
        }

        const float inverseCount = 1.0f / static_cast<float>(clampedCount);
        JoltWaterSurfaceSample fitted;
        fitted.m_position = averagePosition * inverseCount;
        fitted.m_normal = averageNormal.GetNormalizedSafe();
        fitted.m_velocity = averageVelocity * inverseCount;
        fitted.m_jacobian = averageJacobian * inverseCount;
        return fitted;
    }

    JoltWaterSurfaceSample JoltWaterVolume::EvaluateSurface(const AZ::Vector3& worldPoint) const
    {
        // One pass over the locks rather than three. This used to take the surface-function
        // mutex, then the settings mutex twice more inside GetSnapshot and GetSettings,
        // which OnStep avoids by hoisting them out of its body loop but every gameplay
        // caller paid - and multi-sampling makes this path much hotter.
        SurfaceFunction customSurface;
        {
            AZStd::lock_guard lock(m_surfaceFunctionMutex);
            customSurface = m_surfaceFunction;
        }
        if (customSurface)
        {
            return customSurface(worldPoint);
        }

        JoltWaterVolumeSnapshot snapshot;
        JoltWaterVolumeSettings settings;
        JoltGerstnerWaves waves;
        {
            AZStd::lock_guard lock(m_settingsMutex);
            snapshot.m_worldTransform = m_worldTransform;
            snapshot.m_dimensions = m_dimensions;
            snapshot.m_worldBounds = m_worldBounds;
            snapshot.m_shape = m_settings.m_shape;
            snapshot.m_maxDepth = m_settings.m_maxDepth;
            settings = m_settings;
            waves = m_waves;
        }
        snapshot.m_enabled = IsEnabled();
        snapshot.m_owner = this;

        return EvaluateBuiltInSurface(snapshot, settings, waves, worldPoint);
    }

    JoltGerstnerWaves JoltWaterVolume::GetWaves() const
    {
        AZStd::lock_guard lock(m_settingsMutex);
        return m_waves;
    }

    float JoltWaterVolume::GetSignificantWaveHeight() const
    {
        AZStd::lock_guard lock(m_settingsMutex);
        return m_waves.GetSignificantWaveHeight();
    }

    AZ::Vector3 JoltWaterVolume::GetWaterVelocityAt(const AZ::Vector3& worldPoint) const
    {
        return GetSettings().m_fluidVelocity + EvaluateSurface(worldPoint).m_velocity;
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

    bool JoltWaterVolume::IsPointUnderwater(const AZ::Vector3& worldPoint) const
    {
        return GetSnapshot().Contains(worldPoint) && GetDepthAt(worldPoint) >= 0.0f;
    }

    float JoltWaterVolume::GetDepthAt(const AZ::Vector3& worldPoint) const
    {
        // Measured against the live surface, so it follows the waves rather than the
        // volume's flat lid.
        const JoltWaterSurfaceSample surface = EvaluateSurface(worldPoint);
        return (surface.m_position - worldPoint).Dot(surface.m_normal);
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
        const JoltWaterVolumeSettings settings = GetSettings();
        snapshot.m_shape = settings.m_shape;
        snapshot.m_maxDepth = settings.m_maxDepth;
        snapshot.m_fluidDensity = settings.m_fluidDensity;
        snapshot.m_fluidVelocity = settings.m_fluidVelocity;
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
        RebuildBoundsUnlocked();
    }

    void JoltWaterVolume::RebuildBoundsUnlocked()
    {
        // Waves lift the surface above the volume's own lid, and the broadphase query is
        // the only thing deciding which bodies are looked at. Without this padding a light
        // body riding a crest leaves the query box, stops being affected, falls back in,
        // and oscillates.
        AZ::Vector3 padded = m_dimensions;
        if (HasWaves(m_settings) && !m_waves.IsEmpty())
        {
            // Every component contributes, vertically and horizontally. A Gerstner wave
            // drags points sideways toward the crest as well as lifting them, and the
            // horizontal part was not accounted for at all when this was a single sine.
            const float verticalReach = m_waves.GetMaximumHeight();
            const float horizontalReach = m_waves.GetMaximumHorizontalDisplacement();
            padded.SetZ(padded.GetZ() + 2.0f * verticalReach);
            padded.SetX(padded.GetX() + 2.0f * horizontalReach);
            padded.SetY(padded.GetY() + 2.0f * horizontalReach);
        }

        // A sphere is sized by its X extent alone, so its bounds are a cube of that size.
        if (m_settings.m_shape == JoltWaterVolumeShape::Sphere)
        {
            padded = AZ::Vector3(m_dimensions.GetX());
        }

        const AZ::Vector3 halfExtents = padded * 0.5f;
        AZ::Vector3 localMin = -halfExtents;
        const AZ::Vector3 localMax = halfExtents;

        // A plane has no floor at the authored Z dimension, and this is the only place that
        // could honour it. The broadphase query decides which bodies are looked at at all,
        // so a symmetric box here meant a plane silently stopped applying buoyancy one
        // Z dimension below its surface while Contains went on calling it water - a body
        // sinking past that just quietly weighed its full dry weight again.
        //
        // Measured from the unpadded surface, so this floor is exactly the one Contains
        // uses. The wave padding raises the lid and must not also lower the floor, or the
        // two would drift apart by the wave height.
        if (m_settings.m_shape == JoltWaterVolumeShape::Plane)
        {
            localMin.SetZ(m_dimensions.GetZ() * 0.5f - AZ::GetMax(m_settings.m_maxDepth, 0.0f));
        }

        // The broadphase is queried with an axis-aligned box, so a rotated volume needs
        // the bounds of its corners rather than of its dimensions.
        m_worldBounds = AZ::Aabb::CreateNull();
        for (int corner = 0; corner < 8; ++corner)
        {
            const AZ::Vector3 localCorner(
                (corner & 1) ? localMax.GetX() : localMin.GetX(),
                (corner & 2) ? localMax.GetY() : localMin.GetY(),
                (corner & 4) ? localMax.GetZ() : localMin.GetZ());
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

        {
            // A different group means the cached per-layer answers were computed against
            // the old mask.
            AZStd::lock_guard cacheLock(m_layerFilterCacheMutex);
            m_layerFilterCache.clear();
        }

        AZStd::lock_guard lock(m_settingsMutex);
        const bool spectrumChanged = m_waves.IsEmpty() ||
            !AZ::IsClose(settings.m_spectrum.m_beaufort, m_settings.m_spectrum.m_beaufort) ||
            !AZ::IsClose(settings.m_spectrum.m_fetch, m_settings.m_spectrum.m_fetch) ||
            !settings.m_spectrum.m_windDirection.IsClose(m_settings.m_spectrum.m_windDirection) ||
            !AZ::IsClose(settings.m_spectrum.m_directionalSpread, m_settings.m_spectrum.m_directionalSpread) ||
            settings.m_spectrum.m_componentCount != m_settings.m_spectrum.m_componentCount ||
            !AZ::IsClose(settings.m_spectrum.m_amplitudeScale, m_settings.m_spectrum.m_amplitudeScale) ||
            !AZ::IsClose(settings.m_spectrum.m_steepness, m_settings.m_spectrum.m_steepness) ||
            // Depth is consumed only inside Synthesise, in the dispersion solve, so a depth
            // that changes without resynthesising leaves every component on its deep-water
            // wave number - the setting silently does nothing.
            !AZ::IsClose(settings.m_spectrum.m_waterDepth, m_settings.m_spectrum.m_waterDepth) ||
            settings.m_spectrum.m_seed != m_settings.m_spectrum.m_seed;
        // m_speedScale is deliberately absent: it is applied per step in Advance and
        // resynthesising for it would throw away the accumulated phases for nothing.

        m_settings = settings;

        // Resynthesising throws away the accumulated phases, so it only happens when the
        // spectrum actually changed. Scaling speed alone leaves the waves where they are.
        if (spectrumChanged)
        {
            m_waves.Synthesise(m_settings.m_spectrum);
        }

        // Wave reach and shape both change the bounds the broadphase is queried with.
        RebuildBoundsUnlocked();
    }

    bool JoltWaterVolume::ObjectLayerPassesFilter(AZ::u32 objectLayer, AZ::u64 collidesWithMask) const
    {
        {
            AZStd::lock_guard lock(m_layerFilterCacheMutex);
            const auto cached = m_layerFilterCache.find(objectLayer);
            if (cached != m_layerFilterCache.end())
            {
                return cached->second;
            }
        }

        // Only reached the first time this volume sees a given object layer. Layers are
        // handed out as bodies are created and never change meaning, so after the opening
        // steps this never runs again and the physics bus is left alone on the job threads.
        bool matches = true;
        JoltPhysics::JoltPhysicsSystemRequestBus::BroadcastResult(
            matches, &JoltPhysics::JoltPhysicsSystemRequests::ObjectLayerMatchesQueryMask, objectLayer, collidesWithMask);

        AZStd::lock_guard lock(m_layerFilterCacheMutex);
        m_layerFilterCache[objectLayer] = matches;
        return matches;
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

            // Publishing an empty set rather than just returning: a disabled volume holds
            // nothing, so everything it held has left. Skipping this left
            // GetSubmergedFraction answering with pre-disable values and delayed every exit
            // until the volume was switched back on.
            PublishSubmergedSet({}, {});
            m_ownedLastStep.clear();
            return;
        }

        // Copy what gameplay may be writing, so the rest of the step works from a
        // consistent snapshot.
        JoltWaterVolumeSettings settings;
        AZ::Transform worldTransform;
        AZ::Vector3 dimensions;
        AZ::Aabb worldBounds;
        JoltGerstnerWaves waves;
        {
            AZStd::lock_guard lock(m_settingsMutex);
            settings = m_settings;
            worldTransform = m_worldTransform;
            dimensions = m_dimensions;
            worldBounds = m_worldBounds;

            // Each component carries its own phase, advanced by its own frequency and
            // wrapped separately. A shared clock has nothing to wrap to once the periods
            // stop being multiples of each other.
            m_waves.Advance(inContext.mDeltaTime, settings.m_spectrum.m_speedScale);
            waves = m_waves;
        }

        if (!worldBounds.IsValid())
        {
            return;
        }

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
        self.m_shape = settings.m_shape;
        self.m_maxDepth = settings.m_maxDepth;
        self.m_fluidDensity = settings.m_fluidDensity;
        self.m_fluidVelocity = settings.m_fluidVelocity;
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

        AZStd::unordered_map<AZ::EntityId, float> nowSubmerged;
        AZStd::unordered_map<AZ::EntityId, float> entrySpeeds;
        AZStd::unordered_set<AZ::EntityId> ownedThisStep;

        // Added mass resists the change in velocity, so it needs last step's.
        AZStd::unordered_map<AZ::EntityId, AZ::Vector3> velocitiesThisStep;
        const AZStd::unordered_map<AZ::EntityId, AZ::Vector3>& previousVelocities = m_previousVelocities;

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

            // The physics gem stamps every body it creates with its entity id, which is
            // what makes per-body overrides and enter/exit notifications possible at all.
            const AZ::EntityId bodyEntityId(body->GetUserData());

            // The fluid this body sees. Per body, not per volume: the blend below is
            // specific to where this body sits, and writing it back into the shared
            // settings would leak one body's blend into the next.
            float bodyFluidDensity = settings.m_fluidDensity;
            AZ::Vector3 bodyFluidVelocity = settings.m_fluidVelocity;

            // With overlapping volumes, the one holding the body deepest below its surface
            // owns it. Every volume computes this from the same data and reaches the same
            // answer, so it does not matter which order Jolt runs the listener jobs in.
            if (!peerSnapshots.empty())
            {
                const AZ::Vector3 bodyPosition = FromJolt(JPH::Vec3(body->GetCenterOfMassPosition()));
                const bool selfContains = self.Contains(bodyPosition);
                const float ownDepth = self.SubmersionDepth(bodyPosition);
                const bool ownedLastStep = m_ownedLastStep.find(bodyEntityId) != m_ownedLastStep.end();

                bool ownedByPeer = false;
                for (const JoltWaterVolumeSnapshot& peerSnapshot : peerSnapshots)
                {
                    if (!peerSnapshot.Contains(bodyPosition))
                    {
                        continue;
                    }

                    // The body's centre is in a peer but not in us. This is the straddling
                    // case: the body overlaps our query box while sitting in the neighbour.
                    // Arbitrating only when we contained the centre meant neither volume
                    // stood down here, and Jolt measures the whole shape against each
                    // surface, so the body took roughly double the impulse.
                    if (!selfContains)
                    {
                        ownedByPeer = true;
                        break;
                    }

                    const float peerDepth = peerSnapshot.SubmersionDepth(bodyPosition);

                    // Ownership sticks. Without the margin a body drifting along the seam
                    // between two volumes changes hands every few steps, and its current
                    // and fluid density jump each time it does.
                    const float margin = ownedLastStep ? AZStd::max(settings.m_ownershipHysteresis, 0.0f) : 0.0f;
                    if (peerDepth > ownDepth + margin)
                    {
                        ownedByPeer = true;
                        break;
                    }

                    // An exact tie still has to resolve to exactly one owner, or two
                    // identical volumes would both claim or both stand down. Ordered with
                    // AZStd::less rather than raw < on unrelated pointers, which is
                    // unspecified.
                    if (peerDepth == ownDepth && !ownedLastStep &&
                        AZStd::less<const void*>{}(peerSnapshot.m_owner, self.m_owner))
                    {
                        ownedByPeer = true;
                        break;
                    }
                }

                if (ownedByPeer)
                {
                    continue;
                }
                ownedThisStep.insert(bodyEntityId);

                // Owning a body outright and ignoring the neighbour makes an estuary a
                // step change: river current one frame, still ocean the next. The owner
                // still applies the only impulse - no double buoyancy - but blends its
                // fluid toward any peer that also holds the body, weighted by how deeply
                // each one does. The crossing becomes a gradient instead of an edge.
                float ownWeight = AZ::GetMax(ownDepth, 0.0f) + 1.0e-3f;
                float totalWeight = ownWeight;
                AZ::Vector3 blendedVelocity = self.m_fluidVelocity * ownWeight;
                float blendedDensity = self.m_fluidDensity * ownWeight;

                for (const JoltWaterVolumeSnapshot& peerSnapshot : peerSnapshots)
                {
                    if (!peerSnapshot.Contains(bodyPosition))
                    {
                        continue;
                    }
                    const float peerWeight = AZ::GetMax(peerSnapshot.SubmersionDepth(bodyPosition), 0.0f);
                    if (peerWeight <= 0.0f)
                    {
                        continue;
                    }
                    blendedVelocity += peerSnapshot.m_fluidVelocity * peerWeight;
                    blendedDensity += peerSnapshot.m_fluidDensity * peerWeight;
                    totalWeight += peerWeight;
                }

                if (totalWeight > 0.0f)
                {
                    bodyFluidVelocity = blendedVelocity / totalWeight;
                    bodyFluidDensity = blendedDensity / totalWeight;
                }
            }

            // Bodies the volume's collision group excludes never reach the solver, so a
            // volume can be made to float debris while ignoring the player.
            if (collidesWithMask != ~0ull &&
                !ObjectLayerPassesFilter(static_cast<AZ::u32>(body->GetObjectLayer()), collidesWithMask))
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
                    buoyancy = bodyFluidDensity / AZStd::max(bodyDensity, 0.001f);
                }
            }

            // Sampled per body rather than once for the volume, which is what lets the
            // surface be a wave instead of a plane: two boats on the same swell sit at
            // different heights and tilt with their own bit of it.
            //
            // And sampled at several points across the body rather than one at its centre,
            // so a hull long enough to straddle a crest pitches on it instead of merely
            // riding up and down. The footprint comes from the body's own bounds.
            const AZ::Vector3 bodyPosition = FromJolt(JPH::Vec3(body->GetCenterOfMassPosition()));
            const JPH::AABox bodyBounds = body->GetWorldSpaceBounds();
            const float footprintRadius =
                0.5f * AZ::GetMax(bodyBounds.GetSize().GetX(), bodyBounds.GetSize().GetY());
            const JoltWaterSurfaceSample surface = SampleAcrossFootprint(
                self, settings, waves, customSurface, bodyPosition, footprintRadius,
                settings.m_surfaceSamplesPerBody);

            // Read before the impulse: ApplyBuoyancyImpulse damps the velocity, so sampling
            // afterwards reports a splash slower than the one that actually happened.
            const float approachSpeed = AZStd::abs(FromJolt(body->GetLinearVelocity()).Dot(surface.m_normal));

            // Per-body drag multipliers: a streamlined hull cuts through water that its
            // weight alone would say should slow it down.
            //
            // Jolt takes one scalar, so it gets the isotropic floor of the per-axis scales
            // and ApplyExtraHydrodynamics adds back whatever each axis asked for above it.
            // Handing Jolt the largest instead would mean over-damping the streamlined
            // axis and having no way to take it back.
            const AZ::Vector3 perAxisScale =
                bodyOverride.m_directionalDrag * AZStd::max(bodyOverride.m_linearDragMultiplier, 0.0f);
            const float isotropicScale = AZ::GetMax(perAxisScale.GetMinElement(), 0.0f);
            const float angularDrag = settings.m_angularDrag * AZStd::max(bodyOverride.m_angularDragMultiplier, 0.0f);

            const bool needsHydrodynamics =
                bodyOverride.m_addedMass > 0.0f || perAxisScale.GetMaxElement() > isotropicScale + 1.0e-4f;

            // The volume's own current plus the water's orbital motion at this body.
            // Passing only the current makes the sea a conveyor belt; the orbital part is
            // what makes a boat surge down the face of a swell and flotsam gather in lines.
            const AZ::Vector3 waterVelocity = bodyFluidVelocity + surface.m_velocity;

            // How much of the body is wet, asked for up front instead of letting
            // ApplyBuoyancyImpulse work it out internally and keep it. Jolt has a second
            // overload that takes these figures rather than recomputing them, so this is
            // the same single walk of the shape either way - and it is what makes both
            // corrections below possible, since both need the wet fraction before the
            // impulse is applied rather than after.
            float totalVolume = 0.0f;
            float submergedVolume = 0.0f;
            JPH::Vec3 relativeCenterOfBuoyancy = JPH::Vec3::sZero();
            body->GetSubmergedVolume(
                ToJoltR(surface.m_position), ToJolt(surface.m_normal), totalVolume, submergedVolume,
                relativeCenterOfBuoyancy);
            if (submergedVolume <= 0.0f)
            {
                // Exactly the test ApplyBuoyancyImpulse makes before doing anything else,
                // so skipping the call here changes nothing but the wasted work.
                continue;
            }

            const float submergedFraction = totalVolume > 0.0f ? submergedVolume / totalVolume : 0.0f;

            // Jolt scales its angular drag by the submerged fraction but takes the linear
            // drag area from the whole shape's local bounding box, so a hull floating with
            // a tenth of itself wet drags as though fully immersed - superstructure
            // included. That is the entire surface-vessel regime, which is most of what
            // floats. Scaling by the same fraction Jolt already applies to the angular half
            // keeps the two consistent rather than inventing a second model.
            //
            // The density correction is a separate problem in the same number. Jolt derives
            // the fluid density it drags with from the buoyancy factor:
            //     fluid_density = inBuoyancy / (totalVolume * inverseMass)
            // In Automatic mode that is exactly right - the factor is the ratio of fluid to
            // body density, so the body density multiplies straight back out and the
            // correction below comes to 1. Under an Explicit factor it is wrong: the number
            // is an authored fudge, and a sealed hull asking for 3 to float correctly would
            // pay three times the drag for it. Dividing that implied density back out means
            // an explicit factor buys buoyancy and only buoyancy.
            const float inverseMass = body->GetMotionProperties()->GetInverseMass();
            float dragDensityCorrection = 1.0f;
            if (buoyancy > 0.0f && totalVolume > 0.0f && inverseMass > 0.0f)
            {
                const float joltImpliedDensity = buoyancy / (totalVolume * inverseMass);
                if (joltImpliedDensity > 0.0f)
                {
                    dragDensityCorrection = bodyFluidDensity / joltImpliedDensity;
                }
            }

            const float linearDrag =
                settings.m_linearDrag * isotropicScale * submergedFraction * dragDensityCorrection;

            if (body->ApplyBuoyancyImpulse(
                    totalVolume, submergedVolume, relativeCenterOfBuoyancy, buoyancy, linearDrag, angularDrag,
                    ToJolt(waterVelocity), gravity, inContext.mDeltaTime))
            {
                ++submergedCount;

                if (needsHydrodynamics)
                {
                    const auto previous = previousVelocities.find(bodyEntityId);
                    ApplyExtraHydrodynamics(
                        *body, bodyOverride, waterVelocity, bodyFluidDensity, settings.m_linearDrag, isotropicScale,
                        submergedVolume, submergedFraction,
                        previous != previousVelocities.end() ? previous->second : AZ::Vector3::CreateZero(),
                        previous != previousVelocities.end(), inContext.mDeltaTime);
                }

                if (bodyEntityId.IsValid())
                {
                    // The fraction is worked out for every body now that the drag needs it,
                    // so the setting no longer buys back a second walk of the shape - it
                    // only says whether the number is worth keeping.
                    nowSubmerged.emplace(bodyEntityId, settings.m_reportSubmergedFraction ? submergedFraction : 0.0f);
                    entrySpeeds.emplace(bodyEntityId, approachSpeed);
                    velocitiesThisStep.emplace(bodyEntityId, FromJolt(body->GetLinearVelocity()));
                }
            }
        }

        // Counts everything in the water, including the sleepers carried across above. A
        // pool of settled floaters reporting zero would look exactly like a volume that had
        // stopped working, which is the one thing this counter exists to rule out.
        m_submergedBodyCount.store(static_cast<int>(nowSubmerged.size()), AZStd::memory_order_relaxed);
        AZ_UNUSED(submergedCount);

        m_ownedLastStep = AZStd::move(ownedThisStep);
        m_previousVelocities = AZStd::move(velocitiesThisStep);
        if (m_previousVelocities.empty())
        {
            AZStd::unordered_map<AZ::EntityId, AZ::Vector3>().swap(m_previousVelocities);
        }

        if (!toWake.empty())
        {
            AZStd::lock_guard lock(m_pendingWakeMutex);
            m_pendingWake.insert(m_pendingWake.end(), toWake.begin(), toWake.end());
        }

        PublishSubmergedSet(nowSubmerged, entrySpeeds);
    }

    void JoltWaterVolume::ApplyExtraHydrodynamics(
        JPH::Body& body,
        const JoltBuoyancyOverride& bodyOverride,
        const AZ::Vector3& waterVelocity,
        float fluidDensity,
        float baseLinearDrag,
        float isotropicScale,
        float submergedVolume,
        float submergedFraction,
        const AZ::Vector3& previousVelocity,
        bool hadPreviousVelocity,
        float deltaTime)
    {
        JPH::MotionProperties& motion = *body.GetMotionProperties();
        const float inverseMass = motion.GetInverseMass();
        if (inverseMass <= 0.0f || submergedVolume <= 0.0f)
        {
            return;
        }
        const float mass = 1.0f / inverseMass;

        // --- the anisotropic remainder of the drag -------------------------------------
        //
        // Jolt was handed the isotropic floor of the per-axis scales, because that is all a
        // single scalar can carry. Whatever each axis asked for above that floor is applied
        // here, in the same quadratic, area-projected form Jolt uses, so the two halves are
        // the same formula rather than two different models fighting.
        const AZ::Vector3 perAxis = bodyOverride.m_directionalDrag * bodyOverride.m_linearDragMultiplier;
        const AZ::Vector3 remainder(
            AZ::GetMax(perAxis.GetX() - isotropicScale, 0.0f),
            AZ::GetMax(perAxis.GetY() - isotropicScale, 0.0f),
            AZ::GetMax(perAxis.GetZ() - isotropicScale, 0.0f));

        if (baseLinearDrag > 0.0f && remainder.GetMaxElement() > 0.0f)
        {
            const AZ::Vector3 relativeVelocity = waterVelocity - FromJolt(body.GetLinearVelocity());
            const float relativeSpeed = relativeVelocity.GetLength();
            if (relativeSpeed > 1.0e-4f)
            {
                const JPH::Vec3 localSize = body.GetShape()->GetLocalBounds().GetSize();
                // Face areas of the local bounding box, per axis, scaled by how much of the
                // body is actually wet. Without that scaling this repeats the mistake it
                // exists to correct: full bounding-box faces on a hull that is a tenth
                // submerged means the superstructure drags through air.
                const float wetted = AZ::GetClamp(submergedFraction, 0.0f, 1.0f);
                const AZ::Vector3 axisArea = wetted *
                    AZ::Vector3(
                        localSize.GetY() * localSize.GetZ(),
                        localSize.GetZ() * localSize.GetX(),
                        localSize.GetX() * localSize.GetY());

                const JPH::Quat rotation = body.GetRotation();
                const AZ::Vector3 localRelative = FromJolt(rotation.InverseRotate(ToJolt(relativeVelocity)));

                const AZ::Vector3 localImpulse = AZ::Vector3(
                    0.5f * fluidDensity * baseLinearDrag * remainder.GetX() * axisArea.GetX() * deltaTime *
                        localRelative.GetX() * relativeSpeed,
                    0.5f * fluidDensity * baseLinearDrag * remainder.GetY() * axisArea.GetY() * deltaTime *
                        localRelative.GetY() * relativeSpeed,
                    0.5f * fluidDensity * baseLinearDrag * remainder.GetZ() * axisArea.GetZ() * deltaTime *
                        localRelative.GetZ() * relativeSpeed);

                AZ::Vector3 impulse = FromJolt(rotation * ToJolt(localImpulse));

                // Clamped against the velocity it is opposing, exactly as Jolt clamps its
                // own drag: without it a large step or a steep coefficient reverses the
                // body instead of slowing it.
                const AZ::Vector3 deltaVelocity = impulse * inverseMass;
                const float currentSpeed = FromJolt(body.GetLinearVelocity()).GetLength();
                if (deltaVelocity.GetLength() > currentSpeed && deltaVelocity.GetLength() > 0.0f)
                {
                    impulse *= currentSpeed / deltaVelocity.GetLength();
                }
                body.AddImpulse(ToJolt(impulse));
            }
        }

        // --- added mass -----------------------------------------------------------------
        //
        // Water accelerated along with the hull. Doing this properly means adding to the
        // solver's mass matrix, which Jolt does not expose, so this resists the change in
        // velocity after the fact: the body keeps the fraction of its velocity change that
        // its own inertia accounts for, and loses the fraction the entrained water would
        // have absorbed. It is an approximation and behaves like one - it damps
        // acceleration rather than genuinely making the body heavier - but it takes the
        // twitchiness out of heave and pitch, which is what it is for.
        if (bodyOverride.m_addedMass > 0.0f && hadPreviousVelocity)
        {
            const float addedMass = bodyOverride.m_addedMass * fluidDensity * submergedVolume;
            if (addedMass > 0.0f)
            {
                const float ratio = addedMass / (mass + addedMass);
                const AZ::Vector3 velocityChange = FromJolt(body.GetLinearVelocity()) - previousVelocity;
                motion.AddLinearVelocityStep(ToJolt(-ratio * velocityChange));
            }
        }
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
