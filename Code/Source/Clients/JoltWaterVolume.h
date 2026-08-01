#pragma once

#include <AzCore/Math/Aabb.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/parallel/atomic.h>
#include <AzCore/std/parallel/mutex.h>

#include <AzFramework/Physics/Common/PhysicsTypes.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsStepListener.h>

namespace JPH
{
    class PhysicsSystem;
}

namespace JoltBuoyancy
{
    //! Settings describing a body of water.
    struct JoltWaterVolumeSettings
    {
        AZ_CLASS_ALLOCATOR(JoltWaterVolumeSettings, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltWaterVolumeSettings, "{E5F6A7B8-C9D0-4B4C-DE5F-6A7B8C9D0E1F}");

        //! kg/m^3. A body floats when its own density is below this and sinks above it.
        float m_fluidDensity = 1000.0f;
        float m_linearDrag = 0.5f;
        float m_angularDrag = 0.05f;
        AZ::Vector3 m_fluidVelocity = AZ::Vector3::CreateZero();
    };

    //! An immutable copy of a volume's placement, taken under its mutex so that one volume
    //! can read another's without racing gameplay writes.
    struct JoltWaterVolumeSnapshot
    {
        AZ::Transform m_worldTransform = AZ::Transform::CreateIdentity();
        AZ::Vector3 m_dimensions = AZ::Vector3::CreateZero();
        AZ::Aabb m_worldBounds = AZ::Aabb::CreateNull();
        bool m_enabled = false;
        //! Identity only, for breaking ties between volumes with equal submersion depth.
        const void* m_owner = nullptr;

        //! Whether a world-space point lies inside the oriented box.
        bool Contains(const AZ::Vector3& worldPoint) const;

        //! How far the point sits below this volume's surface plane, along the surface
        //! normal. Negative above the surface. Defined for a tilted volume too, which is
        //! why it is a plane distance rather than a difference of Z coordinates.
        float SubmersionDepth(const AZ::Vector3& worldPoint) const;
    };

    //! Applies Jolt's buoyancy impulses to every dynamic body overlapping a box of water,
    //! once per physics step.
    //!
    //! This runs as a JPH::PhysicsStepListener rather than on the tick bus so the impulse
    //! lands inside the step it belongs to, at the same delta time the solver is about to
    //! integrate. Jolt calls step listeners with every body mutex already held, so the
    //! bodies are reached through the no-lock interface and none are added or removed.
    //! Jolt also runs step listeners on several jobs at once, so anything shared between
    //! volumes has to be safe to touch concurrently.
    class JoltWaterVolume final : public JPH::PhysicsStepListener
    {
    public:
        AZ_CLASS_ALLOCATOR(JoltWaterVolume, AZ::SystemAllocator);

        JoltWaterVolume() = default;
        ~JoltWaterVolume() override;

        //! Starts applying buoyancy in the given scene. Returns false when the scene is
        //! not backed by Jolt, which is the case with any other physics backend.
        bool Attach(AzPhysics::SceneHandle sceneHandle);

        //! Attaches directly to a Jolt physics system, for callers that own one rather
        //! than reaching it through an AzPhysics scene.
        bool AttachToPhysicsSystem(JPH::PhysicsSystem* physicsSystem);
        void Detach();
        bool IsAttached() const
        {
            return m_physicsSystem != nullptr;
        }

        //! The water box: an oriented box whose local +Z face is the water surface.
        void SetVolume(const AZ::Transform& worldTransform, const AZ::Vector3& dimensions);

        void SetSettings(const JoltWaterVolumeSettings& settings);
        JoltWaterVolumeSettings GetSettings() const;

        void SetEnabled(bool enabled);
        bool IsEnabled() const
        {
            return m_enabled.load(AZStd::memory_order_relaxed);
        }

        //! Bodies affected during the most recent step; useful for diagnosing a volume
        //! that is not doing anything.
        int GetSubmergedBodyCount() const
        {
            return m_submergedBodyCount.load(AZStd::memory_order_relaxed);
        }

        //! Wakes bodies that OnStep found asleep inside a volume that had just moved,
        //! resized or changed settings.
        //!
        //! **Must be called outside the physics step.** Waking a body takes its mutex, and
        //! every body mutex is already held during the step, so doing this from OnStep
        //! would deadlock - which is why OnStep only queues the ids. The component calls
        //! this from the scene's simulation-finish event.
        //!
        //! Without it a sleeping body ignores water that arrives after it settled: a rising
        //! level or a moving volume slides over it and nothing ever wakes it, because
        //! ApplyBuoyancyImpulse does not. Bodies asleep under water that has not changed are
        //! deliberately left alone - they are in equilibrium, and waking them every step
        //! would stop anything from ever sleeping.
        void WakePendingBodies();

        //! Placement copied under the mutex, for peer volumes and for tests.
        JoltWaterVolumeSnapshot GetSnapshot() const;

        // JPH::PhysicsStepListener
        void OnStep(const JPH::PhysicsStepListenerContext& inContext) override;

    private:
        //! Bumped whenever the water itself changes, so OnStep can tell "the body settled
        //! in water that has not moved since" from "the water just changed under it".
        void BumpGeneration();

        JPH::PhysicsSystem* m_physicsSystem = nullptr;

        //! Guards the volume and settings, which gameplay writes and OnStep reads.
        //! Held only long enough to copy them out, once per step.
        mutable AZStd::mutex m_settingsMutex;
        JoltWaterVolumeSettings m_settings;
        AZ::Transform m_worldTransform = AZ::Transform::CreateIdentity();
        AZ::Vector3 m_dimensions = AZ::Vector3(10.0f, 10.0f, 5.0f);
        AZ::Aabb m_worldBounds = AZ::Aabb::CreateNull();

        AZStd::atomic<bool> m_enabled{ true };
        AZStd::atomic<int> m_submergedBodyCount{ 0 };

        AZStd::atomic<AZ::u32> m_generation{ 1 };
        //! The generation the last wake pass covered. Only read and written in OnStep.
        AZ::u32 m_wakeGeneration = 0;

        //! Ids queued by OnStep for WakePendingBodies. Its own mutex, because OnStep runs
        //! on a job thread while the wake happens on the main thread.
        AZStd::mutex m_pendingWakeMutex;
        AZStd::vector<JPH::BodyID> m_pendingWake;
    };
} // namespace JoltBuoyancy
