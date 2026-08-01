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

#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/functional.h>

#include <AzFramework/Physics/Common/PhysicsTypes.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsStepListener.h>

#include <Clients/JoltBuoyancyOverrideRegistry.h>
#include <Clients/JoltGerstnerWaves.h>
#include <JoltBuoyancy/JoltBuoyancyBus.h>

namespace JPH
{
    class Body;
    class PhysicsSystem;
}

namespace JoltBuoyancy
{
    //! Where the water's surface is at one point, which way it faces, and how the water
    //! there is moving.
    struct JoltWaterSurfaceSample
    {
        AZ::Vector3 m_position = AZ::Vector3::CreateZero();
        AZ::Vector3 m_normal = AZ::Vector3::CreateAxisZ();
        //! Orbital velocity of the water at this point, in world space, excluding the
        //! volume's own current. Added to the current before it reaches the solver, which
        //! is what makes a boat surge down the face of a swell.
        AZ::Vector3 m_velocity = AZ::Vector3::CreateZero();
        //! Below 1 where the surface is compressing toward a crest, at or below 0 where it
        //! has folded - which is where a real sea breaks. Drives foam.
        float m_jacobian = 1.0f;
    };

    //! A body crossing into or out of the water, queued during the step and delivered
    //! afterwards.
    struct JoltWaterVolumeEvent
    {
        AZ::EntityId m_bodyEntityId;
        //! Speed along the surface normal on the way in. Zero for an exit.
        float m_speed = 0.0f;
        bool m_entered = false;
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

        JoltWaterVolumeShape m_shape = JoltWaterVolumeShape::Box;

        //! Enough of the peer's settings to blend with. Ownership is all-or-nothing, so
        //! without this a body crossing from a river into a pool changes current and
        //! density in a single step.
        float m_fluidDensity = 1000.0f;
        AZ::Vector3 m_fluidVelocity = AZ::Vector3::CreateZero();

        //! Whether a world-space point lies inside the volume's region.
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

        //! Replaces the built-in flat-or-wavy surface with an arbitrary one, for water that
        //! has to line up with something the gem knows nothing about - a rendered ocean, a
        //! scripted tide. Given a world point, return the surface above it and its normal.
        //!
        //! Called from Jolt's step listener jobs, once per body, so it must be quick and
        //! safe to call from several threads at once. Pass an empty function to go back to
        //! the built-in surface.
        using SurfaceFunction = AZStd::function<JoltWaterSurfaceSample(const AZ::Vector3& worldPoint)>;
        void SetSurfaceFunction(SurfaceFunction surfaceFunction);

        //! Where the surface sits above a world point, using whatever surface the volume is
        //! currently configured with. Public so tests and gameplay can ask the same question
        //! the solver does.
        JoltWaterSurfaceSample EvaluateSurface(const AZ::Vector3& worldPoint) const;

        //! Fraction of the body under the surface last step, 0 to 1. Always 0 unless the
        //! ReportSubmergedFraction setting is on.
        float GetSubmergedFraction(AZ::EntityId bodyEntityId) const;

        //! Inside the volume and below its surface.
        bool IsPointUnderwater(const AZ::Vector3& worldPoint) const;

        //! How far below the surface a point sits, along the surface normal.
        float GetDepthAt(const AZ::Vector3& worldPoint) const;

        //! Whether the wave settings actually produce a moving surface. The renderer uses
        //! this to decide between a flat lid and a tessellated one.
        static bool HasWaves(const JoltWaterVolumeSettings& settings);

        //! A copy of the current wave set, for the renderer to tessellate and for tests.
        //! Taken under the mutex, so a caller can hold it without racing the simulation.
        JoltGerstnerWaves GetWaves() const;

        //! Significant wave height of the current sea state, in metres.
        float GetSignificantWaveHeight() const;

        //! Velocity of the water at a world point, including orbital motion.
        AZ::Vector3 GetWaterVelocityAt(const AZ::Vector3& worldPoint) const;

        //! Hands over the enter and exit events the last step produced, leaving the queue
        //! empty. Called after the step, for the same reason as WakePendingBodies: a
        //! handler is gameplay code and must not run on a physics job with the body
        //! mutexes held.
        void TakePendingEvents(AZStd::vector<JoltWaterVolumeEvent>& outEvents);

        // JPH::PhysicsStepListener
        void OnStep(const JPH::PhysicsStepListenerContext& inContext) override;

    private:
        //! The built-in surface: the local +Z face, rippled by the waves when they are on.
        //! Takes pre-copied state so it can run without touching the mutex.
        static JoltWaterSurfaceSample EvaluateBuiltInSurface(
            const JoltWaterVolumeSnapshot& snapshot,
            const JoltWaterVolumeSettings& settings,
            const JoltGerstnerWaves& waves,
            const AZ::Vector3& worldPoint);

        //! Fits a plane through several surface samples spread across a body's footprint.
        //!
        //! One sample at the centre of mass gives the solver a plane the whole hull shares,
        //! so a boat long enough to straddle a crest rises and falls without pitching.
        //! Averaging positions and normals across the footprint lets the hull sit on the
        //! slope it is actually on.
        static JoltWaterSurfaceSample SampleAcrossFootprint(
            const JoltWaterVolumeSnapshot& snapshot,
            const JoltWaterVolumeSettings& settings,
            const JoltGerstnerWaves& waves,
            const SurfaceFunction& customSurface,
            const AZ::Vector3& bodyPosition,
            float footprintRadius,
            AZ::u32 sampleCount);

        //! Bumped whenever the water itself changes, so OnStep can tell "the body settled
        //! in water that has not moved since" from "the water just changed under it".
        void BumpGeneration();

        //! Recomputes the world bounds the broadphase is queried with. Assumes the caller
        //! holds m_settingsMutex.
        void RebuildBoundsUnlocked();

        //! Whether a body on this object layer is visible to a query using this mask.
        //! Cached for the life of the volume, so the physics bus is only asked the first
        //! time a layer is seen rather than on every step from a job thread.
        bool ObjectLayerPassesFilter(AZ::u32 objectLayer, AZ::u64 collidesWithMask) const;

        //! Applies the hydrodynamics Jolt does not: the anisotropic remainder of a body's
        //! directional drag, and an added-mass approximation.
        //!
        //! Jolt already does more than it is usually given credit for - its drag is
        //! quadratic, and it varies with the projected area of the body's bounding box
        //! along the flow. What it cannot express is a hull being far more streamlined than
        //! its box, and it has no notion of added mass at all.
        //!
        //! The drag split is approximate. Jolt takes one scalar, so it gets the isotropic
        //! floor and this adds the per-axis remainder; but each half clamps itself against
        //! the current velocity independently, and Jolt applies its share at the centre of
        //! buoyancy while this applies at the centre of mass. Setting every axis to the
        //! same value therefore does not reproduce an unsplit drag of that value exactly.
        //! It is close, and the anisotropy is the point.
        static void ApplyExtraHydrodynamics(
            JPH::Body& body,
            const JoltBuoyancyOverride& bodyOverride,
            const AZ::Vector3& waterVelocity,
            float fluidDensity,
            float baseLinearDrag,
            float isotropicScale,
            float submergedVolume,
            const AZ::Vector3& previousVelocity,
            bool hadPreviousVelocity,
            float deltaTime);

        //! Swaps in this step's submerged set, queueing an enter or exit for every body
        //! that joined or left it.
        void PublishSubmergedSet(
            const AZStd::unordered_map<AZ::EntityId, float>& nowSubmerged,
            const AZStd::unordered_map<AZ::EntityId, float>& entrySpeeds);

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

        //! The synthesised waves, and their phases. Advanced by OnStep rather than read
        //! from a clock, so the water only moves when the simulation does and a paused game
        //! has a still surface. Guarded by m_settingsMutex, which OnStep already takes once
        //! per step to copy the settings out.
        JoltGerstnerWaves m_waves;

        //! Set by SetSurfaceFunction. Guarded because gameplay can swap it while a step is
        //! reading it.
        mutable AZStd::mutex m_surfaceFunctionMutex;
        SurfaceFunction m_surfaceFunction;

        //! The collision group mask resolved from the settings' group id, cached because
        //! resolving it means a bus call that must not happen on a physics job.
        AZStd::atomic<AZ::u64> m_collidesWithMask{ ~0ull };

        //! Which object layers pass the mask, remembered across steps. Object layers are
        //! only assigned as bodies are created and never change meaning, so this is filled
        //! in once per layer and read forever after - which keeps the answer off the
        //! physics jobs entirely rather than merely once per step.
        mutable AZStd::mutex m_layerFilterCacheMutex;
        mutable AZStd::unordered_map<AZ::u64, bool> m_layerFilterCache;

        //! Bodies this volume owned last step. Ownership sticks unless a neighbour holds
        //! the body meaningfully deeper, so a body on the seam between two volumes does not
        //! flip owner every few steps.
        AZStd::unordered_set<AZ::EntityId> m_ownedLastStep;

        //! Who was submerged, and by how much, as of the last step. Compared against the
        //! next step's set to raise enter and exit events.
        mutable AZStd::mutex m_submergedMutex;
        AZStd::unordered_map<AZ::EntityId, float> m_submerged;

        //! Last step's linear velocity per body, for the added-mass approximation, which
        //! resists changes in velocity rather than velocity itself.
        AZStd::unordered_map<AZ::EntityId, AZ::Vector3> m_previousVelocities;

        AZStd::mutex m_pendingEventMutex;
        AZStd::vector<JoltWaterVolumeEvent> m_pendingEvents;
    };
} // namespace JoltBuoyancy
