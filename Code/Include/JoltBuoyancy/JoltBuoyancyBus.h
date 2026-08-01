#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Vector2.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/TypeInfo.h>

#include <AzFramework/Physics/Collision/CollisionGroups.h>

namespace JoltBuoyancy
{
    //! How a body's buoyancy factor is decided.
    enum class JoltBuoyancyMode : AZ::u8
    {
        //! Derived from the body's own density: mass over shape volume, against the fluid.
        //! A dense body sinks and a light one floats without anything being authored.
        Automatic = 0,
        //! Taken from the override component verbatim. This is how a sealed hull floats
        //! despite the average density of its collider saying it should not - the shape
        //! volume of a boat's hull says nothing about the air inside it.
        Explicit = 1,
    };

    //! Settings describing a body of water.
    struct JoltWaterVolumeSettings
    {
        AZ_CLASS_ALLOCATOR(JoltWaterVolumeSettings, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltWaterVolumeSettings, "{E5F6A7B8-C9D0-4B4C-DE5F-6A7B8C9D0E1F}");

        static void Reflect(AZ::ReflectContext* context);

        //! kg/m^3. A body floats when its own density is below this and sinks above it.
        float m_fluidDensity = 1000.0f;
        float m_linearDrag = 0.5f;
        float m_angularDrag = 0.05f;
        AZ::Vector3 m_fluidVelocity = AZ::Vector3::CreateZero();

        //! Ripples the surface instead of leaving it a flat plane. The wave rides the
        //! volume's own surface, so a tilted volume gets a tilted, moving surface.
        bool m_wavesEnabled = false;
        //! Crest-to-flat height, in metres.
        float m_waveAmplitude = 0.25f;
        //! Distance between crests, in metres.
        float m_waveLength = 6.0f;
        //! How fast crests travel, in metres per second.
        float m_waveSpeed = 1.5f;
        //! Travel direction in the volume's local XY plane.
        AZ::Vector2 m_waveDirection = AZ::Vector2(1.0f, 0.0f);

        //! Which bodies the volume even looks at. Bodies the group excludes are skipped
        //! before any impulse is computed, so a volume can be made to affect only, say,
        //! debris and not the player.
        AzPhysics::CollisionGroups::Id m_collisionGroupId;

        //! Compute how much of each body is under the surface and publish it through
        //! GetSubmergedFraction. Off by default: Jolt already works this out inside
        //! ApplyBuoyancyImpulse but does not hand it back, so asking for it means walking
        //! the shape a second time.
        bool m_reportSubmergedFraction = false;
    };

    //! Runtime control of a water volume. The volume applies Jolt's buoyancy impulses to
    //! every rigid body overlapping it, once per physics step.
    class JoltWaterVolumeRequests
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltWaterVolumeRequests() = default;

        //! Density of the fluid in kg/m^3. A body floats when it is less dense than this
        //! and sinks when it is denser, so this is what decides whether wood floats and
        //! stone does not. Fresh water is 1000.
        virtual void SetFluidDensity(float density) = 0;
        virtual float GetFluidDensity() const = 0;

        //! Fraction of a body's velocity removed per second while submerged; higher values
        //! make the water feel thicker.
        virtual void SetLinearDrag(float drag) = 0;
        virtual float GetLinearDrag() const = 0;
        virtual void SetAngularDrag(float drag) = 0;
        virtual float GetAngularDrag() const = 0;

        //! Velocity of the fluid itself, in world space: a current that carries bodies.
        virtual void SetFluidVelocity(const AZ::Vector3& velocity) = 0;
        virtual AZ::Vector3 GetFluidVelocity() const = 0;

        //! Size of the water box in entity space. Its local +Z face is the surface.
        virtual void SetDimensions(const AZ::Vector3& dimensions) = 0;
        virtual AZ::Vector3 GetDimensions() const = 0;

        //! Every setting at once, for callers that would otherwise make a run of individual
        //! calls and have the volume react to each one separately.
        virtual void SetWaterSettings(const JoltWaterVolumeSettings& settings) = 0;
        virtual JoltWaterVolumeSettings GetWaterSettings() const = 0;

        //! Surface waves. Amplitude and length are in metres, speed in metres per second.
        virtual void SetWavesEnabled(bool enabled) = 0;
        virtual bool GetWavesEnabled() const = 0;
        virtual void SetWaveAmplitude(float amplitude) = 0;
        virtual float GetWaveAmplitude() const = 0;

        //! Stops or resumes applying buoyancy without removing the component.
        virtual void SetEnabled(bool enabled) = 0;
        virtual bool IsEnabled() const = 0;

        //! Number of bodies the volume affected during the most recent physics step.
        virtual int GetSubmergedBodyCount() const = 0;

        //! How much of the given body was under the surface last step, from 0 to 1.
        //! Returns 0 unless the volume's ReportSubmergedFraction setting is on.
        virtual float GetSubmergedFraction(AZ::EntityId bodyEntityId) const = 0;
    };

    using JoltWaterVolumeRequestBus = AZ::EBus<JoltWaterVolumeRequests>;

    //! Told when bodies enter and leave a water volume. Addressed by the water volume's
    //! entity, so a listener hears about one body of water rather than all of them.
    //!
    //! Both are raised after the physics step has finished, never from inside it, so a
    //! handler is free to spawn a splash, start a sound, or touch the body it was handed.
    class JoltWaterVolumeNotifications
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltWaterVolumeNotifications() = default;

        //! A body became submerged this step. `speed` is how fast it was travelling along
        //! the surface normal as it went in, which is what tells a big splash from a gentle
        //! one.
        virtual void OnBodyEnteredWater([[maybe_unused]] AZ::EntityId bodyEntityId, [[maybe_unused]] float speed)
        {
        }

        //! A body that was submerged no longer is, either because it left the water or
        //! because it was removed from the scene.
        virtual void OnBodyExitedWater([[maybe_unused]] AZ::EntityId bodyEntityId)
        {
        }
    };

    using JoltWaterVolumeNotificationBus = AZ::EBus<JoltWaterVolumeNotifications>;

    //! Per-body control over how water treats one particular entity, for the cases the
    //! automatic density calculation cannot express.
    class JoltBuoyancyOverrideRequests
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltBuoyancyOverrideRequests() = default;

        //! Keep the body out of the water entirely. Nothing is applied to it, and it never
        //! raises enter or exit notifications.
        virtual void SetExcludedFromWater(bool excluded) = 0;
        virtual bool IsExcludedFromWater() const = 0;

        //! Whether the buoyancy factor is derived from density or taken as authored.
        virtual void SetBuoyancyMode(JoltBuoyancyMode mode) = 0;
        virtual JoltBuoyancyMode GetBuoyancyMode() const = 0;

        //! The factor used in Explicit mode. Jolt's scale is relative: 1 is neutral and
        //! floats half out of the water, above 1 rides higher, below 1 sinks.
        virtual void SetBuoyancyFactor(float factor) = 0;
        virtual float GetBuoyancyFactor() const = 0;
    };

    using JoltBuoyancyOverrideRequestBus = AZ::EBus<JoltBuoyancyOverrideRequests>;

} // namespace JoltBuoyancy
