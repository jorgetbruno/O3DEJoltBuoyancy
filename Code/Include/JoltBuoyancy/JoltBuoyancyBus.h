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

    //! The region a volume fills.
    enum class JoltWaterVolumeShape : AZ::u8
    {
        //! An oriented box. Its local +Z face is the surface.
        Box = 0,
        //! A sphere, sized by the X dimension. Its surface plane sits at the top of the
        //! sphere, so it reads as a tank filled to the brim.
        Sphere = 1,
        //! Everything below the surface within a horizontal extent, with no floor. An
        //! ocean is not a box: a body that sinks past the bottom of one stops being in the
        //! water, which is wrong for open sea. Pair with a follow entity so the broadphase
        //! query box travels with the player instead of spanning the world.
        Plane = 2,
    };

    //! A sea described the way oceanography describes one, rather than as a list of waves.
    //!
    //! Authoring a spectrum rather than individual wave components is deliberate. The
    //! components are synthesised from it, so the physical relationships hold by
    //! construction: long waves travel faster than short ones, and amplitudes fall off with
    //! frequency the way a real wind sea does. Author the components by hand and it is easy
    //! to produce a sea that reads as wrong without being able to say why.
    //!
    //! It is also the schema an FFT ocean would consume. Tessendorf synthesis runs over a
    //! Phillips or JONSWAP spectrum, which is what this describes, so swapping the Gerstner
    //! sum for an FFT later replaces the synthesis without disturbing the authored data,
    //! the editor UI, the bus or the tests.
    struct JoltWaterSpectrum
    {
        AZ_CLASS_ALLOCATOR(JoltWaterSpectrum, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltWaterSpectrum, "{2E7A9C41-5B8D-4E3F-A16C-8D4B2F7E9A31}");

        static void Reflect(AZ::ReflectContext* context);

        //! Beaufort force, 0 (glassy) to 12 (hurricane). One number for the whole sea
        //! state, which is what makes a weather transition a single lerp.
        float m_beaufort = 4.0f;

        //! How far the wind has blown across open water, in metres. Short fetch gives a
        //! choppy, short-wavelength sea however hard the wind blows - the difference
        //! between a lake in a gale and an ocean swell.
        float m_fetch = 50000.0f;

        //! Wind direction in the volume's local XY plane. Waves travel along it.
        AZ::Vector2 m_windDirection = AZ::Vector2(1.0f, 0.0f);

        //! How far components fan out either side of the wind, in radians. Zero gives a
        //! corduroy sea of parallel ridges; real water spreads.
        float m_directionalSpread = 0.6f;

        //! How many Gerstner components to synthesise. More is smoother and costs linearly
        //! on every surface sample.
        AZ::u32 m_componentCount = 6;

        //! Art control on top of the physical result. Amplitude scales wave height,
        //! steepness sharpens crests (1 is the limit before the surface self-intersects),
        //! and speed scales the whole sea's motion without changing its shape.
        float m_amplitudeScale = 1.0f;
        float m_steepness = 0.7f;
        float m_speedScale = 1.0f;

        //! Depth of water under the surface, in metres. Zero means deep water, where the
        //! dispersion relation is w^2 = g k.
        //!
        //! In shallow water the sea floor interferes: w^2 = g k tanh(k d). Long waves feel
        //! the bottom first, slow down and shorten, which is why swell bunches up and
        //! steepens as it reaches a beach. Setting a depth is what makes that happen.
        //!
        //! This is one depth for the whole volume. Real shoaling needs a depth that varies
        //! with the sea floor, which would mean sampling terrain from the physics step -
        //! a dependency this gem does not have and should not take on speculatively.
        float m_waterDepth = 0.0f;

        //! Makes synthesis repeatable, so the same spectrum gives the same sea every run.
        AZ::u32 m_seed = 12345;
    };

    //! Settings describing a body of water.
    struct JoltWaterVolumeSettings
    {
        AZ_CLASS_ALLOCATOR(JoltWaterVolumeSettings, AZ::SystemAllocator);
        AZ_TYPE_INFO(JoltWaterVolumeSettings, "{E5F6A7B8-C9D0-4B4C-DE5F-6A7B8C9D0E1F}");

        static void Reflect(AZ::ReflectContext* context);

        //! Which region the water fills.
        JoltWaterVolumeShape m_shape = JoltWaterVolumeShape::Box;

        //! kg/m^3. A body floats when its own density is below this and sinks above it.
        float m_fluidDensity = 1000.0f;
        float m_linearDrag = 0.5f;
        float m_angularDrag = 0.05f;
        AZ::Vector3 m_fluidVelocity = AZ::Vector3::CreateZero();

        //! Ripples the surface instead of leaving it a flat plane. The waves ride the
        //! volume's own surface, so a tilted volume gets a tilted, moving surface.
        bool m_wavesEnabled = false;

        //! The sea state the waves are synthesised from.
        JoltWaterSpectrum m_spectrum;

        //! How many points across a body's footprint the surface is sampled at, before
        //! fitting a plane through them for the solver.
        //!
        //! One sample means a hull only ever sees the water under its centre, so a boat
        //! long enough to straddle a crest translates up and down without pitching. Four
        //! is enough to make a hull ride a swell; more costs a full surface evaluation per
        //! sample per body per step. Ignored on a flat surface, where every sample would
        //! return the same plane.
        AZ::u32 m_surfaceSamplesPerBody = 4;

        //! Which bodies the volume even looks at. Bodies the group excludes are skipped
        //! before any impulse is computed, so a volume can be made to affect only, say,
        //! debris and not the player.
        AzPhysics::CollisionGroups::Id m_collisionGroupId;

        //! Compute how much of each body is under the surface and publish it through
        //! GetSubmergedFraction. Off by default: Jolt already works this out inside
        //! ApplyBuoyancyImpulse but does not hand it back, so asking for it means walking
        //! the shape a second time.
        bool m_reportSubmergedFraction = false;

        //! How much deeper a neighbouring volume has to hold a body before it takes it
        //! over. Without it, a body drifting along the seam between two volumes flips
        //! owner every few steps and its current and density change abruptly each time.
        float m_ownershipHysteresis = 0.1f;
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

        //! Surface waves, on or off.
        virtual void SetWavesEnabled(bool enabled) = 0;
        virtual bool GetWavesEnabled() const = 0;

        //! The sea state. Setting a spectrum resynthesises the wave components, so this is
        //! how weather changes: lerp the Beaufort number and set it each frame.
        virtual void SetSpectrum(const JoltWaterSpectrum& spectrum) = 0;
        virtual JoltWaterSpectrum GetSpectrum() const = 0;

        //! Beaufort force on its own, for the common case of "make the sea rougher".
        virtual void SetSeaState(float beaufort) = 0;
        virtual float GetSeaState() const = 0;

        //! Wind direction in the volume's local XY plane.
        virtual void SetWindDirection(const AZ::Vector2& direction) = 0;
        virtual AZ::Vector2 GetWindDirection() const = 0;

        //! Significant wave height for the current spectrum, in metres - the average of
        //! the highest third, which is what a forecast quotes. Read-only: it falls out of
        //! the sea state rather than being authored.
        virtual float GetSignificantWaveHeight() const = 0;

        //! Velocity of the water at a point, including the orbital motion of the waves.
        //! At a crest the water moves with the wave and in a trough against it, which is
        //! what makes flotsam gather in lines along the swell.
        virtual AZ::Vector3 GetWaterVelocityAt(const AZ::Vector3& worldPoint) const = 0;

        //! Stops or resumes applying buoyancy without removing the component.
        virtual void SetEnabled(bool enabled) = 0;
        virtual bool IsEnabled() const = 0;

        //! Number of bodies the volume affected during the most recent physics step.
        //! Bodies that have settled and gone to sleep are still counted: they are still in
        //! the water, and a pool of sleeping floaters reporting zero would look exactly
        //! like a volume that had stopped working.
        virtual int GetSubmergedBodyCount() const = 0;

        //! How much of the given body was under the surface last step, from 0 to 1.
        //! Returns 0 unless the volume's ReportSubmergedFraction setting is on.
        virtual float GetSubmergedFraction(AZ::EntityId bodyEntityId) const = 0;

        //! Whether a world point is inside this volume and below its surface. The first
        //! thing gameplay wants: whether to play a bubble effect, drown a character, cut
        //! the engine on a boat.
        virtual bool IsPointUnderwater(const AZ::Vector3& worldPoint) const = 0;

        //! Where the surface sits directly above a world point, and which way it faces.
        //! Follows the waves, so this is what to place a splash or a floating decal on.
        //! Meaningless if the point is outside the volume - check IsPointUnderwater first.
        virtual AZ::Vector3 GetSurfacePositionAt(const AZ::Vector3& worldPoint) const = 0;
        virtual AZ::Vector3 GetSurfaceNormalAt(const AZ::Vector3& worldPoint) const = 0;

        //! How deep a world point sits below the surface, along the surface normal.
        //! Negative above it.
        virtual float GetDepthAt(const AZ::Vector3& worldPoint) const = 0;
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

        //! Scales the volume's drag for this body alone. A streamlined hull cuts through
        //! water its own weight would otherwise be slowed by, which buoyancy factor and
        //! exclusion together cannot express.
        virtual void SetLinearDragMultiplier(float multiplier) = 0;
        virtual float GetLinearDragMultiplier() const = 0;
        virtual void SetAngularDragMultiplier(float multiplier) = 0;
        virtual float GetAngularDragMultiplier() const = 0;

        //! Per body-axis drag scale. Jolt already varies drag with the projected area of
        //! the body's bounding box, so this refines that rather than introducing it - a
        //! hull is much more streamlined along its length than any box can express.
        virtual void SetDirectionalDrag(const AZ::Vector3& perAxisScale) = 0;
        virtual AZ::Vector3 GetDirectionalDrag() const = 0;

        //! Added mass: water dragged along with the body, as a fraction of the mass it
        //! displaces. Takes the twitchiness out of heave and pitch. An approximation -
        //! see JoltBuoyancyOverride for what it does and does not do.
        virtual void SetAddedMass(float coefficient) = 0;
        virtual float GetAddedMass() const = 0;
    };

    using JoltBuoyancyOverrideRequestBus = AZ::EBus<JoltBuoyancyOverrideRequests>;

} // namespace JoltBuoyancy
