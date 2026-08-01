#pragma once

#include <AzCore/Math/Vector2.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>

#include <JoltBuoyancy/JoltBuoyancyBus.h>

namespace JoltBuoyancy
{
    //! One Gerstner wave, synthesised from a spectrum rather than authored.
    struct JoltGerstnerComponent
    {
        //! Unit direction of travel in the volume's local XY plane.
        AZ::Vector2 m_direction = AZ::Vector2(1.0f, 0.0f);
        //! Crest height above the mean surface, in metres.
        float m_amplitude = 0.0f;
        //! Wave number, 2*pi / wavelength.
        float m_waveNumber = 1.0f;
        //! Angular frequency, from the deep-water dispersion relation sqrt(g*k).
        float m_angularFrequency = 1.0f;
        //! How far points are dragged horizontally toward the crest. 0 is a plain sine.
        float m_steepness = 0.0f;

        //! Accumulated phase, wrapped to one turn.
        //!
        //! Each component keeps its own rather than sharing a single elapsed time. With
        //! several incommensurate periods there is no common period to wrap a shared clock
        //! to, so it would either grow without bound and lose float32 precision after a few
        //! hours, or jump the surface when wrapped to any one component's period.
        float m_phase = 0.0f;
    };

    //! What the surface is doing at one point: where it is, which way it faces, and how the
    //! water there is moving.
    struct JoltGerstnerSample
    {
        AZ::Vector3 m_position = AZ::Vector3::CreateZero();
        AZ::Vector3 m_normal = AZ::Vector3::CreateAxisZ();
        //! Orbital velocity of the water itself. Circular in deep water: forward at the
        //! crest, backward in the trough.
        AZ::Vector3 m_velocity = AZ::Vector3::CreateZero();
        //! Determinant of the horizontal displacement Jacobian. Below 1 the surface is
        //! compressing toward a crest, and at or below 0 it has folded over - which is
        //! where a real sea breaks and throws foam.
        float m_jacobian = 1.0f;
    };

    //! Turns a sea state into a set of waves.
    //!
    //! Wind speed comes from the Beaufort number, the peak of the spectrum from wind speed
    //! and fetch, and each component's amplitude from the spectral density in its band.
    //! Speeds are never authored: the deep-water dispersion relation fixes them from
    //! wavelength, which is what keeps long swell outrunning short chop.
    class JoltGerstnerWaves
    {
    public:
        //! Standard gravity. The dispersion relation is only meaningful against the real
        //! constant, so this deliberately does not follow the scene's gravity.
        static constexpr float Gravity = 9.80665f;

        //! Replaces the component set from a spectrum. Existing phases are not preserved,
        //! so avoid calling it every frame with an unchanged spectrum.
        void Synthesise(const JoltWaterSpectrum& spectrum);

        //! Advances every component's phase by its own frequency, each wrapped separately.
        void Advance(float deltaTime, float speedScale);

        //! Surface directly above a point in the volume's local XY plane.
        //!
        //! Gerstner waves are not a heightfield: a point is displaced horizontally as well
        //! as vertically, so the surface above a given XY is not simply Evaluate(XY). This
        //! inverts that displacement - a few fixed-point iterations to find the parameter
        //! point that lands above the query - before evaluating. Skipping it puts bodies
        //! visibly off the crests, leaning the wrong way on a steep face.
        JoltGerstnerSample SampleAbove(const AZ::Vector2& localXY, float meanHeight) const;

        //! Evaluates the wave sum at a parameter point, without inversion. This is the
        //! function a vertex shader runs, and the one a CPU/GPU parity test compares.
        JoltGerstnerSample Evaluate(const AZ::Vector2& parameterXY, float meanHeight) const;

        //! Largest vertical excursion, the sum of the amplitudes.
        float GetMaximumHeight() const;

        //! Largest horizontal excursion, from the steepness terms. The broadphase query box
        //! has to allow for this as well as the height, or a body riding a crest leaves it.
        float GetMaximumHorizontalDisplacement() const;

        //! Significant wave height: the mean of the highest third, and what a forecast
        //! quotes. For a spectrum of independent components this is 4*sqrt(sum of variance).
        float GetSignificantWaveHeight() const;

        bool IsEmpty() const
        {
            return m_components.empty();
        }

        const AZStd::vector<JoltGerstnerComponent>& GetComponents() const
        {
            return m_components;
        }

        //! Wind speed in m/s for a Beaufort force, from the standard scale.
        static float BeaufortToWindSpeed(float beaufort);

    private:
        AZStd::vector<JoltGerstnerComponent> m_components;
    };
} // namespace JoltBuoyancy
