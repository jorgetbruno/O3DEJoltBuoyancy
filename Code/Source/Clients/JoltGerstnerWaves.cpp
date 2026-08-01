#include <Clients/JoltGerstnerWaves.h>

#include <AzCore/Math/MathUtils.h>

#include <cmath>

namespace JoltBuoyancy
{
    namespace
    {
        //! Cheap deterministic hash, so the same seed always gives the same sea.
        float SeededUnit(AZ::u32 seed, AZ::u32 index)
        {
            AZ::u32 hash = seed * 747796405u + index * 2891336453u;
            hash = ((hash >> ((hash >> 28) + 4)) ^ hash) * 277803737u;
            hash = (hash >> 22) ^ hash;
            return static_cast<float>(hash & 0xFFFFFFu) / static_cast<float>(0xFFFFFF);
        }

        //! Pierson-Moskowitz spectral density for a fully developed sea, in m^2 s.
        //! S(w) = alpha g^2 / w^5 * exp(-1.25 (wp/w)^4)
        float PiersonMoskowitzDensity(float angularFrequency, float peakFrequency)
        {
            if (angularFrequency <= 0.0f)
            {
                return 0.0f;
            }

            constexpr float Alpha = 0.0081f;
            const float g2 = JoltGerstnerWaves::Gravity * JoltGerstnerWaves::Gravity;
            const float ratio = peakFrequency / angularFrequency;
            const float ratio4 = ratio * ratio * ratio * ratio;
            const float w5 = std::pow(angularFrequency, 5.0f);
            return Alpha * g2 / w5 * std::exp(-1.25f * ratio4);
        }
    } // namespace

    float JoltGerstnerWaves::BeaufortToWindSpeed(float beaufort)
    {
        // The Beaufort scale is defined by wind speed bands; this is the usual closed form
        // fitted to them, in metres per second.
        return 0.836f * std::pow(AZ::GetMax(beaufort, 0.0f), 1.5f);
    }

    void JoltGerstnerWaves::Synthesise(const JoltWaterSpectrum& spectrum)
    {
        m_components.clear();

        const AZ::u32 componentCount = AZ::GetClamp(spectrum.m_componentCount, 1u, 32u);
        const float windSpeed = BeaufortToWindSpeed(spectrum.m_beaufort);
        if (windSpeed <= 0.0f || spectrum.m_amplitudeScale <= 0.0f)
        {
            // Beaufort 0 is a glassy calm, and that is a legitimate sea state.
            return;
        }

        // Peak of a fully developed sea, then shortened if the fetch is too small for the
        // wind to have raised it. This is what separates a choppy lake from an ocean swell
        // at the same wind speed.
        const float fullyDevelopedPeak = 0.855f * Gravity / windSpeed;
        const float fetch = AZ::GetMax(spectrum.m_fetch, 1.0f);
        const float dimensionlessFetch = Gravity * fetch / (windSpeed * windSpeed);
        const float fetchLimitedPeak = 22.0f * Gravity / windSpeed * std::pow(dimensionlessFetch, -0.33f);
        const float peakFrequency = AZ::GetMax(fullyDevelopedPeak, fetchLimitedPeak);

        // Components span a band either side of the peak, spaced geometrically so each
        // covers a similar share of the spectrum.
        const float lowestFrequency = peakFrequency * 0.6f;
        const float highestFrequency = peakFrequency * 3.0f;
        const float frequencyRatio = std::pow(highestFrequency / lowestFrequency, 1.0f / static_cast<float>(componentCount));

        const AZ::Vector2 windDirection = spectrum.m_windDirection.GetLengthSq() > 0.0f
            ? spectrum.m_windDirection.GetNormalized()
            : AZ::Vector2(1.0f, 0.0f);
        const float windAngle = std::atan2(windDirection.GetY(), windDirection.GetX());

        m_components.reserve(componentCount);
        for (AZ::u32 index = 0; index < componentCount; ++index)
        {
            const float lower = lowestFrequency * std::pow(frequencyRatio, static_cast<float>(index));
            const float upper = lower * frequencyRatio;
            const float centre = 0.5f * (lower + upper);

            // Amplitude of the component standing in for this band: a = sqrt(2 S dw).
            const float density = PiersonMoskowitzDensity(centre, peakFrequency);
            float amplitude = std::sqrt(AZ::GetMax(2.0f * density * (upper - lower), 0.0f));
            amplitude *= spectrum.m_amplitudeScale;
            if (amplitude < 1.0e-4f)
            {
                continue;
            }

            // Fanned either side of the wind, deterministically from the seed.
            const float spread = (SeededUnit(spectrum.m_seed, index) * 2.0f - 1.0f) * spectrum.m_directionalSpread;
            const float angle = windAngle + spread;

            JoltGerstnerComponent component;
            component.m_direction = AZ::Vector2(std::cos(angle), std::sin(angle));
            component.m_amplitude = amplitude;
            // Deep water: w^2 = g k, so the wave number follows from the frequency and the
            // speed c = w/k = sqrt(g/k) follows from that. Long waves outrun short ones
            // without anyone authoring a speed.
            component.m_waveNumber = centre * centre / Gravity;
            component.m_angularFrequency = centre;
            component.m_steepness = AZ::GetClamp(spectrum.m_steepness, 0.0f, 1.0f);
            // Offset so the components do not all start crest-aligned at the origin, which
            // would give one enormous wave on the first frame.
            component.m_phase = SeededUnit(spectrum.m_seed, index + 977u) * AZ::Constants::TwoPi;

            m_components.push_back(component);
        }

        // Steepness is shared across components in the authored settings, but the limit
        // before the surface self-intersects applies to their sum. Scale it down so a
        // steep setting on many components stays physical.
        if (!m_components.empty())
        {
            float totalSteepnessTerm = 0.0f;
            for (const JoltGerstnerComponent& component : m_components)
            {
                totalSteepnessTerm += component.m_steepness * component.m_waveNumber * component.m_amplitude;
            }
            if (totalSteepnessTerm > 1.0f)
            {
                const float correction = 1.0f / totalSteepnessTerm;
                for (JoltGerstnerComponent& component : m_components)
                {
                    component.m_steepness *= correction;
                }
            }
        }
    }

    void JoltGerstnerWaves::Advance(float deltaTime, float speedScale)
    {
        for (JoltGerstnerComponent& component : m_components)
        {
            component.m_phase += component.m_angularFrequency * speedScale * deltaTime;
            // Wrapped per component, since there is no shared period to wrap to.
            component.m_phase = std::fmod(component.m_phase, AZ::Constants::TwoPi);
            if (component.m_phase < 0.0f)
            {
                component.m_phase += AZ::Constants::TwoPi;
            }
        }
    }

    JoltGerstnerSample JoltGerstnerWaves::Evaluate(const AZ::Vector2& parameterXY, float meanHeight) const
    {
        JoltGerstnerSample sample;
        sample.m_position = AZ::Vector3(parameterXY.GetX(), parameterXY.GetY(), meanHeight);

        if (m_components.empty())
        {
            return sample;
        }

        // Partial derivatives of the displacement, accumulated alongside it. They give the
        // tangents for an exact normal and the Jacobian determinant for foam, both for
        // less than the cost of the finite-difference taps they replace.
        float dxdx = 1.0f, dxdy = 0.0f, dydy = 1.0f;
        float dzdx = 0.0f, dzdy = 0.0f;

        for (const JoltGerstnerComponent& component : m_components)
        {
            const float phase =
                component.m_waveNumber * component.m_direction.Dot(parameterXY) - component.m_phase;
            const float sinPhase = std::sin(phase);
            const float cosPhase = std::cos(phase);

            const float dirX = component.m_direction.GetX();
            const float dirY = component.m_direction.GetY();
            const float horizontal = component.m_steepness * component.m_amplitude;
            const float ka = component.m_waveNumber * component.m_amplitude;

            // Horizontal drag toward the crest is what sharpens peaks and broadens troughs
            // - the difference between open water and a wave pool.
            sample.m_position += AZ::Vector3(horizontal * dirX * cosPhase, horizontal * dirY * cosPhase, 0.0f);
            sample.m_position.SetZ(sample.m_position.GetZ() + component.m_amplitude * sinPhase);

            // Water particles orbit: with the wave at the crest, against it in the trough.
            const float wa = component.m_angularFrequency * component.m_amplitude;
            const float wq = component.m_angularFrequency * horizontal;
            sample.m_velocity += AZ::Vector3(wq * dirX * sinPhase, wq * dirY * sinPhase, -wa * cosPhase);

            const float steepTerm = component.m_steepness * ka;
            dxdx -= steepTerm * dirX * dirX * sinPhase;
            dxdy -= steepTerm * dirX * dirY * sinPhase;
            dydy -= steepTerm * dirY * dirY * sinPhase;
            dzdx += ka * dirX * cosPhase;
            dzdy += ka * dirY * cosPhase;
        }

        const AZ::Vector3 tangentX(dxdx, dxdy, dzdx);
        const AZ::Vector3 tangentY(dxdy, dydy, dzdy);
        sample.m_normal = tangentX.Cross(tangentY).GetNormalizedSafe();
        if (sample.m_normal.GetZ() < 0.0f)
        {
            sample.m_normal = -sample.m_normal;
        }

        sample.m_jacobian = dxdx * dydy - dxdy * dxdy;
        return sample;
    }

    JoltGerstnerSample JoltGerstnerWaves::SampleAbove(const AZ::Vector2& localXY, float meanHeight) const
    {
        if (m_components.empty())
        {
            return Evaluate(localXY, meanHeight);
        }

        // Fixed-point inversion: guess that the parameter point is the query point, see
        // where it actually lands, and subtract the error. Converges quickly because the
        // horizontal displacement is bounded well below one wavelength - the same reason
        // the surface does not self-intersect.
        AZ::Vector2 parameter = localXY;
        for (int iteration = 0; iteration < 3; ++iteration)
        {
            const JoltGerstnerSample sample = Evaluate(parameter, meanHeight);
            const AZ::Vector2 landed(sample.m_position.GetX(), sample.m_position.GetY());
            parameter += localXY - landed;
        }

        JoltGerstnerSample sample = Evaluate(parameter, meanHeight);
        // The caller asked about this column of water, so report it there. The residual
        // after three iterations is far below the size of anything that floats.
        sample.m_position.SetX(localXY.GetX());
        sample.m_position.SetY(localXY.GetY());
        return sample;
    }

    float JoltGerstnerWaves::GetMaximumHeight() const
    {
        float total = 0.0f;
        for (const JoltGerstnerComponent& component : m_components)
        {
            total += component.m_amplitude;
        }
        return total;
    }

    float JoltGerstnerWaves::GetMaximumHorizontalDisplacement() const
    {
        float total = 0.0f;
        for (const JoltGerstnerComponent& component : m_components)
        {
            total += component.m_steepness * component.m_amplitude;
        }
        return total;
    }

    float JoltGerstnerWaves::GetSignificantWaveHeight() const
    {
        // Each component contributes a^2/2 to the surface variance; the significant height
        // of a narrow-band sea is four standard deviations.
        float variance = 0.0f;
        for (const JoltGerstnerComponent& component : m_components)
        {
            variance += 0.5f * component.m_amplitude * component.m_amplitude;
        }
        return 4.0f * std::sqrt(variance);
    }
} // namespace JoltBuoyancy
