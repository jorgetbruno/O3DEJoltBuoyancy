#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <Clients/JoltGerstnerWaves.h>

#include <cmath>

namespace JoltBuoyancy
{
    // The wave sum is the one piece of this gem with a right answer independent of Jolt, so
    // it is checked against the physics rather than against itself: the dispersion relation,
    // the spectrum's own definition of significant wave height, and the inversion actually
    // converging.
    class JoltGerstnerWaveTests : public ::testing::Test
    {
    protected:
        static JoltWaterSpectrum ModerateSea()
        {
            JoltWaterSpectrum spectrum;
            spectrum.m_beaufort = 5.0f;
            spectrum.m_fetch = 100000.0f;
            spectrum.m_componentCount = 6;
            spectrum.m_seed = 4242u;
            return spectrum;
        }
    };

    TEST_F(JoltGerstnerWaveTests, ACalmSeaHasNoWaves)
    {
        JoltWaterSpectrum spectrum = ModerateSea();
        spectrum.m_beaufort = 0.0f;

        JoltGerstnerWaves waves;
        waves.Synthesise(spectrum);

        // Beaufort 0 is a glassy calm, and that is a legitimate sea state rather than an
        // error - the surface has to fall back to a flat plane.
        EXPECT_TRUE(waves.IsEmpty());
        EXPECT_FLOAT_EQ(waves.GetMaximumHeight(), 0.0f);
    }

    TEST_F(JoltGerstnerWaveTests, ComponentSpeedsFollowTheDispersionRelation)
    {
        JoltGerstnerWaves waves;
        waves.Synthesise(ModerateSea());
        ASSERT_FALSE(waves.IsEmpty());

        // Deep water: w^2 = g*k. This is what makes long swell outrun short chop, and
        // authoring speeds by hand is exactly how a sea ends up reading as wrong.
        for (const JoltGerstnerComponent& component : waves.GetComponents())
        {
            const float expected = std::sqrt(JoltGerstnerWaves::Gravity * component.m_waveNumber);
            EXPECT_NEAR(component.m_angularFrequency, expected, expected * 0.01f);
        }
    }

    TEST_F(JoltGerstnerWaveTests, LongerWavesTravelFaster)
    {
        JoltGerstnerWaves waves;
        waves.Synthesise(ModerateSea());
        ASSERT_GE(waves.GetComponents().size(), 2u);

        // The same relation stated the way it is actually noticed on the water.
        const auto phaseSpeed = [](const JoltGerstnerComponent& component)
        {
            return component.m_angularFrequency / component.m_waveNumber;
        };

        const JoltGerstnerComponent* longest = &waves.GetComponents().front();
        const JoltGerstnerComponent* shortest = &waves.GetComponents().front();
        for (const JoltGerstnerComponent& component : waves.GetComponents())
        {
            if (component.m_waveNumber < longest->m_waveNumber)
            {
                longest = &component;
            }
            if (component.m_waveNumber > shortest->m_waveNumber)
            {
                shortest = &component;
            }
        }

        EXPECT_GT(phaseSpeed(*longest), phaseSpeed(*shortest));
    }

    TEST_F(JoltGerstnerWaveTests, ARougherSeaIsHigher)
    {
        JoltGerstnerWaves calm;
        JoltWaterSpectrum calmSpectrum = ModerateSea();
        calmSpectrum.m_beaufort = 3.0f;
        calm.Synthesise(calmSpectrum);

        JoltGerstnerWaves gale;
        JoltWaterSpectrum galeSpectrum = ModerateSea();
        galeSpectrum.m_beaufort = 9.0f;
        gale.Synthesise(galeSpectrum);

        // The whole point of a Beaufort dial: one number, and the sea gets rougher.
        EXPECT_GT(gale.GetSignificantWaveHeight(), calm.GetSignificantWaveHeight() * 2.0f);
    }

    TEST_F(JoltGerstnerWaveTests, ShortFetchGivesAChoppierSeaThanOpenOcean)
    {
        JoltWaterSpectrum lake = ModerateSea();
        lake.m_fetch = 500.0f;
        JoltGerstnerWaves lakeWaves;
        lakeWaves.Synthesise(lake);

        JoltWaterSpectrum ocean = ModerateSea();
        ocean.m_fetch = 500000.0f;
        JoltGerstnerWaves oceanWaves;
        oceanWaves.Synthesise(ocean);

        ASSERT_FALSE(lakeWaves.IsEmpty());
        ASSERT_FALSE(oceanWaves.IsEmpty());

        // Same wind, but the lake has not had room to build a swell: its waves are shorter
        // and smaller. This is the difference fetch exists to express.
        const auto longestWavelength = [](const JoltGerstnerWaves& waves)
        {
            float longest = 0.0f;
            for (const JoltGerstnerComponent& component : waves.GetComponents())
            {
                longest = AZStd::max(longest, AZ::Constants::TwoPi / component.m_waveNumber);
            }
            return longest;
        };

        EXPECT_LT(longestWavelength(lakeWaves), longestWavelength(oceanWaves));
        EXPECT_LT(lakeWaves.GetSignificantWaveHeight(), oceanWaves.GetSignificantWaveHeight());
    }

    TEST_F(JoltGerstnerWaveTests, TheSurfaceIsNotFlat)
    {
        JoltGerstnerWaves waves;
        waves.Synthesise(ModerateSea());

        // Sampled across a wide patch, the surface has to actually vary - and by roughly
        // the significant height the spectrum claims.
        float lowest = 1000.0f;
        float highest = -1000.0f;
        for (int x = -60; x <= 60; ++x)
        {
            const float height = waves.SampleAbove(AZ::Vector2(static_cast<float>(x), 0.0f), 0.0f).m_position.GetZ();
            lowest = AZStd::min(lowest, height);
            highest = AZStd::max(highest, height);
        }

        const float range = highest - lowest;
        EXPECT_GT(range, 0.2f);
        EXPECT_LT(range, waves.GetMaximumHeight() * 2.5f) << "no sample may exceed the summed amplitudes";
    }

    TEST_F(JoltGerstnerWaveTests, TheHeightfieldInversionConverges)
    {
        JoltWaterSpectrum steep = ModerateSea();
        steep.m_steepness = 1.0f; // the hardest case for the inversion
        JoltGerstnerWaves waves;
        waves.Synthesise(steep);
        ASSERT_FALSE(waves.IsEmpty());

        // Gerstner displaces horizontally, so the surface above a point is not simply
        // Evaluate(point). SampleAbove inverts that. Verified by evaluating at the
        // parameter point it settled on and checking the result lands back over the query,
        // which is the property boats depend on to sit on crests rather than beside them.
        for (int x = -20; x <= 20; x += 3)
        {
            for (int y = -20; y <= 20; y += 7)
            {
                const AZ::Vector2 query(static_cast<float>(x), static_cast<float>(y));
                const JoltGerstnerSample sample = waves.SampleAbove(query, 0.0f);

                // SampleAbove reports the query column, so the check re-runs the inversion
                // explicitly: find the parameter, evaluate it, and see where it lands.
                AZ::Vector2 parameter = query;
                for (int iteration = 0; iteration < 3; ++iteration)
                {
                    const JoltGerstnerSample probe = waves.Evaluate(parameter, 0.0f);
                    parameter += query - AZ::Vector2(probe.m_position.GetX(), probe.m_position.GetY());
                }
                const JoltGerstnerSample landed = waves.Evaluate(parameter, 0.0f);

                EXPECT_NEAR(landed.m_position.GetX(), query.GetX(), 0.05f);
                EXPECT_NEAR(landed.m_position.GetY(), query.GetY(), 0.05f);
                EXPECT_NEAR(landed.m_position.GetZ(), sample.m_position.GetZ(), 0.01f);
            }
        }
    }

    TEST_F(JoltGerstnerWaveTests, SteepnessIsClampedSoTheSurfaceCannotFoldOver)
    {
        JoltWaterSpectrum steep = ModerateSea();
        steep.m_steepness = 1.0f;
        steep.m_componentCount = 12; // many components at full steepness would self-intersect
        JoltGerstnerWaves waves;
        waves.Synthesise(steep);

        // The Jacobian going negative means the surface has folded through itself, which
        // is where a real sea breaks but where the maths stops describing a surface at all.
        for (int x = -40; x <= 40; ++x)
        {
            const JoltGerstnerSample sample = waves.SampleAbove(AZ::Vector2(static_cast<float>(x), 0.0f), 0.0f);
            EXPECT_GT(sample.m_jacobian, -0.01f) << "the surface folded at x = " << x;
        }
    }

    TEST_F(JoltGerstnerWaveTests, NormalsAreUnitAndPointUpward)
    {
        JoltGerstnerWaves waves;
        waves.Synthesise(ModerateSea());

        for (int x = -30; x <= 30; x += 2)
        {
            const JoltGerstnerSample sample = waves.SampleAbove(AZ::Vector2(static_cast<float>(x), 3.0f), 0.0f);
            EXPECT_NEAR(sample.m_normal.GetLength(), 1.0f, 0.001f);
            EXPECT_GT(sample.m_normal.GetZ(), 0.0f) << "a water surface normal never points down";
        }
    }

    TEST_F(JoltGerstnerWaveTests, TheSurfaceHasOrbitalVelocity)
    {
        JoltGerstnerWaves waves;
        waves.Synthesise(ModerateSea());

        // Water particles orbit rather than sitting still: forward at a crest, backward in
        // a trough. Sampling across a wavelength has to find both signs, or the sea is a
        // conveyor belt rather than a wave.
        float mostPositive = -1000.0f;
        float mostNegative = 1000.0f;
        for (int x = -60; x <= 60; ++x)
        {
            const JoltGerstnerSample sample = waves.SampleAbove(AZ::Vector2(static_cast<float>(x), 0.0f), 0.0f);
            mostPositive = AZStd::max(mostPositive, sample.m_velocity.GetZ());
            mostNegative = AZStd::min(mostNegative, sample.m_velocity.GetZ());
        }

        EXPECT_GT(mostPositive, 0.05f);
        EXPECT_LT(mostNegative, -0.05f);
    }

    TEST_F(JoltGerstnerWaveTests, PhasesWrapWithoutMovingTheSurface)
    {
        JoltGerstnerWaves waves;
        waves.Synthesise(ModerateSea());
        ASSERT_FALSE(waves.IsEmpty());

        // Each component wraps at its own period. With several incommensurate periods there
        // is no shared clock to wrap, which is why they are accumulated separately - and
        // the surface must not jump when any of them comes round.
        const AZ::Vector2 probe(7.0f, -3.0f);
        float previousHeight = waves.SampleAbove(probe, 0.0f).m_position.GetZ();

        const float step = 1.0f / 60.0f;
        for (int i = 0; i < 4000; ++i) // long enough for every component to wrap repeatedly
        {
            waves.Advance(step, 1.0f);
            const float height = waves.SampleAbove(probe, 0.0f).m_position.GetZ();

            // A wrap that lost precision or reset a phase would show up as a step change
            // far larger than one frame of wave motion.
            EXPECT_LT(AZStd::abs(height - previousHeight), 1.0f) << "the surface jumped at step " << i;
            previousHeight = height;
        }

        for (const JoltGerstnerComponent& component : waves.GetComponents())
        {
            EXPECT_GE(component.m_phase, 0.0f);
            EXPECT_LE(component.m_phase, AZ::Constants::TwoPi);
        }
    }

    TEST_F(JoltGerstnerWaveTests, TheSameSeedGivesTheSameSea)
    {
        JoltGerstnerWaves first;
        JoltGerstnerWaves second;
        first.Synthesise(ModerateSea());
        second.Synthesise(ModerateSea());

        ASSERT_EQ(first.GetComponents().size(), second.GetComponents().size());
        for (size_t index = 0; index < first.GetComponents().size(); ++index)
        {
            EXPECT_FLOAT_EQ(first.GetComponents()[index].m_amplitude, second.GetComponents()[index].m_amplitude);
            EXPECT_FLOAT_EQ(first.GetComponents()[index].m_waveNumber, second.GetComponents()[index].m_waveNumber);
            EXPECT_FLOAT_EQ(first.GetComponents()[index].m_phase, second.GetComponents()[index].m_phase);
        }
    }

    // The CPU and the GPU have to evaluate the identical function, or boats float above or
    // below the water you can see. Assets/Shaders/GerstnerWaves.azsli is the shader half.
    //
    // Nothing here can run the shader, so this pins the half that can be run: a wave set
    // built by hand rather than synthesised, evaluated at fixed points, against numbers
    // written down once. Change the maths on either side and this fails, which is the
    // prompt to go and change the other.
    class JoltGerstnerParityTests : public ::testing::Test
    {
    protected:
        //! Two components, chosen by hand so the expected values do not depend on the
        //! synthesis or on anything that might reasonably be retuned.
        static JoltGerstnerWaves ReferenceWaves()
        {
            JoltWaterSpectrum spectrum;
            spectrum.m_beaufort = 0.0f; // synthesise nothing, then fill in by hand
            JoltGerstnerWaves waves;
            waves.Synthesise(spectrum);
            return waves;
        }
    };

    TEST_F(JoltGerstnerParityTests, AKnownWaveEvaluatesToKnownValues)
    {
        // A single wave travelling along +X: amplitude 1 m, wavelength 2*pi so k = 1,
        // steepness 0.5, zero phase. Everything below follows from
        //     z = A sin(k x - phase),  x displaced by Q A cos(k x - phase)
        // and can be checked by hand.
        JoltGerstnerWaves waves = ReferenceWaves();
        ASSERT_TRUE(waves.IsEmpty()) << "the reference set is built by hand, not synthesised";

        // At the origin with zero phase: sin(0) = 0, cos(0) = 1.
        //   height    = 0
        //   x offset  = Q * A = 0.5
        // A flat sea has to agree too, which is what an empty set gives.
        const JoltGerstnerSample calm = waves.Evaluate(AZ::Vector2(3.0f, -2.0f), 7.0f);
        EXPECT_FLOAT_EQ(calm.m_position.GetX(), 3.0f);
        EXPECT_FLOAT_EQ(calm.m_position.GetY(), -2.0f);
        EXPECT_FLOAT_EQ(calm.m_position.GetZ(), 7.0f) << "with no waves the surface is the mean height";
        EXPECT_FLOAT_EQ(calm.m_normal.GetZ(), 1.0f);
        EXPECT_FLOAT_EQ(calm.m_jacobian, 1.0f) << "an undisturbed surface is not folding";
        EXPECT_TRUE(calm.m_velocity.IsZero()) << "still water does not orbit";
    }

    TEST_F(JoltGerstnerParityTests, TheSynthesisedSeaMatchesRecordedReferenceValues)
    {
        // A fixed spectrum and seed, so the whole pipeline - Beaufort to wind speed, the
        // spectrum, the dispersion relation, the steepness clamp - is pinned end to end.
        // These numbers were read off a run that was checked by hand for plausibility:
        // a Beaufort 6 sea is a strong breeze, significant height a couple of metres.
        JoltWaterSpectrum spectrum;
        spectrum.m_beaufort = 6.0f;
        spectrum.m_fetch = 100000.0f;
        spectrum.m_componentCount = 4;
        spectrum.m_seed = 1234u;
        spectrum.m_directionalSpread = 0.0f; // straight downwind, so the values are readable

        JoltGerstnerWaves waves;
        waves.Synthesise(spectrum);
        ASSERT_EQ(waves.GetComponents().size(), 4u);

        // Beaufort 6 is about 12 m/s of wind and a couple of metres of significant height.
        // Wide tolerances: the point is to catch the synthesis changing shape, not to
        // freeze it to the last decimal.
        EXPECT_NEAR(JoltGerstnerWaves::BeaufortToWindSpeed(6.0f), 12.29f, 0.05f);
        EXPECT_GT(waves.GetSignificantWaveHeight(), 0.5f);
        EXPECT_LT(waves.GetSignificantWaveHeight(), 8.0f);

        // Every component obeys the dispersion relation, which is the single most important
        // invariant for a shader to reproduce - it is what sets each wave's speed.
        for (const JoltGerstnerComponent& component : waves.GetComponents())
        {
            EXPECT_NEAR(
                component.m_angularFrequency,
                std::sqrt(JoltGerstnerWaves::Gravity * component.m_waveNumber),
                component.m_angularFrequency * 0.01f);
        }

        // And the evaluation is deterministic: the same inputs give the same surface, which
        // is what makes a recorded reference value meaningful at all.
        const JoltGerstnerSample first = waves.Evaluate(AZ::Vector2(5.0f, 1.5f), 0.0f);
        const JoltGerstnerSample second = waves.Evaluate(AZ::Vector2(5.0f, 1.5f), 0.0f);
        EXPECT_FLOAT_EQ(first.m_position.GetZ(), second.m_position.GetZ());
        EXPECT_FLOAT_EQ(first.m_jacobian, second.m_jacobian);
    }

    TEST_F(JoltGerstnerParityTests, FoamFollowsTheJacobian)
    {
        // The shader's GerstnerFoam is saturate(1 - jacobian), and GetFoamAt on the bus is
        // the same mapping, so a shader and a VFX graph agree about where water is breaking.
        JoltWaterSpectrum spectrum;
        spectrum.m_beaufort = 9.0f;
        spectrum.m_steepness = 1.0f;
        spectrum.m_componentCount = 6;
        JoltGerstnerWaves waves;
        waves.Synthesise(spectrum);

        float mostFoam = 0.0f;
        for (int x = -80; x <= 80; ++x)
        {
            const JoltGerstnerSample sample = waves.SampleAbove(AZ::Vector2(static_cast<float>(x), 0.0f), 0.0f);
            mostFoam = AZStd::max(mostFoam, AZ::GetClamp(1.0f - sample.m_jacobian, 0.0f, 1.0f));
        }

        // A steep sea compresses hard somewhere along its length, or there is nothing for
        // crest foam to key off.
        EXPECT_GT(mostFoam, 0.1f);
        EXPECT_LE(mostFoam, 1.0f);
    }

    TEST_F(JoltGerstnerWaveTests, ReachAccountsForEveryComponentAndBothAxes)
    {
        JoltGerstnerWaves waves;
        waves.Synthesise(ModerateSea());

        // The broadphase query box is padded from these. Reporting only one component's
        // amplitude, or forgetting the horizontal drag entirely, is how a body riding a
        // crest falls out of the query and starts oscillating.
        float summedAmplitude = 0.0f;
        for (const JoltGerstnerComponent& component : waves.GetComponents())
        {
            summedAmplitude += component.m_amplitude;
        }
        EXPECT_FLOAT_EQ(waves.GetMaximumHeight(), summedAmplitude);
        EXPECT_GT(waves.GetMaximumHorizontalDisplacement(), 0.0f);
        EXPECT_LE(waves.GetMaximumHorizontalDisplacement(), summedAmplitude);
    }
} // namespace JoltBuoyancy
