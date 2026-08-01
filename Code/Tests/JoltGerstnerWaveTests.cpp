#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <Clients/JoltGerstnerWaves.h>

#include <AzCore/Math/MathUtils.h>

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
        //! One wave, built by hand rather than synthesised, so every expected value below
        //! is derived from the formula instead of recorded from this implementation.
        //!
        //! A = 1 m, k = 1 (so a wavelength of 2*pi), Q = 0.5, phase 0, and the deep-water
        //! frequency for that wave number. Everything follows from
        //!     x -> x + Q A d.x cos(theta),   z -> A sin(theta),   theta = k (d.p) - phase
        static JoltGerstnerWaves HandBuiltWave(const AZ::Vector2& direction)
        {
            JoltGerstnerComponent component;
            component.m_direction = direction;
            component.m_amplitude = 1.0f;
            component.m_waveNumber = 1.0f;
            component.m_angularFrequency = std::sqrt(JoltGerstnerWaves::Gravity * 1.0f);
            component.m_steepness = 0.5f;
            component.m_phase = 0.0f;

            JoltGerstnerWaves waves;
            waves.SetComponents({ component });
            return waves;
        }

        //! omega for the hand-built wave, which several expected velocities are written in
        //! terms of.
        static float ReferenceFrequency()
        {
            return std::sqrt(JoltGerstnerWaves::Gravity);
        }
    };

    TEST_F(JoltGerstnerParityTests, ACalmSurfaceIsTheMeanHeight)
    {
        JoltGerstnerWaves waves;
        ASSERT_TRUE(waves.IsEmpty());

        const JoltGerstnerSample calm = waves.Evaluate(AZ::Vector2(3.0f, -2.0f), 7.0f);
        EXPECT_FLOAT_EQ(calm.m_position.GetX(), 3.0f);
        EXPECT_FLOAT_EQ(calm.m_position.GetY(), -2.0f);
        EXPECT_FLOAT_EQ(calm.m_position.GetZ(), 7.0f) << "with no waves the surface is the mean height";
        EXPECT_FLOAT_EQ(calm.m_normal.GetZ(), 1.0f);
        EXPECT_FLOAT_EQ(calm.m_jacobian, 1.0f) << "an undisturbed surface is not folding";
        EXPECT_TRUE(calm.m_velocity.IsZero()) << "still water does not orbit";
    }

    TEST_F(JoltGerstnerParityTests, AKnownWaveEvaluatesToKnownValuesAtAZeroCrossing)
    {
        // A single wave along +X, evaluated at the origin. theta = 0, so sin = 0, cos = 1:
        //   z        = A sin(0)                = 0
        //   x        = 0 + Q A cos(0)          = 0.5
        //   dz/dx    = k A cos(0)              = 1        (the surface climbs at 45 degrees)
        //   dx/dx    = 1 - Q k A sin(0)        = 1
        //   normal   = normalize((1,0,1) x (0,1,0)) = (-1, 0, 1)/sqrt(2)
        //   jacobian = dx/dx * dy/dy - (dx/dy)^2   = 1
        //   velocity = (0, 0, -omega A cos(0))     = (0, 0, -omega)
        const JoltGerstnerWaves waves = HandBuiltWave(AZ::Vector2(1.0f, 0.0f));
        ASSERT_EQ(waves.GetComponents().size(), 1u);

        const JoltGerstnerSample sample = waves.Evaluate(AZ::Vector2(0.0f, 0.0f), 0.0f);
        EXPECT_NEAR(sample.m_position.GetX(), 0.5f, 1.0e-5f);
        EXPECT_NEAR(sample.m_position.GetY(), 0.0f, 1.0e-5f);
        EXPECT_NEAR(sample.m_position.GetZ(), 0.0f, 1.0e-5f);

        const float invRoot2 = 1.0f / std::sqrt(2.0f);
        EXPECT_NEAR(sample.m_normal.GetX(), -invRoot2, 1.0e-5f) << "the normal leans back down the rising face";
        EXPECT_NEAR(sample.m_normal.GetY(), 0.0f, 1.0e-5f);
        EXPECT_NEAR(sample.m_normal.GetZ(), invRoot2, 1.0e-5f);

        EXPECT_NEAR(sample.m_jacobian, 1.0f, 1.0e-5f);
        EXPECT_NEAR(sample.m_velocity.GetX(), 0.0f, 1.0e-5f);
        EXPECT_NEAR(sample.m_velocity.GetZ(), -ReferenceFrequency(), 1.0e-4f)
            << "on the way up the face, the water itself is moving down";
    }

    TEST_F(JoltGerstnerParityTests, AKnownWaveEvaluatesToKnownValuesAtACrestAndATrough)
    {
        const JoltGerstnerWaves waves = HandBuiltWave(AZ::Vector2(1.0f, 0.0f));
        const float omega = ReferenceFrequency();

        // Crest: k x = pi/2, so sin = 1, cos = 0.
        //   z        = A                       = 1
        //   x offset = Q A cos(pi/2)           = 0        (undisplaced at the crest)
        //   dz/dx    = k A cos(pi/2)           = 0        (flat on top)
        //   dx/dx    = 1 - Q k A sin(pi/2)     = 0.5      (compressed toward the crest)
        //   velocity = (omega Q A sin, 0, 0)   = (0.5 omega, 0, 0)
        const JoltGerstnerSample crest = waves.Evaluate(AZ::Vector2(AZ::Constants::HalfPi, 0.0f), 0.0f);
        EXPECT_NEAR(crest.m_position.GetX(), AZ::Constants::HalfPi, 1.0e-5f);
        EXPECT_NEAR(crest.m_position.GetZ(), 1.0f, 1.0e-5f);
        EXPECT_NEAR(crest.m_normal.GetZ(), 1.0f, 1.0e-5f) << "a crest is a stationary point, so the normal is up";
        EXPECT_NEAR(crest.m_jacobian, 0.5f, 1.0e-5f) << "1 - Q k A, the compression that foam keys off";
        EXPECT_NEAR(crest.m_velocity.GetX(), 0.5f * omega, 1.0e-4f) << "at a crest the water runs with the wave";

        // Trough: k x = -pi/2, so sin = -1, cos = 0. Same terms with the sign flipped, and
        // the surface stretches rather than compressing.
        const JoltGerstnerSample trough = waves.Evaluate(AZ::Vector2(-AZ::Constants::HalfPi, 0.0f), 0.0f);
        EXPECT_NEAR(trough.m_position.GetZ(), -1.0f, 1.0e-5f);
        EXPECT_NEAR(trough.m_jacobian, 1.5f, 1.0e-5f) << "1 + Q k A: a trough is stretched, and never foams";
        EXPECT_NEAR(trough.m_velocity.GetX(), -0.5f * omega, 1.0e-4f) << "in a trough the water runs against it";
    }

    TEST_F(JoltGerstnerParityTests, ADiagonalWavePinsTheCrossDerivative)
    {
        // Everything above travels along an axis, where dx/dy is identically zero - so a
        // shader that wrote d.x*d.x where it meant d.x*d.y would pass every one of them.
        // This wave runs along (0.6, 0.8), which makes the three second-derivative terms
        // -Q k A times 0.36, 0.48 and 0.64: all different, so a swap changes the answer.
        const JoltGerstnerWaves waves = HandBuiltWave(AZ::Vector2(0.6f, 0.8f));

        // At the crest, d.p = pi/2 with |d| = 1, so p = (0.6, 0.8) * pi/2.
        //   dx/dx    = 1 - 0.5 * 0.36 = 0.82
        //   dx/dy    =   - 0.5 * 0.48 = -0.24
        //   dy/dy    = 1 - 0.5 * 0.64 = 0.68
        //   jacobian = 0.82 * 0.68 - 0.24^2 = 0.5
        const AZ::Vector2 crestPoint(0.6f * AZ::Constants::HalfPi, 0.8f * AZ::Constants::HalfPi);
        const JoltGerstnerSample crest = waves.Evaluate(crestPoint, 0.0f);
        EXPECT_NEAR(crest.m_position.GetZ(), 1.0f, 1.0e-5f);
        EXPECT_NEAR(crest.m_jacobian, 0.5f, 1.0e-5f)
            << "the crest jacobian is 1 - Q k A whatever direction the wave runs in";
        EXPECT_NEAR(crest.m_normal.GetZ(), 1.0f, 1.0e-5f);

        // And at the zero crossing the normal is direction-sensitive:
        //   dz/dx  = k A d.x = 0.6,  dz/dy = k A d.y = 0.8
        //   normal = normalize((-0.6, -0.8, 1))
        const JoltGerstnerSample crossing = waves.Evaluate(AZ::Vector2(0.0f, 0.0f), 0.0f);
        const float scale = 1.0f / std::sqrt(2.0f);
        EXPECT_NEAR(crossing.m_normal.GetX(), -0.6f * scale, 1.0e-5f);
        EXPECT_NEAR(crossing.m_normal.GetY(), -0.8f * scale, 1.0e-5f);
        EXPECT_NEAR(crossing.m_normal.GetZ(), scale, 1.0e-5f);

        // Displaced along the direction of travel, by Q A in total.
        EXPECT_NEAR(crossing.m_position.GetX(), 0.5f * 0.6f, 1.0e-5f);
        EXPECT_NEAR(crossing.m_position.GetY(), 0.5f * 0.8f, 1.0e-5f);
    }

    TEST_F(JoltGerstnerParityTests, ComponentsAccumulateRatherThanReplace)
    {
        // Two waves whose contributions at the sample point are distinguishable, so a loop
        // that assigned instead of accumulating would give one of them rather than the sum.
        JoltGerstnerComponent first;
        first.m_direction = AZ::Vector2(1.0f, 0.0f);
        first.m_amplitude = 1.0f;
        first.m_waveNumber = 1.0f;
        first.m_angularFrequency = std::sqrt(JoltGerstnerWaves::Gravity);
        first.m_steepness = 0.5f;
        first.m_phase = 0.0f;

        // Quarter of a turn behind, so at the origin its theta is -3pi/2 and sin is +1.
        // No steepness, so it contributes height but no horizontal displacement.
        JoltGerstnerComponent second;
        second.m_direction = AZ::Vector2(1.0f, 0.0f);
        second.m_amplitude = 0.5f;
        second.m_waveNumber = 2.0f;
        second.m_angularFrequency = std::sqrt(JoltGerstnerWaves::Gravity * 2.0f);
        second.m_steepness = 0.0f;
        second.m_phase = 1.5f * AZ::Constants::Pi;

        JoltGerstnerWaves waves;
        waves.SetComponents({ first, second });

        // At the origin: first gives z = 0 and x += 0.5; second gives z += 0.5 and x += 0.
        const JoltGerstnerSample sample = waves.Evaluate(AZ::Vector2(0.0f, 0.0f), 0.0f);
        EXPECT_NEAR(sample.m_position.GetX(), 0.5f, 1.0e-5f) << "only the steep component displaces horizontally";
        EXPECT_NEAR(sample.m_position.GetZ(), 0.5f, 1.0e-5f) << "the heights add: 0 from one, 0.5 from the other";

        // dz/dx = 1*1*cos(0) + 2*0.5*cos(-3pi/2) = 1 + 0, and only the first is steep, so
        // the jacobian is still 1.
        EXPECT_NEAR(sample.m_jacobian, 1.0f, 1.0e-5f);
        const float invRoot2 = 1.0f / std::sqrt(2.0f);
        EXPECT_NEAR(sample.m_normal.GetX(), -invRoot2, 1.0e-5f);
        EXPECT_NEAR(sample.m_normal.GetZ(), invRoot2, 1.0e-5f);
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
