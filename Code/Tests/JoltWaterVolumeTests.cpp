#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <Clients/JoltBuoyancyAllocator.h>
#include <Clients/JoltBuoyancyOverrideRegistry.h>
#include <Clients/JoltWaterVolume.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

namespace JoltBuoyancy
{
    namespace
    {
        namespace TestBroadPhaseLayers
        {
            static constexpr JPH::BroadPhaseLayer NonMoving(0);
            static constexpr JPH::BroadPhaseLayer Moving(1);
            static constexpr JPH::uint NumLayers = 2;
        }

        namespace TestObjectLayers
        {
            static constexpr JPH::ObjectLayer NonMoving = 0;
            static constexpr JPH::ObjectLayer Moving = 1;
            static constexpr JPH::ObjectLayer NumLayers = 2;
        }

        // Minimal layer setup: the gem under test only needs a physics system to attach
        // to, so the tests build a plain Jolt world rather than depending on the
        // JoltPhysics gem's scene implementation.
        class TestBroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface
        {
        public:
            JPH::uint GetNumBroadPhaseLayers() const override
            {
                return TestBroadPhaseLayers::NumLayers;
            }
            JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override
            {
                return inLayer == TestObjectLayers::Moving ? TestBroadPhaseLayers::Moving : TestBroadPhaseLayers::NonMoving;
            }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
            const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override
            {
                return "TestLayer";
            }
#endif
        };

        class TestObjectVsBroadPhaseLayerFilter final : public JPH::ObjectVsBroadPhaseLayerFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override
            {
                return inLayer1 == TestObjectLayers::Moving || inLayer2 == TestBroadPhaseLayers::Moving;
            }
        };

        class TestObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter
        {
        public:
            bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::ObjectLayer inLayer2) const override
            {
                return inLayer1 == TestObjectLayers::Moving || inLayer2 == TestObjectLayers::Moving;
            }
        };
    }

    class JoltWaterVolumeTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            // The same installer the gem's modules use, rather than
            // JPH::RegisterDefaultAllocator: the tests previously registered malloc/free
            // here and so never exercised - or noticed the absence of - the gem's own
            // hooks. See JoltBuoyancyAllocator.h.
            JoltBuoyancyAllocator::Install();
            JPH::Factory::sInstance = new JPH::Factory();
            JPH::RegisterTypes();

            m_tempAllocator = AZStd::make_unique<JPH::TempAllocatorImpl>(4 * 1024 * 1024);
            m_jobSystem = AZStd::make_unique<JPH::JobSystemThreadPool>(
                JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers, 1);

            m_physicsSystem = AZStd::make_unique<JPH::PhysicsSystem>();
            m_physicsSystem->Init(
                1024, 0, 1024, 1024, m_broadPhaseLayerInterface, m_objectVsBroadPhaseLayerFilter, m_objectLayerPairFilter);

            // Jolt's default gravity is along -Y; O3DE is Z up, and so is the water
            // volume's surface, so the test world has to match.
            m_physicsSystem->SetGravity(JPH::Vec3(0.0f, 0.0f, -9.81f));
        }

        void TearDown() override
        {
            m_waterVolume.Detach();
            m_physicsSystem.reset();
            m_jobSystem.reset();
            m_tempAllocator.reset();

            JPH::UnregisterTypes();
            delete JPH::Factory::sInstance;
            JPH::Factory::sInstance = nullptr;
        }

        //! A 1 m cube of the given mass, dropped at the given height. Density is
        //! mass / 1 m^3, so the mass alone decides whether it floats in fresh water.
        JPH::BodyID CreateCube(float mass, float height)
        {
            JPH::BodyCreationSettings settings(
                new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)), JPH::RVec3(0.0f, 0.0f, height), JPH::Quat::sIdentity(),
                JPH::EMotionType::Dynamic, TestObjectLayers::Moving);
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            settings.mMassPropertiesOverride.mMass = mass;

            JPH::BodyID bodyId = m_physicsSystem->GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::Activate);
            return bodyId;
        }

        void Simulate(float seconds)
        {
            const float fixedDeltaTime = 1.0f / 60.0f;
            const int steps = static_cast<int>(seconds / fixedDeltaTime);
            for (int i = 0; i < steps; ++i)
            {
                m_physicsSystem->Update(fixedDeltaTime, 1, m_tempAllocator.get(), m_jobSystem.get());

                // Standing in for the component's scene simulation-finish handler: bodies
                // the step found asleep can only be woken once the step has released the
                // body mutexes.
                m_waterVolume.WakePendingBodies();
                for (JoltWaterVolume* volume : m_extraVolumes)
                {
                    volume->WakePendingBodies();
                }
            }
        }

        bool IsBodyAsleep(const JPH::BodyID& bodyId) const
        {
            return !m_physicsSystem->GetBodyInterface().IsActive(bodyId);
        }

        //! A static floor whose top surface sits at the given height.
        void CreateFloor(float topZ = 0.0f)
        {
            JPH::BodyCreationSettings settings(
                new JPH::BoxShape(JPH::Vec3(50.0f, 50.0f, 0.5f)), JPH::RVec3(0.0f, 0.0f, topZ - 0.5f),
                JPH::Quat::sIdentity(), JPH::EMotionType::Static, TestObjectLayers::NonMoving);
            m_physicsSystem->GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::DontActivate);
        }

        //! Stamps a body with an entity id the way the JoltPhysics gem does when it creates
        //! one. That stamp is what lets a volume find per-body overrides and name the body
        //! in an enter or exit event.
        void SetBodyEntityId(const JPH::BodyID& bodyId, AZ::EntityId entityId)
        {
            m_physicsSystem->GetBodyInterface().SetUserData(bodyId, static_cast<AZ::u64>(entityId));
        }

        AZ::Vector3 GetBodyPosition(const JPH::BodyID& bodyId) const
        {
            const JPH::RVec3 position = m_physicsSystem->GetBodyInterface().GetPosition(bodyId);
            return AZ::Vector3(
                static_cast<float>(position.GetX()), static_cast<float>(position.GetY()),
                static_cast<float>(position.GetZ()));
        }

        //! A second 1 m cube at an arbitrary position, for the tests that need two.
        JPH::BodyID CreateCubeAt(float mass, const AZ::Vector3& position)
        {
            JPH::BodyCreationSettings settings(
                new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)),
                JPH::RVec3(position.GetX(), position.GetY(), position.GetZ()), JPH::Quat::sIdentity(),
                JPH::EMotionType::Dynamic, TestObjectLayers::Moving);
            settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            settings.mMassPropertiesOverride.mMass = mass;
            return m_physicsSystem->GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::Activate);
        }

        //! Where the volume's surface plane sits, in world Z, above a given column. Solves
        //! the plane rather than reading a height, so it stays right for a tilted volume.
        static float SurfaceHeightAt(const JoltWaterVolumeSnapshot& snapshot, float x, float y)
        {
            const AZ::Vector3 normal =
                snapshot.m_worldTransform.TransformVector(AZ::Vector3::CreateAxisZ()).GetNormalizedSafe();
            const AZ::Vector3 pointOnSurface = snapshot.m_worldTransform.TransformPoint(
                AZ::Vector3(0.0f, 0.0f, snapshot.m_dimensions.GetZ() * 0.5f));
            return pointOnSurface.GetZ() -
                (normal.GetX() * (x - pointOnSurface.GetX()) + normal.GetY() * (y - pointOnSurface.GetY())) / normal.GetZ();
        }

        float GetBodyZ(const JPH::BodyID& bodyId) const
        {
            return static_cast<float>(m_physicsSystem->GetBodyInterface().GetPosition(bodyId).GetZ());
        }

        //! Water filling z in [-5, 0], so the surface sits at z = 0.
        void CreateWater(float fluidDensity = 1000.0f)
        {
            JoltWaterVolumeSettings settings;
            settings.m_fluidDensity = fluidDensity;
            m_waterVolume.SetSettings(settings);
            m_waterVolume.SetVolume(
                AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), AZ::Vector3(50.0f, 50.0f, 5.0f));
            ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));
        }

        TestBroadPhaseLayerInterface m_broadPhaseLayerInterface;
        TestObjectVsBroadPhaseLayerFilter m_objectVsBroadPhaseLayerFilter;
        TestObjectLayerPairFilter m_objectLayerPairFilter;

        AZStd::unique_ptr<JPH::TempAllocatorImpl> m_tempAllocator;
        AZStd::unique_ptr<JPH::JobSystemThreadPool> m_jobSystem;
        AZStd::unique_ptr<JPH::PhysicsSystem> m_physicsSystem;
        JoltWaterVolume m_waterVolume;

        //! Additional volumes the overlap tests attach, pumped by Simulate alongside the
        //! main one.
        AZStd::vector<JoltWaterVolume*> m_extraVolumes;
    };

    TEST_F(JoltWaterVolumeTests, LightBodyFloatsAtTheSurface)
    {
        CreateWater();
        // 200 kg per cubic metre: much lighter than water, so it should surface and stay.
        auto cube = CreateCube(200.0f, -3.0f);

        Simulate(5.0f);

        // It rises from below and settles around the surface rather than sinking or
        // being launched out of the water.
        const float restingZ = GetBodyZ(cube);
        EXPECT_GT(restingZ, -0.6f);
        EXPECT_LT(restingZ, 0.6f);
    }

    TEST_F(JoltWaterVolumeTests, DenseBodySinks)
    {
        CreateWater();
        // 3000 kg per cubic metre: denser than water, so buoyancy only slows the fall.
        auto cube = CreateCube(3000.0f, 0.5f);

        Simulate(3.0f);

        EXPECT_LT(GetBodyZ(cube), -1.5f);
    }

    TEST_F(JoltWaterVolumeTests, DenserFluidFloatsABodyThatWouldOtherwiseSink)
    {
        // The same body in mercury-like fluid: density decides, not the body alone.
        CreateWater(13500.0f);
        auto cube = CreateCube(3000.0f, -3.0f);

        Simulate(5.0f);

        EXPECT_GT(GetBodyZ(cube), -1.0f);
    }

    TEST_F(JoltWaterVolumeTests, BodyOutsideTheVolumeIsUnaffected)
    {
        CreateWater();
        // Far to the side of the 50 x 50 volume, so it just falls.
        JPH::BodyCreationSettings settings(
            new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)), JPH::RVec3(100.0f, 0.0f, 0.0f), JPH::Quat::sIdentity(),
            JPH::EMotionType::Dynamic, TestObjectLayers::Moving);
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = 200.0f;
        JPH::BodyID cube = m_physicsSystem->GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::Activate);

        Simulate(1.0f);

        // Free fall for a second is about -4.9 m; buoyancy would have slowed it.
        EXPECT_LT(GetBodyZ(cube), -3.0f);
        EXPECT_EQ(m_waterVolume.GetSubmergedBodyCount(), 0);
    }

    TEST_F(JoltWaterVolumeTests, DisabledVolumeAppliesNothing)
    {
        CreateWater();
        auto cube = CreateCube(200.0f, -1.0f);

        m_waterVolume.SetEnabled(false);
        // Half a second of free fall is about 1.2 m, which keeps the body inside the
        // volume - long enough to show it sank, short enough that re-enabling can act
        // on it (the water only reaches down to z = -5).
        Simulate(0.5f);

        // A light body that would have floated instead falls freely.
        EXPECT_LT(GetBodyZ(cube), -1.8f);
        EXPECT_EQ(m_waterVolume.GetSubmergedBodyCount(), 0);

        // Re-enabling brings it back up to the surface.
        m_waterVolume.SetEnabled(true);
        Simulate(5.0f);
        EXPECT_GT(GetBodyZ(cube), -1.0f);
        EXPECT_GT(m_waterVolume.GetSubmergedBodyCount(), 0);
    }

    TEST_F(JoltWaterVolumeTests, FluidVelocityCarriesABodyAlong)
    {
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        settings.m_linearDrag = 2.0f; // a current needs drag to transfer its motion
        settings.m_fluidVelocity = AZ::Vector3(3.0f, 0.0f, 0.0f);
        m_waterVolume.SetSettings(settings);
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), AZ::Vector3(200.0f, 50.0f, 5.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        auto cube = CreateCube(200.0f, -1.0f);

        Simulate(3.0f);

        const float driftX = static_cast<float>(m_physicsSystem->GetBodyInterface().GetPosition(cube).GetX());
        EXPECT_GT(driftX, 1.0f);
    }

    TEST_F(JoltWaterVolumeTests, SleepingBodyWakesWhenTheWaterMovesOntoIt)
    {
        // The bug this pins: ApplyBuoyancyImpulse does not wake a sleeping body, and OnStep
        // used to skip inactive bodies outright, so a rising level or a moving volume slid
        // over anything that had already settled and never touched it again.
        CreateFloor();
        auto cube = CreateCube(200.0f, 0.6f); // light enough to float, dropped onto the floor

        // Attached from the start, but sitting far below the floor so it touches nothing.
        // The volume moving is what has to wake the body, not the volume appearing.
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        m_waterVolume.SetSettings(settings);
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -20.0f)), AZ::Vector3(50.0f, 50.0f, 5.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        // No water on it yet: it lands and goes to sleep.
        Simulate(4.0f);
        ASSERT_TRUE(IsBodyAsleep(cube));
        const float sleepingZ = GetBodyZ(cube);

        // Water rises over it, deep enough that a floating body would climb well clear.
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, 3.0f)), AZ::Vector3(50.0f, 50.0f, 6.0f));

        Simulate(4.0f);

        EXPECT_GT(GetBodyZ(cube), sleepingZ + 1.0f);
        EXPECT_GT(m_waterVolume.GetSubmergedBodyCount(), 0);
    }

    TEST_F(JoltWaterVolumeTests, SleepingBodyIsLeftAloneWhileTheWaterDoesNotChange)
    {
        // The other half of the fix: waking sleepers unconditionally every step would keep
        // every settled body permanently awake and stop anything sleeping at all.
        CreateFloor();
        CreateWater();
        auto cube = CreateCube(3000.0f, 1.0f); // dense: sinks and rests on the floor

        Simulate(6.0f);
        ASSERT_TRUE(IsBodyAsleep(cube));

        // Nothing about the water changes, so it must stay asleep.
        Simulate(3.0f);
        EXPECT_TRUE(IsBodyAsleep(cube));
    }

    TEST_F(JoltWaterVolumeTests, OverlappingVolumesDoNotDoubleTheImpulse)
    {
        // Two identical volumes over the same body used to apply two impulses, so a body
        // denser than the fluid - which must sink - would float instead.
        //
        // The floor sits inside the water so the body settles while still submerged. Left
        // to sink out of the bottom it would leave both volumes and report nothing.
        CreateFloor(-4.0f);
        CreateWater();

        JoltWaterVolume secondVolume;
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        secondVolume.SetSettings(settings);
        secondVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), AZ::Vector3(50.0f, 50.0f, 5.0f));
        ASSERT_TRUE(secondVolume.AttachToPhysicsSystem(m_physicsSystem.get()));
        m_extraVolumes.push_back(&secondVolume);

        // 1500 kg/m^3 against 1000 kg/m^3 water: one impulse leaves it sinking to the
        // floor, two would carry it back up to the surface.
        auto cube = CreateCube(1500.0f, -0.5f);

        // Sampled while it is still sinking. Once it settles on the floor it falls asleep,
        // and a sleeping body is deliberately not counted as submerged by either volume.
        Simulate(0.5f);
        const int claimed = m_waterVolume.GetSubmergedBodyCount() + secondVolume.GetSubmergedBodyCount();
        EXPECT_EQ(claimed, 1) << "exactly one of the two overlapping volumes should own the body";

        Simulate(4.0f);

        EXPECT_LT(GetBodyZ(cube), -3.0f) << "it should be resting on the floor inside the water";

        secondVolume.Detach();
        m_extraVolumes.clear();
    }

    TEST_F(JoltWaterVolumeTests, NonOverlappingVolumesEachKeepTheirOwnBodies)
    {
        // The overlap guard must not make a second volume stop working where the two do not
        // actually overlap.
        CreateWater();

        JoltWaterVolume farVolume;
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        farVolume.SetSettings(settings);
        farVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(200.0f, 0.0f, -2.5f)), AZ::Vector3(50.0f, 50.0f, 5.0f));
        ASSERT_TRUE(farVolume.AttachToPhysicsSystem(m_physicsSystem.get()));
        m_extraVolumes.push_back(&farVolume);

        auto nearCube = CreateCube(200.0f, -3.0f);
        auto farCube = CreateCubeAt(200.0f, AZ::Vector3(200.0f, 0.0f, -3.0f));

        Simulate(5.0f);

        EXPECT_GT(GetBodyZ(nearCube), -0.6f);
        EXPECT_GT(GetBodyPosition(farCube).GetZ(), -0.6f);
        EXPECT_EQ(m_waterVolume.GetSubmergedBodyCount(), 1);
        EXPECT_EQ(farVolume.GetSubmergedBodyCount(), 1);

        farVolume.Detach();
        m_extraVolumes.clear();
    }

    TEST_F(JoltWaterVolumeTests, TiltedVolumeGivesATiltedSurface)
    {
        // The local +Z face is the surface, so rotating the volume is what makes a sloped
        // river possible. Bodies should settle at the plane, not at a single world height.
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        m_waterVolume.SetSettings(settings);

        const AZ::Transform tilted = AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)) *
            AZ::Transform::CreateRotationY(AZ::DegToRad(15.0f));
        m_waterVolume.SetVolume(tilted, AZ::Vector3(60.0f, 20.0f, 8.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        auto nearOrigin = CreateCubeAt(200.0f, AZ::Vector3(0.0f, 0.0f, -3.0f));
        auto downSlope = CreateCubeAt(200.0f, AZ::Vector3(10.0f, 0.0f, -3.0f));

        Simulate(6.0f);

        // Buoyancy pushes along the surface normal, which on a tilted volume has a
        // horizontal component, so a floating body slides downhill. Each is therefore
        // checked against the surface above wherever it actually ended up, not where it
        // started - the point being that the surface is a plane, not one world height.
        const JoltWaterVolumeSnapshot snapshot = m_waterVolume.GetSnapshot();
        const AZ::Vector3 nearOriginEnd = GetBodyPosition(nearOrigin);
        const AZ::Vector3 downSlopeEnd = GetBodyPosition(downSlope);

        const float expectedNearOrigin = SurfaceHeightAt(snapshot, nearOriginEnd.GetX(), nearOriginEnd.GetY());
        const float expectedDownSlope = SurfaceHeightAt(snapshot, downSlopeEnd.GetX(), downSlopeEnd.GetY());

        EXPECT_NEAR(nearOriginEnd.GetZ(), expectedNearOrigin, 0.8f);
        EXPECT_NEAR(downSlopeEnd.GetZ(), expectedDownSlope, 0.8f);

        // And the two really did settle at different world heights, which is the whole
        // difference between a tilted surface and a flat one.
        EXPECT_GT(AZStd::abs(nearOriginEnd.GetZ() - downSlopeEnd.GetZ()), 0.5f);
    }

    TEST_F(JoltWaterVolumeTests, ExplicitBuoyancyFloatsABodyDenserThanTheFluid)
    {
        // The sealed hull case: a boat's collider volume is mostly air, so the density the
        // gem would derive says it must sink. An explicit factor is how that is authored.
        CreateWater();
        auto cube = CreateCube(3000.0f, -3.0f);
        const AZ::EntityId hull(0x5EA1EDu);

        JoltBuoyancyOverride hullOverride;
        hullOverride.m_mode = JoltBuoyancyMode::Explicit;
        hullOverride.m_factor = 2.0f;
        JoltBuoyancyOverrideRegistry::Get().Set(hull, hullOverride);
        SetBodyEntityId(cube, hull);

        Simulate(5.0f);

        // Without the override this is DenseBodySinks, which ends up below -1.5.
        EXPECT_GT(GetBodyZ(cube), -1.0f);

        JoltBuoyancyOverrideRegistry::Get().Remove(hull);
    }

    TEST_F(JoltWaterVolumeTests, ExcludedBodyIsIgnoredByWater)
    {
        CreateWater();
        auto cube = CreateCube(200.0f, -1.0f); // light: would float without the override
        const AZ::EntityId excluded(0xEC1DEDu);

        JoltBuoyancyOverride bodyOverride;
        bodyOverride.m_excluded = true;
        JoltBuoyancyOverrideRegistry::Get().Set(excluded, bodyOverride);
        SetBodyEntityId(cube, excluded);

        Simulate(1.0f);

        // Free fall for a second is about -4.9 m; buoyancy would have stopped it.
        EXPECT_LT(GetBodyZ(cube), -3.0f);
        EXPECT_EQ(m_waterVolume.GetSubmergedBodyCount(), 0);

        JoltBuoyancyOverrideRegistry::Get().Remove(excluded);
    }

    TEST_F(JoltWaterVolumeTests, WavesMoveTheSurfaceUpAndDown)
    {
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        settings.m_wavesEnabled = true;
        settings.m_waveAmplitude = 1.0f;
        settings.m_waveLength = 8.0f;
        settings.m_waveSpeed = 4.0f;
        m_waterVolume.SetSettings(settings);
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), AZ::Vector3(50.0f, 50.0f, 5.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        // A crest and a trough: a quarter and three quarters along the wave. Sampling half
        // a wavelength apart would compare two zero crossings and see no difference at all.
        const float atCrest = m_waterVolume.EvaluateSurface(AZ::Vector3(2.0f, 0.0f, 0.0f)).m_position.GetZ();
        const float atTrough = m_waterVolume.EvaluateSurface(AZ::Vector3(6.0f, 0.0f, 0.0f)).m_position.GetZ();
        EXPECT_GT(atCrest - atTrough, 1.5f) << "amplitude 1 means about 2 m between crest and trough";

        // The normal tilts off vertical where the surface slopes, which is what makes a
        // floating body rock rather than just bob. Sampled at a zero crossing, where the
        // slope is steepest - at the crest the surface is flat and the normal is straight up.
        const AZ::Vector3 slopedNormal = m_waterVolume.EvaluateSurface(AZ::Vector3::CreateZero()).m_normal;
        EXPECT_LT(slopedNormal.GetZ(), 0.999f);
        EXPECT_NEAR(slopedNormal.GetLength(), 1.0f, 0.01f);

        // And the wave travels: the same point is at a different height a moment later.
        const float atOrigin = m_waterVolume.EvaluateSurface(AZ::Vector3::CreateZero()).m_position.GetZ();
        Simulate(0.5f);
        const float atOriginLater = m_waterVolume.EvaluateSurface(AZ::Vector3::CreateZero()).m_position.GetZ();
        EXPECT_GT(AZStd::abs(atOriginLater - atOrigin), 0.1f);
    }

    TEST_F(JoltWaterVolumeTests, ACustomSurfaceFunctionReplacesTheBuiltInOne)
    {
        CreateWater();

        // Water that is not a plane at all, for lining up with something the gem knows
        // nothing about.
        // Deliberately inside the box: the volume still decides which bodies are considered
        // - that comes from its bounds - so a surface placed above the box would let bodies
        // rise out of the query and stop being affected.
        m_waterVolume.SetSurfaceFunction(
            []([[maybe_unused]] const AZ::Vector3& worldPoint)
            {
                JoltWaterSurfaceSample sample;
                sample.m_position = AZ::Vector3(0.0f, 0.0f, -2.0f);
                sample.m_normal = AZ::Vector3::CreateAxisZ();
                return sample;
            });

        EXPECT_NEAR(m_waterVolume.EvaluateSurface(AZ::Vector3::CreateZero()).m_position.GetZ(), -2.0f, 0.001f);

        // A body floats to the surface the function describes, not the volume's own face,
        // which for this volume would be z = 0.
        auto cube = CreateCube(200.0f, -4.5f);
        Simulate(6.0f);
        EXPECT_GT(GetBodyZ(cube), -2.7f);
        EXPECT_LT(GetBodyZ(cube), -1.3f);

        m_waterVolume.SetSurfaceFunction({});
    }

    TEST_F(JoltWaterVolumeTests, EnteringAndLeavingTheWaterRaisesEvents)
    {
        CreateWater();
        const AZ::EntityId floater(0xF10A7Eu);
        auto cube = CreateCube(200.0f, 4.0f); // starts above the surface at z = 0
        SetBodyEntityId(cube, floater);

        AZStd::vector<JoltWaterVolumeEvent> events;

        // Nothing yet: it is still in the air.
        m_waterVolume.TakePendingEvents(events);
        EXPECT_TRUE(events.empty());

        Simulate(2.0f);
        m_waterVolume.TakePendingEvents(events);

        // The first thing that happens is the entry. A body dropped from a height splashes
        // and can break the surface again on the way back up, so the count is not asserted -
        // only that it went in first, and how fast.
        ASSERT_FALSE(events.empty());
        EXPECT_TRUE(events[0].m_entered);
        EXPECT_EQ(events[0].m_bodyEntityId, floater);
        // It fell about 4 m before touching down, so it went in at a decent clip - which is
        // what tells a splash from a gentle drift.
        EXPECT_GT(events[0].m_speed, 1.0f);

        // Let it settle, discarding the churn from the splash.
        Simulate(4.0f);
        m_waterVolume.TakePendingEvents(events);

        // Floating quietly now, or asleep: either way it is still in the water, so nothing
        // more is raised.
        Simulate(1.0f);
        m_waterVolume.TakePendingEvents(events);
        EXPECT_TRUE(events.empty());

        // Taken out of the water: the volume notices it has gone.
        m_physicsSystem->GetBodyInterface().SetPosition(cube, JPH::RVec3(500.0f, 0.0f, 0.0f), JPH::EActivation::Activate);
        Simulate(0.2f);
        m_waterVolume.TakePendingEvents(events);

        ASSERT_EQ(events.size(), 1u);
        EXPECT_FALSE(events[0].m_entered);
        EXPECT_EQ(events[0].m_bodyEntityId, floater);
    }

    TEST_F(JoltWaterVolumeTests, SubmergedFractionIsReportedWhenAskedFor)
    {
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        settings.m_reportSubmergedFraction = true;
        m_waterVolume.SetSettings(settings);
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), AZ::Vector3(50.0f, 50.0f, 5.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        // A floor inside the water, so the dense body settles while still submerged instead
        // of sinking out through the bottom of the volume.
        CreateFloor(-4.0f);

        const AZ::EntityId heavy(0xDEE9u);
        const AZ::EntityId light(0xB0BBu);

        auto sunk = CreateCube(3000.0f, -1.0f); // dense: sinks to the floor, fully under
        SetBodyEntityId(sunk, heavy);
        auto bobbing = CreateCubeAt(200.0f, AZ::Vector3(6.0f, 0.0f, -3.0f)); // light: rides the surface
        SetBodyEntityId(bobbing, light);

        Simulate(4.0f);

        EXPECT_NEAR(m_waterVolume.GetSubmergedFraction(heavy), 1.0f, 0.05f);

        const float lightFraction = m_waterVolume.GetSubmergedFraction(light);
        EXPECT_GT(lightFraction, 0.05f);
        EXPECT_LT(lightFraction, 0.95f) << "a floating body should be partly out of the water";
    }

    TEST_F(JoltWaterVolumeTests, SubmergedFractionIsZeroUnlessRequested)
    {
        // Off by default, because it costs a second pass over each body's shape.
        CreateFloor(-4.0f);
        CreateWater();
        const AZ::EntityId body(0x0FFu);
        // Dense and still sinking when sampled: fully under the surface, and awake, so it
        // is definitely one of the bodies the volume applied to this step.
        auto cube = CreateCube(3000.0f, -1.0f);
        SetBodyEntityId(cube, body);

        Simulate(0.5f);

        EXPECT_GT(m_waterVolume.GetSubmergedBodyCount(), 0);
        EXPECT_EQ(m_waterVolume.GetSubmergedFraction(body), 0.0f);
    }

    TEST_F(JoltWaterVolumeTests, CompoundShapeFloatsOnItsWholeVolume)
    {
        // A compound is what a real authored object is: several colliders on one body.
        // Jolt sums the submerged volume across the parts, so the whole thing should float
        // on its combined displacement rather than on one part of it.
        CreateWater();

        JPH::StaticCompoundShapeSettings compoundSettings;
        compoundSettings.AddShape(JPH::Vec3(-0.75f, 0.0f, 0.0f), JPH::Quat::sIdentity(), new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)));
        compoundSettings.AddShape(JPH::Vec3(0.75f, 0.0f, 0.0f), JPH::Quat::sIdentity(), new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)));
        JPH::ShapeSettings::ShapeResult compoundResult = compoundSettings.Create();
        ASSERT_FALSE(compoundResult.HasError()) << compoundResult.GetError().c_str();

        // Two 1 m^3 boxes at 400 kg total is 200 kg/m^3, well under water.
        JPH::BodyCreationSettings bodySettings(
            compoundResult.Get(), JPH::RVec3(0.0f, 0.0f, -3.0f), JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic,
            TestObjectLayers::Moving);
        bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        bodySettings.mMassPropertiesOverride.mMass = 400.0f;
        JPH::BodyID compound = m_physicsSystem->GetBodyInterface().CreateAndAddBody(bodySettings, JPH::EActivation::Activate);

        Simulate(6.0f);

        const float restingZ = GetBodyPosition(compound).GetZ();
        EXPECT_GT(restingZ, -0.8f);
        EXPECT_LT(restingZ, 0.8f);
    }

    TEST_F(JoltWaterVolumeTests, AttachingToNothingFails)
    {
        EXPECT_FALSE(m_waterVolume.AttachToPhysicsSystem(nullptr));
        EXPECT_FALSE(m_waterVolume.IsAttached());
    }

    TEST_F(JoltWaterVolumeTests, AttachingAllocatesThroughThisModulesJoltHooks)
    {
        // Attaching grows PhysicsSystem::mStepListeners, and that Array is reallocated
        // by the copy of Jolt linked into whichever module called AddStepListener. With
        // the hooks left null - the state every module starts in - this is a call to
        // address zero, which is how the Editor crashed on entering game mode.
        //
        // Only an omission *in this module* is caught here; a module that never calls
        // Install still crashes, so keep the call in every module entry point.
        ASSERT_NE(JPH::Reallocate, nullptr);
        ASSERT_NE(JPH::Allocate, nullptr);
        ASSERT_NE(JPH::Free, nullptr);
        ASSERT_NE(JPH::AlignedAllocate, nullptr);
        ASSERT_NE(JPH::AlignedFree, nullptr);

        EXPECT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));
        m_waterVolume.Detach();
    }

} // namespace JoltBuoyancy
