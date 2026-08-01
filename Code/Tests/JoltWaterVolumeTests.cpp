#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <Clients/JoltBuoyancyAllocator.h>
#include <Clients/JoltBuoyancyOverrideRegistry.h>
#include <Clients/JoltGerstnerWaves.h>
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
        settings.m_spectrum.m_beaufort = 7.0f;
        settings.m_spectrum.m_componentCount = 4;
        m_waterVolume.SetSettings(settings);
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), AZ::Vector3(200.0f, 200.0f, 5.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        // With a spectrum there is no single wavelength to pick a crest and a trough from,
        // so the surface is swept instead. The detailed wave maths lives in
        // JoltGerstnerWaveTests; this checks the volume actually wires it up.
        float lowest = 1000.0f;
        float highest = -1000.0f;
        for (int x = -80; x <= 80; ++x)
        {
            const float height =
                m_waterVolume.EvaluateSurface(AZ::Vector3(static_cast<float>(x), 0.0f, 0.0f)).m_position.GetZ();
            lowest = AZStd::min(lowest, height);
            highest = AZStd::max(highest, height);
        }
        EXPECT_GT(highest - lowest, 0.3f) << "the volume's surface should not be flat with waves on";

        // The normal tilts off vertical somewhere on a wavy surface, which is what makes a
        // floating body rock rather than just bob.
        bool foundSlope = false;
        for (int x = -80; x <= 80 && !foundSlope; ++x)
        {
            const AZ::Vector3 normal =
                m_waterVolume.EvaluateSurface(AZ::Vector3(static_cast<float>(x), 0.0f, 0.0f)).m_normal;
            EXPECT_NEAR(normal.GetLength(), 1.0f, 0.01f);
            foundSlope = normal.GetZ() < 0.999f;
        }
        EXPECT_TRUE(foundSlope);

        // And the sea moves: the same point is at a different height a moment later.
        const AZ::Vector3 probe(3.0f, 0.0f, 0.0f);
        const float before = m_waterVolume.EvaluateSurface(probe).m_position.GetZ();
        Simulate(0.5f);
        EXPECT_GT(AZStd::abs(m_waterVolume.EvaluateSurface(probe).m_position.GetZ() - before), 0.02f);
    }

    TEST_F(JoltWaterVolumeTests, TheWaterHasOrbitalVelocity)
    {
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        settings.m_wavesEnabled = true;
        settings.m_spectrum.m_beaufort = 7.0f;
        settings.m_spectrum.m_componentCount = 4;
        m_waterVolume.SetSettings(settings);
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), AZ::Vector3(200.0f, 200.0f, 5.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        // Passing only the volume's current makes the sea a conveyor belt. Real water
        // orbits, and that is what a boat surges down a swell on.
        float mostPositive = -1000.0f;
        float mostNegative = 1000.0f;
        for (int x = -80; x <= 80; ++x)
        {
            const AZ::Vector3 velocity = m_waterVolume.GetWaterVelocityAt(AZ::Vector3(static_cast<float>(x), 0.0f, 0.0f));
            mostPositive = AZStd::max(mostPositive, velocity.GetZ());
            mostNegative = AZStd::min(mostNegative, velocity.GetZ());
        }
        EXPECT_GT(mostPositive, 0.05f);
        EXPECT_LT(mostNegative, -0.05f);

        // The volume's own current is still in there on top of the orbital part.
        settings.m_fluidVelocity = AZ::Vector3(100.0f, 0.0f, 0.0f);
        m_waterVolume.SetSettings(settings);
        EXPECT_GT(m_waterVolume.GetWaterVelocityAt(AZ::Vector3::CreateZero()).GetX(), 50.0f);
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

    TEST_F(JoltWaterVolumeTests, AStraddlingBodyIsOnlyClaimedByTheVolumeHoldingIt)
    {
        // The hole the first overlap fix left. Arbitration only ran when a volume contained
        // the body's centre, so for a body sitting in A while still overlapping B's query
        // box, A applied and B never arbitrated at all. Jolt measures the whole shape
        // against each surface, so the straddler took roughly double the impulse.
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        m_waterVolume.SetSettings(settings);
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(-5.0f, 0.0f, -2.5f)), AZ::Vector3(10.0f, 10.0f, 5.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        // Sits edge to edge with the first, so their boxes meet at x = 0.
        JoltWaterVolume neighbour;
        neighbour.SetSettings(settings);
        neighbour.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(5.0f, 0.0f, -2.5f)), AZ::Vector3(10.0f, 10.0f, 5.0f));
        ASSERT_TRUE(neighbour.AttachToPhysicsSystem(m_physicsSystem.get()));
        m_extraVolumes.push_back(&neighbour);

        // Denser than the fluid, so one impulse leaves it sinking and two float it.
        // Placed just inside the first volume but close enough to the seam that its shape
        // still overlaps the second's box.
        CreateFloor(-4.0f);
        auto cube = CreateCubeAt(1500.0f, AZ::Vector3(-0.2f, 0.0f, -0.5f));

        Simulate(0.5f);
        const int claimed = m_waterVolume.GetSubmergedBodyCount() + neighbour.GetSubmergedBodyCount();
        EXPECT_EQ(claimed, 1) << "the straddling body should be claimed once, not by both volumes";

        Simulate(4.0f);
        EXPECT_LT(GetBodyPosition(cube).GetZ(), -3.0f) << "it should still sink, not be floated by a doubled impulse";

        neighbour.Detach();
        m_extraVolumes.clear();
    }

    TEST_F(JoltWaterVolumeTests, DisablingClearsTheSubmergedStateAndRaisesExits)
    {
        // A floor inside the water and a dense body, so it is reliably still submerged and
        // awake when the state is sampled rather than bobbing through the surface.
        CreateFloor(-4.0f);
        CreateWater();
        const AZ::EntityId floater(0xD15AB1u);
        auto cube = CreateCube(1500.0f, -1.0f);
        SetBodyEntityId(cube, floater);

        AZStd::vector<JoltWaterVolumeEvent> events;
        Simulate(0.5f);
        m_waterVolume.TakePendingEvents(events);
        ASSERT_FALSE(events.empty());
        ASSERT_TRUE(events[0].m_entered);
        ASSERT_GT(m_waterVolume.GetSubmergedBodyCount(), 0);

        // Switching off used to skip publishing entirely, so the fraction and the
        // submerged set kept their pre-disable values and the exit was not noticed until
        // the volume was switched back on.
        m_waterVolume.SetEnabled(false);
        Simulate(0.2f);

        m_waterVolume.TakePendingEvents(events);
        ASSERT_EQ(events.size(), 1u);
        EXPECT_FALSE(events[0].m_entered);
        EXPECT_EQ(events[0].m_bodyEntityId, floater);
        EXPECT_EQ(m_waterVolume.GetSubmergedBodyCount(), 0);
        EXPECT_FLOAT_EQ(m_waterVolume.GetSubmergedFraction(floater), 0.0f);
    }

    TEST_F(JoltWaterVolumeTests, SettledSleepingBodiesAreStillCounted)
    {
        // A pool of floaters that have settled and gone to sleep used to report zero, which
        // is exactly what a volume that had stopped working reports - so the counter was
        // useless for the one thing it exists to diagnose.
        CreateFloor();
        CreateWater();
        auto cube = CreateCube(200.0f, -1.0f);
        SetBodyEntityId(cube, AZ::EntityId(0x51EEDu));

        Simulate(8.0f);

        ASSERT_TRUE(IsBodyAsleep(cube)) << "the test needs it to have settled";
        EXPECT_GT(m_waterVolume.GetSubmergedBodyCount(), 0);
    }

    TEST_F(JoltWaterVolumeTests, SphereVolumeContainsAndFloats)
    {
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        settings.m_shape = JoltWaterVolumeShape::Sphere;
        m_waterVolume.SetSettings(settings);
        // A 10 m sphere centred at the origin: its surface sits at z = +5.
        m_waterVolume.SetVolume(AZ::Transform::CreateIdentity(), AZ::Vector3(10.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        const JoltWaterVolumeSnapshot snapshot = m_waterVolume.GetSnapshot();
        EXPECT_TRUE(snapshot.Contains(AZ::Vector3(0.0f, 0.0f, 0.0f)));
        EXPECT_TRUE(snapshot.Contains(AZ::Vector3(4.9f, 0.0f, 0.0f)));
        // Inside the bounding box but outside the sphere, which a box volume would accept.
        EXPECT_FALSE(snapshot.Contains(AZ::Vector3(4.0f, 4.0f, 0.0f)));

        auto cube = CreateCube(200.0f, -3.0f);
        Simulate(5.0f);
        EXPECT_GT(GetBodyZ(cube), 4.0f) << "a light body should rise to the top of the sphere";
    }

    TEST_F(JoltWaterVolumeTests, WaterQueriesAnswerForGameplay)
    {
        CreateWater(); // z from -5 to 0, surface at z = 0

        EXPECT_TRUE(m_waterVolume.IsPointUnderwater(AZ::Vector3(0.0f, 0.0f, -2.0f)));
        EXPECT_FALSE(m_waterVolume.IsPointUnderwater(AZ::Vector3(0.0f, 0.0f, 1.0f))) << "above the surface";
        EXPECT_FALSE(m_waterVolume.IsPointUnderwater(AZ::Vector3(100.0f, 0.0f, -2.0f))) << "outside the volume";

        EXPECT_NEAR(m_waterVolume.GetDepthAt(AZ::Vector3(0.0f, 0.0f, -2.0f)), 2.0f, 0.001f);
        EXPECT_LT(m_waterVolume.GetDepthAt(AZ::Vector3(0.0f, 0.0f, 1.0f)), 0.0f);
        EXPECT_NEAR(m_waterVolume.EvaluateSurface(AZ::Vector3(3.0f, 0.0f, -2.0f)).m_position.GetZ(), 0.0f, 0.001f);
    }

    TEST_F(JoltWaterVolumeTests, WaveCrestsStayInsideTheQueryBounds)
    {
        // The waves lift the surface above the volume's own lid and drag it sideways, but
        // the broadphase query decides which bodies are looked at. Unpadded, a body riding
        // a crest leaves the query, stops being affected, falls back in and oscillates.
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        settings.m_wavesEnabled = true;
        settings.m_spectrum.m_beaufort = 8.0f;
        settings.m_spectrum.m_componentCount = 4;
        m_waterVolume.SetSettings(settings);

        const AZ::Vector3 dimensions(100.0f, 100.0f, 5.0f);
        m_waterVolume.SetVolume(AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), dimensions);
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        const JoltGerstnerWaves waves = m_waterVolume.GetWaves();
        ASSERT_FALSE(waves.IsEmpty());
        const JoltWaterVolumeSnapshot snapshot = m_waterVolume.GetSnapshot();

        // Every component contributes, and the horizontal drag counts too - a single
        // component's amplitude is not enough padding.
        EXPECT_GE(snapshot.m_worldBounds.GetMax().GetZ(), waves.GetMaximumHeight() * 0.99f);
        EXPECT_GE(
            snapshot.m_worldBounds.GetMax().GetX(),
            dimensions.GetX() * 0.5f + waves.GetMaximumHorizontalDisplacement() * 0.99f);

        // No point on the surface may escape the padded bounds.
        for (int x = -50; x <= 50; x += 2)
        {
            const AZ::Vector3 surface =
                m_waterVolume.EvaluateSurface(AZ::Vector3(static_cast<float>(x), 0.0f, 0.0f)).m_position;
            EXPECT_LE(surface.GetZ(), snapshot.m_worldBounds.GetMax().GetZ() + 0.01f);
        }
    }

    TEST_F(JoltWaterVolumeTests, DragMultiplierChangesHowFastABodyIsSlowed)
    {
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        settings.m_linearDrag = 4.0f; // thick water, so the multiplier has something to scale
        m_waterVolume.SetSettings(settings);
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), AZ::Vector3(200.0f, 50.0f, 5.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        const AZ::EntityId streamlined(0x51EEC);
        auto draggy = CreateCubeAt(900.0f, AZ::Vector3(0.0f, -5.0f, -2.0f));
        auto sleek = CreateCubeAt(900.0f, AZ::Vector3(0.0f, 5.0f, -2.0f));
        SetBodyEntityId(sleek, streamlined);

        JoltBuoyancyOverride sleekOverride;
        sleekOverride.m_linearDragMultiplier = 0.05f;
        JoltBuoyancyOverrideRegistry::Get().Set(streamlined, sleekOverride);

        // Same shove to both.
        const JPH::Vec3 push(12.0f, 0.0f, 0.0f);
        m_physicsSystem->GetBodyInterface().SetLinearVelocity(draggy, push);
        m_physicsSystem->GetBodyInterface().SetLinearVelocity(sleek, push);

        Simulate(1.5f);

        // The streamlined one keeps more of its shove, which buoyancy factor and exclusion
        // together could not express.
        EXPECT_GT(GetBodyPosition(sleek).GetX(), GetBodyPosition(draggy).GetX() + 1.0f);

        JoltBuoyancyOverrideRegistry::Get().Remove(streamlined);
    }

    TEST_F(JoltWaterVolumeTests, DirectionalDragLetsAHullHoldItsHeading)
    {
        // Jolt's drag is already quadratic and already varies with the projected area of
        // the body's bounding box - it is not isotropic. What a box cannot express is a
        // hull being far more streamlined along its length than across it, which is what
        // this override adds, and what stops a boat sliding sideways through a turn.
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        settings.m_linearDrag = 3.0f;
        m_waterVolume.SetSettings(settings);
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), AZ::Vector3(400.0f, 400.0f, 5.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        const AZ::EntityId hullId(0x5111Du);
        auto hull = CreateCubeAt(800.0f, AZ::Vector3(0.0f, -8.0f, -2.0f));
        auto plain = CreateCubeAt(800.0f, AZ::Vector3(0.0f, 8.0f, -2.0f));
        SetBodyEntityId(hull, hullId);

        // Streamlined along X, ordinary across it.
        JoltBuoyancyOverride hullOverride;
        hullOverride.m_directionalDrag = AZ::Vector3(0.05f, 1.0f, 1.0f);
        JoltBuoyancyOverrideRegistry::Get().Set(hullId, hullOverride);

        const JPH::Vec3 alongX(10.0f, 0.0f, 0.0f);
        m_physicsSystem->GetBodyInterface().SetLinearVelocity(hull, alongX);
        m_physicsSystem->GetBodyInterface().SetLinearVelocity(plain, alongX);

        Simulate(2.0f);

        EXPECT_GT(GetBodyPosition(hull).GetX(), GetBodyPosition(plain).GetX() + 1.0f)
            << "the streamlined axis should keep more of its way on";

        // The anisotropy itself: the same hull, the same shove, across its beam instead of
        // along its length, must not carry nearly as far. Compared against itself rather
        // than against the unmodified body, because the drag is split between Jolt and the
        // extra pass and the two clamp independently - the total is deliberately not
        // identical to a single unsplit drag of the same magnitude.
        const float travelledAlongAxis = GetBodyPosition(hull).GetX();

        auto hullSideways = CreateCubeAt(800.0f, AZ::Vector3(-60.0f, -8.0f, -2.0f));
        SetBodyEntityId(hullSideways, hullId);
        m_physicsSystem->GetBodyInterface().SetLinearVelocity(hullSideways, JPH::Vec3(0.0f, 10.0f, 0.0f));

        Simulate(2.0f);
        const float travelledAcrossAxis = GetBodyPosition(hullSideways).GetY() - (-8.0f);

        EXPECT_GT(travelledAlongAxis, travelledAcrossAxis * 2.0f)
            << "a hull streamlined along X should carry much further along it than across it";

        JoltBuoyancyOverrideRegistry::Get().Remove(hullId);
    }

    TEST_F(JoltWaterVolumeTests, AddedMassDampsAccelerationWithoutStoppingTheBody)
    {
        // Water dragged along with the hull. Jolt has no notion of it, and doing it
        // properly needs the solver's mass matrix, so this resists changes in velocity
        // after the fact - enough to take the twitchiness out of heave.
        CreateWater();

        const AZ::EntityId heavyWater(0xADDEDu);
        auto withAddedMass = CreateCubeAt(500.0f, AZ::Vector3(0.0f, -6.0f, -2.0f));
        auto without = CreateCubeAt(500.0f, AZ::Vector3(0.0f, 6.0f, -2.0f));
        SetBodyEntityId(withAddedMass, heavyWater);

        JoltBuoyancyOverride addedMassOverride;
        addedMassOverride.m_addedMass = 1.5f;
        JoltBuoyancyOverrideRegistry::Get().Set(heavyWater, addedMassOverride);

        // Both start at rest and are floated by the same water, so the only difference is
        // how quickly they can change velocity.
        Simulate(0.35f);

        const float withSpeed = AZStd::abs(
            static_cast<float>(m_physicsSystem->GetBodyInterface().GetLinearVelocity(withAddedMass).GetZ()));
        const float withoutSpeed = AZStd::abs(
            static_cast<float>(m_physicsSystem->GetBodyInterface().GetLinearVelocity(without).GetZ()));

        EXPECT_LT(withSpeed, withoutSpeed) << "added mass should slow how fast the body picks up speed";

        // It damps acceleration, not motion: the body still rises to the surface.
        Simulate(6.0f);
        EXPECT_GT(GetBodyPosition(withAddedMass).GetZ(), -1.5f);

        JoltBuoyancyOverrideRegistry::Get().Remove(heavyWater);
    }

    TEST_F(JoltWaterVolumeTests, APlaneVolumeHasNoFloor)
    {
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1000.0f;
        settings.m_shape = JoltWaterVolumeShape::Plane;
        m_waterVolume.SetSettings(settings);
        // Surface at z = 0, horizontal extent 100 m, nominal depth 10 m.
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -5.0f)), AZ::Vector3(100.0f, 100.0f, 10.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        const JoltWaterVolumeSnapshot snapshot = m_waterVolume.GetSnapshot();

        // Inside the extent and below the surface is water, however deep. A box stops
        // being water at its bottom face, which is wrong for open sea - a body that sinks
        // out of it suddenly weighs its full dry weight again.
        EXPECT_TRUE(snapshot.Contains(AZ::Vector3(0.0f, 0.0f, -2.0f)));
        EXPECT_TRUE(snapshot.Contains(AZ::Vector3(0.0f, 0.0f, -500.0f))) << "a plane has no floor";
        EXPECT_FALSE(snapshot.Contains(AZ::Vector3(0.0f, 0.0f, 2.0f))) << "above the surface is not water";
        EXPECT_FALSE(snapshot.Contains(AZ::Vector3(200.0f, 0.0f, -2.0f))) << "outside the extent is not water";
    }

    TEST_F(JoltWaterVolumeTests, ShallowWaterShortensTheLongestWaves)
    {
        // w^2 = g k tanh(k d). Long waves feel the bottom first, slow and shorten, which is
        // swell steepening as it runs into a beach.
        JoltWaterSpectrum deep;
        deep.m_beaufort = 6.0f;
        deep.m_waterDepth = 0.0f;
        JoltGerstnerWaves deepWaves;
        deepWaves.Synthesise(deep);

        JoltWaterSpectrum shallow = deep;
        shallow.m_waterDepth = 3.0f;
        JoltGerstnerWaves shallowWaves;
        shallowWaves.Synthesise(shallow);

        ASSERT_FALSE(deepWaves.IsEmpty());
        ASSERT_EQ(deepWaves.GetComponents().size(), shallowWaves.GetComponents().size());

        const auto longestWavelength = [](const JoltGerstnerWaves& waves)
        {
            float longest = 0.0f;
            for (const JoltGerstnerComponent& component : waves.GetComponents())
            {
                longest = AZStd::max(longest, AZ::Constants::TwoPi / component.m_waveNumber);
            }
            return longest;
        };

        EXPECT_LT(longestWavelength(shallowWaves), longestWavelength(deepWaves));
    }

    TEST_F(JoltWaterVolumeTests, OverlappingVolumesBlendTheirCurrents)
    {
        // An estuary: a river current meeting still water. Ownership is all or nothing, so
        // without blending a body crossing the seam changes current in a single step.
        JoltWaterVolumeSettings still;
        still.m_fluidDensity = 1000.0f;
        still.m_linearDrag = 2.0f;
        m_waterVolume.SetSettings(still);
        m_waterVolume.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -2.5f)), AZ::Vector3(40.0f, 40.0f, 5.0f));
        ASSERT_TRUE(m_waterVolume.AttachToPhysicsSystem(m_physicsSystem.get()));

        JoltWaterVolume river;
        JoltWaterVolumeSettings flowing = still;
        flowing.m_fluidVelocity = AZ::Vector3(6.0f, 0.0f, 0.0f);
        river.SetSettings(flowing);
        // Overlaps the still water, sitting slightly lower so the still volume owns bodies
        // in the shared region.
        river.SetVolume(
            AZ::Transform::CreateTranslation(AZ::Vector3(0.0f, 0.0f, -3.0f)), AZ::Vector3(40.0f, 40.0f, 5.0f));
        ASSERT_TRUE(river.AttachToPhysicsSystem(m_physicsSystem.get()));
        m_extraVolumes.push_back(&river);

        // Neutrally buoyant and placed well below both surfaces, so each volume genuinely
        // holds it. A body floating at the river's own surface has almost no depth in the
        // river and would correctly get almost none of its current.
        auto cube = CreateCube(1000.0f, -3.0f);
        Simulate(3.0f);

        // Exactly one volume owns it, so it is not double-pushed - but the current it feels
        // is a blend, so it does move downstream rather than sitting still.
        const float driftX = GetBodyPosition(cube).GetX();
        EXPECT_GT(driftX, 0.5f) << "the owner should blend in the overlapping river's current";
        EXPECT_LT(driftX, 18.0f) << "but not feel the full current as though it were the river's alone";

        river.Detach();
        m_extraVolumes.clear();
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
