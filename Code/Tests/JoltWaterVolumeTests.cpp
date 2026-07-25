#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <Clients/JoltBuoyancyAllocator.h>
#include <Clients/JoltWaterVolume.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
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
            }
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
