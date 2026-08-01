#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Component/Entity.h>

#include <Clients/JoltBuoyancyOverrideComponent.h>
#include <Clients/JoltBuoyancyOverrideRegistry.h>
#include <Clients/JoltWaterVolumeComponent.h>

namespace JoltBuoyancy
{
    // The volume tests drive JoltWaterVolume directly. These cover the layer above it: that
    // the component actually forwards what the inspector authored, and keeps forwarding it
    // when gameplay changes a value. A setter that updated only the component's own copy
    // would pass every physics test and still do nothing in a level.
    namespace
    {
        //! The component's bus overrides are protected, as they should be - callers reach
        //! them through the EBus. These tests want them without standing up an entity and a
        //! transform component, so a subclass opens them up.
        class TestableWaterVolumeComponent : public JoltWaterVolumeComponent
        {
        public:
            using JoltWaterVolumeComponent::GetAngularDrag;
            using JoltWaterVolumeComponent::GetDimensions;
            using JoltWaterVolumeComponent::GetFluidDensity;
            using JoltWaterVolumeComponent::GetFluidVelocity;
            using JoltWaterVolumeComponent::GetLinearDrag;
            using JoltWaterVolumeComponent::GetSubmergedBodyCount;
            using JoltWaterVolumeComponent::GetSubmergedFraction;
            using JoltWaterVolumeComponent::GetWaterSettings;
            using JoltWaterVolumeComponent::GetSeaState;
            using JoltWaterVolumeComponent::GetSignificantWaveHeight;
            using JoltWaterVolumeComponent::GetWavesEnabled;
            using JoltWaterVolumeComponent::SetAngularDrag;
            using JoltWaterVolumeComponent::SetDimensions;
            using JoltWaterVolumeComponent::SetFluidDensity;
            using JoltWaterVolumeComponent::SetFluidVelocity;
            using JoltWaterVolumeComponent::SetLinearDrag;
            using JoltWaterVolumeComponent::SetWaterSettings;
            using JoltWaterVolumeComponent::SetSeaState;
            using JoltWaterVolumeComponent::SetWavesEnabled;
        };
    } // namespace

    class JoltWaterVolumeComponentTests : public ::testing::Test
    {
    protected:
        TestableWaterVolumeComponent m_component;
    };

    TEST_F(JoltWaterVolumeComponentTests, AuthoredSettingsAreReadableThroughTheBus)
    {
        // What BuildGameEntity fills in before activation.
        m_component.AccessDimensions() = AZ::Vector3(4.0f, 5.0f, 6.0f);
        m_component.AccessSettings().m_fluidDensity = 1234.0f;
        m_component.AccessSettings().m_wavesEnabled = true;
        m_component.AccessSettings().m_spectrum.m_beaufort = 6.0f;

        EXPECT_EQ(m_component.GetDimensions(), AZ::Vector3(4.0f, 5.0f, 6.0f));
        EXPECT_FLOAT_EQ(m_component.GetFluidDensity(), 1234.0f);
        EXPECT_TRUE(m_component.GetWavesEnabled());
        EXPECT_FLOAT_EQ(m_component.GetSeaState(), 6.0f);
    }

    TEST_F(JoltWaterVolumeComponentTests, SettersRoundTrip)
    {
        m_component.SetFluidDensity(800.0f);
        m_component.SetLinearDrag(3.0f);
        m_component.SetAngularDrag(0.25f);
        m_component.SetFluidVelocity(AZ::Vector3(1.0f, 2.0f, 3.0f));
        m_component.SetDimensions(AZ::Vector3(7.0f, 8.0f, 9.0f));
        m_component.SetWavesEnabled(true);
        m_component.SetSeaState(7.0f);

        EXPECT_FLOAT_EQ(m_component.GetFluidDensity(), 800.0f);
        EXPECT_FLOAT_EQ(m_component.GetLinearDrag(), 3.0f);
        EXPECT_FLOAT_EQ(m_component.GetAngularDrag(), 0.25f);
        EXPECT_EQ(m_component.GetFluidVelocity(), AZ::Vector3(1.0f, 2.0f, 3.0f));
        EXPECT_EQ(m_component.GetDimensions(), AZ::Vector3(7.0f, 8.0f, 9.0f));
        EXPECT_TRUE(m_component.GetWavesEnabled());
        EXPECT_FLOAT_EQ(m_component.GetSeaState(), 7.0f);
    }

    TEST_F(JoltWaterVolumeComponentTests, WholeSettingsGoInAndComeBackOut)
    {
        JoltWaterVolumeSettings settings;
        settings.m_fluidDensity = 1025.0f; // sea water
        settings.m_linearDrag = 2.0f;
        settings.m_wavesEnabled = true;
        settings.m_spectrum.m_beaufort = 8.0f;
        settings.m_reportSubmergedFraction = true;

        m_component.SetWaterSettings(settings);

        const JoltWaterVolumeSettings readBack = m_component.GetWaterSettings();
        EXPECT_FLOAT_EQ(readBack.m_fluidDensity, 1025.0f);
        EXPECT_FLOAT_EQ(readBack.m_linearDrag, 2.0f);
        EXPECT_TRUE(readBack.m_wavesEnabled);
        EXPECT_FLOAT_EQ(readBack.m_spectrum.m_beaufort, 8.0f);
        EXPECT_TRUE(readBack.m_reportSubmergedFraction);
    }

    TEST_F(JoltWaterVolumeComponentTests, DimensionsAreClampedAwayFromZero)
    {
        // A zero-sized volume would hand the broadphase a degenerate box.
        m_component.SetDimensions(AZ::Vector3::CreateZero());

        const AZ::Vector3 dimensions = m_component.GetDimensions();
        EXPECT_GT(dimensions.GetX(), 0.0f);
        EXPECT_GT(dimensions.GetY(), 0.0f);
        EXPECT_GT(dimensions.GetZ(), 0.0f);
    }

    TEST_F(JoltWaterVolumeComponentTests, QueriesAnswerWithNoSceneAttached)
    {
        // Never activated, so there is no volume in any scene. These must answer rather
        // than crash, which is the state a component is in before the physics scene exists.
        EXPECT_FLOAT_EQ(m_component.GetSubmergedFraction(AZ::EntityId(0x1234u)), 0.0f);
        EXPECT_EQ(m_component.GetSubmergedBodyCount(), 0);
    }

    // The override component is the other half of the authoring story: it has to reach the
    // registry that water volumes read while stepping, not just hold its own values. It is
    // driven through the EBus here, which is the real path and needs no subclass.
    class JoltBuoyancyOverrideComponentTests : public ::testing::Test
    {
    protected:
        static constexpr AZ::u64 TestEntityId = 0xB0A7u;

        void SetUp() override
        {
            m_entity = AZStd::make_unique<AZ::Entity>(AZ::EntityId(TestEntityId));
            m_entity->CreateComponent<JoltBuoyancyOverrideComponent>();
            m_entity->Init();
            m_entity->Activate();
        }

        void TearDown() override
        {
            if (m_entity && m_entity->GetState() == AZ::Entity::State::Active)
            {
                m_entity->Deactivate();
            }
            m_entity.reset();
            JoltBuoyancyOverrideRegistry::Get().Remove(AZ::EntityId(TestEntityId));
        }

        AZStd::unique_ptr<AZ::Entity> m_entity;
    };

    TEST_F(JoltBuoyancyOverrideComponentTests, SettingAnExplicitFactorReachesTheRegistry)
    {
        JoltBuoyancyOverrideRequestBus::Event(
            AZ::EntityId(TestEntityId), &JoltBuoyancyOverrideRequests::SetBuoyancyMode, JoltBuoyancyMode::Explicit);
        JoltBuoyancyOverrideRequestBus::Event(
            AZ::EntityId(TestEntityId), &JoltBuoyancyOverrideRequests::SetBuoyancyFactor, 3.0f);

        const JoltBuoyancyOverride published = JoltBuoyancyOverrideRegistry::Get().Find(AZ::EntityId(TestEntityId));
        EXPECT_EQ(published.m_mode, JoltBuoyancyMode::Explicit);
        EXPECT_FLOAT_EQ(published.m_factor, 3.0f);
    }

    TEST_F(JoltBuoyancyOverrideComponentTests, DeactivatingRemovesTheOverride)
    {
        JoltBuoyancyOverrideRequestBus::Event(
            AZ::EntityId(TestEntityId), &JoltBuoyancyOverrideRequests::SetExcludedFromWater, true);
        ASSERT_TRUE(JoltBuoyancyOverrideRegistry::Get().Find(AZ::EntityId(TestEntityId)).m_excluded);

        m_entity->Deactivate();

        // A removed entity must not keep steering water volumes.
        EXPECT_FALSE(JoltBuoyancyOverrideRegistry::Get().Find(AZ::EntityId(TestEntityId)).m_excluded);
        EXPECT_TRUE(JoltBuoyancyOverrideRegistry::Get().IsEmpty());
    }
} // namespace JoltBuoyancy
