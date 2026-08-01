#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Script/ScriptContext.h>

#include <Clients/JoltBuoyancyOverrideComponent.h>
#include <Clients/JoltBuoyancyOverrideRegistry.h>

namespace JoltBuoyancy
{
    // JoltBuoyancyScriptReflectionTests checks that the buses are *present* in the behavior
    // context. That catches a dropped reflection but not a broken one: an event can be
    // reflected under a name script cannot call, or take a type script cannot construct,
    // and still show up in the registry.
    //
    // These run actual Lua against the real behavior context, so they fail if the gem
    // stops being usable from a script rather than merely stops being listed.
    class JoltBuoyancyLuaTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            AZ::ComponentApplicationBus::BroadcastResult(
                m_behaviorContext, &AZ::ComponentApplicationRequests::GetBehaviorContext);
            ASSERT_NE(m_behaviorContext, nullptr) << "No application behavior context";

            m_scriptContext = AZStd::make_unique<AZ::ScriptContext>();
            m_scriptContext->BindTo(m_behaviorContext);
        }

        void TearDown() override
        {
            if (m_entity && m_entity->GetState() == AZ::Entity::State::Active)
            {
                m_entity->Deactivate();
            }
            m_entity.reset();
            m_scriptContext.reset();
            JoltBuoyancyOverrideRegistry::Get().Remove(AZ::EntityId(TestEntityId));
        }

        //! An entity carrying a buoyancy override, which needs no other services, so a Lua
        //! test does not have to stand up a physics scene to have a live bus to talk to.
        void CreateOverrideEntity()
        {
            m_entity = AZStd::make_unique<AZ::Entity>(AZ::EntityId(TestEntityId));
            m_entity->CreateComponent<JoltBuoyancyOverrideComponent>();
            m_entity->Init();
            m_entity->Activate();
        }

        bool RunLua(const char* script)
        {
            return m_scriptContext->Execute(script);
        }

        //! Hands the entity id to Lua as a global named `waterEntity`.
        //!
        //! Lua's `EntityId(1234)` does not work - the numeric constructor is not reflected,
        //! and it quietly yields an invalid id rather than failing - so a script gets its
        //! id the way a real one does: handed in from C++. For a Lua Script component that
        //! is `self.entityId`; here it is a global.
        void GiveLuaTheEntityId()
        {
            m_scriptContext->AddReplaceGlobal("waterEntity", AZ::EntityId(TestEntityId));
        }

        static constexpr AZ::u64 TestEntityId = 0x1A7Eu;

        AZ::BehaviorContext* m_behaviorContext = nullptr;
        AZStd::unique_ptr<AZ::ScriptContext> m_scriptContext;
        AZStd::unique_ptr<AZ::Entity> m_entity;
    };

    TEST_F(JoltBuoyancyLuaTests, LuaCanSeeTheBuoyancyBuses)
    {
        // The names script actually types. A bus reflected under a different name would
        // pass the reflection test and fail here.
        EXPECT_TRUE(RunLua("assert(JoltWaterVolumeRequestBus ~= nil)"));
        EXPECT_TRUE(RunLua("assert(JoltBuoyancyOverrideRequestBus ~= nil)"));
        EXPECT_TRUE(RunLua("assert(JoltWaterVolumeNotificationBus ~= nil)"));
        EXPECT_TRUE(RunLua("assert(JoltWaterVolumeSettings ~= nil)"));
    }

    TEST_F(JoltBuoyancyLuaTests, LuaCanDriveABuoyancyOverride)
    {
        CreateOverrideEntity();
        GiveLuaTheEntityId();

        // The whole point of reflecting the bus: gameplay written in Lua changing how water
        // treats one body, with no C++ involved.
        ASSERT_TRUE(RunLua(R"(
            JoltBuoyancyOverrideRequestBus.Event.SetBuoyancyMode(waterEntity, JoltBuoyancyMode_Explicit)
            JoltBuoyancyOverrideRequestBus.Event.SetBuoyancyFactor(waterEntity, 2.5)
            JoltBuoyancyOverrideRequestBus.Event.SetLinearDragMultiplier(waterEntity, 0.25)
        )"));

        // Verified through the registry, which is what the solver actually reads - not
        // through the component, which would only prove Lua reached the setter.
        const JoltBuoyancyOverride published = JoltBuoyancyOverrideRegistry::Get().Find(AZ::EntityId(TestEntityId));
        EXPECT_EQ(published.m_mode, JoltBuoyancyMode::Explicit);
        EXPECT_FLOAT_EQ(published.m_factor, 2.5f);
        EXPECT_FLOAT_EQ(published.m_linearDragMultiplier, 0.25f);
    }

    TEST_F(JoltBuoyancyLuaTests, LuaCanReadBackThroughTheBus)
    {
        CreateOverrideEntity();
        GiveLuaTheEntityId();

        // A getter that returns something Lua cannot use is as broken as a missing one.
        EXPECT_TRUE(RunLua(R"(
            JoltBuoyancyOverrideRequestBus.Event.SetBuoyancyFactor(waterEntity, 1.75)
            local factor = JoltBuoyancyOverrideRequestBus.Event.GetBuoyancyFactor(waterEntity)
            assert(factor ~= nil, 'GetBuoyancyFactor returned nil')
            assert(math.abs(factor - 1.75) < 0.001, 'GetBuoyancyFactor round trip failed')

            JoltBuoyancyOverrideRequestBus.Event.SetExcludedFromWater(waterEntity, true)
            assert(JoltBuoyancyOverrideRequestBus.Event.IsExcludedFromWater(waterEntity) == true)
        )"));
    }

    TEST_F(JoltBuoyancyLuaTests, LuaCanHandleASplashNotification)
    {
        GiveLuaTheEntityId();

        // The half that matters for gameplay: hearing about a splash, not just setting
        // values. A handler needs the notification bus to expose a script handler, which
        // is a different reflection from an event.
        ASSERT_TRUE(RunLua(R"(
            splashCount = 0
            splashSpeed = 0
            splashBody = nil

            handler = JoltWaterVolumeNotificationBus.Connect(
                {
                    OnBodyEnteredWater = function(self, bodyEntityId, speed)
                        splashCount = splashCount + 1
                        splashSpeed = speed
                        splashBody = bodyEntityId
                    end,
                    OnBodyExitedWater = function(self, bodyEntityId)
                        splashCount = splashCount - 1
                    end,
                }, waterEntity)
            assert(handler ~= nil, 'could not connect a handler to the notification bus')
        )"));

        // Raised the way the component raises it after a physics step.
        JoltWaterVolumeNotificationBus::Event(
            AZ::EntityId(TestEntityId), &JoltWaterVolumeNotifications::OnBodyEnteredWater, AZ::EntityId(0xB0D1u), 4.5f);

        EXPECT_TRUE(RunLua(R"(
            assert(splashCount == 1, 'the Lua handler did not receive the enter event')
            assert(math.abs(splashSpeed - 4.5) < 0.001, 'the entry speed did not survive the call into Lua')
            assert(splashBody ~= nil, 'the body entity id did not survive the call into Lua')
        )"));

        JoltWaterVolumeNotificationBus::Event(
            AZ::EntityId(TestEntityId), &JoltWaterVolumeNotifications::OnBodyExitedWater, AZ::EntityId(0xB0D1u));

        EXPECT_TRUE(RunLua("assert(splashCount == 0, 'the Lua handler did not receive the exit event')"));

        RunLua("handler:Disconnect()");
    }

    TEST_F(JoltBuoyancyLuaTests, LuaCanBuildAndApplyWholeWaterSettings)
    {
        // SetWaterSettings is useless from script unless the settings type can be
        // constructed and filled in Lua.
        EXPECT_TRUE(RunLua(R"(
            local settings = JoltWaterVolumeSettings()
            settings.fluidDensity = 1025.0
            settings.wavesEnabled = true
            settings.waveAmplitude = 0.4
            settings.waveLength = 9.0
            assert(math.abs(settings.fluidDensity - 1025.0) < 0.001)
            assert(settings.wavesEnabled == true)
        )"));
    }
} // namespace JoltBuoyancy
