#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/RTTI/BehaviorContext.h>

namespace JoltBuoyancy
{
    // The gem's buses are only reachable from Lua and Script Canvas if they are reflected
    // to the behavior context. Nothing else in the gem fails when a reflection is dropped -
    // every C++ caller keeps working - so it is pinned here, the same way the JoltPhysics
    // gem pins its own gameplay buses.
    //
    // The context comes from the test application, which registers the gem's component
    // descriptors exactly as the runtime module does, so this exercises the real path.
    class JoltBuoyancyScriptReflectionTests : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            AZ::ComponentApplicationBus::BroadcastResult(
                m_behaviorContext, &AZ::ComponentApplicationRequests::GetBehaviorContext);
            ASSERT_NE(m_behaviorContext, nullptr) << "No application behavior context";
        }

        const AZ::BehaviorEBus* FindBus(const char* name) const
        {
            const auto it = m_behaviorContext->m_ebuses.find(name);
            return it != m_behaviorContext->m_ebuses.end() ? it->second : nullptr;
        }

        void ExpectBusHasEvents(const char* busName, const AZStd::vector<const char*>& eventNames) const
        {
            const AZ::BehaviorEBus* bus = FindBus(busName);
            ASSERT_NE(bus, nullptr) << busName << " is not reflected to script";
            for (const char* eventName : eventNames)
            {
                EXPECT_NE(bus->m_events.find(eventName), bus->m_events.end())
                    << busName << " is missing the event " << eventName;
            }
        }

        AZ::BehaviorContext* m_behaviorContext = nullptr;
    };

    TEST_F(JoltBuoyancyScriptReflectionTests, WaterVolumeBusIsReflectedWithItsFluidAndWaveEvents)
    {
        ExpectBusHasEvents("JoltWaterVolumeRequestBus",
            { "SetFluidDensity", "GetFluidDensity", "SetLinearDrag", "GetLinearDrag", "SetAngularDrag",
              "GetAngularDrag", "SetFluidVelocity", "GetFluidVelocity", "SetDimensions", "GetDimensions",
              "SetWaterSettings", "GetWaterSettings", "SetWavesEnabled", "GetWavesEnabled",
              "SetSpectrum", "GetSpectrum", "SetSeaState", "GetSeaState", "SetWindDirection", "GetWindDirection",
              "GetSignificantWaveHeight", "GetWaterVelocityAt",
              "SetEnabled", "IsEnabled", "GetSubmergedBodyCount", "GetSubmergedFraction" });
    }

    TEST_F(JoltBuoyancyScriptReflectionTests, BuoyancyOverrideBusIsReflected)
    {
        ExpectBusHasEvents("JoltBuoyancyOverrideRequestBus",
            { "SetExcludedFromWater", "IsExcludedFromWater", "SetBuoyancyFactor", "GetBuoyancyFactor",
              "SetBuoyancyMode", "GetBuoyancyMode", "SetDirectionalDrag", "GetDirectionalDrag",
              "SetAddedMass", "GetAddedMass" });
    }

    TEST_F(JoltBuoyancyScriptReflectionTests, WaterVolumeNotificationBusCanBeHandledFromScript)
    {
        // Requests without notifications would let script change the water but never hear
        // about a splash, which is the half that gameplay actually wants.
        const AZ::BehaviorEBus* bus = FindBus("JoltWaterVolumeNotificationBus");
        ASSERT_NE(bus, nullptr) << "JoltWaterVolumeNotificationBus is not reflected to script";
        EXPECT_NE(bus->m_createHandler, nullptr) << "the notification bus has no script handler";
    }

    TEST_F(JoltBuoyancyScriptReflectionTests, WaterSettingsAreReflectedAsAScriptType)
    {
        // SetWaterSettings is useless from script unless the settings type itself is
        // reflected with readable properties.
        const auto found = m_behaviorContext->m_classes.find("JoltWaterVolumeSettings");
        ASSERT_NE(found, m_behaviorContext->m_classes.end()) << "JoltWaterVolumeSettings is not reflected to script";

        const AZ::BehaviorClass* settingsClass = found->second;
        for (const char* propertyName : { "fluidDensity", "linearDrag", "angularDrag", "fluidVelocity",
                                          "wavesEnabled", "spectrum", "surfaceSamplesPerBody" })
        {
            EXPECT_NE(settingsClass->m_properties.find(propertyName), settingsClass->m_properties.end())
                << "JoltWaterVolumeSettings is missing the property " << propertyName;
        }
    }
} // namespace JoltBuoyancy
