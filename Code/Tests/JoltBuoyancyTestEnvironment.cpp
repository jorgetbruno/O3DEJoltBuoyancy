#include <AzTest/GemTestEnvironment.h>
#include <AzFramework/Application/Application.h>

#include <Clients/JoltWaterVolumeComponent.h>

namespace JoltBuoyancy
{
    class JoltBuoyancyTestEnvironment : public AZ::Test::GemTestEnvironment
    {
    protected:
        AZ::ComponentApplication* CreateApplicationInstance() override
        {
            return aznew AzFramework::Application();
        }

        void AddGemsAndComponents() override
        {
            AddComponentDescriptors({ JoltWaterVolumeComponent::CreateDescriptor() });
        }
    };
} // namespace JoltBuoyancy

AZ_UNIT_TEST_HOOK(new JoltBuoyancy::JoltBuoyancyTestEnvironment);
