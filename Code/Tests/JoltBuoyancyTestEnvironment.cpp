#include <AzTest/GemTestEnvironment.h>
#include <AzFramework/Application/Application.h>

#include <Clients/JoltBuoyancyOverrideComponent.h>
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
            // Registering the descriptors is what runs Reflect, which is what the script
            // reflection test checks. Spelled out as a vector because a braced list of two
            // deduces the wrong span type.
            AZStd::vector<AZ::ComponentDescriptor*> descriptors;
            descriptors.push_back(JoltWaterVolumeComponent::CreateDescriptor());
            descriptors.push_back(JoltBuoyancyOverrideComponent::CreateDescriptor());
            AddComponentDescriptors(descriptors);
        }
    };
} // namespace JoltBuoyancy

AZ_UNIT_TEST_HOOK(new JoltBuoyancy::JoltBuoyancyTestEnvironment);
