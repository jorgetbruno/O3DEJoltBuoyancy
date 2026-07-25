#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include <Clients/JoltWaterVolumeComponent.h>

namespace JoltBuoyancy
{
    class JoltBuoyancyModule : public AZ::Module
    {
    public:
        AZ_RTTI(JoltBuoyancyModule, "{C3D4E5F6-A7B8-492A-BC3D-4E5F6A7B8C9D}", AZ::Module);
        AZ_CLASS_ALLOCATOR(JoltBuoyancyModule, AZ::SystemAllocator);

        JoltBuoyancyModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    JoltWaterVolumeComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            // The gem is entirely component driven: a water volume attaches itself to the
            // physics scene, so there is nothing to keep running system-wide.
            return AZ::ComponentTypeList();
        }
    };
} // namespace JoltBuoyancy

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), JoltBuoyancy::JoltBuoyancyModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_JoltBuoyancy, JoltBuoyancy::JoltBuoyancyModule)
#endif
