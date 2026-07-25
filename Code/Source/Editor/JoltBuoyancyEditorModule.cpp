#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/Module/Module.h>

#include <Clients/JoltWaterVolumeComponent.h>
#include <Editor/EditorJoltWaterVolumeComponent.h>

namespace JoltBuoyancy
{
    class JoltBuoyancyEditorModule : public AZ::Module
    {
    public:
        AZ_RTTI(JoltBuoyancyEditorModule, "{D4E5F6A7-B8C9-4A3B-CD4E-5F6A7B8C9D0E}", AZ::Module);
        AZ_CLASS_ALLOCATOR(JoltBuoyancyEditorModule, AZ::SystemAllocator);

        JoltBuoyancyEditorModule()
        {
            m_descriptors.insert(
                m_descriptors.end(),
                {
                    // The runtime component stays registered so BuildGameEntity can spawn
                    // it and prefabs referencing it keep loading.
                    JoltWaterVolumeComponent::CreateDescriptor(),
                    EditorJoltWaterVolumeComponent::CreateDescriptor(),
                });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList();
        }
    };
} // namespace JoltBuoyancy

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, AZ_JOIN(O3DE_GEM_NAME, _Editor)), JoltBuoyancy::JoltBuoyancyEditorModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_JoltBuoyancy_Editor, JoltBuoyancy::JoltBuoyancyEditorModule)
#endif
