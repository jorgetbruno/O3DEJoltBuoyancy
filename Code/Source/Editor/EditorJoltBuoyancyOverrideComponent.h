#pragma once

#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <JoltBuoyancy/JoltBuoyancyBus.h>

namespace JoltBuoyancy
{
    //! Editor Buoyancy Override: authors the per-body water settings and spawns the runtime
    //! JoltBuoyancyOverrideComponent via BuildGameEntity.
    //!
    //! Without this the runtime component could only be put on an entity wrapped in a
    //! GenericComponentWrapper, which is awkward to author and impossible to write into a
    //! generated prefab by hand.
    class EditorJoltBuoyancyOverrideComponent
        : public AzToolsFramework::Components::EditorComponentBase
    {
    public:
        AZ_EDITOR_COMPONENT(EditorJoltBuoyancyOverrideComponent, "{D8E9F0A1-B2C3-44D5-E6F7-A8B9C0D1E2F3}",
            AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        // EditorComponentBase
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        JoltBuoyancyMode m_mode = JoltBuoyancyMode::Automatic;
        float m_factor = 1.2f;
        bool m_excluded = false;
    };
} // namespace JoltBuoyancy
