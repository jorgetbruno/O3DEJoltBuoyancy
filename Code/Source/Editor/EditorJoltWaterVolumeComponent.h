#pragma once

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <Clients/JoltWaterVolume.h>

namespace JoltBuoyancy
{
    //! Editor Water Volume: draws the water box in the Edit viewport and spawns the
    //! runtime JoltWaterVolumeComponent via BuildGameEntity, mirroring the JoltPhysics
    //! gem's editor/runtime component split.
    class EditorJoltWaterVolumeComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
    {
    public:
        AZ_COMPONENT(EditorJoltWaterVolumeComponent, "{B2C3D4E5-F6A7-4819-AB2C-3D4E5F6A7B8C}",
            AzToolsFramework::Components::EditorComponentBase);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // EditorComponentBase
        void Activate() override;
        void Deactivate() override;
        void BuildGameEntity(AZ::Entity* gameEntity) override;

    private:
        // AzFramework::EntityDebugDisplayEvents
        void DisplayEntityViewport(
            const AzFramework::ViewportInfo& viewportInfo, AzFramework::DebugDisplayRequests& debugDisplay) override;

        AZ::Vector3 m_dimensions = AZ::Vector3(10.0f, 10.0f, 5.0f);
        JoltWaterVolumeSettings m_settings;
        bool m_visible = true;
    };
} // namespace JoltBuoyancy
