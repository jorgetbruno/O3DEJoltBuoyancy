#pragma once

#include <AzCore/Component/TransformBus.h>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/Common/PhysicsEvents.h>

#include <AzToolsFramework/ComponentMode/ComponentModeDelegate.h>
#include <AzToolsFramework/Manipulators/BoxManipulatorRequestBus.h>
#include <AzToolsFramework/ToolsComponents/EditorComponentBase.h>

#include <Clients/JoltWaterVolume.h>

namespace JoltBuoyancy
{
    //! Editor Water Volume: draws the water box in the Edit viewport and spawns the
    //! runtime JoltWaterVolumeComponent via BuildGameEntity, mirroring the JoltPhysics
    //! gem's editor/runtime component split.
    //!
    //! It also runs a real water volume in the editor's own physics scene, so bodies with
    //! editor colliders float in the Edit viewport without entering game mode. That scene
    //! only exists because the physics gem added it; before that this could only be a
    //! drawing.
    class EditorJoltWaterVolumeComponent
        : public AzToolsFramework::Components::EditorComponentBase
        , private AzFramework::EntityDebugDisplayEventBus::Handler
        , private AZ::TransformNotificationBus::Handler
        , private AzToolsFramework::BoxManipulatorRequestBus::Handler
    {
    public:
        AZ_EDITOR_COMPONENT(EditorJoltWaterVolumeComponent, "{B2C3D4E5-F6A7-4819-AB2C-3D4E5F6A7B8C}",
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

        // AZ::TransformNotificationBus
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // AzToolsFramework::BoxManipulatorRequestBus - drives AzToolsFramework's own
        // BoxComponentMode, so the water box is dragged out by its faces with the same
        // handles as an engine box shape rather than only typed into the inspector.
        AZ::Vector3 GetDimensions() const override;
        void SetDimensions(const AZ::Vector3& dimensions) override;
        AZ::Transform GetCurrentLocalTransform() const override;

        //! Pushes the authored values into the preview volume, and is the change handler
        //! for every property in the inspector.
        void OnConfigurationChanged();

        AZ::Vector3 m_dimensions = AZ::Vector3(10.0f, 10.0f, 5.0f);
        JoltWaterVolumeSettings m_settings;
        bool m_visible = true;

        //! Lets the entity be dragged out by its faces in the viewport, the way the
        //! physics gem's box collider is edited, instead of only by typing numbers.
        AzToolsFramework::ComponentModeFramework::ComponentModeDelegate m_componentModeDelegate;

        //! The volume running in the editor's physics scene.
        JoltWaterVolume m_previewVolume;
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_previewSimulationFinishHandler;
    };
} // namespace JoltBuoyancy
