#include <Editor/EditorJoltWaterVolumeComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/SystemBus.h>

#include <AzToolsFramework/ComponentModes/BoxComponentMode.h>

#include <Clients/JoltBuoyancyDebugDraw.h>
#include <Clients/JoltWaterVolumeComponent.h>
#include <Clients/JoltWaterVolumeRender.h>

namespace JoltBuoyancy
{
    void EditorJoltWaterVolumeComponent::Reflect(AZ::ReflectContext* context)
    {
        // AzToolsFramework's delegate is reflected by whichever gem reaches it first, and
        // the physics gem's colliders already do. Registering it again is a duplicated-Uuid
        // error, which is easy to miss in the noise at editor startup.
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
            serializeContext &&
            serializeContext->FindClassData(
                azrtti_typeid<AzToolsFramework::ComponentModeFramework::ComponentModeDelegate>()) == nullptr)
        {
            AzToolsFramework::ComponentModeFramework::ComponentModeDelegate::Reflect(context);
        }

        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltWaterVolumeComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("Dimensions", &EditorJoltWaterVolumeComponent::m_dimensions)
                ->Field("Settings", &EditorJoltWaterVolumeComponent::m_settings)
                ->Field("Visible", &EditorJoltWaterVolumeComponent::m_visible)
                ->Field("ComponentMode", &EditorJoltWaterVolumeComponent::m_componentModeDelegate)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltWaterVolumeComponent>(
                    "Jolt Water Volume", "A box of water that makes Jolt rigid bodies float")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltWaterVolumeComponent::m_dimensions,
                        "Dimensions", "Size of the water box in entity space; its top (local +Z) face is the surface.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify,
                            &EditorJoltWaterVolumeComponent::OnConfigurationChanged)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltWaterVolumeComponent::m_settings,
                        "Water", "Fluid properties")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify,
                            &EditorJoltWaterVolumeComponent::OnConfigurationChanged)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltWaterVolumeComponent::m_componentModeDelegate,
                        "Component Mode", "Water volume component mode")
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &EditorJoltWaterVolumeComponent::m_visible,
                        "Visible", "Draw the water as a translucent box, in the viewport and in game mode. There is "
                        "no water mesh or material: this drawing is the volume itself, so it always matches what the "
                        "solver uses.")
                    ;
            }
        }
    }

    void EditorJoltWaterVolumeComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltWaterVolumeService"));
    }

    void EditorJoltWaterVolumeComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltWaterVolumeService"));
    }

    void EditorJoltWaterVolumeComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void EditorJoltWaterVolumeComponent::Activate()
    {
        AzToolsFramework::Components::EditorComponentBase::Activate();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusConnect(GetEntityId());
        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());

        // Attaches to the editor's own physics scene, so bodies with editor colliders float
        // in the Edit viewport. Returns false and does nothing when there is no editor
        // scene, which is the case in a tools build without the physics gem's editor world.
        AzPhysics::SceneHandle editorSceneHandle = AzPhysics::InvalidSceneHandle;
        Physics::EditorWorldBus::BroadcastResult(
            editorSceneHandle, &Physics::EditorWorldRequests::GetEditorSceneHandle);

        OnConfigurationChanged();
        if (editorSceneHandle != AzPhysics::InvalidSceneHandle)
        {
            m_previewVolume.Attach(editorSceneHandle);

            m_previewSimulationFinishHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
                [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, [[maybe_unused]] float fixedDeltaTime)
                {
                    m_previewVolume.WakePendingBodies();
                });
            if (auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get())
            {
                sceneInterface->RegisterSceneSimulationFinishHandler(editorSceneHandle, m_previewSimulationFinishHandler);
            }
        }

        // AzToolsFramework ships the box mode and its manipulators; all this component
        // supplies is the dimensions bus below.
        const AZ::EntityComponentIdPair entityComponentIdPair(GetEntityId(), GetId());
        AzToolsFramework::BoxManipulatorRequestBus::Handler::BusConnect(entityComponentIdPair);
        // No selection handler: unlike the physics gem's colliders this component supplies
        // no picking bounds of its own, so the entity is selected through its transform.
        m_componentModeDelegate.ConnectWithSingleComponentMode<
            EditorJoltWaterVolumeComponent, AzToolsFramework::BoxComponentMode>(entityComponentIdPair, nullptr);
    }

    AZ::Vector3 EditorJoltWaterVolumeComponent::GetDimensions() const
    {
        return m_dimensions;
    }

    void EditorJoltWaterVolumeComponent::SetDimensions(const AZ::Vector3& dimensions)
    {
        m_dimensions = dimensions;
        OnConfigurationChanged();
    }

    AZ::Transform EditorJoltWaterVolumeComponent::GetCurrentLocalTransform() const
    {
        return AZ::Transform::CreateIdentity();
    }

    void EditorJoltWaterVolumeComponent::Deactivate()
    {
        m_componentModeDelegate.Disconnect();
        AzToolsFramework::BoxManipulatorRequestBus::Handler::BusDisconnect();
        m_previewSimulationFinishHandler.Disconnect();
        m_previewVolume.Detach();

        AZ::TransformNotificationBus::Handler::BusDisconnect();
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
    }

    void EditorJoltWaterVolumeComponent::OnTransformChanged(
        const AZ::Transform& /*local*/, const AZ::Transform& world)
    {
        m_previewVolume.SetVolume(world, m_dimensions);
    }

    void EditorJoltWaterVolumeComponent::OnConfigurationChanged()
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        m_previewVolume.SetSettings(m_settings);
        m_previewVolume.SetVolume(worldTransform, m_dimensions);
    }

    void EditorJoltWaterVolumeComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltWaterVolumeComponent>())
        {
            component->AccessDimensions() = m_dimensions;
            component->AccessSettings() = m_settings;
            component->AccessVisible() = m_visible;
        }
    }

    void EditorJoltWaterVolumeComponent::DisplayEntityViewport(
        const AzFramework::ViewportInfo& /*viewportInfo*/, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        // Drained even with the box hidden, and before the early-out: the diagnostic is
        // asked for by console variable rather than by this component's Visible setting,
        // and it describes the bodies rather than the volume.
        JoltBuoyancyDebugDraw::Get().Flush(debugDisplay);

        if (!m_visible)
        {
            return;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        DrawWaterVolume(debugDisplay, worldTransform, m_dimensions, &m_settings, &m_previewVolume);
    }

} // namespace JoltBuoyancy
