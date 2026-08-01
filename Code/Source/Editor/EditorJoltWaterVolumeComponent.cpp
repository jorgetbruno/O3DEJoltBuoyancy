#include <Editor/EditorJoltWaterVolumeComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/JoltWaterVolumeComponent.h>
#include <Clients/JoltWaterVolumeRender.h>

namespace JoltBuoyancy
{
    void EditorJoltWaterVolumeComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltWaterVolumeComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(2)
                ->Field("Dimensions", &EditorJoltWaterVolumeComponent::m_dimensions)
                ->Field("Settings", &EditorJoltWaterVolumeComponent::m_settings)
                ->Field("Visible", &EditorJoltWaterVolumeComponent::m_visible)
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
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltWaterVolumeComponent::m_settings,
                        "Water", "Fluid properties")
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
    }

    void EditorJoltWaterVolumeComponent::Deactivate()
    {
        AzFramework::EntityDebugDisplayEventBus::Handler::BusDisconnect();
        AzToolsFramework::Components::EditorComponentBase::Deactivate();
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
        if (!m_visible)
        {
            return;
        }

        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        DrawWaterVolume(debugDisplay, worldTransform, m_dimensions);
    }

} // namespace JoltBuoyancy
