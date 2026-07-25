#include <Editor/EditorJoltWaterVolumeComponent.h>

#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/JoltWaterVolumeComponent.h>

namespace JoltBuoyancy
{
    namespace
    {
        // The 12 edges of a box, as pairs of corner indices.
        constexpr AZ::u32 BoxEdges[][2] = {
            { 0, 1 }, { 1, 3 }, { 3, 2 }, { 2, 0 },
            { 4, 5 }, { 5, 7 }, { 7, 6 }, { 6, 4 },
            { 0, 4 }, { 1, 5 }, { 2, 6 }, { 3, 7 },
        };

        AZ::Vector3 BoxCorner(const AZ::Vector3& halfExtents, AZ::u32 corner)
        {
            return AZ::Vector3(
                (corner & 1) ? halfExtents.GetX() : -halfExtents.GetX(),
                (corner & 2) ? halfExtents.GetY() : -halfExtents.GetY(),
                (corner & 4) ? halfExtents.GetZ() : -halfExtents.GetZ());
        }
    }

    void EditorJoltWaterVolumeComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltWaterVolumeComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("Dimensions", &EditorJoltWaterVolumeComponent::m_dimensions)
                ->Field("Settings", &EditorJoltWaterVolumeComponent::m_settings)
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
            component->GetDimensions() = m_dimensions;
            component->GetSettings() = m_settings;
        }
    }

    void EditorJoltWaterVolumeComponent::DisplayEntityViewport(
        const AzFramework::ViewportInfo& /*viewportInfo*/, AzFramework::DebugDisplayRequests& debugDisplay)
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);

        const AZ::Vector3 halfExtents = m_dimensions * 0.5f;
        const AZ::Vector4 volumeColor(0.2f, 0.5f, 1.0f, 1.0f);
        for (const auto& edge : BoxEdges)
        {
            debugDisplay.DrawLine(
                worldTransform.TransformPoint(BoxCorner(halfExtents, edge[0])),
                worldTransform.TransformPoint(BoxCorner(halfExtents, edge[1])), volumeColor, volumeColor);
        }

        // Mark the surface (the top face) so it is obvious which way up the volume is.
        const AZ::Vector4 surfaceColor(0.6f, 0.9f, 1.0f, 1.0f);
        for (const AZ::u32 topEdge : { 4u, 5u, 6u, 7u })
        {
            debugDisplay.DrawLine(
                worldTransform.TransformPoint(BoxCorner(halfExtents, BoxEdges[topEdge][0])),
                worldTransform.TransformPoint(BoxCorner(halfExtents, BoxEdges[topEdge][1])), surfaceColor, surfaceColor);
        }
    }

} // namespace JoltBuoyancy
