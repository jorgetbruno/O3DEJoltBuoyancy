#include <Editor/EditorJoltBuoyancyOverrideComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/JoltBuoyancyOverrideComponent.h>

namespace JoltBuoyancy
{
    void EditorJoltBuoyancyOverrideComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<EditorJoltBuoyancyOverrideComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(1)
                ->Field("Mode", &EditorJoltBuoyancyOverrideComponent::m_mode)
                ->Field("Factor", &EditorJoltBuoyancyOverrideComponent::m_factor)
                ->Field("Excluded", &EditorJoltBuoyancyOverrideComponent::m_excluded)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<EditorJoltBuoyancyOverrideComponent>(
                    "Jolt Buoyancy Override", "Controls how water treats this body")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &EditorJoltBuoyancyOverrideComponent::m_excluded,
                        "Excluded from water", "Water ignores this body completely: no impulse, and no enter or exit "
                        "notifications.")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &EditorJoltBuoyancyOverrideComponent::m_mode,
                        "Buoyancy", "Automatic derives the factor from the body's density. Explicit takes the value "
                        "below, which is how a hollow hull floats even though its collider volume says it should sink.")
                        ->EnumAttribute(JoltBuoyancyMode::Automatic, "Automatic (from density)")
                        ->EnumAttribute(JoltBuoyancyMode::Explicit, "Explicit")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &EditorJoltBuoyancyOverrideComponent::m_factor,
                        "Buoyancy factor", "1 is neutral and floats half out of the water, above 1 rides higher, "
                        "below 1 sinks.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &EditorJoltBuoyancyOverrideComponent::m_mode)
                    ;
            }
        }
    }

    void EditorJoltBuoyancyOverrideComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltBuoyancyOverrideService"));
    }

    void EditorJoltBuoyancyOverrideComponent::GetIncompatibleServices(
        AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltBuoyancyOverrideService"));
    }

    void EditorJoltBuoyancyOverrideComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        if (auto* component = gameEntity->CreateComponent<JoltBuoyancyOverrideComponent>())
        {
            component->AccessMode() = m_mode;
            component->AccessFactor() = m_factor;
            component->AccessExcluded() = m_excluded;
        }
    }
} // namespace JoltBuoyancy
