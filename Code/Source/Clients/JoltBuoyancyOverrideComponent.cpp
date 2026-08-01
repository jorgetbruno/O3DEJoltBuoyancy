#include <Clients/JoltBuoyancyOverrideComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <Clients/JoltBuoyancyOverrideRegistry.h>

namespace JoltBuoyancy
{
    void JoltBuoyancyOverrideComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltBuoyancyOverrideComponent, AZ::Component>()
                ->Version(1)
                ->Field("Mode", &JoltBuoyancyOverrideComponent::m_mode)
                ->Field("Factor", &JoltBuoyancyOverrideComponent::m_factor)
                ->Field("Excluded", &JoltBuoyancyOverrideComponent::m_excluded)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltBuoyancyOverrideComponent>(
                    "Jolt Buoyancy Override", "Controls how water treats this body")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltBuoyancyOverrideComponent::m_excluded,
                        "Excluded from water", "Water ignores this body completely: no impulse, and no enter or exit "
                        "notifications.")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &JoltBuoyancyOverrideComponent::m_mode,
                        "Buoyancy", "Automatic derives the factor from the body's density. Explicit takes the value "
                        "below, which is how a hollow hull floats even though its collider volume says it should sink.")
                        ->EnumAttribute(JoltBuoyancyMode::Automatic, "Automatic (from density)")
                        ->EnumAttribute(JoltBuoyancyMode::Explicit, "Explicit")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltBuoyancyOverrideComponent::m_factor,
                        "Buoyancy factor", "1 is neutral and floats half out of the water, above 1 rides higher, "
                        "below 1 sinks.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                        ->Attribute(AZ::Edit::Attributes::Visibility, &JoltBuoyancyOverrideComponent::m_mode)
                    ;
            }
        }
    }

    void JoltBuoyancyOverrideComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltBuoyancyOverrideService"));
    }

    void JoltBuoyancyOverrideComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltBuoyancyOverrideService"));
    }

    void JoltBuoyancyOverrideComponent::Activate()
    {
        Publish();
        JoltBuoyancyOverrideRequestBus::Handler::BusConnect(GetEntityId());
    }

    void JoltBuoyancyOverrideComponent::Deactivate()
    {
        JoltBuoyancyOverrideRequestBus::Handler::BusDisconnect();
        JoltBuoyancyOverrideRegistry::Get().Remove(GetEntityId());
    }

    void JoltBuoyancyOverrideComponent::Publish()
    {
        JoltBuoyancyOverride override;
        override.m_mode = m_mode;
        override.m_factor = m_factor;
        override.m_excluded = m_excluded;
        JoltBuoyancyOverrideRegistry::Get().Set(GetEntityId(), override);
    }

    void JoltBuoyancyOverrideComponent::SetExcludedFromWater(bool excluded)
    {
        m_excluded = excluded;
        Publish();
    }

    bool JoltBuoyancyOverrideComponent::IsExcludedFromWater() const
    {
        return m_excluded;
    }

    void JoltBuoyancyOverrideComponent::SetBuoyancyMode(JoltBuoyancyMode mode)
    {
        m_mode = mode;
        Publish();
    }

    JoltBuoyancyMode JoltBuoyancyOverrideComponent::GetBuoyancyMode() const
    {
        return m_mode;
    }

    void JoltBuoyancyOverrideComponent::SetBuoyancyFactor(float factor)
    {
        m_factor = factor;
        Publish();
    }

    float JoltBuoyancyOverrideComponent::GetBuoyancyFactor() const
    {
        return m_factor;
    }
} // namespace JoltBuoyancy
