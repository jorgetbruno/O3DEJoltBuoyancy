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
                ->Field("LinearDragMultiplier", &JoltBuoyancyOverrideComponent::m_linearDragMultiplier)
                ->Field("AngularDragMultiplier", &JoltBuoyancyOverrideComponent::m_angularDragMultiplier)
                ->Field("DirectionalDrag", &JoltBuoyancyOverrideComponent::m_directionalDrag)
                ->Field("AddedMass", &JoltBuoyancyOverrideComponent::m_addedMass)
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
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltBuoyancyOverrideComponent::m_linearDragMultiplier,
                        "Linear drag scale", "Scales the water's drag on this body. Below 1 is streamlined, above 1 "
                        "drags more than its size suggests.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltBuoyancyOverrideComponent::m_angularDragMultiplier,
                        "Angular drag scale", "Scales how strongly the water damps this body's spin.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltBuoyancyOverrideComponent::m_directionalDrag,
                        "Directional drag", "Per body-axis drag scale. Jolt already varies drag with the projected "
                        "area of the bounding box; lower the forward axis to make a hull streamlined along its "
                        "length, which is what lets a boat hold a heading.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltBuoyancyOverrideComponent::m_addedMass,
                        "Added mass", "Water dragged along with the body, as a fraction of the mass it displaces. "
                        "Around 0.5 for a blunt hull. An approximation: Jolt does not expose the mass matrix, so "
                        "this resists velocity changes after the fact.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
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
        override.m_linearDragMultiplier = m_linearDragMultiplier;
        override.m_angularDragMultiplier = m_angularDragMultiplier;
        override.m_directionalDrag = m_directionalDrag;
        override.m_addedMass = m_addedMass;
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

    void JoltBuoyancyOverrideComponent::SetLinearDragMultiplier(float multiplier)
    {
        m_linearDragMultiplier = multiplier;
        Publish();
    }

    float JoltBuoyancyOverrideComponent::GetLinearDragMultiplier() const
    {
        return m_linearDragMultiplier;
    }

    void JoltBuoyancyOverrideComponent::SetAngularDragMultiplier(float multiplier)
    {
        m_angularDragMultiplier = multiplier;
        Publish();
    }

    float JoltBuoyancyOverrideComponent::GetAngularDragMultiplier() const
    {
        return m_angularDragMultiplier;
    }

    void JoltBuoyancyOverrideComponent::SetDirectionalDrag(const AZ::Vector3& perAxisScale)
    {
        m_directionalDrag = perAxisScale;
        Publish();
    }

    AZ::Vector3 JoltBuoyancyOverrideComponent::GetDirectionalDrag() const
    {
        return m_directionalDrag;
    }

    void JoltBuoyancyOverrideComponent::SetAddedMass(float coefficient)
    {
        m_addedMass = coefficient;
        Publish();
    }

    float JoltBuoyancyOverrideComponent::GetAddedMass() const
    {
        return m_addedMass;
    }
} // namespace JoltBuoyancy
