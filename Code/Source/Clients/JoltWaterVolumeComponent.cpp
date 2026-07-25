#include <Clients/JoltWaterVolumeComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzFramework/Physics/SystemBus.h>

namespace JoltBuoyancy
{
    void JoltWaterVolumeComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<JoltWaterVolumeSettings>()
                ->Version(1)
                ->Field("FluidDensity", &JoltWaterVolumeSettings::m_fluidDensity)
                ->Field("LinearDrag", &JoltWaterVolumeSettings::m_linearDrag)
                ->Field("AngularDrag", &JoltWaterVolumeSettings::m_angularDrag)
                ->Field("FluidVelocity", &JoltWaterVolumeSettings::m_fluidVelocity)
                ;

            serializeContext->Class<JoltWaterVolumeComponent, AZ::Component>()
                ->Version(1)
                ->Field("Dimensions", &JoltWaterVolumeComponent::m_dimensions)
                ->Field("Settings", &JoltWaterVolumeComponent::m_settings)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<JoltWaterVolumeSettings>("Water Settings", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_fluidDensity,
                        "Fluid density", "Density of the fluid. A body floats when it is less dense than this and "
                        "sinks when it is denser. Fresh water is 1000.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " kg/m^3")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_linearDrag,
                        "Linear drag", "How strongly the water slows a body down. Higher feels thicker.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_angularDrag,
                        "Angular drag", "How strongly the water damps a body's spin.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeSettings::m_fluidVelocity,
                        "Fluid velocity", "Velocity of the water itself: a current that carries bodies along.")
                    ;

                editContext->Class<JoltWaterVolumeComponent>(
                    "Jolt Water Volume", "A box of water that makes Jolt rigid bodies float")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::Category, "Jolt Physics")
                        ->Attribute(AZ::Edit::Attributes::RemoveableByUser, true)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeComponent::m_dimensions,
                        "Dimensions", "Size of the water box in entity space; its top (local +Z) face is the surface.")
                        ->Attribute(AZ::Edit::Attributes::Min, 0.001f)
                        ->Attribute(AZ::Edit::Attributes::Suffix, " m")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &JoltWaterVolumeComponent::m_settings,
                        "Water", "Fluid properties")
                    ;
            }
        }
    }

    void JoltWaterVolumeComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JoltWaterVolumeService"));
    }

    void JoltWaterVolumeComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JoltWaterVolumeService"));
    }

    void JoltWaterVolumeComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    void JoltWaterVolumeComponent::Activate()
    {
        Physics::DefaultWorldBus::BroadcastResult(
            m_attachedSceneHandle, &Physics::DefaultWorldRequests::GetDefaultSceneHandle);

        m_waterVolume.SetSettings(m_settings);
        RefreshVolume();
        m_waterVolume.Attach(m_attachedSceneHandle);

        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        JoltWaterVolumeRequestBus::Handler::BusConnect(GetEntityId());
    }

    void JoltWaterVolumeComponent::Deactivate()
    {
        JoltWaterVolumeRequestBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();

        m_waterVolume.Detach();
        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltWaterVolumeComponent::RefreshVolume()
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        m_waterVolume.SetVolume(worldTransform, m_dimensions);
    }

    void JoltWaterVolumeComponent::OnTransformChanged(const AZ::Transform& /*local*/, const AZ::Transform& world)
    {
        m_waterVolume.SetVolume(world, m_dimensions);
    }

    void JoltWaterVolumeComponent::SetFluidDensity(float density)
    {
        m_settings.m_fluidDensity = density;
        m_waterVolume.SetSettings(m_settings);
    }

    float JoltWaterVolumeComponent::GetFluidDensity() const
    {
        return m_settings.m_fluidDensity;
    }

    void JoltWaterVolumeComponent::SetLinearDrag(float drag)
    {
        m_settings.m_linearDrag = drag;
        m_waterVolume.SetSettings(m_settings);
    }

    float JoltWaterVolumeComponent::GetLinearDrag() const
    {
        return m_settings.m_linearDrag;
    }

    void JoltWaterVolumeComponent::SetAngularDrag(float drag)
    {
        m_settings.m_angularDrag = drag;
        m_waterVolume.SetSettings(m_settings);
    }

    float JoltWaterVolumeComponent::GetAngularDrag() const
    {
        return m_settings.m_angularDrag;
    }

    void JoltWaterVolumeComponent::SetFluidVelocity(const AZ::Vector3& velocity)
    {
        m_settings.m_fluidVelocity = velocity;
        m_waterVolume.SetSettings(m_settings);
    }

    AZ::Vector3 JoltWaterVolumeComponent::GetFluidVelocity() const
    {
        return m_settings.m_fluidVelocity;
    }

    void JoltWaterVolumeComponent::SetEnabled(bool enabled)
    {
        m_waterVolume.SetEnabled(enabled);
    }

    bool JoltWaterVolumeComponent::IsEnabled() const
    {
        return m_waterVolume.IsEnabled();
    }

    int JoltWaterVolumeComponent::GetSubmergedBodyCount() const
    {
        return m_waterVolume.GetSubmergedBodyCount();
    }

} // namespace JoltBuoyancy
