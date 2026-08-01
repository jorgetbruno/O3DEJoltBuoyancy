#include <Clients/JoltWaterVolumeComponent.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

#include <AzCore/Interface/Interface.h>

#include <AzFramework/Entity/EntityDebugDisplayBus.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Physics/SystemBus.h>

#include <Clients/JoltWaterVolumeRender.h>

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
                ->Version(2)
                ->Field("Dimensions", &JoltWaterVolumeComponent::m_dimensions)
                ->Field("Settings", &JoltWaterVolumeComponent::m_settings)
                ->Field("Visible", &JoltWaterVolumeComponent::m_visible)
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
                    ->DataElement(AZ::Edit::UIHandlers::CheckBox, &JoltWaterVolumeComponent::m_visible,
                        "Visible", "Draw the water as a translucent box. There is no water mesh or material: this "
                        "drawing is the volume itself, so it always matches what the solver uses.")
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

        // Bodies the step found asleep inside changed water are woken here rather than in
        // OnStep, where every body mutex is held and waking would deadlock.
        m_simulationFinishHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
            [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, [[maybe_unused]] float fixedDeltaTime)
            {
                m_waterVolume.WakePendingBodies();
            });
        if (auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            sceneInterface && m_attachedSceneHandle != AzPhysics::InvalidSceneHandle)
        {
            sceneInterface->RegisterSceneSimulationFinishHandler(m_attachedSceneHandle, m_simulationFinishHandler);
        }

        AZ::TransformNotificationBus::Handler::BusConnect(GetEntityId());
        JoltWaterVolumeRequestBus::Handler::BusConnect(GetEntityId());

        // Connected regardless of m_visible so the volume can be shown and hidden at
        // runtime; OnTick returns immediately when it is off.
        AZ::TickBus::Handler::BusConnect();
    }

    void JoltWaterVolumeComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        JoltWaterVolumeRequestBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();

        m_simulationFinishHandler.Disconnect();
        m_waterVolume.Detach();
        m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    }

    void JoltWaterVolumeComponent::RefreshVolume()
    {
        AZ::Transform worldTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(worldTransform, GetEntityId(), &AZ::TransformBus::Events::GetWorldTM);
        m_worldTransform = worldTransform;
        m_waterVolume.SetVolume(worldTransform, m_dimensions);
    }

    void JoltWaterVolumeComponent::OnTransformChanged(const AZ::Transform& /*local*/, const AZ::Transform& world)
    {
        m_worldTransform = world;
        m_waterVolume.SetVolume(world, m_dimensions);
    }

    void JoltWaterVolumeComponent::OnTick(float /*deltaTime*/, AZ::ScriptTimePoint /*time*/)
    {
        if (!m_visible)
        {
            return;
        }

        // Bound per frame rather than cached: the viewport's debug display handler is
        // not guaranteed to exist when the component activates, and this is a hash
        // lookup against a frame of rendering.
        AzFramework::DebugDisplayRequestBus::BusPtr debugDisplayBus;
        AzFramework::DebugDisplayRequestBus::Bind(debugDisplayBus, AzFramework::g_defaultSceneEntityDebugDisplayId);
        if (auto* debugDisplay = AzFramework::DebugDisplayRequestBus::FindFirstHandler(debugDisplayBus))
        {
            DrawWaterVolume(*debugDisplay, m_worldTransform, m_dimensions);
        }
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
