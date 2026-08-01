#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>

#include <AzFramework/Physics/Common/PhysicsEvents.h>
#include <AzFramework/Physics/Common/PhysicsTypes.h>

#include <Clients/JoltWaterVolume.h>
#include <JoltBuoyancy/JoltBuoyancyBus.h>

namespace JoltBuoyancy
{
    //! Turns the entity into a box of water: Jolt rigid bodies overlapping it float,
    //! sink or drift according to their own density and the fluid settings.
    //! The volume follows the entity's transform, and its local +Z face is the surface.
    class JoltWaterVolumeComponent
        : public AZ::Component
        , private AZ::TransformNotificationBus::Handler
        , private AZ::TickBus::Handler
        , private JoltWaterVolumeRequestBus::Handler
    {
    public:
        AZ_COMPONENT(JoltWaterVolumeComponent, "{A1B2C3D4-E5F6-4708-9A1B-2C3D4E5F6A7B}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        //! Direct access for the editor component's BuildGameEntity, which fills these in
        //! before the component is activated. Named apart from the bus getters, which are
        //! const and answer from the live volume.
        AZ::Vector3& AccessDimensions()
        {
            return m_dimensions;
        }
        JoltWaterVolumeSettings& AccessSettings()
        {
            return m_settings;
        }
        bool& AccessVisible()
        {
            return m_visible;
        }

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AZ::TransformNotificationBus
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // AZ::TickBus
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

        // JoltWaterVolumeRequestBus
        void SetFluidDensity(float density) override;
        float GetFluidDensity() const override;
        void SetLinearDrag(float drag) override;
        float GetLinearDrag() const override;
        void SetAngularDrag(float drag) override;
        float GetAngularDrag() const override;
        void SetFluidVelocity(const AZ::Vector3& velocity) override;
        AZ::Vector3 GetFluidVelocity() const override;
        void SetDimensions(const AZ::Vector3& dimensions) override;
        AZ::Vector3 GetDimensions() const override;
        void SetWaterSettings(const JoltWaterVolumeSettings& settings) override;
        JoltWaterVolumeSettings GetWaterSettings() const override;
        void SetWavesEnabled(bool enabled) override;
        bool GetWavesEnabled() const override;
        void SetWaveAmplitude(float amplitude) override;
        float GetWaveAmplitude() const override;
        void SetEnabled(bool enabled) override;
        bool IsEnabled() const override;
        int GetSubmergedBodyCount() const override;
        float GetSubmergedFraction(AZ::EntityId bodyEntityId) const override;

    private:
        void RefreshVolume();

        //! Delivers the enter and exit events the last step produced. Runs on the scene's
        //! simulation-finish event, so handlers are gameplay code on the main thread rather
        //! than something running on a physics job.
        void DispatchWaterEvents();

        AZ::Vector3 m_dimensions = AZ::Vector3(10.0f, 10.0f, 5.0f);
        JoltWaterVolumeSettings m_settings;
        bool m_visible = true;

        //! Kept in step with the volume so drawing needs no per-frame transform query.
        AZ::Transform m_worldTransform = AZ::Transform::CreateIdentity();

        JoltWaterVolume m_waterVolume;
        AzPhysics::SceneHandle m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;

        //! Reused each frame so dispatching events does not allocate per step.
        AZStd::vector<JoltWaterVolumeEvent> m_eventScratch;

        //! Fires after the step, with no body mutexes held, which is the only place bodies
        //! the step found asleep can safely be woken.
        AzPhysics::SceneEvents::OnSceneSimulationFinishHandler m_simulationFinishHandler;
    };
} // namespace JoltBuoyancy
