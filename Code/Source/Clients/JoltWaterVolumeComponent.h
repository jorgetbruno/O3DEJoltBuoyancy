#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TransformBus.h>

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
        , private JoltWaterVolumeRequestBus::Handler
    {
    public:
        AZ_COMPONENT(JoltWaterVolumeComponent, "{A1B2C3D4-E5F6-4708-9A1B-2C3D4E5F6A7B}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        AZ::Vector3& GetDimensions()
        {
            return m_dimensions;
        }
        JoltWaterVolumeSettings& GetSettings()
        {
            return m_settings;
        }

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // AZ::TransformNotificationBus
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        // JoltWaterVolumeRequestBus
        void SetFluidDensity(float density) override;
        float GetFluidDensity() const override;
        void SetLinearDrag(float drag) override;
        float GetLinearDrag() const override;
        void SetAngularDrag(float drag) override;
        float GetAngularDrag() const override;
        void SetFluidVelocity(const AZ::Vector3& velocity) override;
        AZ::Vector3 GetFluidVelocity() const override;
        void SetEnabled(bool enabled) override;
        bool IsEnabled() const override;
        int GetSubmergedBodyCount() const override;

    private:
        void RefreshVolume();

        AZ::Vector3 m_dimensions = AZ::Vector3(10.0f, 10.0f, 5.0f);
        JoltWaterVolumeSettings m_settings;

        JoltWaterVolume m_waterVolume;
        AzPhysics::SceneHandle m_attachedSceneHandle = AzPhysics::InvalidSceneHandle;
    };
} // namespace JoltBuoyancy
