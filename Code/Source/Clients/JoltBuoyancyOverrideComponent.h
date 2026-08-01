#pragma once

#include <AzCore/Component/Component.h>

#include <JoltBuoyancy/JoltBuoyancyBus.h>

namespace JoltBuoyancy
{
    //! Overrides how water treats the body on this entity.
    //!
    //! Without it, buoyancy is derived from the body's own density - mass over collider
    //! volume - which is right for solid objects and wrong for anything hollow. A boat hull
    //! is mostly air, but its collider volume says otherwise, so it sinks. Explicit mode is
    //! how that gets authored, and exclusion is how something is kept out of the water
    //! entirely without moving it to another collision layer.
    class JoltBuoyancyOverrideComponent
        : public AZ::Component
        , private JoltBuoyancyOverrideRequestBus::Handler
    {
    public:
        AZ_COMPONENT(JoltBuoyancyOverrideComponent, "{C7D8E9F0-A1B2-43C4-D5E6-F7A8B9C0D1E2}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        JoltBuoyancyMode& AccessMode()
        {
            return m_mode;
        }
        float& AccessFactor()
        {
            return m_factor;
        }
        bool& AccessExcluded()
        {
            return m_excluded;
        }
        float& AccessLinearDragMultiplier()
        {
            return m_linearDragMultiplier;
        }
        float& AccessAngularDragMultiplier()
        {
            return m_angularDragMultiplier;
        }

    protected:
        // AZ::Component
        void Activate() override;
        void Deactivate() override;

        // JoltBuoyancyOverrideRequestBus
        void SetExcludedFromWater(bool excluded) override;
        bool IsExcludedFromWater() const override;
        void SetBuoyancyMode(JoltBuoyancyMode mode) override;
        JoltBuoyancyMode GetBuoyancyMode() const override;
        void SetBuoyancyFactor(float factor) override;
        float GetBuoyancyFactor() const override;
        void SetLinearDragMultiplier(float multiplier) override;
        float GetLinearDragMultiplier() const override;
        void SetAngularDragMultiplier(float multiplier) override;
        float GetAngularDragMultiplier() const override;

    private:
        //! Pushes the current values into the registry water volumes read while stepping.
        void Publish();

        JoltBuoyancyMode m_mode = JoltBuoyancyMode::Automatic;
        float m_factor = 1.2f;
        float m_linearDragMultiplier = 1.0f;
        float m_angularDragMultiplier = 1.0f;
        bool m_excluded = false;
    };
} // namespace JoltBuoyancy
