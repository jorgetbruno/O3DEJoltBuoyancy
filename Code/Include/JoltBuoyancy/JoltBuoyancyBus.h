#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Math/Vector3.h>

namespace JoltBuoyancy
{
    //! Runtime control of a water volume. The volume applies Jolt's buoyancy impulses to
    //! every rigid body overlapping it, once per physics step.
    class JoltWaterVolumeRequests
        : public AZ::ComponentBus
    {
    public:
        virtual ~JoltWaterVolumeRequests() = default;

        //! Density of the fluid in kg/m^3. A body floats when it is less dense than this
        //! and sinks when it is denser, so this is what decides whether wood floats and
        //! stone does not. Fresh water is 1000.
        virtual void SetFluidDensity(float density) = 0;
        virtual float GetFluidDensity() const = 0;

        //! Fraction of a body's velocity removed per second while submerged; higher values
        //! make the water feel thicker.
        virtual void SetLinearDrag(float drag) = 0;
        virtual float GetLinearDrag() const = 0;
        virtual void SetAngularDrag(float drag) = 0;
        virtual float GetAngularDrag() const = 0;

        //! Velocity of the fluid itself, in world space: a current that carries bodies.
        virtual void SetFluidVelocity(const AZ::Vector3& velocity) = 0;
        virtual AZ::Vector3 GetFluidVelocity() const = 0;

        //! Stops or resumes applying buoyancy without removing the component.
        virtual void SetEnabled(bool enabled) = 0;
        virtual bool IsEnabled() const = 0;

        //! Number of bodies the volume affected during the most recent physics step.
        virtual int GetSubmergedBodyCount() const = 0;
    };

    using JoltWaterVolumeRequestBus = AZ::EBus<JoltWaterVolumeRequests>;

} // namespace JoltBuoyancy
