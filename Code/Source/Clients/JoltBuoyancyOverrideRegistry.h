#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/parallel/shared_mutex.h>

#include <JoltBuoyancy/JoltBuoyancyBus.h>

namespace JoltBuoyancy
{
    //! What one entity's buoyancy override says.
    struct JoltBuoyancyOverride
    {
        JoltBuoyancyMode m_mode = JoltBuoyancyMode::Automatic;
        float m_factor = 1.0f;
        //! Scale the volume's drag for this body. 1 leaves it alone.
        float m_linearDragMultiplier = 1.0f;
        float m_angularDragMultiplier = 1.0f;

        //! Per body-axis drag scale, refining what Jolt already derives from the body's
        //! bounding box. Jolt is not isotropic - it projects the box along the flow - but a
        //! hull is far more streamlined along its length than its box suggests, and no box
        //! can express that. Lowering the forward axis is what lets a boat hold a heading
        //! instead of sliding sideways through turns.
        AZ::Vector3 m_directionalDrag = AZ::Vector3::CreateOne();

        //! Added-mass coefficient: water dragged along with the hull, as a fraction of the
        //! displaced mass. 0 is off, around 0.5 is typical for a blunt body.
        //!
        //! An approximation, and deliberately so. Doing it properly means altering the
        //! solver's mass matrix, which Jolt does not expose, so this resists changes in
        //! velocity after the fact instead. It takes the twitchiness out of heave and pitch
        //! without pretending to be the real thing.
        float m_addedMass = 0.0f;
        bool m_excluded = false;
    };

    //! Per-entity buoyancy overrides, looked up by water volumes while they step.
    //!
    //! A volume reaches this from Jolt's step listener jobs, where an EBus call into a
    //! component would be dispatching to gameplay code on a physics thread. Override
    //! components instead push their values in here when they change, and volumes only
    //! read - under a shared lock, so several volumes can look up at once.
    //!
    //! Empty in the common case, which is the signal for a volume to skip the lookup
    //! altogether.
    class JoltBuoyancyOverrideRegistry
    {
    public:
        static JoltBuoyancyOverrideRegistry& Get();

        void Set(AZ::EntityId entityId, const JoltBuoyancyOverride& override);
        void Remove(AZ::EntityId entityId);

        bool IsEmpty() const;

        //! The override for this entity, or the default when it has none.
        JoltBuoyancyOverride Find(AZ::EntityId entityId) const;

    private:
        mutable AZStd::shared_mutex m_mutex;
        AZStd::unordered_map<AZ::EntityId, JoltBuoyancyOverride> m_overrides;
    };
} // namespace JoltBuoyancy
