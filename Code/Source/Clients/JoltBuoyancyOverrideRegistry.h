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
