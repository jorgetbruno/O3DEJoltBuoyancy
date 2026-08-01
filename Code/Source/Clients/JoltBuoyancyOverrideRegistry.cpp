#include <Clients/JoltBuoyancyOverrideRegistry.h>

#include <AzCore/std/parallel/lock.h>

namespace JoltBuoyancy
{
    JoltBuoyancyOverrideRegistry& JoltBuoyancyOverrideRegistry::Get()
    {
        static JoltBuoyancyOverrideRegistry s_registry;
        return s_registry;
    }

    void JoltBuoyancyOverrideRegistry::Set(AZ::EntityId entityId, const JoltBuoyancyOverride& override)
    {
        if (!entityId.IsValid())
        {
            return;
        }

        AZStd::unique_lock lock(m_mutex);
        m_overrides[entityId] = override;
    }

    void JoltBuoyancyOverrideRegistry::Remove(AZ::EntityId entityId)
    {
        AZStd::unique_lock lock(m_mutex);
        m_overrides.erase(entityId);

        // Hand the buckets back once the last override goes away: this is a function-local
        // static, so whatever it still holds outlives AZ::SystemAllocator and is reported
        // as a leak.
        if (m_overrides.empty())
        {
            AZStd::unordered_map<AZ::EntityId, JoltBuoyancyOverride>().swap(m_overrides);
        }
    }

    bool JoltBuoyancyOverrideRegistry::IsEmpty() const
    {
        AZStd::shared_lock lock(m_mutex);
        return m_overrides.empty();
    }

    JoltBuoyancyOverride JoltBuoyancyOverrideRegistry::Find(AZ::EntityId entityId) const
    {
        AZStd::shared_lock lock(m_mutex);
        const auto found = m_overrides.find(entityId);
        return found != m_overrides.end() ? found->second : JoltBuoyancyOverride();
    }
} // namespace JoltBuoyancy
