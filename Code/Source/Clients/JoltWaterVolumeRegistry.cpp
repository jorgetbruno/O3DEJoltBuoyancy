#include <Clients/JoltWaterVolumeRegistry.h>

#include <AzCore/std/parallel/lock.h>

#include <algorithm>

namespace JoltBuoyancy
{
    JoltWaterVolumeRegistry& JoltWaterVolumeRegistry::Get()
    {
        // Function-local static: constructed on first attach, which is well after the
        // module's allocation hooks are installed.
        static JoltWaterVolumeRegistry s_registry;
        return s_registry;
    }

    void JoltWaterVolumeRegistry::Register(JPH::PhysicsSystem* physicsSystem, JoltWaterVolume* volume)
    {
        if (!physicsSystem || !volume)
        {
            return;
        }

        AZStd::lock_guard lock(m_mutex);
        const auto found = AZStd::find_if(m_entries.begin(), m_entries.end(),
            [physicsSystem, volume](const Entry& entry)
            {
                return entry.m_physicsSystem == physicsSystem && entry.m_volume == volume;
            });
        if (found == m_entries.end())
        {
            m_entries.push_back(Entry{ physicsSystem, volume });
        }
    }

    void JoltWaterVolumeRegistry::Unregister(JPH::PhysicsSystem* physicsSystem, JoltWaterVolume* volume)
    {
        AZStd::lock_guard lock(m_mutex);
        m_entries.erase(
            AZStd::remove_if(m_entries.begin(), m_entries.end(),
                [physicsSystem, volume](const Entry& entry)
                {
                    return entry.m_physicsSystem == physicsSystem && entry.m_volume == volume;
                }),
            m_entries.end());

        // Hand the buffer back once the last volume detaches. This registry is a
        // function-local static, so anything it still holds outlives AZ::SystemAllocator
        // and is reported as a leak - which is how the unit test environment found it.
        if (m_entries.empty())
        {
            AZStd::vector<Entry>().swap(m_entries);
        }
    }

    void JoltWaterVolumeRegistry::CollectPeers(
        JPH::PhysicsSystem* physicsSystem, const JoltWaterVolume* self, AZStd::vector<JoltWaterVolume*>& outPeers) const
    {
        outPeers.clear();
        if (!physicsSystem)
        {
            return;
        }

        AZStd::lock_guard lock(m_mutex);
        for (const Entry& entry : m_entries)
        {
            if (entry.m_physicsSystem == physicsSystem && entry.m_volume != self)
            {
                outPeers.push_back(entry.m_volume);
            }
        }
    }
} // namespace JoltBuoyancy
