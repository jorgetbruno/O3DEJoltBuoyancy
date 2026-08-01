#pragma once

#include <AzCore/std/containers/vector.h>
#include <AzCore/std/parallel/mutex.h>

namespace JPH
{
    class PhysicsSystem;
}

namespace JoltBuoyancy
{
    class JoltWaterVolume;

    //! Tracks which water volumes are attached to which physics system, so a volume can
    //! see its overlapping neighbours.
    //!
    //! This exists to stop two overlapping volumes both applying an impulse to the same
    //! body, which previously doubled its buoyancy. Rather than have volumes claim bodies
    //! first-come-first-served - which would depend on the order Jolt happens to run its
    //! step listener jobs in, and so vary between frames - each volume works out
    //! independently whether it is the one that owns a given body. Every volume reaches
    //! the same answer from the same data, so no arbitration, no per-step bookkeeping, and
    //! nothing that breaks when the listeners run concurrently.
    class JoltWaterVolumeRegistry
    {
    public:
        //! One registry for the process. Volumes are keyed by physics system, so volumes
        //! in different scenes never see each other.
        static JoltWaterVolumeRegistry& Get();

        void Register(JPH::PhysicsSystem* physicsSystem, JoltWaterVolume* volume);
        void Unregister(JPH::PhysicsSystem* physicsSystem, JoltWaterVolume* volume);

        //! Every other volume in the same physics system. Left empty in the ordinary
        //! single-volume case, which is the signal to skip the overlap work entirely.
        void CollectPeers(
            JPH::PhysicsSystem* physicsSystem, const JoltWaterVolume* self, AZStd::vector<JoltWaterVolume*>& outPeers) const;

    private:
        struct Entry
        {
            JPH::PhysicsSystem* m_physicsSystem = nullptr;
            JoltWaterVolume* m_volume = nullptr;
        };

        //! Guards registration and lookup. Registration happens on the main thread when a
        //! volume attaches; lookup happens once per step per volume, on a job thread.
        mutable AZStd::mutex m_mutex;
        AZStd::vector<Entry> m_entries;
    };
} // namespace JoltBuoyancy
