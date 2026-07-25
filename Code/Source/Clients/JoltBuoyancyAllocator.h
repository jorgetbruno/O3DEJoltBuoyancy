#pragma once

#include <cstddef>

namespace JoltBuoyancy
{
    //! Installs this module's Jolt allocation hooks.
    //!
    //! Jolt reaches memory through five global function pointers (JPH::Allocate,
    //! JPH::Reallocate, JPH::Free, JPH::AlignedAllocate, JPH::AlignedFree). Jolt is
    //! statically linked into every module that uses it, so *each module owns its own
    //! copy of those pointers*, and they start out null. The JoltPhysics gem installing
    //! them does nothing for this gem: any Jolt code the linker resolved locally - the
    //! header-only collision collectors, and non-virtual calls like
    //! PhysicsSystem::AddStepListener - allocates through this module's copy and jumps
    //! to address zero if it was never filled in.
    //!
    //! The hooks must also *match* the physics gem's, because allocations cross the
    //! module boundary: the step listener array is grown by this module's
    //! AddStepListener and released by the physics gem's ~PhysicsSystem. Both therefore
    //! forward to AZ::SystemAllocator, which is one process-wide instance shared by
    //! every module. JPH::RegisterDefaultAllocator would compile and run, but it routes
    //! to malloc/free and would hand the physics gem a block AZ::SystemAllocator never
    //! handed out.
    class JoltBuoyancyAllocator
    {
    public:
        //! Idempotent, and safe to call from any module entry point.
        static void Install();

    private:
        static void* Allocate(size_t size);
        static void* Reallocate(void* block, size_t oldSize, size_t newSize);
        static void Free(void* block);
        static void* AlignedAllocate(size_t size, size_t alignment);
        static void AlignedFree(void* block);
    };
} // namespace JoltBuoyancy
