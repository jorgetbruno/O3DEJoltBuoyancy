#include <Clients/JoltBuoyancyAllocator.h>

#include <AzCore/Memory/SystemAllocator.h>

#include <Jolt/Jolt.h>
#include <Jolt/Core/Memory.h>

namespace JoltBuoyancy
{
    void* JoltBuoyancyAllocator::Allocate(size_t size)
    {
        return AZ::AllocatorInstance<AZ::SystemAllocator>::Get().allocate(size, 16);
    }

    void* JoltBuoyancyAllocator::Reallocate(void* block, [[maybe_unused]] size_t oldSize, size_t newSize)
    {
        if (block)
        {
            return AZ::AllocatorInstance<AZ::SystemAllocator>::Get().reallocate(block, newSize, 16);
        }
        return Allocate(newSize);
    }

    void JoltBuoyancyAllocator::Free(void* block)
    {
        if (block)
        {
            AZ::AllocatorInstance<AZ::SystemAllocator>::Get().deallocate(block);
        }
    }

    void* JoltBuoyancyAllocator::AlignedAllocate(size_t size, size_t alignment)
    {
        return AZ::AllocatorInstance<AZ::SystemAllocator>::Get().allocate(size, alignment);
    }

    void JoltBuoyancyAllocator::AlignedFree(void* block)
    {
        if (block)
        {
            AZ::AllocatorInstance<AZ::SystemAllocator>::Get().deallocate(block);
        }
    }

    void JoltBuoyancyAllocator::Install()
    {
        JPH::Allocate = &JoltBuoyancyAllocator::Allocate;
        JPH::Reallocate = &JoltBuoyancyAllocator::Reallocate;
        JPH::Free = &JoltBuoyancyAllocator::Free;
        JPH::AlignedAllocate = &JoltBuoyancyAllocator::AlignedAllocate;
        JPH::AlignedFree = &JoltBuoyancyAllocator::AlignedFree;
    }

    // Deliberately no Uninstall, for the same reason the physics gem has none: Jolt
    // allocations can outlive whatever installed the hooks, and freeing through a null
    // pointer is exactly the crash this file exists to prevent. The hooks only forward
    // to AZ::SystemAllocator, which lives for the process.

} // namespace JoltBuoyancy
