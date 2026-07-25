#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>

namespace AzFramework
{
    class DebugDisplayRequests;
}

namespace JoltBuoyancy
{
    //! Draws a water volume as a translucent box with its surface face picked out.
    //!
    //! Shared by the editor component (viewport) and the runtime component (game mode)
    //! so the two cannot drift apart, and driven by the same world transform and
    //! dimensions the solver uses - the visual is the volume, not a copy of it.
    void DrawWaterVolume(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& worldTransform, const AZ::Vector3& dimensions);
} // namespace JoltBuoyancy
