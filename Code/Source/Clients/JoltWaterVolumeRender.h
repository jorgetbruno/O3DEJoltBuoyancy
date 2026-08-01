#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/Math/Vector3.h>

#include <JoltBuoyancy/JoltBuoyancyBus.h>

namespace AzFramework
{
    class DebugDisplayRequests;
}

namespace JoltBuoyancy
{
    class JoltWaterVolume;

    //! Draws a water volume as a translucent box with its surface face picked out.
    //!
    //! Shared by the editor component (viewport) and the runtime component (game mode)
    //! so the two cannot drift apart, and driven by the same world transform and
    //! dimensions the solver uses - the visual is the volume, not a copy of it.
    //! Pass the settings and the volume's wave phase to draw the surface as it actually
    //! is. Without them the lid is drawn flat, which is a lie whenever waves are on.
    void DrawWaterVolume(
        AzFramework::DebugDisplayRequests& debugDisplay,
        const AZ::Transform& worldTransform,
        const AZ::Vector3& dimensions,
        const JoltWaterVolumeSettings* settings = nullptr,
        const JoltWaterVolume* volume = nullptr);
} // namespace JoltBuoyancy
