#include <Clients/JoltWaterVolumeRender.h>

#include <AzCore/Math/Color.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>

namespace JoltBuoyancy
{
    namespace
    {
        // Deep enough to read as water against the grey test level, transparent enough
        // to watch a body sink through it - the whole point of the level.
        const AZ::Color BodyColor(0.10f, 0.42f, 0.65f, 0.25f);
        const AZ::Color SurfaceColor(0.45f, 0.80f, 0.95f, 0.35f);
        const AZ::Color EdgeColor(0.55f, 0.85f, 1.00f, 0.90f);

        AZ::Vector3 Corner(const AZ::Vector3& halfExtents, bool xPositive, bool yPositive, bool zPositive)
        {
            return AZ::Vector3(
                xPositive ? halfExtents.GetX() : -halfExtents.GetX(),
                yPositive ? halfExtents.GetY() : -halfExtents.GetY(),
                zPositive ? halfExtents.GetZ() : -halfExtents.GetZ());
        }
    }

    void DrawWaterVolume(
        AzFramework::DebugDisplayRequests& debugDisplay, const AZ::Transform& worldTransform, const AZ::Vector3& dimensions)
    {
        const AZ::Vector3 halfExtents = dimensions * 0.5f;

        // Drawing in the volume's own space, so a tilted volume tilts with it and the
        // surface stays the local +Z face - the same face the solver treats as the
        // waterline.
        debugDisplay.PushMatrix(worldTransform);

        // Translucent geometry has no depth sorting here, so writing depth would let
        // the box hide the bodies floating in it. Culling off keeps the fill visible
        // from inside the volume as well as outside.
        debugDisplay.DepthWriteOff();
        debugDisplay.CullOff();

        debugDisplay.SetColor(BodyColor);
        debugDisplay.DrawSolidBox(-halfExtents, halfExtents);

        // The surface again on its own, brighter: it is the one face with physical
        // meaning, and a plain box gives no clue which way up it is.
        debugDisplay.SetColor(SurfaceColor);
        debugDisplay.DrawQuad(
            Corner(halfExtents, false, false, true), Corner(halfExtents, true, false, true),
            Corner(halfExtents, true, true, true), Corner(halfExtents, false, true, true));

        // Wireframe edges last, nearly opaque, so the extents stay readable where the
        // fill washes out against a bright background.
        debugDisplay.DepthWriteOn();
        debugDisplay.SetColor(EdgeColor);
        debugDisplay.DrawWireBox(-halfExtents, halfExtents);

        debugDisplay.CullOn();
        debugDisplay.PopMatrix();
    }
} // namespace JoltBuoyancy
