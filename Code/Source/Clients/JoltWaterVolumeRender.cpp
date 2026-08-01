#include <Clients/JoltWaterVolumeRender.h>

#include <AzCore/Math/Color.h>
#include <AzCore/Math/MathUtils.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>

#include <Clients/JoltWaterVolume.h>

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
        AzFramework::DebugDisplayRequests& debugDisplay,
        const AZ::Transform& worldTransform,
        const AZ::Vector3& dimensions,
        const JoltWaterVolumeSettings* settings,
        const JoltWaterVolume* volume)
    {
        const AZ::Vector3 halfExtents = dimensions * 0.5f;
        const bool isSphere = settings && settings->m_shape == JoltWaterVolumeShape::Sphere;
        // The drawn surface has to come from the same waves the solver is using, or the
        // water you can see is not the water bodies float on.
        JoltGerstnerWaves waves;
        if (volume != nullptr)
        {
            waves = volume->GetWaves();
        }
        const bool hasWaves = settings && JoltWaterVolume::HasWaves(*settings) && !waves.IsEmpty();

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
        if (isSphere)
        {
            debugDisplay.DrawBall(AZ::Vector3::CreateZero(), dimensions.GetX() * 0.5f);
        }
        else
        {
            debugDisplay.DrawSolidBox(-halfExtents, halfExtents);
        }

        // The surface on its own, brighter: it is the one face with physical meaning, and
        // a plain box gives no clue which way up it is.
        debugDisplay.SetColor(SurfaceColor);
        const float surfaceHeight = isSphere ? dimensions.GetX() * 0.5f : halfExtents.GetZ();

        if (hasWaves && !isSphere)
        {
            // Tessellated, so the drawing shows the surface the solver is actually using.
            // A flat lid over a rippled surface is the sort of mismatch that sends someone
            // hunting for a physics bug that is not there.
            const AZ::Vector2 extents(dimensions.GetX(), dimensions.GetY());
            // Enough subdivisions to resolve the shortest component present.
            float shortestWavelength = extents.GetX();
            for (const JoltGerstnerComponent& component : waves.GetComponents())
            {
                shortestWavelength =
                    AZ::GetMin(shortestWavelength, AZ::Constants::TwoPi / AZ::GetMax(component.m_waveNumber, 0.001f));
            }
            const int steps = AZ::GetClamp(static_cast<int>(extents.GetX() / (shortestWavelength * 0.25f)), 8, 64);

            const auto surfacePoint = [&](int ix, int iy)
            {
                const float x = -halfExtents.GetX() + extents.GetX() * (static_cast<float>(ix) / steps);
                const float y = -halfExtents.GetY() + extents.GetY() * (static_cast<float>(iy) / steps);
                // Evaluated, not inverted: the mesh is drawn at the displaced parameter
                // points, which is exactly what a Gerstner vertex shader does.
                return waves.Evaluate(AZ::Vector2(x, y), surfaceHeight).m_position;
            };

            for (int ix = 0; ix < steps; ++ix)
            {
                for (int iy = 0; iy < steps; ++iy)
                {
                    debugDisplay.DrawQuad(
                        surfacePoint(ix, iy), surfacePoint(ix + 1, iy), surfacePoint(ix + 1, iy + 1),
                        surfacePoint(ix, iy + 1));
                }
            }
        }
        else
        {
            debugDisplay.DrawQuad(
                AZ::Vector3(-halfExtents.GetX(), -halfExtents.GetY(), surfaceHeight),
                AZ::Vector3(halfExtents.GetX(), -halfExtents.GetY(), surfaceHeight),
                AZ::Vector3(halfExtents.GetX(), halfExtents.GetY(), surfaceHeight),
                AZ::Vector3(-halfExtents.GetX(), halfExtents.GetY(), surfaceHeight));
        }

        // Wireframe edges last, nearly opaque, so the extents stay readable where the
        // fill washes out against a bright background.
        debugDisplay.DepthWriteOn();
        debugDisplay.SetColor(EdgeColor);
        if (isSphere)
        {
            debugDisplay.DrawWireSphere(AZ::Vector3::CreateZero(), dimensions.GetX() * 0.5f);
        }
        else
        {
            debugDisplay.DrawWireBox(-halfExtents, halfExtents);
        }

        debugDisplay.CullOn();
        debugDisplay.PopMatrix();
    }
} // namespace JoltBuoyancy
