#include <Clients/JoltBuoyancyDebugDraw.h>

#include <AzCore/std/parallel/lock.h>
#include <AzFramework/Entity/EntityDebugDisplayBus.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Renderer/DebugRendererSimple.h>
#endif

namespace JoltBuoyancy
{
    namespace
    {
        AZ::Vector3 FromJoltR(const JPH::RVec3& v)
        {
            return AZ::Vector3(static_cast<float>(v.GetX()), static_cast<float>(v.GetY()), static_cast<float>(v.GetZ()));
        }

#ifdef JPH_DEBUG_RENDERER
        AZ::Color FromJoltColor(const JPH::Color& color)
        {
            return AZ::Color(color.r, color.g, color.b, color.a);
        }
#endif
    } // namespace

#ifdef JPH_DEBUG_RENDERER
    //! Records what Jolt asks for instead of drawing it.
    //!
    //! Constructing one installs it as this module's DebugRenderer::sInstance, which is the
    //! whole point - see the class comment on JoltBuoyancyDebugDraw.
    //!
    //! Solid triangles come through as their three edges, via the base class fallback. For
    //! a wireframe diagnostic drawn over a translucent water box that is the more readable
    //! answer anyway, and it keeps this to two overrides.
    class JoltBuoyancyDebugDraw::BufferingRenderer final : public JPH::DebugRendererSimple
    {
    public:
        explicit BufferingRenderer(JoltBuoyancyDebugDraw& owner)
            : m_owner(owner)
        {
        }

        void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override
        {
            m_owner.RecordLine(FromJoltR(inFrom), FromJoltR(inTo), FromJoltColor(inColor));
        }

        void DrawText3D(
            JPH::RVec3Arg inPosition, const std::string_view& inString, JPH::ColorArg inColor, float /*inHeight*/) override
        {
            m_owner.RecordLabel(
                FromJoltR(inPosition), AZStd::string(inString.data(), inString.size()), FromJoltColor(inColor));
        }

    private:
        JoltBuoyancyDebugDraw& m_owner;
    };
#else
    //! Nothing to record against in a build without Jolt's debug renderer. Declared so the
    //! unique_ptr member has a complete type either way.
    class JoltBuoyancyDebugDraw::BufferingRenderer
    {
    };
#endif

    JoltBuoyancyDebugDraw& JoltBuoyancyDebugDraw::Get()
    {
        static JoltBuoyancyDebugDraw instance;
        return instance;
    }

    JoltBuoyancyDebugDraw::~JoltBuoyancyDebugDraw()
    {
        // Leaving the flag set while the renderer is destroyed is precisely the state this
        // class exists to make impossible, and at static teardown there is nobody left to
        // notice it.
        SetSubmergedVolumesEnabled(false);
    }

    bool JoltBuoyancyDebugDraw::SetSubmergedVolumesEnabled(bool enabled)
    {
#ifdef JPH_DEBUG_RENDERER
        AZStd::lock_guard lock(m_mutex);
        if (enabled == m_enabled)
        {
            return true;
        }

        if (enabled)
        {
            // Renderer first, flag second. Jolt reads the flag on job threads, so between
            // the two writes it must never find drawing on with nowhere to draw.
            m_renderer = AZStd::make_unique<BufferingRenderer>(*this);
            m_overflowed = false;
            JPH::Shape::sDrawSubmergedVolumes = true;
        }
        else
        {
            // And the reverse on the way out.
            JPH::Shape::sDrawSubmergedVolumes = false;
            m_renderer.reset();

            // Released rather than merely cleared: this lives in a function-local static
            // that outlives AZ::SystemAllocator, and a container still holding its buffer
            // at teardown reads as a leak.
            AZStd::vector<JoltDebugLine>().swap(m_lines);
            AZStd::vector<JoltDebugLabel>().swap(m_labels);
        }

        m_enabled = enabled;
        return true;
#else
        // Release builds compile Jolt without its debug renderer, so there is no drawing
        // to enable and no flag to set.
        return !enabled;
#endif
    }

    bool JoltBuoyancyDebugDraw::IsSubmergedVolumesEnabled() const
    {
        AZStd::lock_guard lock(m_mutex);
        return m_enabled;
    }

    void JoltBuoyancyDebugDraw::RecordLine(const AZ::Vector3& from, const AZ::Vector3& to, const AZ::Color& color)
    {
        AZStd::lock_guard lock(m_mutex);
        if (m_lines.size() >= MaxRecordedLines)
        {
            m_overflowed = true;
            return;
        }
        m_lines.push_back({ from, to, color });
    }

    void JoltBuoyancyDebugDraw::RecordLabel(const AZ::Vector3& position, AZStd::string text, const AZ::Color& color)
    {
        AZStd::lock_guard lock(m_mutex);
        if (m_labels.size() >= MaxRecordedLines)
        {
            m_overflowed = true;
            return;
        }
        m_labels.push_back({ position, AZStd::move(text), color });
    }

    void JoltBuoyancyDebugDraw::Drain(AZStd::vector<JoltDebugLine>& outLines, AZStd::vector<JoltDebugLabel>& outLabels)
    {
        bool overflowed = false;
        {
            AZStd::lock_guard lock(m_mutex);

            // Moved out and then released, rather than swapped. A swap hands the caller's
            // old buffer back to these members, and this object is a function-local static
            // that outlives AZ::SystemAllocator - so a caller draining twice into the same
            // vector leaves a buffer parked here until teardown, where it reads as a leak.
            outLines = AZStd::move(m_lines);
            outLabels = AZStd::move(m_labels);
            AZStd::vector<JoltDebugLine>().swap(m_lines);
            AZStd::vector<JoltDebugLabel>().swap(m_labels);

            overflowed = m_overflowed;
            m_overflowed = false;
        }

        AZ_Warning("JoltBuoyancy", !overflowed,
            "jolt_DebugSubmergedVolumes produced more than %zu primitives in one frame and the rest were dropped. "
            "Either a great many bodies are in the water, or nothing is draining the buffer - the diagnostic is drawn "
            "by the water volume component, so it needs one active in the scene.",
            MaxRecordedLines);
    }

    void JoltBuoyancyDebugDraw::Flush(AzFramework::DebugDisplayRequests& debugDisplay)
    {
        if (!IsSubmergedVolumesEnabled())
        {
            return;
        }

        AZStd::vector<JoltDebugLine> lines;
        AZStd::vector<JoltDebugLabel> labels;
        Drain(lines, labels);
        if (lines.empty() && labels.empty())
        {
            return;
        }

        // Depth on: the point of the overlay is to see which part of a hull the solver
        // thinks is under the surface, which means reading it against the hull.
        debugDisplay.DepthTestOn();

        AZ::Color currentColor = AZ::Colors::White;
        debugDisplay.SetColor(currentColor);
        for (const JoltDebugLine& line : lines)
        {
            if (line.m_color != currentColor)
            {
                currentColor = line.m_color;
                debugDisplay.SetColor(currentColor);
            }
            debugDisplay.DrawLine(line.m_from, line.m_to);
        }

        for (const JoltDebugLabel& label : labels)
        {
            debugDisplay.SetColor(label.m_color);
            debugDisplay.DrawTextLabel(label.m_position, 1.0f, label.m_text.c_str(), true);
        }
    }
} // namespace JoltBuoyancy
