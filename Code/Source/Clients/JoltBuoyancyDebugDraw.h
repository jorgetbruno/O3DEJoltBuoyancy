#pragma once

#include <AzCore/Math/Color.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzCore/std/string/string.h>

namespace AzFramework
{
    class DebugDisplayRequests;
}

namespace JoltBuoyancy
{
    //! One line Jolt asked to be drawn during a physics step, in world space.
    struct JoltDebugLine
    {
        AZ::Vector3 m_from = AZ::Vector3::CreateZero();
        AZ::Vector3 m_to = AZ::Vector3::CreateZero();
        AZ::Color m_color = AZ::Colors::White;
    };

    //! A piece of text Jolt asked to be drawn, in world space.
    struct JoltDebugLabel
    {
        AZ::Vector3 m_position = AZ::Vector3::CreateZero();
        AZStd::string m_text;
        AZ::Color m_color = AZ::Colors::White;
    };

    //! Jolt's own buoyancy diagnostic, made safe to turn on from this module.
    //!
    //! `Shape::sDrawSubmergedVolumes` draws the slice of each shape that is under the
    //! surface, its centre of buoyancy and its submerged volume. It is far more use than
    //! the translucent box when a body floats at the wrong height, because it shows what
    //! the *solver* thinks is wet rather than where the water is.
    //!
    //! Two things make it dangerous to reach for directly, and both are the same trap as
    //! the allocator (see JoltBuoyancyAllocator.h): **Jolt's statics are per module.**
    //!
    //!   - `Shape::sDrawSubmergedVolumes` is one bool per statically linked module. Setting
    //!     it here turns drawing on for this module's copy of Jolt's shape code, which is
    //!     the copy the water volume calls - so that part works as you would expect.
    //!   - `DebugRenderer::sInstance` is also one pointer per module, and it is set by the
    //!     DebugRenderer constructor. The physics gem builds one on the stack inside its
    //!     own draw call, which sets *its* copy for the duration of that call. This
    //!     module's copy is null and stays null.
    //!
    //! So the flag alone turned every submerged-volume calculation into a null dereference,
    //! on a physics job thread, inside the step. This class closes that: the flag is only
    //! ever true while this module owns a renderer, and the two are set and cleared
    //! together in the right order.
    //!
    //! The renderer records rather than draws. Jolt calls it from step listener jobs with
    //! every body mutex held, where O3DE's debug display bus must not be touched, so
    //! primitives are buffered under a mutex and handed to whichever water volume component
    //! draws next frame.
    class JoltBuoyancyDebugDraw
    {
    public:
        static JoltBuoyancyDebugDraw& Get();

        //! Turns Jolt's submerged-volume drawing on or off for this module, installing or
        //! removing the renderer that makes it safe.
        //!
        //! @return false if the request could not be honoured - a release build has no
        //!         debug renderer compiled in at all - in which case nothing was changed
        //!         and drawing stays off.
        bool SetSubmergedVolumesEnabled(bool enabled);

        bool IsSubmergedVolumesEnabled() const;

        //! Records a line. Called from Jolt's job threads.
        void RecordLine(const AZ::Vector3& from, const AZ::Vector3& to, const AZ::Color& color);

        //! Records a label. Called from Jolt's job threads.
        void RecordLabel(const AZ::Vector3& position, AZStd::string text, const AZ::Color& color);

        //! Takes everything recorded since the last call, leaving the buffers empty.
        //! Whichever component drains first gets the primitives; the rest get nothing,
        //! which is what stops two water volumes drawing the same diagnostic twice.
        void Drain(AZStd::vector<JoltDebugLine>& outLines, AZStd::vector<JoltDebugLabel>& outLabels);

        //! Drains and draws in one step, for callers that already hold a debug display.
        //! Does nothing when drawing is off, so it is cheap to call unconditionally.
        void Flush(AzFramework::DebugDisplayRequests& debugDisplay);

        //! A step can produce a great many triangles, and nothing drains the buffer when
        //! no volume is visible. Past this the recording stops and says so once.
        static constexpr size_t MaxRecordedLines = 60000;

    private:
        JoltBuoyancyDebugDraw() = default;
        ~JoltBuoyancyDebugDraw();

        JoltBuoyancyDebugDraw(const JoltBuoyancyDebugDraw&) = delete;
        JoltBuoyancyDebugDraw& operator=(const JoltBuoyancyDebugDraw&) = delete;

        class BufferingRenderer;

        mutable AZStd::mutex m_mutex;
        AZStd::vector<JoltDebugLine> m_lines;
        AZStd::vector<JoltDebugLabel> m_labels;
        AZStd::unique_ptr<BufferingRenderer> m_renderer;
        bool m_enabled = false;
        bool m_overflowed = false;
    };
} // namespace JoltBuoyancy
