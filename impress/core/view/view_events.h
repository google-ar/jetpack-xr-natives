/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_EVENTS_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_EVENTS_H_

#include <functional>
#include <optional>
#include <string>

#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Renderer.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/window/window_rotation.h"

namespace imp {

// Event sent to the dispatcher when the view's size changes.
struct ViewSizeChangedEvent : public Event {
  // The size of the view in pixels.
  uint2 size;
  // The margins of the view in pixels in the order of left, right, top, bottom.
  uint4 margins;
};

// Event sent to the dispatcher when the orientation of the view changes.
struct ViewRotationChangedEvent : public Event {
  // The new orientation of the view being rendered.
  window::WindowRotation rotation;
};

// Event sent to the dispatcher when the call stack is ready to accept IMGUI
// render commands.  Only sent when a DevModeExtension is installed.
struct ImGuiPreRenderEvent : public Event {};

// An iOS-specific event used allow the native runtime to react to a device
// rotation transition, maintaining a smooth presentation.
//
// On iOS, when the user rotates their device, a sequence of events happens:
// - The user interface orientation changes instantly to the new orientation.
// - The view bounds (and layer/swapchain) are changed to the new size.
// - The view gets rendered at the new size.
// - The presentation layer uses a view animation to transform the new view over
//   a transition time determined by the system:
//   - On the first frame, the _new_ rendered frame is scaled, and the
//     containing window is rotated, such that the top-left and bottom-right
//     corners of the new image match the on-panel locations of the old image
//     (i.e. is squished vertically or horizontally and rotated 90 or 180
//     degrees away from the final post-transition presentation).
//   - Over the system-determined transition time, the presentation layer bounds
//     animates to the new size, continually scaling subsequent images rendered
//     at the destination size and orientation.  The window rotates during this
//     time as well.
//
// This event is sent every frame during a view transition.
// - `size_scale` communicates the scale applied to the _presentation_ layer to
//   the rendered view, allowing the latter to counter-scale the aspect ratio.
// - `counter_rotation` communicates the rotation that would
//   be necessary to render the frame at the "true" orientation.  For 3D
//   content, this isn't necessary to deal with because the view scale is
//   enough to match the pre-rotation frame exactly.  For AR content, the change
//   in orientation causes a 90 or 180 degree jump in the orientation of the
//   camera feed covering the frame, which gradually interpolates to 0.
struct ViewTransitionParametersChangedEvent : public Event {
  float2 size_scale;
  float counter_rotation;
};

// Event sent to the dispatcher just before the frame is updated.
// Allows for receivers of this event to indicate to Impress that this frame
// should be skipped. If any receiver requests the frame to be skipped, it is
// skipped.
// TODO: Make it possible to report a reason for skipping a frame.
class ViewPreFrameUpdateEvent : public Event {
 public:
  ViewPreFrameUpdateEvent() = default;

  // Created by View::OnHostPreRender with a functor that causes the frame to be
  // skipped.
  explicit ViewPreFrameUpdateEvent(
      std::function<void(absl::optional<absl::Duration>)> skip_frame_fn);

  // Tells Impress to skip this frame. Subsequent calls to this on the same
  // frame will do nothing.
  //
  // This will do the following:
  // 1. Prevent the View::Update from being called.
  // 2. Prevent Component Updates from being called.
  // 3. Prevent the foreground executor from pumping this frame.
  // 3. Prevent this frame from being rendered.
  //
  // if a time_until_retry is passed in, then the platform will attempt to
  // advance and render another frame after the specified amount of time elapses
  // instead of waiting until the next time a frame would render normally. As an
  // example, this is used when running Ar to re-try a frame more quickly if we
  // skipped a frame because there was no new Ar frame yet. Currently, this is
  // only implemented on Android, and for this to function the app must let
  // Impress control the frame loop.
  //
  // **NOTE** This will do nothing if FilamentHost::IsNextRenderRequired() is
  // true. It is not safe to skip frames when rendering is required, which
  // occurs when resize and rotation occurs.
  void SkipFrame(absl::optional<absl::Duration> time_until_retry) const;

 private:
  std::function<void(absl::optional<absl::Duration>)> skip_frame_fn_;
};

// Event sent to the dispatcher just after the frame is updated.
//
// This is sent after the main update loop, but before the rendering phase
// starts.
//
// This event is not sent if the frame is skipped.
//
// However, it *is* sent if the frame is updated with no time elapsed, which
// happens in scuba tests.
class ViewPostFrameUpdateEvent : public Event {};

// Event sent to the dispatcher just before the main visibility group is
// rendered but after the rendering phase has started.
//
// This event can be used to perform offscreen rendering to a texture.
//
// Note: This is not the same as UpdatePhase::kPreRender.
//
// *IMPORTANT* This event takes place just after filament::Renderer::beginFrame
// is called but before Impress renders the main pass. This means that it is not
// the right place to do any work (i.e. update material parameters) that should
// be done from outside of the rendering phase. This event is intended for
// rendering to a RenderTarget only.
class ViewPreRenderEvent : public Event {
 public:
  explicit ViewPreRenderEvent(filament::Renderer* filament_renderer);

  filament::Renderer* GetRenderer() const;

 private:
  filament::Renderer* filament_renderer_;
};

// Event sent to the dispatcher just before skipping rendering the frame.
class ViewSkippedFrameEvent : public Event {
 public:
  ViewSkippedFrameEvent() = default;
};

// Event sent to the dispatcher just after the main visibility group is
// rendered.
class ViewPostRenderEvent : public Event {
 public:
  ViewPostRenderEvent() = default;
};

// Event sent to the dispatcher just after the main render ends. Used to
// schedule secondary windows/renders.
class ViewSecondaryRenderEvent : public Event {
 public:
  ViewSecondaryRenderEvent() = default;
};

// Event sent to the dispatcher just after the frame is fully complete.
class ViewPostFrameEvent : public Event {
 public:
  ViewPostFrameEvent() = default;
};

// Event sent to the dispatcher whenever the Impress view is paused.
class ViewPausedEvent : public Event {
 public:
  ViewPausedEvent() = default;
};

// Event sent to the dispatcher whenever the Impress view is started/resumed.
class ViewResumedEvent : public Event {
 public:
  ViewResumedEvent() = default;
};

// Event sent to the dispatcher right before everything in the Impress view is
// cleaned up and released.
class ViewCleanupEvent : public Event {
 public:
  ViewCleanupEvent() = default;
};

// Event sent when a file is dragged onto the app.
struct DropFileEvent : public Event {
  explicit DropFileEvent(absl::string_view file,
                         std::optional<absl::Cord> file_data = std::nullopt)
      : filename(file), data(file_data) {}

  std::string filename;
  std::optional<absl::Cord> data;
};

// Event sent when a filament::View is created via View::CreateFilamentView().
class FilamentViewCreatedEvent : public Event {
 public:
  explicit FilamentViewCreatedEvent(filament::View* v) : view(v) {}

  filament::View* view;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_VIEW_EVENTS_H_
