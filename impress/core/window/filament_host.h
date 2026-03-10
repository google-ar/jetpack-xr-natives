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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAMENT_HOST_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAMENT_HOST_H_

#include <stdbool.h>
#include <stdint.h>

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>  // NOLINT: Need to use threads available in bazel.
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/Scene.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "filament/filament/include/filament/View.h"
#include "core/common/buffer_access.h"
#include "core/common/enum_flags.h"
#include "core/common/invocable.h"
#include "core/common/optional_error.h"
#include "core/common/pass_key.h"
// TODO: Remove config.h and fix breaks.
#include "absl/status/statusor.h"
#include "core/common/robin_set.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/monitor/monitor.h"
#include "core/view/utils/proto/filament_feature_flag.proto.imp.h"
#include "core/window/clipboard/clipboard_handler.h"
#include "core/window/filament_host_input.h"
#include "core/window/filament_view.h"
#include "core/window/window_rotation.h"

namespace imp {

namespace loader {
class GltfAnimation;
class ModelInstance;
}  // namespace loader

namespace editor {
class EditorImpl;
}  // namespace editor

namespace window {

class ImGuiRenderer;

// FilamentHost is a wrapper class that allows you to create, use, and destroy
// a filament engine and its many appendages.  Clients can subclass the 'State'
// inner to customize behavior.
class FilamentHost {
 public:
  // A set of flags to control or influence the core Impress update loop.
  class UpdateStageFlags {
   public:
    using UpdateStageFlag = uint8_t;
    // Skips updating time on Impress, which means all classes that perform
    // logic on the update loop will not be updated this frame. The time will be
    // accumulated for the next frame.
    constexpr static UpdateStageFlag kSkipUpdate = (1 << 0);
    // Skips pumping the foreground executor
    constexpr static UpdateStageFlag kSkipForeground = (1 << 1);
    // Skips pumping the background executor
    constexpr static UpdateStageFlag kSkipBackground = (1 << 2);
    // Skips the entire frame, which is the entire update and render pass. This
    // is here for migration reasons, and should not be used.
    // TODO Remove this once all clients are migrated to the new
    // frame pacing API
    constexpr static UpdateStageFlag kSkipFrame = (1 << 3);

    void SetFlag(UpdateStageFlag flag);
    void UnsetFlag(UpdateStageFlag flag);
    bool HasFlag(UpdateStageFlag flag) const;

   private:
    UpdateStageFlag flags_ = -0;
  };

  // Flags to configure the behavior of IsolatedPreRender().
  enum class IsolatedPreRenderFlags : uint8_t {
    // Force the dev-mode extension to render regardless of state. Useful for
    // scuba tests that are instrumenting rendering.
    kAlwaysRenderDevMode = (1 << 0),
    kNeverRenderDevMode = (1 << 1),
  };

  // Enum type used to return flags from RenderNextFrame()
  enum class RenderResultFlags : uint8_t {
    // This flag is true if we skipped rendering this frame.
    // This can happen if the host thread has gotten too far ahead of filament's
    // render thread, or if State::PreRender has requested the frame be skipped.
    kSkippedRender = (1 << 0),
    // We may be actively animating viewer values, or otherwise desire
    // continuous rendering.  This flag indicates to caller that continuous
    // rendering should continue.  It's false during steady states, when further
    // frames would be identical to the current one.
    kIsAnimating = (1 << 1),
  };

  using DragBegin = detail::PointerDown;
  using Drag = detail::PointerMove;
  using DragEnd = detail::PointerUp;
  using Wheel = detail::Wheel;
  using MouseInput = detail::MouseInput;

  using FrameScheduledCallback = filament::backend::FrameScheduledCallback;
  using FrameCompletedCallback = filament::SwapChain::FrameCompletedCallback;

  struct RenderResult {
    Flags<RenderResultFlags> flags;
    absl::optional<absl::Duration> time_until_retry;
  };

  // FilamentHost expects an instance of a state object at construction time.
  // Subclasses of the state object implement app-specific behavior.
  class State {
   public:
    constexpr static uint2 kDefaultDesktopDimensions = {1600, 900};
    constexpr static uint2 kDefaultMobileDimensions = {540, 960};

    State() = default;
    virtual ~State() = default;

    // Called before first render. Errors from this function prevent render.
    virtual OptionalError Setup(FilamentHost* host) { return NoError(); }

    // Called before the actual `Update` happens, and the output flags will
    // modify the behavior of `Update`. Any state updates should go here.
    virtual absl::Status PreUpdate(
        FilamentHost* host, absl::Duration last_vsync,
        absl::Duration next_vsync, UpdateStageFlags* out_flags,
        absl::optional<absl::Duration>* out_time_until_retry) {
      return absl::OkStatus();
    }

    // Performs the actual core Update loop, which advances time in Impress.
    virtual absl::Status Update(FilamentHost* host, absl::Duration last_vsync,
                                absl::Duration next_vsync,
                                const UpdateStageFlags& update_flags) {
      return absl::OkStatus();
    }

    // Called after `Update` has completed and time has been advanced. This is
    // not called if `Update` is skipped.
    virtual absl::Status PostUpdate(FilamentHost* host) {
      return absl::OkStatus();
    }

    // Called before rendering. Verifies we should render another frame.
    virtual bool IsStillRendering(FilamentHost* host) { return false; }
    // Called before rendering.
    virtual absl::Status PreRender(FilamentHost* host) {
      return absl::OkStatus();
    }
    // Called during rendering, for rendering to render targets. If a client
    // wants to use a renderer object, they must override this.
    // TODO make this return void instead.
    virtual OptionalError OffscreenRender(FilamentHost* host,
                                          filament::Renderer* renderer) {
      return NoError();
    }
    // Call after main pass. For e.g. helper masks and secondary camera
    virtual OptionalError MultiPassRender() { return NoError(); }
    // Called after rendering. For e.g. saving out screenshots.
    virtual OptionalError PostRender(FilamentHost* host) { return NoError(); }
    // Called after ending the main render. Used to render to additional
    // views/windows.
    virtual OptionalError SecondaryViewRender(FilamentHost* host) {
      return NoError();
    }
    // Called after all frame actions. For e.g. waiting on render fences.
    virtual OptionalError PostFrame(FilamentHost* host) { return NoError(); }
    // Called at setup time to determine if we build out a UI context.
    virtual bool IsUiDesired(FilamentHost* host) { return false; }
    // Called for opt-in UI via imgui.
    virtual OptionalError UiRender(FilamentHost* host) { return NoError(); }
    // Called after rendering.  Returning true ensures a re-render next frame.
    virtual bool IsAnimating(FilamentHost* host) { return false; }
    // Called after rendering is complete.
    virtual OptionalError Cleanup(FilamentHost* host) { return NoError(); }
    virtual void NotifyLast(FilamentHost* host) {}
    // Called at setup time; the desired title text for our window.
    virtual std::string Title(FilamentHost* host) {
      return "FilamentHost.State";
    }
    // Called when the view becomes hidden.
    virtual OptionalError Pause() { return NoError(); }
    // Called when the view becomes visible.
    virtual OptionalError Resume() { return NoError(); }
    // Called at setup time; the desired dimensions for our window.
    virtual uint2 DesiredDimensions(FilamentHost* host) {
#if IMP_PLATFORM(DESKTOP)
      return kDefaultDesktopDimensions;
#else
      return kDefaultMobileDimensions;
#endif
    }
    // Called at setup time; the desired near/far planes for our window.
    virtual float2 DepthRange(FilamentHost* host) { return {0.05f, 5.0f}; }
    // Width/Height (tall pixels have a ratio < 1).  For e.g. ascii rendering.
    virtual float PixelAspectRatio(FilamentHost* host) { return 1.0f; }
    // Called at setup time; the desired vertical field of view for our window.
    virtual float VerticalFovDegrees(FilamentHost* host) { return 45.0f; }
    // TODO: iOS should call this on display orientation changes.
    virtual OptionalError SetDisplayRotation(FilamentHost* host,
                                             window::WindowRotation rotation) {
      return NoError();
    }
    // Called externally by the owner of the state with the updated dimensions
    // and subpixel ratio.
    virtual void OnResize(FilamentHost* host, uint2 dimensions, uint4 margins,
                          float2 subpixel_ratio) {}
    // Called in response to mouse input.
    virtual OptionalError OnMouseInput(FilamentHost* host,
                                       detail::MouseInput mouse_input) {
      return NoError();
    }
    // Called in response to a host-initiated file-load, e.g. via drag-and-drop.
    virtual OptionalError OnFileDrop(FilamentHost* host,
                                     absl::string_view path) {
      return NoError();
    }

    virtual filament::Engine::Config GetEngineConfig() const { return {}; }
    virtual filament::backend::FeatureLevel GetMaximumEngineFeatureLevel()
        const {
      return filament::backend::FeatureLevel::FEATURE_LEVEL_1;
    }
    virtual std::vector<FilamentFeatureFlag> GetFilamentFeatureFlags() const {
      return {};
    }
    virtual bool ShouldStartPaused() const { return false; }

    // When rendering using OpenGL, determines if a shared Gl context should be
    // passed into filament. This is necessary for ARCore and C9, but can be
    // turned off in some use cases where it isn't required.
    virtual bool ShouldUseSharedGlContext() const { return true; }

    virtual bool ShouldUseSystemFrameScheduledHandler() const { return false; }

    virtual bool ShouldUseSrgbSwapChain() const { return false; }

    virtual bool ShouldUseStencilSwapChain() const { return false; }

    virtual bool ShouldUseMsaaSwapChain() const { return false; }

    virtual bool ShouldUseTransparentSwapChain() const { return false; }

    // When true, the host will set the presentation time on the renderer.
    virtual bool ShouldSetPresentationTime() const { return false; }

    virtual filament::Engine::Backend GetPreferredBackend() const {
      return filament::Engine::Backend::DEFAULT;
    }
  };

  // Dev mode is optionally installed and operates via this abstract interface.
  struct DevModeExtension {
    using ImGuiCommand = std::function<void(void)>;

    virtual ~DevModeExtension() = default;
    // Manage any setup related work and acquire a pointer to the host.
    virtual absl::Status Setup(FilamentHost& host) = 0;
    // called prior to view being cleaned up to allow for any cleanup that
    // needs to happen before the view is destroyed.
    virtual void PreCleanup() = 0;
    // Dispose any extension specific resources.
    virtual void Cleanup() = 0;
    // Filters legacy MouseInput for FilamentHost.
    virtual bool TryConsumeMouseInput(const MouseInput& latest_input) = 0;
    // Advances time ahead of rendering.
    virtual void PreRender(absl::Duration previous_vsync,
                           absl::Duration next_vsync, bool force) = 0;
    // Creates a render target and sets it on the ImGui-specific view.
    virtual void ApplyTextureRenderTarget(filament::Texture* texture) {}
    // Returns true if the ImGui-specific view has a render target.
    virtual bool HasRenderTarget() { return false; }
    // Called during rendering. Used for rendering to render targets.
    virtual void OffscreenRender() {}
    // Renders the next frame.
    virtual void Render() = 0;
    // Informs the extension of the current viewport dimensions.
    virtual void UpdateCameraAndViewport(uint2 actual_size,
                                         float2 subpixel_ratio) = 0;

    // Queues an ImGui callback to be executed during RenderDev.
    virtual void QueueImGuiCommandBlock(ImGuiCommand cmd) = 0;
    // Renders dev mode UI.
    virtual void RenderDevModeUI() = 0;
    virtual void SetEnabled(bool is_enabled) = 0;
    virtual bool IsEnabled() = 0;

    // Gets called when FilamentHost::SetClipboardHandler gets called.
    virtual void OnClipboardHandlerChanged(
        ClipboardHandler* clipboard_handler) = 0;

    // Returns a pointer to the ImGuiRenderer interface.
    virtual ImGuiRenderer* GetImGuiRenderer() = 0;
  };

  struct RenderPassOptions {
    bool use_main_view_projection_matrix = true;
  };

  // Construct with a state object constructed by the caller.  We take over
  // ownership of it, and delete it at destruction time.
  explicit FilamentHost(std::unique_ptr<State>&& state)
      : state_(std::move(state)), monitor_(std::make_unique<Monitor>()) {}
  virtual ~FilamentHost() {}

  // Setup the host with an automatically setup backend based on how filament
  // was compiled. If using the NOOP backend, that must be set manually.
  OptionalError Setup(filament::Engine::Platform* platform = nullptr,
                      void* shared_gl_context = nullptr,
                      bool skip_color_grading = false);
  // Setup the host with the specified backend.
  // If skip_color_grading is true, Setup will not set a color grading. Instead
  // the default color grading for the engine will be used.
  OptionalError Setup(filament::Engine::Backend backend,
                      filament::Engine::Platform* platform = nullptr,
                      void* shared_gl_context = nullptr,
                      bool skip_color_grading = false);
  OptionalError Setup(filament::Engine* engine, filament::Renderer* renderer,
                      filament::View* view, filament::Scene* scene);
  OptionalError Cleanup();
  OptionalError Pause();
  OptionalError Resume();

  void SetSharedGlContext(void* shared_gl_context = nullptr);

  // Returns true if the filament engine is created and owned by this host.
  // Returns false if the filament engine is passed in from an external source.
  bool OwnsFilament();

  // Lifecycle queries.
  bool IsCleaningUp();

  // Returns true if the session is running in an XR environment.
  virtual bool IsInXr() const;

  // Pass through a void* representing the system resource we want to create a
  // swap chain with.  The real type is known by Engine::Platform.
  // Automatically sets the swap chain as the active swap chain.
  OptionalError CreateSwapChain(void* native_window, uint64_t flags = 0);

  // Creates a headless swap chain for running unit tests or other no-display.
  // Does not honor requests for sRGB or stencil swap chains.
  OptionalError CreateHeadlessSwapChain(uint32_t width, uint32_t height,
                                        uint64_t flags = 0);

  // Creates a new filament::SwapChain, adds it to an internal set of known
  // swap chains, and returns a pointer to it. SwapChains created through this
  // API can later be set as the active swap chain through SetActiveSwapChain().
  absl::StatusOr<const filament::SwapChain*> AddSwapChain(void* native_window,
                                                          uint64_t flags = 0);
  // Sets the active swap chain to one previously created with AddSwapChain().
  absl::Status SetActiveSwapChain(const filament::SwapChain* swap_chain);

  // Destroys all swap chains created by CreateSwapChain() & AddSwapChain().
  OptionalError DestroySwapChain();

  // Returns true if there is any active swap chain.
  bool HasSwapChain() const;

  // Returns true if the platform supports sRGB swapchains.
  bool IsSRGBSwapChainSupported();

  // Convenience function that returns true if IsSRGBSwapChainSupported() is
  // true _and_ the host state is requesting an sRGB swapchain.
  bool SwapChainWillBeSRGB();

  // TODO: iOS should call this on display orientation changes.
  OptionalError SetDisplayRotation(window::WindowRotation orientation);
  // Called by our venue when our display dimensions change.  'dimensions' is
  // in physical pixels in UI space; 'subpixel_ratio' is the UI-dependant
  // scale; e.g. on a retina display it is (2,2). margins is the left / right /
  // top / bottom margins applied.
  void Resize(uint2 pixel_dimensions, float2 subpixel_ratio,
              uint4 margins = {});
  uint2 GetDimensions() const { return dimensions_; }
  uint2 GetPixelDimensions() const { return pixel_dimensions_; }
  float2 GetSubpixelRatio() const { return subpixel_ratio_; }

  void EnsureNextRenderCompletes();

  // Returns true for the upcoming frame if EnsureNextRenderCompletes() was
  // called or if something has occurred that requires a new frame to be
  // rendered like resizing.
  bool IsNextRenderRequired() const;

  void SetFrameScheduledCallback(
      FrameScheduledCallback&& frame_scheduled_callback = {});
  void SetFrameCompletedCallback(
      FrameCompletedCallback&& frame_completed_callback = {});

  // Called right before rendering begins. Override this if you need something
  // to happen as late as possible in the frame loop before rendering begins.
  virtual absl::Status PreBeginRender() { return absl::OkStatus(); }

  absl::StatusOr<RenderResult> RenderNextFrame(absl::Duration previous_vsync,
                                               absl::Duration next_vsync);

  absl::Status UpdateNextFrame(
      absl::Duration previous_vsync, absl::Duration next_vsync,
      UpdateStageFlags* out_flags,
      absl::optional<absl::Duration>* out_time_until_retry = nullptr);

  // If something else is controlling the Filament render, call this before
  // rendering.
  absl::Status IsolatedPreRender(
      absl::Duration previous_vsync, absl::Duration next_vsync,
      Flags<RenderResultFlags>* out_flags,
      absl::optional<absl::Duration>* out_time_until_retry = nullptr,
      Flags<IsolatedPreRenderFlags> isolated_pre_render_flags = {});

  // If something else is controlling the Filament render, call this after
  // rendering.
  OptionalError IsolatedPostRender(Flags<RenderResultFlags>* out_flags);

  OptionalError QueueMouseInput(detail::MouseInput mouse_input);
  OptionalError OnFileDrop(absl::string_view path);
  // Force refresh of our projection matrix, for when e.g. VerticalFovDegrees
  // or DepthRange changes.
  void UpdateCamerasForWindow();
  void SetSampleCount(size_t sample_count);

  State* GetState() { return state_.get(); }
  const State* GetState() const { return state_.get(); }
  filament::Engine* GetEngine() { return engine_; }
  filament::Scene* GetScene() { return scene_; }
  filament::Renderer* GetRenderer() { return renderer_; }
  filament::View* GetView();
  filament::View* GetUiView();
  const Monitor* GetMonitor() const { return monitor_.get(); }
  Monitor* GetMonitor() { return monitor_.get(); }
  virtual filament::Engine::Config GetEngineConfig();

  using FramebufferHandler = std::function<void(BufferAccess, uint2)>;
  OptionalError GetRenderedImage(FramebufferHandler framebuffer_handler,
                                 bool clear_alpha = true, uint32_t stride = 0);

  // These functions are defined via filament_host_details_*.cc
  OptionalError SaveRenderedImage(const char* filename,
                                  bool clear_alpha = true);

  void CopyFrame(filament::SwapChain* destination_swap_chain,
                 const filament::Viewport& destination_viewport,
                 const filament::Viewport& source_viewport, uint32_t flags);

  // Queues an ImGui callback to be executed during RenderDev.
  void QueueImGuiCommandBlock(DevModeExtension::ImGuiCommand cmd);

  // Processes any queued ImGui.
  void ProcessImGuiCommands();

  absl::Status RegisterExtension(std::unique_ptr<DevModeExtension> extension);
  DevModeExtension* TryGetExtension();

  // Sets a hidden override to use for the editor camera, which is applied last-
  // minute (right before calling into filament::Renderer::render()).
  void SetEditorCameraOverride(PassKey<editor::EditorImpl> key,
                               filament::Camera* camera);

  void SetClipboardHandler(std::unique_ptr<ClipboardHandler> clipboard_handler);

  // Performs rendering by calling filament::Renderer::Render on the view using
  // the renderer provided by GetRenderer().
  //
  // Can be overridden to perform rendering in a special way. For instance, for
  // OpenXR this can be used to override how rendering occurs so that both the
  // left eye and the right eye are rendered with the correct camera settings.
  virtual void PerformRender(filament::View* view, RenderPassOptions options);

  void PerformRender(filament::View* view) {
    PerformRender(view, {.use_main_view_projection_matrix = true});
  }

  // Pause or resume the rendering thread.
  void SetPaused(bool paused);

  // Overwrite the vsync time used by Filament.  Optionally call as close as
  // possible to the beginning of work for each frame.
  void CaptureVsyncTime();

  // This should be enabled all the time, but making the behavior opt-in because
  // it caused (broken link).
  // TODO: Investigate why skipFrame causes crashes, after
  // resolving make it non-optional
  void SetCallSkipFrameWhenRenderingSkipped(
      bool call_skip_frame_when_rendering_skipped);

  // Updates flags for a pending call to createSwapChain() based on features
  // requested by State, if any.
  uint64_t UpdateSwapChainFlagsFromState(uint64_t flags) const;

  // Asserts that the current thread is the frame thread.
  //
  // The frame thread is the same thread that the ForegroundExecutor runs on.
  //
  // This is a little bit more reliable than checking that the CurrentExecutor
  // is the ForegroundExecutor, because if there are multiple Impress threads
  // and something is called on the wrong Impress thread, there may be a valid
  // but incorrect ForegroundExecutor on that thread.
  void CheckOnFrameThread() const;

  // Note that types that inherit from FilamentHost and override RenderNextFrame
  // will need to implement these.
  void SetPerformMainRender(bool should_draw) {
    should_perform_main_render_ = should_draw;
  }

  void SetPerformSecondaryViewRender(bool should_draw) {
    should_perform_secondary_view_render_ = should_draw;
  }

 protected:
  enum class LifeCycleState {
    kNone,
    kPreSetup,
    kSettingUp,
    kRunning,
    kPausing,
    kPaused,
    kResuming,
    kCleaningUp,
    kDead,
  };

  OptionalError InternalSetup();

  std::unique_ptr<State> state_;
  bool owns_filament_ = true;
  bool has_rendered_since_last_state_change_ = false;
  void* shared_gl_context_ = nullptr;
  filament::Engine* engine_ = nullptr;
  filament::Scene* scene_ = nullptr;
  filament::SwapChain* swap_chain_ = nullptr;
  RobinSet<filament::SwapChain*> swap_chains_;
  filament::Renderer* renderer_ = nullptr;
  LifeCycleState life_cycle_state_ = LifeCycleState::kNone;
  detail::FilamentView render_view_;
  absl::Duration legacy_time_cursor_ = absl::ZeroDuration();
  uint2 dimensions_;
  uint2 pixel_dimensions_;
  uint4 margins_;
  float2 subpixel_ratio_ = {1.0f, 1.0f};
  std::vector<detail::MouseInput> pending_mouse_inputs_;
  std::unique_ptr<Monitor> monitor_;
  std::unique_ptr<DevModeExtension> dev_mode_extension_;
  filament::Camera* editor_camera_override_ = nullptr;
  // May or may not be valid depending on whether we have a custom handler for
  // the specific platform the app is running on.
  std::unique_ptr<ClipboardHandler> clipboard_handler_;
  bool force_sync_on_resize_ = false;

  // Ensures the earliest timestamp captured by Filament is used each frame.
  // True indicates Filament has already captured the timestamp.
  // Set to False after each frame finishes (or skips) rendering.
  bool is_vsync_time_captured_since_last_frame_ = false;

  bool call_skip_frame_when_rendering_skipped_ = false;

  // Indicates whether we are currently in-between the calls to
  // filament::Renderer::beginFrame() and filament::Renderer::endFrame().
  //
  // This allows us to implement checks that ensure we don't call into APIs that
  // are disallowed during this time.
  bool is_within_filament_render_frame_ = false;

  bool should_perform_main_render_ = true;
  bool should_perform_secondary_view_render_ = true;

  std::thread::id frame_thread_id_;
};

}  // namespace window
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAMENT_HOST_H_
