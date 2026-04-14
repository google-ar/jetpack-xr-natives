// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/window/filament_host.h"

#include <stdbool.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>  // NOLINT: Need to use threads available in bazel.
#include <utility>

#include "absl/cleanup/cleanup.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "filament/filament/include/filament/Camera.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/SwapChain.h"
#include "filament/filament/include/filament/TransformManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/filament/include/filament/View.h"
#include "filament/filament/include/filament/Viewport.h"
#include "core/common/buffer_access.h"
#include "core/common/enum_flags.h"
#include "core/common/filament_engine_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/pass_key.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/monitor/monitor_helpers.h"
#include "core/monitor/scoped_duration_measurement.h"
#include "core/view/utils/proto/filament_feature_flag.proto.imp.h"
#include "core/window/clipboard/clipboard_handler.h"
#include "core/window/filament_host_input.h"
#include "core/window/projection_helpers.h"
#include "core/window/shared_host_state.h"
#include "core/window/window_rotation.h"
#include "mediapipe/framework/port/status_macros.h"

#if IMP_RUNTIME(DEV)
#include "core/common/file_helpers.h"
#include "stblib/stb_image_write.h"
#endif  // IMP_RUNTIME(DEV)

namespace imp::window {

using filament::Engine;

OptionalError FilamentHost::CreateSwapChain(void* native_window,
                                            uint64_t flags) {
  if (!owns_filament_) {
    return NoError();
  }
  if (!engine_) {
    return Error("no engine");
  }
  if (swap_chain_) {
    MP_RETURN_IF_ERROR(DestroySwapChain());
  }

  if (!native_window &&
      engine_->getBackend() != filament::Engine::Backend::NOOP) {
    return Error("Cannot create swap chain with null native_window");
  }

  flags = UpdateSwapChainFlagsFromState(flags);
  swap_chain_ = engine_->createSwapChain(native_window, flags);
  if (!swap_chain_) {
    return Error("Filament engine failed to create swap chain.");
  }

  swap_chains_.insert(swap_chain_);
  return NoError();
}

OptionalError FilamentHost::CreateHeadlessSwapChain(uint32_t width,
                                                    uint32_t height,
                                                    uint64_t flags) {
  if (!owns_filament_) {
    return NoError();
  }
  if (!engine_) {
    return Error("no engine");
  }
  if (swap_chain_) {
    MP_RETURN_IF_ERROR(DestroySwapChain());
  }
  flags = UpdateSwapChainFlagsFromState(flags);
  swap_chain_ = engine_->createSwapChain(width, height, flags);
  swap_chains_.insert(swap_chain_);
  return NoError();
}

absl::StatusOr<const filament::SwapChain*> FilamentHost::AddSwapChain(
    void* native_window, uint64_t flags) {
  IMP_TRACE();
  if (!engine_) {
    return absl::FailedPreconditionError("No Filament engine has been set.");
  }
  flags = UpdateSwapChainFlagsFromState(flags);
  filament::SwapChain* swap_chain =
      engine_->createSwapChain(native_window, flags);
  swap_chains_.insert(swap_chain);
  return swap_chain;
}

absl::Status FilamentHost::SetActiveSwapChain(
    const filament::SwapChain* swap_chain) {
  IMP_TRACE();
  if (!swap_chain) {
    swap_chain_ = nullptr;
    return absl::OkStatus();
  }

  // Note: const_cast is necessary because the underlying set stores non-const
  // pointers and an implicit cast away from const even to find is invalid.
  auto entry = swap_chains_.find(const_cast<filament::SwapChain*>(swap_chain));
  if (entry == swap_chains_.end()) {
    return absl::FailedPreconditionError(
        "Swap chain must have been created via "
        "FilamentHost::AddSwapChain(...)");
  }
  swap_chain_ = *entry;
  return absl::OkStatus();
}

OptionalError FilamentHost::DestroySwapChain() {
  if (!owns_filament_) {
    return NoError();
  }
  if (!swap_chain_ && swap_chains_.empty()) return Error("no swap chain");

  // If the swap chain is destroyed while rendering, it will cause filament to
  // crash. We FATAL in this case as an unrecoverable error instead of waiting
  // for filament to crash so we can provide a more useful error message and
  // identify why the swap chain was destroyed.
  if (is_within_filament_render_frame_) {
    IMP_LOG(imp::FATAL) << "Cannot destroy the swap chain while rendering.";
  }

  // Destroy all swap chains created through AddSwapChain().
  for (filament::SwapChain* swap_chain : swap_chains_) {
    engine_->destroy(swap_chain);
    if (swap_chain_ == swap_chain) {
      swap_chain_ = nullptr;
    }
  }
  swap_chains_.clear();

  // Destroy the active swap chain if it was not already destroyed.
  if (swap_chain_) {
    engine_->destroy(swap_chain_);
    swap_chain_ = nullptr;
  }
  return NoError();
}

bool FilamentHost::HasSwapChain() const { return swap_chain_ != nullptr; }

bool FilamentHost::IsSRGBSwapChainSupported() {
#if IMP_MATERIAL_API(METAL)
  // On iOS, Metal universally supports sRGB swapchains, but usage is controlled
  // at the IMPView level (by setting that format on its CAMetalLayer in
  // response to State::ShouldUseSrgbSwapChain()), instead of Filament's driver
  // controlling it. The CONFIG_SRGB_COLORSPACE flag has no effect in Metal.
  //
  // For that reason, MetalDriver::isSRGBSwapChainSupported() always returns
  // false (since the flag is ignored), even though this feature _is_ supported.
  return true;
#else
  return engine_ && filament::SwapChain::isSRGBSwapChainSupported(*engine_);
#endif
}

bool FilamentHost::SwapChainWillBeSRGB() {
  if (!engine_ || !state_) {
    return false;
  }

  // FL0 devices can't run post-processing, so they MUST attempt to use
  // sRGB swapchains if they're available. (See UpdateSwapChainFlagsFromState())
  const bool is_fl0 = (engine_->getActiveFeatureLevel() ==
                       filament::backend::FeatureLevel::FEATURE_LEVEL_0);
  const bool requested_srgb_swapchain =
      is_fl0 || state_->ShouldUseSrgbSwapChain();

  // Assume that if the driver supports sRGB, and the host app asked for it,
  // then it's being used.
  //
  // This is the only option at present; there's no API to ask the SwapChain
  // what its creation flags were, or what its texture format is.  Note that
  // this means it'll incorrectly return `false` if we create a swapchain with
  // CONFIG_SRGB_COLORSPACE manually set, versus using ShouldUseSrgbSwapChain().
  return IsSRGBSwapChainSupported() && requested_srgb_swapchain;
}

OptionalError FilamentHost::Setup(Engine::Platform* platform,
                                  void* shared_gl_context,
                                  bool skip_color_grading) {
  filament::Engine::Backend backend = state_->GetPreferredBackend();
  if (backend == filament::Engine::Backend::DEFAULT) {
#if IMP_MATERIAL_API(OPENGL)
    backend = filament::Engine::Backend::OPENGL;
#elif IMP_MATERIAL_API(VULKAN)
    backend = filament::Engine::Backend::VULKAN;
#elif IMP_MATERIAL_API(METAL)
    backend = filament::Engine::Backend::METAL;
#else
#error missing Material API declaration
#endif
  }

  return Setup(backend, platform, shared_gl_context, skip_color_grading);
}

OptionalError FilamentHost::Setup(Engine::Backend backend,
                                  Engine::Platform* platform,
                                  void* shared_gl_context,
                                  bool skip_color_grading) {
  frame_thread_id_ = std::this_thread::get_id();
  life_cycle_state_ = LifeCycleState::kPreSetup;
  auto& shared_state = SharedHostState::GetInstance();

  // Use the provided shared_gl_context, falling back to the one externally set
  // by calling SetSharedGlContext().
  MP_ASSIGN_OR_RETURN(
      engine_,
      shared_state.GetOrCreateEngine(
          backend, platform,
          shared_gl_context ? shared_gl_context : shared_gl_context_,
          state_->ShouldUseSharedGlContext(), GetEngineConfig(),
          state_->GetMaximumEngineFeatureLevel(),
          state_->GetFilamentFeatureFlags(), state_->ShouldStartPaused(), {}));

  shared_state.RegisterHost(this);
  renderer_ = engine_->createRenderer();
  renderer_->setClearOptions({.clearColor = {0, 0, 0, 0}, .clear = true});

  MP_RETURN_IF_ERROR(render_view_.Setup(engine_, "render", skip_color_grading));
  scene_ = engine_->createScene();
  MP_RETURN_IF_ERROR(InternalSetup());
#if IMP_PLATFORM(WASM)
  // WASM doesn't own its window so we don't ever get a request to create a swap
  // chain. Filament expects a swapchain even when we don't own one.
  MP_RETURN_IF_ERROR(CreateHeadlessSwapChain(2, 2, 0));
#endif  // IMP_PLATFORM(WASM)
  return NoError();
}

OptionalError FilamentHost::Setup(Engine* engine, filament::Renderer* renderer,
                                  filament::View* view,
                                  filament::Scene* scene) {
  frame_thread_id_ = std::this_thread::get_id();
  life_cycle_state_ = LifeCycleState::kPreSetup;
  assert(engine_ == nullptr);
  owns_filament_ = false;
  auto& shared_state = SharedHostState::GetInstance();
  shared_state.RegisterHost(this);
  engine_ = engine;
  renderer_ = renderer;
  scene_ = scene;

  MP_RETURN_IF_ERROR(render_view_.SetupShared(engine_, view, "render"));
  MP_RETURN_IF_ERROR(InternalSetup());
  return NoError();
}

OptionalError FilamentHost::InternalSetup() {
  IMP_TRACE();
  life_cycle_state_ = LifeCycleState::kSettingUp;

  render_view_.Get()->setVisibleLayers(0x4, 0x4);
  bool isAtLeastFeatureLevel1 =
      engine_->getActiveFeatureLevel() >=
      filament::backend::FeatureLevel::FEATURE_LEVEL_1;
  render_view_.Get()->setPostProcessingEnabled(isAtLeastFeatureLevel1);
  render_view_.Get()->setShadowingEnabled(isAtLeastFeatureLevel1);
  render_view_.Get()->setStencilBufferEnabled(
      state_->ShouldUseStencilSwapChain());
  if (owns_filament_) {
    render_view_.Get()->setScene(scene_);
  }

  MP_RETURN_IF_ERROR(state_->Setup(this));

  life_cycle_state_ = LifeCycleState::kRunning;

  //  Request a Histogram with lower bounds of 0 to 32 ms for actual render
  //  time.
  ScopedDurationMeasurement::AddHistogram(
      *GetMonitor(), imp::kFilamentFrameTiming, absl::ZeroDuration(),
      absl::Milliseconds(2), 16);

  return NoError();
}

bool FilamentHost::OwnsFilament() { return owns_filament_; }

OptionalError FilamentHost::GetRenderedImage(
    FilamentHost::FramebufferHandler framebuffer_handler, bool clear_alpha,
    uint32_t stride) {
  constexpr auto kChannels = 4;  // RGBA
  struct Packet {
    BufferAccess storage;
    FramebufferHandler framebuffer_handler;
    uint2 dimensions;
    bool clear_alpha;
    // The alpha channel in a rendered image is not user data; clear to opaque.
    void ClearAlpha() {
      uint8_t* rgba_begin = const_cast<uint8_t*>(storage.Data());
      uint8_t* rgba_end = rgba_begin + dimensions.x * dimensions.y * kChannels;
      for (uint8_t* rgba = rgba_begin; rgba != rgba_end; rgba += kChannels) {
        rgba[3] = 0xff;
      }
    }
  };

  const size_t framebuffer_size =
      (stride ? stride : pixel_dimensions_.x) * pixel_dimensions_.y * kChannels;
  Packet* packet =
      new Packet{.framebuffer_handler = std::move(framebuffer_handler),
                 .dimensions = pixel_dimensions_,
                 .clear_alpha = clear_alpha};
  uint8_t* framebuffer_data =
      BufferAccess::Create(framebuffer_size, &packet->storage);

  // Submit request to have back_buffer_contents filled in.
  renderer_->readPixels(
      0, 0, pixel_dimensions_.x, pixel_dimensions_.y,
      filament::backend::PixelBufferDescriptor(
          framebuffer_data, framebuffer_size,
          filament::backend::PixelDataFormat::RGBA,
          filament::backend::PixelDataType::UBYTE, 1, 0, 0, stride,
          [](void* buffer, size_t size, void* user) {
            auto* packet = reinterpret_cast<Packet*>(user);
            // Sanity checks.
            assert(packet->storage.Data() == buffer);
            assert(packet->storage.Size() == size);

            if (packet->clear_alpha) {
              packet->ClearAlpha();
            }

            packet->framebuffer_handler(std::move(packet->storage),
                                        packet->dimensions);
            delete packet;
          },
          packet));

  return NoError();
}

// A CallbackHandler that simply calls the callback on the same thread that
// calls post. When passing a custom CallbackHandler to
// setFrameCompletedCallback or setFrameScheduledCallback, this is guaranteed to
// not be the main Filament thread.
class FrameCallbackHandler : public filament::backend::CallbackHandler {
 public:
  void post(void* user,
            filament::backend::CallbackHandler::Callback callback) override {
    callback(user);
  };

  ~FrameCallbackHandler() override = default;
};

void FilamentHost::SetFrameScheduledCallback(
    FrameScheduledCallback&& frame_scheduled_callback) {
  if (state_->ShouldUseSystemFrameScheduledHandler()) {
    swap_chain_->setFrameScheduledCallback(
        nullptr, std::move(frame_scheduled_callback),
        filament::SwapChain::CALLBACK_DEFAULT_USE_METAL_COMPLETION_HANDLER);
  } else {
    static FrameCallbackHandler filamentCallbackHandler;
    swap_chain_->setFrameScheduledCallback(&filamentCallbackHandler,
                                           std::move(frame_scheduled_callback));
  }
}

void FilamentHost::SetFrameCompletedCallback(
    FrameCompletedCallback&& frame_completed_callback) {
  swap_chain_->setFrameCompletedCallback(nullptr,
                                         std::move(frame_completed_callback));
}

absl::StatusOr<FilamentHost::RenderResult> FilamentHost::RenderNextFrame(
    absl::Duration previous_vsync, absl::Duration next_vsync) {
  IMP_PROFILE_START_FRAME();
  IMP_TRACE();

  RenderResult result;
  if (!owns_filament_) {
    IMP_LOG(imp::FATAL) << "FilamentHost doesn't own Filament";
    return Error("RenderNextFrame failed");
  }

  if (!swap_chain_) {
    return Error("Cannot call RenderNextFrame without a swap chain");
  }

  CheckOnFrameThread();

  // Filament requests the earliest time possible in the frame, do not overwrite
  // the timestamp if it has already been captured.
  if (!is_vsync_time_captured_since_last_frame_) {
    CaptureVsyncTime();
  }

  MP_RETURN_IF_ERROR(IsolatedPreRender(previous_vsync, next_vsync, &result.flags,
                                    &result.time_until_retry));

  // IsolatedPreRender indicated that rendering should be skipped this frame
  // so we return early.
  //
  // Don't skip if rendering this frame is required. This is a last ditch
  // effort, IsolatedPreRender should not set kSkippedRender in the first place
  // if rendering is required.
  if ((result.flags.Test(RenderResultFlags::kSkippedRender) &&
       !IsNextRenderRequired()) ||
      !swap_chain_) {
    is_vsync_time_captured_since_last_frame_ = false;
    if (call_skip_frame_when_rendering_skipped_) {
      renderer_->skipFrame();
    }

    if (!swap_chain_) {
      IMP_LOG(imp::ERROR) << "The SwapChain was destroyed while the frame was updating.";
    }

    return result;
  }

  MP_RETURN_IF_ERROR(PreBeginRender());

  {
    IMP_TRACE_NAME("FilamentHost::FilamentRenderPass");
    ScopedDurationMeasurement filament_frame_duration(GetMonitor(),
                                                      kFilamentFrameTiming);
    // beginFrame returns a 'should' result (in terms of minimizing latency
    // between the CPU and GPU), but under some circumstances (like after a
    // resize) we ignore the suggestion and render.

    // vsynctime is 0 here because the setVsyncTime api is used instead.
    bool should_render_frame = false;

    {
      IMP_TRACE_NAME("Renderer::BeginFrame");
      should_render_frame = renderer_->beginFrame(swap_chain_, 0);
    }

    // beginFrame consumes the captured vsync timing, so reset it.
    is_vsync_time_captured_since_last_frame_ = false;

    if (!should_render_frame && has_rendered_since_last_state_change_) {
      result.flags |= RenderResultFlags::kSkippedRender;
      filament_frame_duration.CancelSample();
    } else {
      is_within_filament_render_frame_ = true;
      absl::Cleanup render_frame_cleanup = [this] {
        is_within_filament_render_frame_ = false;
      };

      MP_RETURN_IF_ERROR(state_->OffscreenRender(this, renderer_));

      if (dev_mode_extension_) {
        dev_mode_extension_->OffscreenRender();
      }

      filament::Camera* camera = &GetView()->getCamera();

      if (editor_camera_override_) {
        GetView()->setCamera(editor_camera_override_);
      }

      if (should_perform_main_render_) {
        PerformRender(render_view_.Get());
      }

      GetView()->setCamera(camera);

      MP_RETURN_IF_ERROR(state_->MultiPassRender());

      if (dev_mode_extension_) {
        dev_mode_extension_->Render();
      }

      MP_RETURN_IF_ERROR(state_->PostRender(this));

      if (state_->ShouldSetPresentationTime()) {
        renderer_->setPresentationTime(absl::ToInt64Nanoseconds(next_vsync));
      }

      {
        IMP_TRACE_NAME("Renderer::EndFrame");
        renderer_->endFrame();
      }

      if (should_perform_secondary_view_render_) {
        MP_RETURN_IF_ERROR(state_->SecondaryViewRender(this));
      }

      if (state_->IsAnimating(this)) {
        result.flags |= RenderResultFlags::kIsAnimating;
      }
      if (!has_rendered_since_last_state_change_) {
        has_rendered_since_last_state_change_ = true;
      }
    }
  }

#if IMP_PLATFORM(WASM)
  // The wasm build is single threaded, so no render thread, so pump manually.
  if (engine_) {
    IMP_TRACE_NAME("Engine::Execute");
    engine_->execute();
  }
#endif  // IMP_PLATFORM(WASM)

  MP_RETURN_IF_ERROR(state_->PostFrame(this));

  return result;
}

absl::Status FilamentHost::UpdateNextFrame(
    absl::Duration previous_vsync, absl::Duration next_vsync,
    UpdateStageFlags* out_flags,
    absl::optional<absl::Duration>* out_time_until_retry) {
  IMP_TRACE();

  MP_RETURN_IF_ERROR(state_->PreUpdate(this, previous_vsync, next_vsync, out_flags,
                                    out_time_until_retry));

  MP_RETURN_IF_ERROR(state_->Update(this, previous_vsync, next_vsync, *out_flags));

  if (!out_flags->HasFlag(UpdateStageFlags::kSkipUpdate) &&
      !out_flags->HasFlag(UpdateStageFlags::kSkipFrame)) {
    MP_RETURN_IF_ERROR(state_->PostUpdate(this));
  }

  return absl::OkStatus();
}

OptionalError FilamentHost::IsolatedPreRender(
    absl::Duration previous_vsync, absl::Duration next_vsync,
    Flags<RenderResultFlags>* out_flags,
    absl::optional<absl::Duration>* out_time_until_retry,
    Flags<IsolatedPreRenderFlags> isolated_pre_render_flags) {
  CheckOnFrameThread();
  IMP_TRACE();
  *out_flags = {};
  // We need to send mouse input before beginFrame so that uniform buffer
  // updates generated by mouse input get submitted before render.
  if (!pending_mouse_inputs_.empty()) {
    for (detail::MouseInput& input : pending_mouse_inputs_) {
      if (dev_mode_extension_ &&
          dev_mode_extension_->TryConsumeMouseInput(input)) {
        continue;
      }

      MP_RETURN_IF_ERROR(state_->OnMouseInput(this, input))
          << "Handling Mouse Input";
    }
    pending_mouse_inputs_.clear();
  }

  // TODO Move away from using out-params
  UpdateStageFlags update_flags = {};
  MP_RETURN_IF_ERROR(UpdateNextFrame(previous_vsync, next_vsync, &update_flags,
                                  out_time_until_retry));

  // TODO Remove this after migrating users to kSkipAdvance
  if (update_flags.HasFlag(UpdateStageFlags::kSkipFrame)) {
    *out_flags |= RenderResultFlags::kSkippedRender;
  }

  auto status = state_->PreRender(this);

  if (dev_mode_extension_ && dev_mode_extension_->IsEnabled() &&
      !out_flags->Test(RenderResultFlags::kSkippedRender) &&
      !isolated_pre_render_flags.Test(
          IsolatedPreRenderFlags::kNeverRenderDevMode)) {
    dev_mode_extension_->PreRender(
        previous_vsync, next_vsync,
        (isolated_pre_render_flags.Test(
            IsolatedPreRenderFlags::kAlwaysRenderDevMode)) ||
            !has_rendered_since_last_state_change_);
    dev_mode_extension_->RenderDevModeUI();
  }

  return status;
}

OptionalError FilamentHost::IsolatedPostRender(
    Flags<RenderResultFlags>* out_flags) {
  CheckOnFrameThread();
  IMP_TRACE();
  *out_flags = {};

  MP_RETURN_IF_ERROR(state_->PostRender(this));
  if (state_->IsAnimating(this)) {
    *out_flags |= RenderResultFlags::kIsAnimating;
  }

  return NoError();
}

OptionalError FilamentHost::Cleanup() {
  switch (life_cycle_state_) {
    case LifeCycleState::kNone:
    case LifeCycleState::kPreSetup:
    case LifeCycleState::kSettingUp:
    case LifeCycleState::kCleaningUp:
    case LifeCycleState::kDead:
      return Error(
          "FilamentHost::Cleanup called while the host is in an invalid state "
          "(%d). Skipping cleanup.",
          life_cycle_state_);
    default:
      break;
  }

  CheckOnFrameThread();

  life_cycle_state_ = LifeCycleState::kCleaningUp;

  FlushEngineAndWait(engine_);

  OptionalError cleanup_result = state_->Cleanup(this);
  if (engine_) {
    render_view_.Cleanup(engine_);
    if (dev_mode_extension_) {
      dev_mode_extension_->Cleanup();
    }

    if (owns_filament_) {
      if (renderer_) engine_->destroy(renderer_);
      if (swap_chain_) engine_->destroy(swap_chain_);
      if (scene_) engine_->destroy(scene_);
    }

#if IMP_PLATFORM(WASM) && !defined(__EMSCRIPTEN_PTHREADS__)
    SharedHostState::GetInstance().RequestSynchronousShutdown();
#endif

    // Note: If we're the last host, this destroys the engine.
    if (SharedHostState::GetInstance().UnregisterHostAndReturnIsLast(this)) {
      state_->NotifyLast(this);
    }

    renderer_ = nullptr;
    swap_chain_ = nullptr;
    scene_ = nullptr;
    engine_ = nullptr;
  }

  life_cycle_state_ = LifeCycleState::kDead;

  return cleanup_result;
}

OptionalError FilamentHost::Pause() {
  CheckOnFrameThread();

  life_cycle_state_ = LifeCycleState::kPausing;
  auto result = state_->Pause();
  life_cycle_state_ = LifeCycleState::kPaused;
  return result;
}

OptionalError FilamentHost::Resume() {
  CheckOnFrameThread();

  life_cycle_state_ = LifeCycleState::kResuming;
  auto result = state_->Resume();
  life_cycle_state_ = LifeCycleState::kRunning;
  has_rendered_since_last_state_change_ = false;
  return result;
}

void FilamentHost::SetSharedGlContext(void* shared_gl_context) {
  shared_gl_context_ = shared_gl_context;
}

bool FilamentHost::IsCleaningUp() {
  return life_cycle_state_ == LifeCycleState::kCleaningUp;
}

bool FilamentHost::IsInXr() const { return false; }

OptionalError FilamentHost::SetDisplayRotation(
    window::WindowRotation orientation) {
  MP_RETURN_IF_ERROR(state_->SetDisplayRotation(this, orientation));
  has_rendered_since_last_state_change_ = false;
  return NoError();
}

void FilamentHost::Resize(uint2 pixel_dimensions, float2 subpixel_ratio,
                          uint4 margins) {
  CheckOnFrameThread();

  dimensions_ = uint2{pixel_dimensions / subpixel_ratio};
  subpixel_ratio = float2{pixel_dimensions} / float2{dimensions_};
  // Since we are converting to uint when computing dimensions, it is possible
  // to get 0 width / height which can lead to assertion failures down the line
  // when computing aspect ratio. Ensure that they are at least 1 if the
  // corresponding pixel dimension is greater than 0.
  // See (broken link) for more info.
  if (dimensions_.x == 0 && pixel_dimensions.x > 0) {
    dimensions_.x = 1;
  }
  if (dimensions_.y == 0 && pixel_dimensions.y > 0) {
    dimensions_.y = 1;
  }
  margins_ = margins;
  subpixel_ratio_ = subpixel_ratio;
  pixel_dimensions_ = pixel_dimensions;

  // Sample the state and reconstruct viewport and projection matrix.
  UpdateCamerasForWindow();

  // Trigger resize event after the viewport has been updated.
  state_->OnResize(this, dimensions_, margins, subpixel_ratio);

  has_rendered_since_last_state_change_ = false;
}

void FilamentHost::EnsureNextRenderCompletes() {
  has_rendered_since_last_state_change_ = false;
}

bool FilamentHost::IsNextRenderRequired() const {
  return !has_rendered_since_last_state_change_;
}

filament::View* FilamentHost::GetView() { return render_view_.Get(); }

OptionalError FilamentHost::QueueMouseInput(detail::MouseInput mouse_input) {
  pending_mouse_inputs_.push_back(std::move(mouse_input));
  return NoError();
}

OptionalError FilamentHost::OnFileDrop(absl::string_view path) {
  MP_RETURN_IF_ERROR(state_->OnFileDrop(this, path));
  return NoError();
}

void FilamentHost::UpdateCamerasForWindow() {
  int32_t offset_left = margins_.x * subpixel_ratio_.x;
  int32_t offset_bottom = margins_.w * subpixel_ratio_.y;
  uint32_t width = pixel_dimensions_.x;
  uint32_t height = pixel_dimensions_.y;
  const auto viewport =
      filament::Viewport{offset_left, offset_bottom, width, height};
  if (owns_filament_) {
    render_view_.Get()->setViewport(viewport);
    if (dev_mode_extension_) {
      dev_mode_extension_->UpdateCameraAndViewport(pixel_dimensions_,
                                                   subpixel_ratio_);
    }
  }
}

void FilamentHost::SetSampleCount(size_t sample_count) {
  render_view_.Get()->setSampleCount(sample_count);
}

OptionalError FilamentHost::SaveRenderedImage(const char* filename,
                                              bool clear_alpha) {
#if IMP_PLATFORM(WASM)
  return Error("SaveRenderedImage is not available on this platform");
#elif IMP_RUNTIME(DEV)
  std::string filename_storage = filename;
  MP_RETURN_IF_ERROR(GetRenderedImage(
      [filename_storage](BufferAccess access, uint2 dimensions) {
        if (stbi_write_png_to_func(
                [](void* context, void* data, int size) {
                  char* filename = reinterpret_cast<char*>(context);
                  BufferAccess access =
                      BufferAccess::Wrap(const_cast<const uint8_t*>(
                                             reinterpret_cast<uint8_t*>(data)),
                                         static_cast<size_t>(size));
                  if (auto status = SaveBinary(filename, access);
                      !status.ok()) {
                    IMP_LOG(imp::ERROR) << status;
                    return;
                  }
                  IMP_LOG(imp::INFO)
                      << "Saved '" << filename << "' (" << size << " bytes)";
                },
                const_cast<char*>(filename_storage.c_str()), dimensions.x,
                dimensions.y, 4, access.Data(), dimensions.x * 4) == 0) {
          IMP_LOG(imp::ERROR) << "Failed to save rendered image to '" << filename_storage
                     << "'";
        }
      },
      clear_alpha));
  return NoError();
#else
  return Error("SaveRenderedImage disabled in ship builds");
#endif
}

void FilamentHost::CopyFrame(filament::SwapChain* destination_swap_chain,
                             const filament::Viewport& destination_viewport,
                             const filament::Viewport& source_viewport,
                             uint32_t flags) {
  renderer_->copyFrame(destination_swap_chain, destination_viewport,
                       source_viewport, flags);
}

void FilamentHost::QueueImGuiCommandBlock(DevModeExtension::ImGuiCommand cmd) {
  if (dev_mode_extension_ && dev_mode_extension_->IsEnabled()) {
    dev_mode_extension_->QueueImGuiCommandBlock(std::move(cmd));
  }
}

absl::Status FilamentHost::RegisterExtension(
    std::unique_ptr<DevModeExtension> extension) {
  DevModeExtension* extension_ptr = extension.get();

  switch (life_cycle_state_) {
    case LifeCycleState::kNone:
      // Ideal case
      break;
    case LifeCycleState::kSettingUp: {
      MP_RETURN_IF_ERROR(extension_ptr->Setup(*this));
      break;
    }
    case LifeCycleState::kRunning:
    case LifeCycleState::kPreSetup:
    case LifeCycleState::kPausing:
    case LifeCycleState::kPaused:
    case LifeCycleState::kResuming: {
      MP_RETURN_IF_ERROR(extension_ptr->Setup(*this));
      extension_ptr->UpdateCameraAndViewport(pixel_dimensions_,
                                             subpixel_ratio_);
      break;
    }
    case LifeCycleState::kCleaningUp:
    case LifeCycleState::kDead: {
      return absl::InternalError("Ignoring Extension (registered too late)");
    }
  }

  dev_mode_extension_ = std::move(extension);
  return absl::OkStatus();
}

FilamentHost::DevModeExtension* FilamentHost::TryGetExtension() {
  return dev_mode_extension_.get();
}

void FilamentHost::SetEditorCameraOverride(PassKey<editor::EditorImpl> key,
                                           filament::Camera* camera) {
  editor_camera_override_ = camera;
}

void FilamentHost::SetClipboardHandler(
    std::unique_ptr<ClipboardHandler> clipboard_handler) {
  clipboard_handler_ = std::move(clipboard_handler);
  if (dev_mode_extension_) {
    dev_mode_extension_->OnClipboardHandlerChanged(clipboard_handler_.get());
  }
}

void FilamentHost::PerformRender(filament::View* view,
                                 RenderPassOptions options) {
  filament::Camera& camera = view->getCamera();
  mat4 camera_model_matrix;
  mat4 camera_projection_matrix;
  float camera_near_clip;
  float camera_far_clip;
  if (options.projection_quad.has_value()) {
    // Save the camera's original matrices.
    camera_model_matrix = camera.getModelMatrix();
    camera_projection_matrix = camera.getProjectionMatrix();
    camera_near_clip = camera.getNear();
    camera_far_clip = camera.getCullingFar();

    // Adjust the viewport to match a portal (aka a projection quad) exactly.
    // modifies the projection and view matrices.
    AimCameraToFitQuad(engine_, &view->getCamera(),
                       options.projection_quad.value());
  }

  renderer_->render(view);

  if (options.projection_quad.has_value()) {
    // Restore the camera's original matrices.
    camera.setModelMatrix(camera_model_matrix);
    camera.setCustomProjection(camera_projection_matrix, camera_near_clip,
                               camera_far_clip);
  }
}

filament::Engine::Config FilamentHost::GetEngineConfig() {
  return state_->GetEngineConfig();
}

void FilamentHost::SetPaused(bool paused) {
  CheckOnFrameThread();

  engine_->setPaused(paused);
}

void FilamentHost::CaptureVsyncTime() {
  is_vsync_time_captured_since_last_frame_ = true;
  // Note: Uses the setVsyncTime API in Filament to ensure right clock is used.
  // The vsync time used by Filament has different meaning and runs on a
  // different clock than the times used elsewhere.  Do not assume this time can
  // used interchangeably with other numbers called vsync time.
  renderer_->setVsyncTime(renderer_->getEngine()->getSteadyClockTimeNano());
}

void FilamentHost::SetCallSkipFrameWhenRenderingSkipped(
    bool call_skip_frame_when_rendering_skipped) {
  call_skip_frame_when_rendering_skipped_ =
      call_skip_frame_when_rendering_skipped;
}

uint64_t FilamentHost::UpdateSwapChainFlagsFromState(uint64_t flags) const {
  if (engine_ && engine_->getActiveFeatureLevel() ==
                     filament::backend::FeatureLevel::FEATURE_LEVEL_0) {
    // Enable SRGB output for feature level 0 as it does not support full
    // postprocessing.
    flags |= filament::SwapChain::CONFIG_SRGB_COLORSPACE;
  }
  if (state_) {
    if (state_->ShouldUseSrgbSwapChain()) {
      flags |= filament::SwapChain::CONFIG_SRGB_COLORSPACE;
    }
    if (state_->ShouldUseStencilSwapChain()) {
      flags |= filament::SwapChain::CONFIG_HAS_STENCIL_BUFFER;
    }
    if (state_->ShouldUseMsaaSwapChain()) {
      flags |= filament::SwapChain::CONFIG_MSAA_4_SAMPLES;
    }
    if (state_->ShouldUseTransparentSwapChain()) {
      flags |= filament::SwapChain::CONFIG_TRANSPARENT;
    }
  }
  return flags;
}

void FilamentHost::CheckOnFrameThread() const {
  
}

void FilamentHost::UpdateStageFlags::SetFlag(UpdateStageFlag flag) {
  flags_ |= flag;
}

void FilamentHost::UpdateStageFlags::UnsetFlag(UpdateStageFlag flag) {
  flags_ &= ~flag;
}

bool FilamentHost::UpdateStageFlags::HasFlag(UpdateStageFlag flag) const {
  return flags_ & flag;
}

}  // namespace imp::window
