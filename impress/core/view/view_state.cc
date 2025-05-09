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

#include "core/view/view_state.h"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/View.h"
#include "core/common/enum_flags.h"
#include "core/common/optional_error.h"
#include "core/common/pass_key.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/base_view.h"
#include "core/view/view_events.h"
#include "core/view/view_host.h"
#include "core/window/filament_host.h"
#include "core/window/window_rotation.h"
#include "mediapipe/framework/port/status_macros.h"

#if IMP_RUNTIME(DEV)
#include "dear_imgui/imgui.h"
#include "core/common/invocable.h"
#include "core/editor/editor.h"
#include "core/editor/editor_plugin.h"
#include "core/input/dev_mode_input_interceptor.h"
#include "core/window/dev_mode_extension.h"
#endif  // IMP_RUNTIME(DEV)

namespace imp {

using window::FilamentHost;

ViewState::ViewState(std::unique_ptr<imp::BaseView> view)
    : view_(std::move(view)),
      view_hooks_(view_->GetViewHooks({})),
      title_(view_->GetTitle()),
      is_multisampled_(false) {}

ViewState::~ViewState() = default;

OptionalError ViewState::Setup(FilamentHost* filament_host) {
  IMP_TRACE();
  ViewHost* host = static_cast<ViewHost*>(filament_host);
  filament::View* view = filament_host->GetView();

  view->setDynamicResolutionOptions(
      filament::View::DynamicResolutionOptions{.enabled = true});
  view->setAntiAliasing(filament::View::AntiAliasing::FXAA);
  view->setRenderQuality(
      filament::View::RenderQuality{filament::View::QualityLevel::MEDIUM});

  // Magic constants courtesy of sceneform/rendering/Renderer.java:72
  const float kCameraAperture = 4.0f;
  const float kCameraShutterSpeed = 1.0f / 30.0f;
  const float kCameraIso = 320;

  view->getCamera().setExposure(kCameraAperture, kCameraShutterSpeed,
                                kCameraIso);

  BaseView::SetSharedEngine(host->GetEngine());

#if IMP_RUNTIME(DEV)
  // If dev mode is set up at compile time, automatically install the default
  // dev mode extensions into the filament host if one already isn't registered.
  if (!host->TryGetExtension()) {
    MP_RETURN_IF_ERROR(
        host->RegisterExtension(imp::window::CreateDefaultDevModeExtension()));
    view_->GetInputManager().AddInterceptor(
        std::make_unique<DevModeInputInterceptor>(view_.get()));
  }
  // Delay Editor creation until dependencies have been set up.
  // Passing this logic in as an Invocable removes what would be a circular
  // dependency between the Editor implementation and the View.
  view_hooks_.OnHostSetup(
      DesiredDimensions(host), [this, host](bool is_app_sandbox_target) {
        editor::GetOrCreateEditor(view_.get(), host->CreateEditorPlugin(),
                                  is_app_sandbox_target);
      });
#else
  view_hooks_.OnHostSetup(DesiredDimensions(host), {});
#endif  // IMP_RUNTIME(DEV)

  return NoError();
}

void ViewState::OnHostCreated(FilamentHost* filament_host) {
  IMP_TRACE();
  view_hooks_.OnHostCreated(filament_host);
}

bool ViewState::IsStillRendering(FilamentHost* filament_host) { return true; }

bool ViewState::IsUiDesired(FilamentHost* filament_host) {
  return IMP_RUNTIME(DEV);
}

absl::Status ViewState::PreUpdate(
    FilamentHost* host, absl::Duration last_vsync, absl::Duration next_vsync,
    UpdateStageFlags* out_flags,
    absl::optional<absl::Duration>* out_time_until_retry) {
  IMP_TRACE();
  MP_RETURN_IF_ERROR(window::FilamentHost::State::PreUpdate(
      host, last_vsync, next_vsync, out_flags, out_time_until_retry));
  view_hooks_.OnHostPreUpdate(host, last_vsync, next_vsync, out_flags,
                              out_time_until_retry);
  return absl::OkStatus();
}

absl::Status ViewState::Update(FilamentHost* host, absl::Duration last_vsync,
                               absl::Duration next_vsync,
                               const UpdateStageFlags& update_flags) {
  IMP_TRACE();
  MP_RETURN_IF_ERROR(window::FilamentHost::State::Update(
      host, last_vsync, next_vsync, update_flags));
  view_hooks_.OnHostUpdate(host, last_vsync, next_vsync, update_flags);
  return absl::OkStatus();
}

absl::Status ViewState::PostUpdate(FilamentHost* host) {
  IMP_TRACE();
  MP_RETURN_IF_ERROR(window::FilamentHost::State::PostUpdate(host));
  view_hooks_.OnHostPostUpdate();
  return absl::OkStatus();
}

absl::Status ViewState::PreRender(window::FilamentHost* filament_host) {
  IMP_TRACE();
  MP_RETURN_IF_ERROR(window::FilamentHost::State::PreRender(filament_host));
  view_hooks_.OnHostPreRender(filament_host);
  return absl::OkStatus();
}

OptionalError ViewState::OffscreenRender(
    window::FilamentHost*, filament::Renderer* filament_renderer) {
  IMP_TRACE();
  view_hooks_.OnHostOffscreenRender(filament_renderer);
  return NoError();
}

OptionalError ViewState::MultiPassRender() {
  view_hooks_.OnHostMultiPassRender();
  return NoError();
}

OptionalError ViewState::PostRender(window::FilamentHost* filament_host) {
  IMP_TRACE();
  MP_RETURN_IF_ERROR(window::FilamentHost::State::PostRender(filament_host));
  view_hooks_.OnHostPostRender();
  return NoError();
}

OptionalError ViewState::SecondaryViewRender(
    window::FilamentHost* filament_host) {
  IMP_TRACE();
  MP_RETURN_IF_ERROR(
      window::FilamentHost::State::SecondaryViewRender(filament_host));
  view_hooks_.OnHostSecondaryViewRender();
  return NoError();
}

OptionalError ViewState::PostFrame(window::FilamentHost* filament_host) {
  IMP_TRACE();
  MP_RETURN_IF_ERROR(window::FilamentHost::State::PostFrame(filament_host));
  view_hooks_.OnHostPostFrame();
  return NoError();
}

OptionalError ViewState::UiRender(FilamentHost* filament_host) {
  IMP_TRACE();
  // Generate immediate commands.
  view_->RenderDev();
  return NoError();
}

OptionalError ViewState::Cleanup(FilamentHost* filament_host) {
  IMP_TRACE();
  view_hooks_.OnHostCleanup();

  // Reset the view pointer.
  // This is important to do during Cleanup so that the view can deallocate
  // any filament resources it's holding in its destructor before the
  // filament::Engine is destroyed without needing to manually clear
  // everything in View::Cleanup.
  view_.reset();

  return NoError();
}

std::string ViewState::Title(FilamentHost* filament_host) { return title_; }

OptionalError ViewState::SetDisplayRotation(
    window::FilamentHost* filament_host, window::WindowRotation orientation) {
  view_hooks_.OnHostSetDisplayRotation(orientation);
  return NoError();
}

void ViewState::OnResize(FilamentHost* filament_host, uint2 dimensions,
                         uint4 margins, float2 subpixel_ratio) {
  IMP_TRACE();
  ViewHost* host = static_cast<ViewHost*>(filament_host);
  // Subpixel ratios are >1 on desktop platforms with retina-style displays, or
  // modern android/iOS devices, and are fractional in some cases (zoomed page
  // in the wasm build). Detecting a sufficiently high DPI display disables
  // multisampling for the sake of memory pressure and performance.
  // They're == 1 on, for example, glinux workstations, and when run under test.
  // TODO: (broken link) - Remove this logic, as it currently clobbers whatever
  // the application specifies. This would be better handled at the application
  // level.
  constexpr bool is_mobile = IMP_PLATFORM(ANDROID) || IMP_PLATFORM(IOS);
  bool should_be_multisampled =
      !is_mobile && std::max(subpixel_ratio.x, subpixel_ratio.y) < 1.5f;

  if (should_be_multisampled != is_multisampled_) {
    if (should_be_multisampled) {
      host->SetSampleCount(4);
    } else {
      host->SetSampleCount(1);
    }

    is_multisampled_ = should_be_multisampled;
  }
  view_hooks_.OnHostResize(dimensions, margins, subpixel_ratio);
}

OptionalError ViewState::OnFileDrop(window::FilamentHost* filament_host,
                                    absl::string_view path) {
  IMP_TRACE();
  view_->GetDispatcher().Send(DropFileEvent(path));
  return NoError();
}

void ViewState::NotifyLast(FilamentHost* host) {
  IMP_TRACE();
  BaseView::SetSharedEngine(nullptr);
}
OptionalError ViewState::Pause() {
  IMP_TRACE();
  view_hooks_.Pause();
  return NoError();
}

OptionalError ViewState::Resume() {
  IMP_TRACE();
  view_hooks_.Resume();
  return NoError();
}

filament::Engine::Config ViewState::GetEngineConfig() const {
  return view_->GetEngineConfig();
}

filament::backend::FeatureLevel ViewState::GetMaximumEngineFeatureLevel()
    const {
  return view_->GetMaximumEngineFeatureLevel();
}

bool ViewState::ShouldStartPaused() const { return view_->ShouldStartPaused(); }

bool ViewState::ShouldUseSharedGlContext() const {
  return view_->ShouldUseSharedGlContext();
}

bool ViewState::ShouldUseSystemFrameScheduledHandler() const {
  return view_->GetConfig().use_system_frame_scheduled_handler.value_or(false);
}

}  // namespace imp
