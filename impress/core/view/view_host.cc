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

#include "core/view/view_host.h"

#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Fence.h"
#include "filament/filament/include/filament/View.h"
#include "core/async/executor.h"
#include "core/common/filament_engine_helpers.h"
#include "core/config.h"
#include "core/view/view_state.h"
#include "core/window/filament_host.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

ViewHost::ViewHost(std::unique_ptr<imp::BaseView> view)
    : window::FilamentHost(std::make_unique<ViewState>(std::move(view))) {
  ViewState* view_state = static_cast<ViewState*>(GetState());
  view_state->OnHostCreated(this);
}

imp::BaseView* ViewHost::GetView() {
  auto view_state = static_cast<ViewState*>(GetState());
  return view_state->GetView();
}

const imp::BaseView* ViewHost::GetView() const {
  auto view_state = static_cast<const ViewState*>(GetState());
  return view_state->GetView();
}

void ViewHost::DrainAllExecutorsForTest() {
  bool work_to_do = true;
  while (work_to_do) {
    work_to_do = imp::Executor::BackgroundExecutor()->Pump(true);
    work_to_do |= imp::Executor::ForegroundExecutor()->Pump(true);
  }
}

absl::Status ViewHost::StaticRenderForTest() {
  // First, override some rendering settings to try to guarantee that the render
  // produces consistent results between runs.
  filament::View* filament_view =
      static_cast<imp::window::FilamentHost*>(this)->GetView();
  // Ensure relatively consistent rendering
  filament_view->setDynamicResolutionOptions(
      filament::View::DynamicResolutionOptions{.enabled = false});
  filament_view->setAntiAliasing(filament::View::AntiAliasing::NONE);
  filament_view->setDithering(filament::View::Dithering::NONE);
  // filament_view->setDepthPrepass(filament::View::DepthPrepass::DISABLED);
  filament_view->setSampleCount(1);
  // filament_view->setClearTargets(true, true, true);
  filament_view->setAmbientOcclusion(filament::View::AmbientOcclusion::NONE);

  // Then, render a frame without any time elapsing and flush the engine.
  // Keep looping until the frame isn't skipped, which can happen if the
  // filament render thread is running behind which is likely in a test where
  // we are simulating multiple frames while blocking.
  auto* engine = imp::BaseView::GetSharedEngine();
  if (!engine) {
    return absl::InternalError("Tried to flush with no engine");
  }

  // Ensure the next render isn't skipped.
  EnsureNextRenderCompletes();

  // Perform the render.
  FilamentHost::RenderResult render_result;
  MP_ASSIGN_OR_RETURN(render_result,
                   RenderNextFrame(absl::ZeroDuration(), absl::ZeroDuration()));

  if (render_result.flags & FilamentHost::RenderResultFlags::kSkippedRender) {
    return absl::InternalError("Skipped render");
  }

  // Wait until the GPU work has been kicked off.
  // This is necessary to do before calling FlushEngineAndWait to prevent
  // FlushEngineAndWait from timing out when a lot of work is enqueued, which is
  // typical in tests since everything is setup synchronously.
  // Wasm is single-threaded so we need to sip the fence otherwise it will hang.
#if !defined(__EMSCRIPTEN__) && !defined(FILAMENT_SINGLE_THREADED)
  filament::Fence* fence = GetEngine()->createFence();
  fence->wait();
  GetEngine()->destroy(fence);
#endif

  // Wait until the render is completed on the GPU.
  imp::FlushEngineAndWait(engine);

  return absl::OkStatus();
}

#if IMP_RUNTIME(DEV)
std::unique_ptr<imp::editor::EditorPlugin> ViewHost::CreateEditorPlugin() {
  return GetView()->CreateEditorPlugin();
}
#endif

}  // namespace imp
