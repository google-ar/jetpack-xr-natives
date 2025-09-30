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

#include <memory>
#include <utility>

#include "absl/strings/string_view.h"
#include "core/split_engine/split_engine_renderer.h"
#include "core/split_engine/split_engine_renderer_impl.h"
#include "imp.h"

namespace imp {
namespace split_engine {

constexpr float kFarClip = 250.0f;

class SplitEngineView : public View {
 public:
  SplitEngineView() = default;

 protected:
  void Setup() override;
};

void SplitEngineView::Setup() {
  // Match XR Platform settings.
  GetHost()->GetView()->setPostProcessingEnabled(false);
  GetCameraManager().GetCamera()->SetFarClip(kFarClip);

  auto renderer = std::make_unique<SplitEngineRendererImpl>(*this);
  GetRegistry().Register<imp::split_engine::SplitEngineRenderer>(
      std::move(renderer));
}

}  // namespace split_engine

// Tells the platform-code which View to instantiate.
const bool kIsCreateViewAssigned =
    client_api::SetCreateViewFn([](absl::string_view identifier) {
      return View::Create<split_engine::SplitEngineView>("SplitEngineView");
    });

}  // namespace imp
