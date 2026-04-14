// Copyright 2026 Google LLC
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

#include "core/editor/widgets/viewport/viewport_widget.h"

#include <cstdint>

#include "absl/cleanup/cleanup.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/editor/widgets/viewport/viewport_render_target.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"
#include "core/view/utils/device.h"

namespace imp::editor {

namespace {

constexpr ImVec2 kUv0 = ImVec2(0.0f, 1.0f);
constexpr ImVec2 kUv1 = ImVec2(1.0f, 0.0f);

}  // namespace

ViewportWidget::ViewportWidget(BaseView& view,
                               ViewportRenderTarget* viewport_render_target)
    : view_(view), viewport_render_target_(viewport_render_target) {}

absl::string_view ViewportWidget::GetName() const {
  return kViewportWindowName;
}

void ViewportWidget::DrawImGui() {
  // Setting padding to 0 to use full window space for image.
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
  const absl::Cleanup pop_window_padding = ([] { ImGui::PopStyleVar(); });

  const ImVec2 viewport_panel_size = ImGui::GetContentRegionAvail();

  if (!viewport_render_target_ || viewport_panel_size.x <= 0 ||
      viewport_panel_size.y <= 0) {
    return;
  }

  const float2 pixel_ratio = view_.GetDevice().GetPhysicalPixelRatio();

  viewport_render_target_->SetSize(
      {static_cast<uint32_t>(viewport_panel_size.x * pixel_ratio.x),
       static_cast<uint32_t>(viewport_panel_size.y * pixel_ratio.y)});

  ImGui::Image(viewport_render_target_->GetColorTexture(), viewport_panel_size,
               kUv0, kUv1);
}

}  // namespace imp::editor
