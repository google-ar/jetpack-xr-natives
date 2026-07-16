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

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/editor/editor.h"
#include "core/editor/widgets/viewport/viewport_helpers.h"
#include "core/editor/widgets/viewport/viewport_render_target.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"

namespace imp::editor {

namespace {

constexpr ImVec2 kUv0 = ImVec2(0.0f, 1.0f);
constexpr ImVec2 kUv1 = ImVec2(1.0f, 0.0f);

}  // namespace

ViewportWidget::ViewportWidget(Editor& editor, BaseView& view,
                               ViewportRenderTarget* viewport_render_target)
    : editor_(editor),
      view_(view),
      viewport_render_target_(viewport_render_target) {}

absl::string_view ViewportWidget::GetName() const {
  return kViewportWindowName;
}

void ViewportWidget::DrawImGui() {
  if (!viewport_render_target_) return;

  const ImVec2 viewport_panel_size = ImGui::GetContentRegionAvail();

  if (viewport_panel_size.x <= 0 || viewport_panel_size.y <= 0) return;

  const float2 pixel_ratio = editor::GetPhysicalPixelRatio(view_);

  viewport_render_target_->SetSize(
      {static_cast<uint32_t>(viewport_panel_size.x * pixel_ratio.x),
       static_cast<uint32_t>(viewport_panel_size.y * pixel_ratio.y)});

  // Must get this value BEFORE drawing the image or it will be incorrect.
  // If this value is incorrect, input coords will not be transformed properly.
  const ImVec2 screen_pos = ImGui::GetCursorScreenPos();

  ImGui::Image(viewport_render_target_->GetColorTexture(), viewport_panel_size,
               kUv0, kUv1);

  // Update the viewport rect in the editor so input can be correctly
  // transformed.
  const float2 center = {screen_pos.x + viewport_panel_size.x * 0.5f,
                         screen_pos.y + viewport_panel_size.y * 0.5f};
  const float2 half_extent = {viewport_panel_size.x * 0.5f,
                              viewport_panel_size.y * 0.5f};
  editor_.SetViewportRect(Rect{center, half_extent});
}

}  // namespace imp::editor
