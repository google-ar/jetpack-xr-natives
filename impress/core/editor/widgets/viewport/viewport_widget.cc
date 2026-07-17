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
#include <optional>
#include <string>

#include "absl/cleanup/cleanup.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/editor_info.h"
#include "core/editor/file_loader_helper.h"
#include "core/editor/ui/drag_and_drop.h"
#include "core/editor/widgets/viewport/viewport_helpers.h"
#include "core/editor/widgets/viewport/viewport_render_target.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"
#include "core/window/filament_host.h"

namespace imp::editor {

namespace {

#if IMP_MATERIAL_API(OPENGL)
// Flipped UVs for a vertically flipped texture.
constexpr ImVec2 kUv0 = ImVec2(0.0f, 1.0f);
constexpr ImVec2 kUv1 = ImVec2(1.0f, 0.0f);
#else
// Default UVs for a non-flipped texture.
constexpr ImVec2 kUv0 = ImVec2(0.0f, 0.0f);
constexpr ImVec2 kUv1 = ImVec2(1.0f, 1.0f);
#endif  // IMP_MATERIAL_API(OPENGL)

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

  const uint2 target_size = {
      static_cast<uint32_t>(viewport_panel_size.x * pixel_ratio.x),
      static_cast<uint32_t>(viewport_panel_size.y * pixel_ratio.y)};

  viewport_render_target_->SetSize(target_size);
#if IMP_RUNTIME(DEV)
  view_.SetSizeOverride(target_size);
#endif

  // Must get this value BEFORE drawing the image or it will be incorrect.
  // If this value is incorrect, input coords will not be transformed properly.
  const ImVec2 screen_pos = ImGui::GetCursorScreenPos();

  ImGui::Image(viewport_render_target_->GetColorTexture(), viewport_panel_size,
               kUv0, kUv1);

  HandleAssetDragDrop(screen_pos);

  // Update the viewport rect in the editor so input can be correctly
  // transformed.
  const float2 center = {screen_pos.x + viewport_panel_size.x * 0.5f,
                         screen_pos.y + viewport_panel_size.y * 0.5f};
  const float2 half_extent = {viewport_panel_size.x * 0.5f,
                              viewport_panel_size.y * 0.5f};
  editor_.SetViewportRect(Rect{center, half_extent});
}

void ViewportWidget::HandleAssetDragDrop(const ImVec2 screen_pos) {
  if (!ImGui::BeginDragDropTarget()) return;
  // Lets us return early without having to manually call EndDragDropTarget.
  const absl::Cleanup cleanup([]() { ImGui::EndDragDropTarget(); });

  const std::optional<std::string> payload =
      AcceptDragAndDropPayload(DragAndDropType::kNodeAsset);

  if (!payload.has_value()) return;

  const ImVec2 mouse_pos = ImGui::GetIO().MousePos;
  const float2 cursor_pos = {mouse_pos.x - screen_pos.x,
                             mouse_pos.y - screen_pos.y};
  LoadAssetFileAtCursor(view_, *payload, LoadAssetFileFromPathSource::kAsset,
                        cursor_pos);
}

bool ViewportWidget::HasContent() const {
  // Do not use the viewport widget in WorldspaceUI or Remote Editor mode.
  window::FilamentHost::DevModeExtension* dev_mode_extension =
      view_.GetHost()->TryGetExtension();
  const bool isWorldspaceUI =
      dev_mode_extension && dev_mode_extension->HasRenderTarget();
  const bool isRemoteScreen =
      editor_.GetDisplayMode() == EditorInfo::DisplayMode::kRemoteScreen;

  return !isWorldspaceUI && !isRemoteScreen;
}

}  // namespace imp::editor
