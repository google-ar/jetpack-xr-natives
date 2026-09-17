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

#include "core/editor/widgets/settings_toolbar.h"

#include <string>

#include "absl/strings/str_format.h"
#include "dear_imgui/imgui.h"
#include "dear_imgui/imgui_internal.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/editor_settings.h"
#include "core/editor/widgets/icons/texture_assets.h"
#include "core/render/texture_asset.h"
#include "core/resources/resource_definition.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"

namespace imp::editor {

namespace {

// Padding around the text for tooltips.
constexpr ImVec2 kTooltipPadding(4.0f, 2.0f);

// The size of the icons in the settings toolbar.
constexpr ImVec2 kButtonSize(16, 16);

// Spacing between buttons in the settings toolbar.
constexpr ImVec2 kButtonSpacing(4.0f, 0.0f);

// Fixed text width for the space mode label.
constexpr float kLocalWorldSpaceTextWidth = 50.0f;

// Label strings.
constexpr char kLocalLabel[] = "Local";
constexpr char kWorldLabel[] = "World";

// Tooltip strings.
constexpr char kLocalSpaceTooltip[] = "Local Space";
constexpr char kWorldSpaceTooltip[] = "World Space";
constexpr char kToggleGridTooltip[] = "Toggle Grid";
constexpr char kToggleSnappingTooltip[] = "Toggle Snapping";
constexpr char kGridSpacingTooltip[] = "Grid Spacing";

// Format strings.
constexpr char kStringFormat[] = "%s";
constexpr char kFloatFormat[] = "%g";

// The label for the translate button.
constexpr char kTranslateButtonLabel[] = "Translate";

// The label for the rotate button.
constexpr char kRotateButtonLabel[] = "Rotate";

// The label for the scale button.
constexpr char kScaleButtonLabel[] = "Scale";

}  // namespace

SettingsToolbar::SettingsToolbar(BaseView& view) : view_(view) {
  LoadIcon<&SettingsToolbar::translate_icon_>(
      texture_data::kTransformTranslatePng);
  LoadIcon<&SettingsToolbar::rotate_icon_>(texture_data::kTransformRotatePng);
  LoadIcon<&SettingsToolbar::scale_icon_>(texture_data::kTransformScalePng);
  LoadIcon<&SettingsToolbar::local_worldspace_toggle_icon_>(
      texture_data::kLocalWorldspaceTogglePng);
  LoadIcon<&SettingsToolbar::grid_on_icon_>(texture_data::kGridONPng);
  LoadIcon<&SettingsToolbar::grid_off_icon_>(texture_data::kGridOffPng);
  LoadIcon<&SettingsToolbar::snap_on_icon_>(texture_data::kSnapOnPng);
  LoadIcon<&SettingsToolbar::snap_off_icon_>(texture_data::kSnapOffPng);
  LoadIcon<&SettingsToolbar::grid_mini_icon_>(texture_data::kGridMiniPng);
}

void SettingsToolbar::DrawImGui() {
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, kButtonSpacing);

  Editor& editor = view_.GetRegistry().Get<Editor>()->get();

  const TransformWidgetMode current_mode =
      editor.GetSettings().transform_widget_mode.Get();

  ImGui::SameLine();
  DrawButton<TransformWidgetMode::kTranslate,
             &SettingsToolbar::translate_icon_>(
      editor, current_mode, "##translate_button", kTranslateButtonLabel);

  ImGui::SameLine();
  DrawButton<TransformWidgetMode::kRotate, &SettingsToolbar::rotate_icon_>(
      editor, current_mode, "##rotate_button", kRotateButtonLabel);

  ImGui::SameLine();
  DrawButton<TransformWidgetMode::kScale, &SettingsToolbar::scale_icon_>(
      editor, current_mode, "##scale_button", kScaleButtonLabel);

  ImGui::SameLine();
  DrawLocalWorldSpaceOption(editor);

  ImGui::SameLine();
  DrawGridToggle(editor);

  ImGui::SameLine();
  DrawSnapToggle(editor);

  ImGui::SameLine();
  DrawGridSpacingOption(editor);

  ImGui::PopStyleVar();  // ImGuiStyleVar_ItemSpacing
}

void SettingsToolbar::DrawLocalWorldSpaceOption(Editor& editor) {
  const bool local_space =
      editor.GetSettings().transform_widget_local_space.Get();
  const float total_width = kButtonSize.x + ImGui::GetStyle().ItemSpacing.x +
                            kLocalWorldSpaceTextWidth + ImGui::GetFrameHeight();

  ImGui::SetNextItemWidth(total_width);
  if (ImGui::BeginCombo("##local_worldspace_combo", nullptr,
                        ImGuiComboFlags_CustomPreview)) {
    if (ImGui::Selectable(kLocalLabel, local_space)) {
      editor.GetSettings().transform_widget_local_space.Set(true);
    }

    if (ImGui::Selectable(kWorldLabel, !local_space)) {
      editor.GetSettings().transform_widget_local_space.Set(false);
    }

    ImGui::EndCombo();
  }

  DrawTooltipOnHover(local_space ? kLocalSpaceTooltip : kWorldSpaceTooltip);

  if (ImGui::BeginComboPreview()) {
    if (local_worldspace_toggle_icon_) {
      ImGui::Image(local_worldspace_toggle_icon_->GetFilamentTexture(),
                   kButtonSize);
      ImGui::SameLine();
    }
    ImGui::Text(kStringFormat, local_space ? kLocalLabel : kWorldLabel);
    ImGui::EndComboPreview();
  }
}

void SettingsToolbar::DrawGridToggle(Editor& editor) {
  bool grid_enabled = editor.GetSettings().show_grid.Get();
  const auto& icon = grid_enabled ? grid_on_icon_ : grid_off_icon_;
  if (icon && ImGui::ImageButton("##grid_toggle_button",
                                 icon->GetFilamentTexture(), kButtonSize)) {
    editor.GetSettings().show_grid.Set(!grid_enabled);
  }

  DrawTooltipOnHover(kToggleGridTooltip);
}

void SettingsToolbar::DrawSnapToggle(Editor& editor) {
  bool snap_enabled = editor.GetSettings().snap_enabled.Get();
  const auto& icon = snap_enabled ? snap_on_icon_ : snap_off_icon_;
  if (icon && ImGui::ImageButton("##snap_toggle_button",
                                 icon->GetFilamentTexture(), kButtonSize)) {
    editor.GetSettings().snap_enabled.Set(!snap_enabled);
  }

  DrawTooltipOnHover(kToggleSnappingTooltip);
}

void SettingsToolbar::DrawGridSpacingOption(Editor& editor) {
  float current_spacing = editor.GetSettings().grid_spacing.Get();
  constexpr float kGridSpacingOptions[] = {0.25f, 0.5f, 1.0f,  2.0f,
                                           4.0f,  8.0f, 16.0f, 32.0f};

  // Fixed text width for the spacing preview.
  constexpr float kGridSpacingTextWidth = 40.0f;
  const float total_width = kButtonSize.x + ImGui::GetStyle().ItemSpacing.x +
                            kGridSpacingTextWidth + ImGui::GetFrameHeight();
  ImGui::SetNextItemWidth(total_width);

  if (ImGui::BeginCombo("##grid_spacing_combo", nullptr,
                        ImGuiComboFlags_CustomPreview)) {
    for (float option : kGridSpacingOptions) {
      std::string label = absl::StrFormat(kFloatFormat, option);
      bool is_selected = (current_spacing == option);
      if (ImGui::Selectable(label.c_str(), is_selected)) {
        editor.GetSettings().grid_spacing.Set(option);
      }
    }
    ImGui::EndCombo();
  }

  DrawTooltipOnHover(kGridSpacingTooltip);

  if (ImGui::BeginComboPreview()) {
    if (grid_mini_icon_) {
      ImGui::Image(grid_mini_icon_->GetFilamentTexture(), kButtonSize);
      ImGui::SameLine();
    }
    ImGui::Text(kStringFormat,
                absl::StrFormat(kFloatFormat, current_spacing).c_str());
    ImGui::EndComboPreview();
  }
}

template <AssetPtr<TextureAsset> SettingsToolbar::* IconMember>
void SettingsToolbar::LoadIcon(
    const resources::ResourceDefinition& texture_definition) {
  view_.GetAssetManager()
      .LoadTexture(texture_definition)
      .Then([this](AssetPtr<TextureAsset> icon) mutable {
        this->*IconMember = icon;
      })
      .KeptBy(&rememberer_);
}

template <TransformWidgetMode Mode,
          AssetPtr<TextureAsset> SettingsToolbar::* IconMember>
void SettingsToolbar::DrawButton(Editor& editor,
                                 const TransformWidgetMode current_mode,
                                 const char* button_id, const char* label) {
  const bool is_active = current_mode == Mode;

  if (is_active) {
    ImGui::PushStyleColor(ImGuiCol_Button,
                          ImGui::GetStyle().Colors[ImGuiCol_ButtonActive]);
  }

  const AssetPtr<TextureAsset>& icon = this->*IconMember;
  if (icon &&
      ImGui::ImageButton(button_id, icon->GetFilamentTexture(), kButtonSize)) {
    editor.GetSettings().transform_widget_mode.Set(Mode);
  }

  if (is_active) {
    ImGui::PopStyleColor();  // ImGuiCol_Button
  }

  DrawTooltipOnHover(label);
}

void SettingsToolbar::DrawTooltipOnHover(const char* text) {
  if (!ImGui::IsItemHovered()) return;

  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, kTooltipPadding);
  ImGui::SetTooltip("%s", text);
  ImGui::PopStyleVar();  // ImGuiStyleVar_WindowPadding
}

}  // namespace imp::editor
