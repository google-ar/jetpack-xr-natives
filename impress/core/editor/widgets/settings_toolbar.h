/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_SETTINGS_TOOLBAR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_SETTINGS_TOOLBAR_H_

#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/render/texture_asset.h"
#include "core/view/base_view.h"

namespace imp::resources {
class ResourceDefinition;
}

namespace imp::editor {

class Editor;
enum class TransformWidgetMode;

// A small toolbar for settings below the existing Menu bar.
class SettingsToolbar : public Widget {
 public:
  explicit SettingsToolbar(BaseView& view);

  absl::string_view GetName() const override { return "Settings Toolbar"; }
  void DrawImGui() override;

 private:
  template <AssetPtr<TextureAsset> SettingsToolbar::* IconMember>
  void LoadIcon(const resources::ResourceDefinition& texture_definition);

  // Draws a button for the given transform widget mode with the given icon.
  template <TransformWidgetMode Mode,
            AssetPtr<TextureAsset> SettingsToolbar::* IconMember>
  void DrawButton(Editor& editor, TransformWidgetMode current_mode,
                  const char* button_id, const char* label);
  // Draws a hover tooltip for a setting with the given text.
  void DrawTooltipOnHover(const char* text);

  void DrawLocalWorldSpaceOption(Editor& editor);
  void DrawGridToggle(Editor& editor);
  void DrawSnapToggle(Editor& editor);
  void DrawGridSpacingOption(Editor& editor);

  BaseView& view_;
  Rememberer rememberer_;
  AssetPtr<TextureAsset> translate_icon_;
  AssetPtr<TextureAsset> rotate_icon_;
  AssetPtr<TextureAsset> scale_icon_;
  AssetPtr<TextureAsset> local_worldspace_toggle_icon_;
  AssetPtr<TextureAsset> grid_on_icon_;
  AssetPtr<TextureAsset> grid_off_icon_;
  AssetPtr<TextureAsset> snap_on_icon_;
  AssetPtr<TextureAsset> snap_off_icon_;
  AssetPtr<TextureAsset> grid_mini_icon_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_SETTINGS_TOOLBAR_H_
