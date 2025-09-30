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

#include "core/editor/widgets/editor_mode_toggle.h"

#include <tuple>
#include <utility>

#include "dear_imgui/imgui.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/editor_info.h"
#include "core/editor/widgets/icons/texture_assets.h"
#include "core/render/texture_asset.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"

namespace imp::editor {

namespace {
constexpr ImVec2 kButtonSize(24, 24);

}  // namespace

EditorModeToggle::EditorModeToggle(BaseView& view) : view_(view) {
  // Load all the icons.
  Future<AssetPtr<TextureAsset>> play_icon_future =
      view_.GetAssetManager().LoadTexture(texture_data::kPlayPng);
  Future<AssetPtr<TextureAsset>> stop_icon_future =
      view_.GetAssetManager().LoadTexture(texture_data::kStopPng);
  Future<AssetPtr<TextureAsset>> pause_icon_future =
      view_.GetAssetManager().LoadTexture(texture_data::kPausePng);
  Future<AssetPtr<TextureAsset>> resume_icon_future =
      view_.GetAssetManager().LoadTexture(texture_data::kResumePng);
  Future<AssetPtr<TextureAsset>> step_icon_future =
      view_.GetAssetManager().LoadTexture(texture_data::kStepPng);

  play_icon_future
      .Merge(stop_icon_future, pause_icon_future, resume_icon_future,
             step_icon_future)
      .Then([this](std::tuple<
                   AssetPtr<imp::TextureAsset>, AssetPtr<imp::TextureAsset>,
                   AssetPtr<imp::TextureAsset>, AssetPtr<imp::TextureAsset>,
                   AssetPtr<imp::TextureAsset>>
                       tuple) mutable {
        std::tie(play_icon_, stop_icon_, pause_icon_, resume_icon_,
                 step_icon_) = std::move(tuple);
      })
      .KeptBy(&rememberer_);
}

void EditorModeToggle::DrawImGui() {
  // If one icon is loaded, it's guaranteed they all are.
  if (!play_icon_) {
    return;
  }

  Editor& editor = view_.GetRegistry().Get<Editor>()->get();

  bool is_disabled =
      editor.GetRunMode() == EditorInfo::RunMode::kSwitchingToEditMode ||
      editor.GetRunMode() == EditorInfo::RunMode::kSwitchingToPlayMode;

  if (is_disabled) {
    ImGui::BeginDisabled();
  }

  // To keep the buttons on the same line with button from the previous widget.
  ImGui::SameLine();

  // First button in the toolbar: Play / Stop button.
  if (editor.GetRunMode() == EditorInfo::RunMode::kPlayMode) {
    if (ImGui::ImageButton(stop_icon_->GetFilamentTexture(), kButtonSize)) {
      editor.SetInEditMode(true);
    }
  } else {
    if (ImGui::ImageButton(play_icon_->GetFilamentTexture(), kButtonSize)) {
      editor.SetInEditMode(false);
    }
  }

  ImGui::SameLine();

  // Second button in the toolbar: Pause / Resume button.
  filament::Texture* second_button_texture =
      editor.IsPaused() ? resume_icon_->GetFilamentTexture()
                        : pause_icon_->GetFilamentTexture();

  if (ImGui::ImageButton(second_button_texture, kButtonSize)) {
    editor.SetPaused(!editor.IsPaused());
  }

  ImGui::SameLine();

  // Third button in the toolbar: Step button.
  bool disabled = editor.GetRunMode() != EditorInfo::RunMode::kPlayMode ||
                  !editor.IsPaused();
  if (disabled) ImGui::BeginDisabled();
  if (ImGui::ImageButton(step_icon_->GetFilamentTexture(), kButtonSize)) {
    editor.StepNextFrame();
  }
  if (disabled) ImGui::EndDisabled();

  if (is_disabled) {
    ImGui::EndDisabled();
  }
}

}  // namespace imp::editor
