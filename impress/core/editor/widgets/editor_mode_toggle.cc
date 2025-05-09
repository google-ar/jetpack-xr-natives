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
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/editor_info.h"
#include "core/editor/widgets/icons/texture_assets.h"
#include "core/render/image_asset.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"

namespace imp::editor {

namespace {
constexpr ImVec2 kButtonSize(24, 24);

}  // namespace

EditorModeToggle::EditorModeToggle(BaseView& view) : view_(view) {
  // Load all the icons.
  Future<AssetPtr<ImageAsset>> play_icon_future =
      view_.GetAssetManager().LoadImage(texture_data::kPlayPng);
  Future<AssetPtr<ImageAsset>> stop_icon_future =
      view_.GetAssetManager().LoadImage(texture_data::kStopPng);
  Future<AssetPtr<ImageAsset>> pause_icon_future =
      view_.GetAssetManager().LoadImage(texture_data::kPausePng);
  Future<AssetPtr<ImageAsset>> resume_icon_future =
      view_.GetAssetManager().LoadImage(texture_data::kResumePng);
  Future<AssetPtr<ImageAsset>> step_icon_future =
      view_.GetAssetManager().LoadImage(texture_data::kStepPng);

  play_icon_future
      .Merge(stop_icon_future, pause_icon_future, resume_icon_future,
             step_icon_future)
      .Then([this](
                std::tuple<AssetPtr<imp::ImageAsset>, AssetPtr<imp::ImageAsset>,
                           AssetPtr<imp::ImageAsset>, AssetPtr<imp::ImageAsset>,
                           AssetPtr<imp::ImageAsset>>
                    tuple) mutable {
        auto [play_icon, stop_icon, pause_icon, resume_icon, step_icon] =
            std::move(tuple);
        play_icon_ = view_.GetTextureFactory().CreateTexture(*play_icon);
        stop_icon_ = view_.GetTextureFactory().CreateTexture(*stop_icon);
        pause_icon_ = view_.GetTextureFactory().CreateTexture(*pause_icon);
        resume_icon_ = view_.GetTextureFactory().CreateTexture(*resume_icon);
        step_icon_ = view_.GetTextureFactory().CreateTexture(*step_icon);
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

  // First button in the toolbar: Play / Stop button.
  if (editor.GetRunMode() == EditorInfo::RunMode::kPlayMode) {
    if (ImGui::ImageButton(stop_icon_->GetTexture(), kButtonSize)) {
      editor.SetInEditMode(true);
    }
  } else {
    if (ImGui::ImageButton(play_icon_->GetTexture(), kButtonSize)) {
      editor.SetInEditMode(false);
    }
  }

  ImGui::SameLine();

  // Second button in the toolbar: Pause / Resume button.
  Texture* second_button_texture =
      editor.IsPaused() ? resume_icon_.get() : pause_icon_.get();

  if (ImGui::ImageButton(second_button_texture->GetTexture(), kButtonSize)) {
    editor.SetPaused(!editor.IsPaused());
  }

  ImGui::SameLine();

  // Third button in the toolbar: Step button.
  bool disabled = editor.GetRunMode() != EditorInfo::RunMode::kPlayMode ||
                  !editor.IsPaused();
  if (disabled) ImGui::BeginDisabled();
  if (ImGui::ImageButton(step_icon_->GetTexture(), kButtonSize)) {
    editor.StepNextFrame();
  }
  if (disabled) ImGui::EndDisabled();

  if (is_disabled) {
    ImGui::EndDisabled();
  }
}

}  // namespace imp::editor
