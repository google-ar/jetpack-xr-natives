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

#include "core/editor/widgets/toggle_camera.h"

#include "dear_imgui/imgui.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/editor/widgets/icons/texture_assets.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/render/texture_asset.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"

namespace imp::editor {

namespace {
constexpr ImVec2 kButtonSize(24, 24);
}  // namespace

ToggleCamera::ToggleCamera(BaseView& view) : view_(view) {
  view_.GetAssetManager()
      .LoadTexture(texture_data::kCameraSwitchPng)
      .Then([this](AssetPtr<imp::TextureAsset> switch_camera_icon) mutable {
        switch_camera_icon_ = switch_camera_icon;
      })
      .KeptBy(&rememberer_);
}

void ToggleCamera::DrawImGui() {
  if (switch_camera_icon_ &&
      ImGui::ImageButton("##toggle_camera",
                         switch_camera_icon_->GetFilamentTexture(),
                         kButtonSize)) {
    Editor& editor = view_.GetRegistry().Get<Editor>()->get();
    editor.GetDispatcher().Send(ToggleCameraEvent());
  }
}

}  // namespace imp::editor
