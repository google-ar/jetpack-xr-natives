/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_ARCORE_AR_IMP_CORE_EDITOR_WIDGETS_EDITOR_MODE_TOGGLE_H
#define THIRD_PARTY_ARCORE_AR_IMP_CORE_EDITOR_WIDGETS_EDITOR_MODE_TOGGLE_H

#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/render/texture_asset.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Widget that creates a Play/Stop and Pause/Resume button at the top-middle of
// the Editor.
class EditorModeToggle : public Widget {
 public:
  explicit EditorModeToggle(BaseView& view);
  absl::string_view GetName() const override { return "##Editor Mode Toggle"; }
  void DrawImGui() override;

 private:
  BaseView& view_;
  imp::Rememberer rememberer_;

  // Impress textures for the icons.
  imp::AssetPtr<TextureAsset> play_icon_;
  imp::AssetPtr<TextureAsset> stop_icon_;
  imp::AssetPtr<TextureAsset> pause_icon_;
  imp::AssetPtr<TextureAsset> resume_icon_;
  imp::AssetPtr<TextureAsset> step_icon_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_ARCORE_AR_IMP_CORE_EDITOR_WIDGETS_EDITOR_MODE_H
