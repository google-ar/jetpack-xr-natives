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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_EDITOR_PLUGIN_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_EDITOR_PLUGIN_H_

#include "core/editor/editor_plugin.h"
#include "core/editor/layout/layout_composer.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

// An EditorPlugin that spawns a world-space Editor panel and provides an
// XR-specific LayoutComposer.
class XrEditorPlugin : public EditorPlugin {
 public:
  explicit XrEditorPlugin(BaseView* view) : EditorPlugin(view) {}
  void OnEditorInitialized() override;
  std::unique_ptr<LayoutComposer> CreateLayoutComposer() override;
  CameraConfiguration GetCameraConfiguration() const override;

 private:
  NodeHandle editor_node_;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_EDITOR_PLUGIN_H_
