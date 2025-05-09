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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FALLBACK_COMPONENT_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FALLBACK_COMPONENT_WIDGET_H_

#include <string>

#include "absl/strings/string_view.h"
#include "core/editor/command_manager.h"
#include "core/editor/widget.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"

namespace imp::editor {

// A widget that displays a component that does not include IsfInfo.
//
// It simply displays the component name and provides the ability to toggle
// the component's enabled state.
//
// Component's without IsfInfo can't be saved as authored content, but it is
// still useful to represent them in the editor for debugging purposes.
class FallbackComponentWidget : public Widget {
 public:
  FallbackComponentWidget(NodeHandle node, BaseComponentPool* pool);

  absl::string_view GetName() const override;

  bool HasContent() const override;

  void DrawImGui() override;

 protected:
  virtual void SetComponentEnabled(bool enabled);

  CommandManager& GetCommandManager() const;

  ComponentHandle<SceneMetadata> GetMetadata() const;

  virtual const SceneMetadata::ComponentSource* GetBaseComponentSource() const;

 private:
  NodeHandle node_;
  ComponentHandle<SceneMetadata> metadata_;
  BaseComponentPool* pool_;
  ComponentHandle<Component> component_;
  CommandManager& command_manager_;
  std::string name_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FALLBACK_COMPONENT_WIDGET_H_
