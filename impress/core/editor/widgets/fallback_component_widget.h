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

#include <memory>
#include <string>
#include <vector>

#include "absl/container/flat_hash_set.h"
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
  FallbackComponentWidget(absl::flat_hash_set<NodeHandle> nodes,
                          BaseComponentPool* pool);

  absl::string_view GetName() const override;

  bool IsCloseable() const override { return false; }

  bool HasContent() const override;

  void DrawImGui() override;

 protected:
  virtual void SetComponentEnabled(bool enabled);

  CommandManager& GetCommandManager() const;

  ComponentHandle<SceneMetadata> GetMetadata() const;

  virtual const SceneMetadata::ComponentSource* GetBaseComponentSource() const;

  const absl::flat_hash_set<NodeHandle>& GetNodes() const { return nodes_; }

 private:
  absl::flat_hash_set<NodeHandle> nodes_;
  NodeHandle reference_node_;
  ComponentHandle<SceneMetadata> metadata_;
  BaseComponentPool* pool_;
  absl::flat_hash_set<ComponentHandle<Component>> components_;
  CommandManager& command_manager_;
  std::string name_;

  void DrawActiveLabel(
      const absl::flat_hash_set<ComponentHandle<Component>>& active_components);

  void CreateUndoCommand(
      const absl::flat_hash_set<ComponentHandle<Component>>& components,
      bool enabled);
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_FALLBACK_COMPONENT_WIDGET_H_
