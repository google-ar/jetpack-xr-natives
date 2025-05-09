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

#include "core/editor/editor_touch.h"

#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"

namespace imp::editor {

UndoEditorTouchFn EditorTouch(NodeHandle node) {
  auto scene_metadata = node->GetComponent<SceneMetadata>();
  bool had_metadata_before_touch = scene_metadata.IsValid();

  // If the node already has metadata, don't need to do anything. Just return an
  // empty undo function.
  if (had_metadata_before_touch) {
    return []() {};
  }

  // Add metadata component to the node to indicate that it has been touched by
  // the editor.
  node->AddComponent<SceneMetadata>();

  // Undo function removes the metadata component.
  return [node]() {
    // Make sure node still exists.
    if (node) {
      node->RemoveComponent<SceneMetadata>();
    }
  };
}

}  // namespace imp::editor
