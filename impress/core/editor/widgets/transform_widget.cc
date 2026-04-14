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

#include "core/editor/widgets/transform_widget.h"

#include <vector>

#include "absl/container/flat_hash_set.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_system.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

namespace {

// Minimum distance from the camera to the transform widget.
const float kMinCameraDistance = 1e-5f;

}  // namespace

void TransformWidget::Setup() {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  Dispatcher& editor_dispatcher = editor.GetDispatcher();

  editor_dispatcher.Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        const absl::flat_hash_set<NodeHandle>& selected_nodes =
            GetView().GetRegistry().Get<Editor>()->get().GetSelectedNodes();
        active_nodes_.assign(selected_nodes.begin(), selected_nodes.end());
        GetNode()->SetEnabled(!active_nodes_.empty());
      },
      this);
}

void TransformWidget::Update(const FrameTime& frame_time) {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  if (!editor.GetEditorRoot()->IsActive()) return;
  if (active_nodes_.empty()) {
    // TODO: Remove this once we have a better way to hide the
    // transform widget.
    GetNode()->SetEnabled(false);
    return;
  }

  // Compute the centroid of the active nodes.
  float3 sum_positions = {0.0f, 0.0f, 0.0f};
  int valid_nodes_count = 0;
  for (const NodeHandle& node : active_nodes_) {
    if (!node.IsValid()) continue;
    sum_positions += node->GetWorldPosition();
    valid_nodes_count++;
  }

  // If there are no valid nodes, disable the transform widget and return.
  if (valid_nodes_count == 0) {
    GetNode()->SetEnabled(false);
    return;
  }

  const float3 centroid = sum_positions / static_cast<float>(valid_nodes_count);
  const float3 camera_position =
      editor.GetCamera()->GetNode()->GetWorldPosition();
  const float3 camera_to_target = centroid - camera_position;

  // Avoid divide by zero when normalizing a zero vector.
  float3 dif;
  if (length(camera_to_target) > kMinCameraDistance) {
    dif = normalize(camera_to_target);
  } else {
    // If the camera is at the centroid, place the widget slightly in front.
    dif = float3{0.0f, 0.0f, -1.0f};
  }

  GetNode()->SetWorldPosition(camera_position + dif);
  // TODO: Enable for local mode.
  // GetNode()->SetWorldRotation(active_nodes_[0]->GetWorldRotation());
}

}  // namespace imp::editor
