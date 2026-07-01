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

#include "core/editor/widgets/transform_widget_aspect_translate.h"

#include <memory>
#include <optional>
#include <vector>

#include "core/editor/command.h"
#include "core/editor/events.proto.imp.h"
#include "core/editor/node_value_command.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"

namespace imp::editor {

float3 TransformWidgetAspectTranslate::GetAxis() const {
  return state_.axis.Value();
}

void TransformWidgetAspectTranslate::UpdateAspect(
    const bool commit, std::vector<std::unique_ptr<Command>>& commands) {
  // Calculate the closest point on the widget's axis to the user's pointer.
  // This is in world-space.
  const std::optional<float3> projection = ComputeClosestPoint();

  if (!projection.has_value()) return;  // Pointer and axis are parallel.

  // The projection on the axis is the new position of the centroid.
  const float3 new_centroid = *projection;
  // Calculate how much the centroid has moved since the drag started.
  // Multiply by state_.axis to constrain translation to the active axes,
  // particularly useful for planar translation so we don't accidentally
  // translate along the normal due to floating point inaccuracies.
  const float3 delta = (new_centroid - centroid_start_) * state_.axis.Value();

  // Apply the same translation delta to all selected nodes.
  for (const NodeTransformData& data : initial_node_transform_data_) {
    if (!data.node.IsValid()) continue;

    const float3 new_pos = data.position_start + delta;
    data.node->SetWorldPosition(new_pos);

    // If commit is false, we only update the node visually but don't create
    // undo/redo commands. This happens continuously during the drag.
    // Commands are only added to the stack when the drag finishes and the
    // position has changed enough to matter.
    if (!commit || RoughlyEqual(data.position_start, new_pos)) continue;

    commands.push_back(std::make_unique<NodeValueCommand<float3>>(
        data.node, data.position_start, new_pos,
        [editor = editor_](NodeHandle target, const float3 value) {
          target->SetWorldPosition(value);
          NodeUpdatedEvent event;
          event.target = target;
          event.translation = value;
          editor->GetDispatcher().Send(event);
        }));
  }
}

}  // namespace imp::editor
