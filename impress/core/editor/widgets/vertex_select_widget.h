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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VERTEX_SELECT_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VERTEX_SELECT_WIDGET_H_

#include <array>
#include <cstddef>
#include <vector>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/collision/collision_helpers.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

/* The VertexSelectWidget is a plugin module that handles vertex selection on
 * static/skinned/animated meshes.
 */
class VertexSelectWidget : public Widget, public Rememberer {
 public:
  explicit VertexSelectWidget(BaseView& view, Dispatcher& dispatcher,
                              NodeHandle editor_root_node);
  void DrawImGui() override {}
  absl::string_view GetName() const override {
    return "##Vertex Select Widget";
  }

 private:
  // Highlight the collision point and candidate vertex to be selected.
  void Draw();
  // Make sure picked_triangle_ has value before calling this.
  absl::optional<std::array<double3, 3>> GetVertexPositionsPrecise();

  BaseView& view_;
  Dispatcher& dispatcher_;
  NodeHandle editor_root_node_;
  NodeHandle active_node_;
  absl::optional<collision::CollidedTriangle> picked_triangle_;
  // The vertex in the triangle, that is the closest to the collision point.
  // Returns 0, 1 or 2.
  size_t picked_vertex_id_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VERTEX_SELECT_WIDGET_H_
