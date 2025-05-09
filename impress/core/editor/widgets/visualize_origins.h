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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VISUALIZE_ORIGINS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VISUALIZE_ORIGINS_H_

#include "absl/strings/string_view.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

// Visualizes the origins of the nodes within a glTF for the previewer.
class VisualizeOrigins : public editor::Widget, public imp::Rememberer {
 public:
  explicit VisualizeOrigins(BaseView& view);
  absl::string_view GetName() const override { return "##Visualize Origins"; }
  void DrawImGui() override;
  void Update(const FrameTime& frame_time);

 private:
  enum class Mode {
    // Draw the origins only for the node selected in the hierarchy widget.
    // The origins will be drawn in a pink color.
    kShowSelectedOrigins,
    // Draw the origins for every node in the glTF.
    kShowAllOrigins,
  };

  void DrawOriginsForAllNodes();
  void DrawOriginsForNodeRecursive(NodeHandle node);
  void DrawOriginForNode(NodeHandle node);

  BaseView& view_;
  Mode mode_ = Mode::kShowSelectedOrigins;
  NodeHandle selected_node_;
  float3 empty_node_origins_size_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VISUALIZE_ORIGINS_H_
