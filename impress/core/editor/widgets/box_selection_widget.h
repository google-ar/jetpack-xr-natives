/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_BOX_SELECTION_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_BOX_SELECTION_WIDGET_H_

#include "absl/container/flat_hash_set.h"
#include "absl/strings/string_view.h"
#include "core/common/rememberer.h"
#include "core/editor/widget.h"
#include "core/input/key_codes.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Adds box selection functionality to an Editor viewport.
class BoxSelectionWidget : public Widget, public imp::Rememberer {
 public:
  // Modes for box selection.
  enum class SelectionMode {
    // Select nodes that intersect the selection box.
    kIntersect,
    // Select nodes that are fully contained within the selection box.
    kContains,
  };

  explicit BoxSelectionWidget(BaseView& view);

  // This name must start with ## otherwise a blank ImGui window will be shown.
  absl::string_view GetName() const override { return "##Box Selection"; }
  void DrawImGui() override;
  bool HasContent() const override { return dragging_; }

  void SetSelectionMode(SelectionMode mode) { selection_mode_ = mode; }
  SelectionMode GetSelectionMode() const { return selection_mode_; }

 private:
  void OnDragStart(float2 pos, NodeHandle target);
  void OnDragUpdate(float2 pos);
  void OnDragFinish(float2 pos, bool cancelled);

  // Performs the box selection after a drag gesture is finished.
  void PerformSelection();
  // Draws the selection box.
  void DrawBox();

  bool dragging_ = false;
  BaseView& view_;
  SelectionMode selection_mode_ = SelectionMode::kContains;
  float2 start_pos_;
  float2 current_pos_;
  absl::flat_hash_set<VirtualKeyCode> held_add_select_keys_;
  absl::flat_hash_set<VirtualKeyCode> held_remove_select_keys_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_BOX_SELECTION_WIDGET_H_
