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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ADD_COMPONENT_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ADD_COMPONENT_H_

#include "absl/strings/string_view.h"
#include "core/common/rememberer.h"
#include "core/common/robin_map.h"
#include "core/editor/editor.h"
#include "core/editor/widget.h"
#include "core/editor/widget_layout_info.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Shows UI to add Components to a selected node and UI for existing components.
class ComponentUi : public editor::Widget, public imp::Rememberer {
 public:
  explicit ComponentUi(BaseView& view,
                       WidgetLayoutInfo component_widgets_layout_info);
  void DrawImGui() override;
  bool HasContent() const override;
  absl::string_view GetName() const override { return "Component Library"; }

 private:
  // Loops through all components on the active model and updates the UI.
  void UpdateComponentWidgets();
  // Destroys all component widgets that may exist.
  void DestroyComponentWidgets();

  BaseView& view_;
  Editor& editor_;
  NodeHandle selected_node_;
  imp::RobinMap<BaseComponentPool*, Widget*> component_widgets_;
  WidgetLayoutInfo component_widgets_layout_info_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ADD_COMPONENT_H_
