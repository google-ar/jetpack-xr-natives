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

#include "core/editor/widgets/component_ui.h"

#include <functional>
#include <utility>

#if IMP_RUNTIME(DEV)
#include <memory>
#endif  // IMP_RUNTIME(DEV)

#include "core/common/registry.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/editor/layout/helpers.h"
#include "core/editor/widget_layout_info.h"
#include "core/editor/widget_ui_system.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_system.h"
#if IMP_RUNTIME(DEV)
#include "absl/container/btree_map.h"
#include "absl/container/flat_hash_set.h"
#include "core/editor/editor_info.h"
#include "core/editor/widget.h"
#include "core/editor/widgets/fallback_component_widget.h"
#include "core/ncsb/scene_metadata.h"
#include "core/view/framework/scene/scene_system.h"
#endif  // IMP_RUNTIME(DEV)

namespace imp::editor {

ComponentUi::ComponentUi(BaseView& view,
                         WidgetLayoutInfo component_widgets_layout_info)
    : view_(view),
      editor_(view_.GetRegistry().Get<Editor>()->get()),
      component_widgets_layout_info_(component_widgets_layout_info) {
  editor_.GetDispatcher().Connect(
      [this](const editor::NodeSelectionChangedEvent& event) mutable {
        selected_nodes_ = editor_.GetSelectedNodes();

        DestroyComponentWidgets();
        UpdateComponentWidgets(selected_nodes_);
      },
      this);
}

void ComponentUi::DestroyComponentWidgets() {
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  WidgetUiSystem& widget_ui_system = editor.GetWidgetUiSystem();
  for (auto& entry : component_widgets_) {
    widget_ui_system.RemoveWidget(entry.second);
  }
  component_widgets_.clear();
}

void ComponentUi::UpdateComponentWidgets(
    const absl::flat_hash_set<NodeHandle>& selected_nodes) {
  if (selected_nodes.empty()) {
    return;
  }
#if IMP_RUNTIME(DEV)
  Editor& editor = view_.GetRegistry().Get<Editor>()->get();
  WidgetUiSystem& widget_ui_system = editor.GetWidgetUiSystem();

  NodeHandle first_node = *selected_nodes.begin();
  ComponentHandle<SceneMetadata> scene_metadata =
      first_node->GetComponent<SceneMetadata>();
  bool is_in_edit_mode = editor::IsInEditMode(view_.GetRegistry());

  // Gather the set of all components that should be visualized on all selected
  // nodes.
  absl::flat_hash_set<BaseComponentPool*> component_pools;
  bool first = true;
  for (const NodeHandle& node : selected_nodes) {
    absl::flat_hash_set<BaseComponentPool*> node_pools;
    // TODO : Don't iterate over all the component pools.
    view_.GetComponentManager().ForEachPool(
        [&node_pools, node](BaseComponentPool* pool) {
          if (pool->Has(node->GetEntity()) && !pool->IsExcludedFromEditor()) {
            node_pools.insert(pool);
          }
        });
    if (first) {
      component_pools = std::move(node_pools);
      first = false;
    } else {
      // Only keep components that are present on every selected node.
      absl::erase_if(component_pools, [&node_pools](BaseComponentPool* pool) {
        return !node_pools.contains(pool);
      });
    }
  }
  // Remove all components that we have visualizers for but no longer exist.
  for (auto it = component_widgets_.begin(); it != component_widgets_.end();) {
    if (!component_pools.contains(it->first)) {
      widget_ui_system.RemoveWidget(it->second);
      it = component_widgets_.erase(it);
    } else {
      it++;
    }
  }

  // Put the widgets in a temporary map so they will be added in sorted order.
  absl::btree_map<std::string, std::unique_ptr<Widget>> component_widgets;

  // Add new component visualizers that we don't have.
  for (BaseComponentPool* pool : component_pools) {
    if (component_widgets_.find(pool) == component_widgets_.end()) {
      std::optional<HashValue> state_type_url_hash =
          pool->GetStateTypeUrlHash();
      std::unique_ptr<Widget> component_widget;
      if (state_type_url_hash) {
        // When in edit mode, we don't want to let the user edit components that
        // are dynamically created by other components, so just represent them
        // as a fallback widget to express to the user that the component
        // exists.
        //
        // These dynamic components are also not saved to the isf file, as they
        // are created dynamically by another component when the isf file is
        // loaded.
        //
        // TODO: Create a special way to represent dynamically
        // created components in the UI in EditMode. For instance, show a
        // "dynamic" icon in the component header.
        bool use_fallback_in_edit_mode =
            (!scene_metadata ||
             !scene_metadata->IsComponentAuthored(*state_type_url_hash)) &&
            is_in_edit_mode;

        if (use_fallback_in_edit_mode) {
          component_widget =
              std::make_unique<FallbackComponentWidget>(selected_nodes, pool);
        } else {
          component_widget = view_.GetSceneSystem().CreateComponentWidget(
              state_type_url_hash.value(), selected_nodes,
              editor_.GetDispatcher());
        }

        if (!component_widget) {
          IMP_LOG(imp::ERROR) << "Failed to create component widget for "
                     << pool->GetTypeName()
                     << ". Was the component's IsfInfo registered with the "
                        "SceneSystem?";
          continue;
        }
      } else {
        component_widget =
            std::make_unique<FallbackComponentWidget>(selected_nodes, pool);
      }

      component_widgets_[pool] = component_widget.get();
      component_widgets[std::string(component_widget->GetName())] =
          std::move(component_widget);
    }
  }

  // Add the widgets in a deterministic, sorted order.
  for (auto& entry : component_widgets) {
    widget_ui_system.AddWidget(component_widgets_layout_info_,
                               std::move(entry.second));
  }
  component_widgets.clear();

#endif  // IMP_RUNTIME(DEV)
}

bool ComponentUi::HasContent() const {
  for (const NodeHandle& node : selected_nodes_) {
    if (node) return true;
  }
  return false;
}

void ComponentUi::DrawImGui() {
  PruneInvalidHandles(selected_nodes_);
  UpdateComponentWidgets(selected_nodes_);

#if IMP_RUNTIME(DEV)
  if (!selected_nodes_.empty()) {
    view_.GetSceneSystem().ShowAddComponentUi(selected_nodes_);
  }
#endif  // IMP_RUNTIME(DEV)
}

}  // namespace imp::editor
