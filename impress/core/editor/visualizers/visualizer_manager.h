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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_VISUALIZER_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_VISUALIZER_MANAGER_H_

#include <algorithm>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/common/invocable.h"
#include "core/common/registry.h"
#include "core/common/robin_map.h"
#include "core/editor/editor.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/ncsb/system.h"
#include "core/view/base_view.h"

namespace imp::editor {

// VisualizerManager updates visualizers of all registered type, create new
// visualizer, clean up outdated ones.
class VisualizerManager : public System {
 public:
  explicit VisualizerManager(BaseView* view);

  template <typename ComponentT, typename VisualizerT>
  void RegisterVisualizer() {
    creators_.push_back([this]() {
      GetView().GetComponentManager().ForEach<ComponentT>(
          [this](ComponentT* comp) mutable {
            NodeHandle target = comp->GetNode();
            absl::string_view type_name = type_traits::kTypeName<ComponentT>;
            Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
            if (GetView().GetPathManager().IsAncestorOf(editor.GetEditorRoot(),
                                                        target)) {
              return;
            }
            RobinMap<NodeHandle, NodeHandle>& typed_visualizer_map =
                visualizer_map_[type_name];
            auto iter = typed_visualizer_map.find(target);
            if (iter == typed_visualizer_map.end()) {
              NodeHandle node = GetView().CreateNode();
              editor.AddNode(node);
              visualizers_[type_traits::kTypeName<ComponentT>].push_back(node);
              typed_visualizer_map.insert({target, node});
              node->AddComponent<VisualizerT>(target);
            }
          });
    });

    removers_.push_back([this]() {
      std::vector<NodeHandle> to_remove;
      absl::string_view type_name = type_traits::kTypeName<ComponentT>;
      for (auto& visualizer : visualizers_[type_name]) {
        NodeHandle target =
            visualizer->GetComponent<VisualizerT>()->GetTarget();
        if (target && target->GetComponent<ComponentT>()) {
          continue;
        } else {
          to_remove.push_back(visualizer);
        }
      }
      for (auto& visualizer : to_remove) {
        visualizers_[type_name].erase(
            std::remove(visualizers_[type_name].begin(),
                        visualizers_[type_name].end(), visualizer),
            visualizers_[type_name].end());
        visualizer_map_[type_name].erase(
            visualizer->GetComponent<VisualizerT>()->GetTarget());
        GetView().DestroyNode(visualizer);
      }
    });
  }

 private:
  using CreatorFn = Invocable<void()>;
  using RemoverFn = Invocable<void()>;
  std::vector<CreatorFn> creators_;
  std::vector<RemoverFn> removers_;

  // The correspondence between target and its visualizer.
  using VisualizerPairMap =
      RobinMap<absl::string_view, RobinMap<NodeHandle, NodeHandle>>;
  VisualizerPairMap visualizer_map_;

  // Hold all types of visualizers.
  using VisualizerListMap =
      RobinMap<absl::string_view, std::vector<NodeHandle>>;
  VisualizerListMap visualizers_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_VISUALIZER_MANAGER_H_
