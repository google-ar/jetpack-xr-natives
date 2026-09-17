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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_STATELESS_ISF_COMPONENT_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_STATELESS_ISF_COMPONENT_WIDGET_H_

#include <memory>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/common/invocable.h"
#include "core/editor/command.h"
#include "core/editor/composite_command.h"
#include "core/editor/node_value_command.h"
#include "core/editor/widgets/component_editor_helpers.h"
#include "core/editor/widgets/fallback_component_widget.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"

namespace imp::editor {

// A widget that displays a component that includes Stateless IsfInfo.
template <typename T>
class StatelessIsfComponentWidget : public FallbackComponentWidget {
 public:
  StatelessIsfComponentWidget(absl::flat_hash_set<NodeHandle> nodes,
                              ComponentHandle<T> component);

  Invocable<void()> OnCloseButton() override;
  bool IsCloseable() const override {
    return GetBaseComponentSource() == nullptr;
  }

 protected:
  ComponentHandle<T> GetComponent() const;

  const SceneMetadata::ComponentSource* GetBaseComponentSource() const override;

 private:
  void OnCloseButtonImpl();
  static absl::Status ApplyCloseState(NodeHandle target, bool value);

  ComponentHandle<T> component_;
};

template <typename T>

StatelessIsfComponentWidget<T>::StatelessIsfComponentWidget(
    absl::flat_hash_set<NodeHandle> nodes, ComponentHandle<T> component)
    : FallbackComponentWidget(
          std::move(nodes),
          component->GetNode()
              ->GetView()
              .GetComponentManager()
              .GetComponentPoolById(component->GetComponentId())),
      component_(component) {}

template <typename T>
ComponentHandle<T> StatelessIsfComponentWidget<T>::GetComponent() const {
  return component_;
}

template <typename T>
Invocable<void()> StatelessIsfComponentWidget<T>::OnCloseButton() {
  // If this component comes from a base isf file then it can't be removed.
  if (GetBaseComponentSource()) return {};

  return [this]() { OnCloseButtonImpl(); };
}

template <typename T>
void StatelessIsfComponentWidget<T>::OnCloseButtonImpl() {
  const absl::flat_hash_set<NodeHandle>& nodes = this->GetNodes();
  std::vector<std::unique_ptr<Command>> commands;
  commands.reserve(nodes.size());

  for (const NodeHandle& node : nodes) {
    // Although widgets are only displayed for components present on all
    // selected nodes, defensively check in case the component was removed
    // after the widget was created.
    if (!node->GetComponent<T>()) continue;

    // If this component comes from a base isf file then it can't be removed.
    const ComponentHandle<SceneMetadata> metadata =
        node->GetComponent<SceneMetadata>();

    if (metadata &&
        metadata->GetBaseComponentSource(T::IsfInfo::kTypeUrlHash)) {
      continue;
    }

    commands.push_back(std::make_unique<NodeValueCommand<bool>>(
        node, true, false, ApplyCloseState));
  }

  if (commands.empty()) return;

  this->GetCommandManager().template PerformCommand<CompositeCommand>(
      std::move(commands));
}

template <typename T>
absl::Status StatelessIsfComponentWidget<T>::ApplyCloseState(NodeHandle target,
                                                             bool value) {
  return UpdateComponentAuthoredSync<T>(target, value);
}

template <typename T>
const SceneMetadata::ComponentSource*
StatelessIsfComponentWidget<T>::GetBaseComponentSource() const {
  ComponentHandle<SceneMetadata> metadata = GetMetadata();
  if (!metadata) {
    return nullptr;
  }

  return metadata->GetBaseComponentSource(T::IsfInfo::kTypeUrlHash);
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_STATELESS_ISF_COMPONENT_WIDGET_H_
