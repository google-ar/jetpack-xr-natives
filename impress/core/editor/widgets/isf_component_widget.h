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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_COMPONENT_EDITOR_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_COMPONENT_EDITOR_WIDGET_H_

#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/container/inlined_vector.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/common/invocable.h"
#include "core/editor/command.h"
#include "core/editor/command_manager.h"
#include "core/editor/component_editor.proto.imp.h"
#include "core/editor/composite_command.h"
#include "core/editor/editor_info.h"
#include "core/editor/editor_proto_visitor.h"
#include "core/editor/events.proto.imp.h"
#include "core/editor/node_value_command.h"
#include "core/editor/widgets/component_editor_helpers.h"
#include "core/editor/widgets/stateless_isf_component_widget.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_traits.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"

namespace imp::editor {

// A widget that displays a component that includes IsfInfo.
//
// This widget will display the component's state and allow the user to edit
// the state.
//
// It will also handle undo/redo of state changes and notify the component of
// state changes.
template <typename T>
class IsfComponentWidget : public StatelessIsfComponentWidget<T> {
 public:
  using IsfInfo = typename T::IsfInfo;
  using StateT = typename IsfInfo::StateT;

  IsfComponentWidget(absl::flat_hash_set<NodeHandle> nodes,
                     ComponentHandle<T> component,
                     Dispatcher& editor_dispatcher);

  void DrawImGui() override;

  Invocable<void()> OnCloseButton() override;

  void SetComponentEnabled(bool enabled) override;

 private:
  static absl::Status ApplyCloseState(NodeHandle target,
                                      const std::optional<StateT>& state);
  void OnCloseButtonImpl();
  void InvokeOnIsfStateChanged();
  void NotifyComponentStateChanged();

  EditorProtoVisitor<StateT> visitor_;
  Dispatcher& editor_dispatcher_;

  // Stores the states of the components before they change, for creating a
  // command to undo the change.
  absl::flat_hash_map<NodeHandle, StateT> prev_states_;

  std::optional<StateT> base_state_;
};

template <typename T>
IsfComponentWidget<T>::IsfComponentWidget(absl::flat_hash_set<NodeHandle> nodes,
                                          ComponentHandle<T> component,
                                          Dispatcher& editor_dispatcher)
    : StatelessIsfComponentWidget<T>(std::move(nodes), component),
      visitor_(IsfInfo::GetState(component), &component->GetView()),
      editor_dispatcher_(editor_dispatcher) {
  const SceneMetadata::ComponentSource* source =
      StatelessIsfComponentWidget<T>::GetBaseComponentSource();
  if (source) {
    StateT& base_state = base_state_.emplace();
    for (absl::string_view component_binary_data : source->component_bytes) {
      proto::ParseMessage(component_binary_data, &base_state);
    }
  }
}

template <typename T>
void IsfComponentWidget<T>::DrawImGui() {
  // Draws the basic component Ui as well as custom Ui.
  StatelessIsfComponentWidget<T>::DrawImGui();

  // This resets the tracking of which fields have been edited.
  visitor_.Reset();

  // Grab a reference to the current state on the component.
  // Note: Due to non-type-dependent lookup issues, "this" is required here.
  ComponentHandle<T> component = this->GetComponent();
  StateT& state = IsfInfo::GetState(component);

  // gather their states so we can bulk update them.
  const absl::flat_hash_set<NodeHandle>& nodes = this->GetNodes();
  const bool record_prev_state = prev_states_.empty();

  absl::InlinedVector<StateT*, kActiveStatesInlineCapacity> active_states;
  for (const NodeHandle& node : nodes) {
    if (!node.IsValid()) continue;
    const ComponentHandle<T> comp = node->GetComponent<T>();

    if (!comp) continue;

    active_states.push_back(&IsfInfo::GetState(comp));

    if (!record_prev_state) continue;  // Undo/redo recording.

    prev_states_[node] = IsfInfo::GetState(comp);
  }
  visitor_.SetActiveStates(active_states);

  // Draws the editor UI for the component's state.
  //
  // EditorProtoVisitor uses an integer as the cursor to track the current
  // field being visited. We pass in zero as the initial field index being
  // visited.
  //
  // If the component has state coming from a base isf file then we pass in a
  // pointer to the base state so that the editor UI can express the difference
  // between the base state and the current state.
  StateT* other = base_state_.has_value() ? &base_state_.value() : nullptr;
  state.Visit(visitor_, /*cursor*/ 0, other);

  // If any of the state fields were edited, then we need to create a command
  // for notifying the components of the change & making it possible to undo the
  // change across all selected nodes.
  if (!visitor_.AnyFieldEdited()) return;

  CommandManager& command_manager = this->GetCommandManager();
  std::vector<std::unique_ptr<Command>> commands;
  commands.reserve(nodes.size());

  for (const NodeHandle& node : nodes) {
    if (!node.IsValid()) continue;
    const ComponentHandle<T> comp = node->GetComponent<T>();
    if (!comp) continue;
    auto it = prev_states_.find(node);
    const StateT node_prev_state =
        (it != prev_states_.end()) ? it->second : IsfInfo::GetState(comp);

    // ImGui controls in EditorProtoVisitor edit the fields inline on all
    // passed active_states. Therefore, the comp's current state
    // (IsfInfo::GetState(comp)) already holds the modified state.
    const StateT edited_state = IsfInfo::GetState(comp);

    // Since the state was already modified inline by the ImGui visitor, we
    // skip updating the state on the first execution of the command.
    // Subsequent executions (undo/redo) will apply the state changes.
    bool is_first_perform = true;

    const auto command = [&editor_dispatcher = editor_dispatcher_, comp,
                          is_first_perform](const NodeHandle target,
                                            const StateT& new_state) mutable {
      if (!comp) {
        return absl::NotFoundError("Component not found");
      }

      if (!is_first_perform) {
        // Only set the state if this is the not first perform. This
        // avoids an unnecessary copy of the state, because on the first
        // perform it's already correct. It only needs to be updated during
        // undo/redo actions.
        IsfInfo::GetState(comp) = new_state;
      }

      const absl::Status status = InvokeOnIsfStateChangedSync(comp);

      if (!status.ok()) {
        IMP_LOG(imp::ERROR) << "Failed to apply changes to component: " << status;
      }

      ComponentStateChanged component_state_changed;
      component_state_changed.component_state =
          *proto::PackAny(IsfInfo::GetState(comp));
      component_state_changed.component_enabled = comp->IsEnabled();

      NodeUpdatedEvent event;
      event.target = target;
      event.enabled = target->IsEnabled();
      event.component = std::move(component_state_changed);
      editor_dispatcher.Send(event);

      is_first_perform = false;
      return absl::OkStatus();
    };

    commands.push_back(std::make_unique<NodeValueCommand<StateT>>(
        node, node_prev_state, edited_state, command));
  }

  if (!commands.empty()) {
    command_manager.template PerformCommand<CompositeCommand>(
        std::move(commands));
  }
  prev_states_.clear();
}

template <typename T>
Invocable<void()> IsfComponentWidget<T>::OnCloseButton() {
  // If this component comes from a base isf file then it can't be removed.
  if (StatelessIsfComponentWidget<T>::GetBaseComponentSource()) return {};

  return [this]() { OnCloseButtonImpl(); };
}

template <typename T>
void IsfComponentWidget<T>::OnCloseButtonImpl() {
  const absl::flat_hash_set<NodeHandle>& nodes = this->GetNodes();
  std::vector<std::unique_ptr<Command>> commands;
  commands.reserve(nodes.size());

  for (const NodeHandle& node : nodes) {
    // Although widgets are only displayed for components present on all
    // selected nodes, defensively check in case the component was removed
    // after the widget was created.
    const ComponentHandle<T> component = node->GetComponent<T>();
    if (!component) continue;

    // If this component comes from a base isf file then it can't be removed.
    const ComponentHandle<SceneMetadata> metadata =
        node->GetComponent<SceneMetadata>();
    if (metadata && metadata->GetBaseComponentSource(IsfInfo::kTypeUrlHash)) {
      continue;
    }

    const StateT& state = IsfInfo::GetState(component);
    commands.push_back(
        std::make_unique<NodeValueCommand<std::optional<StateT>>>(
            node, state, std::nullopt, ApplyCloseState));
  }

  if (commands.empty()) return;

  this->GetCommandManager().template PerformCommand<CompositeCommand>(
      std::move(commands));
}

template <typename T>
absl::Status IsfComponentWidget<T>::ApplyCloseState(
    NodeHandle target, const std::optional<StateT>& state) {
  return UpdateComponentWithStateAuthoredSync<T>(target, state);
}

template <typename T>
void IsfComponentWidget<T>::SetComponentEnabled(bool enabled) {
  StatelessIsfComponentWidget<T>::SetComponentEnabled(enabled);
  NotifyComponentStateChanged();
}

template <typename T>
void IsfComponentWidget<T>::InvokeOnIsfStateChanged() {
  ComponentHandle<T> component = this->GetComponent();
  if constexpr (!component_traits::kShouldRunInEditMode<T>) {
    if (editor::IsInEditMode(component->GetView().GetRegistry())) {
      return;
    }
  }

  absl::Status status = InvokeOnIsfStateChangedSync(component);
  // TODO: handle this case more gracefully. Maybe we can
  // call OnIsfStateChanged again with the old values (if we have them).
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to apply changes to component: " << status;
  }
}

template <typename T>
void IsfComponentWidget<T>::NotifyComponentStateChanged() {
  ComponentHandle<T> component = this->GetComponent();
  if (!component) {
    return;
  }
  NodeHandle node = component->GetNode();

  ComponentStateChanged component_state_changed;
  component_state_changed.component_state =
      *proto::PackAny(IsfInfo::GetState(component));
  component_state_changed.component_enabled = component->IsEnabled();

  NodeUpdatedEvent event;
  event.target = node;
  event.enabled = node->IsEnabled();
  event.component = std::move(component_state_changed);
  editor_dispatcher_.Send(event);
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_COMPONENT_EDITOR_WIDGET_H_
