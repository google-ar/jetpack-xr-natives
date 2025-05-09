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

#include <optional>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/common/invocable.h"
#include "core/editor/command_manager.h"
#include "core/editor/component_editor.proto.imp.h"
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

  explicit IsfComponentWidget(ComponentHandle<T> component,
                              Dispatcher& editor_dispatcher);

  void DrawImGui() override;

  Invocable<void()> OnCloseButton() override;

  void SetComponentEnabled(bool enabled) override;

 private:
  void InvokeOnIsfStateChanged();
  void NotifyComponentStateChanged();

  EditorProtoVisitor<StateT> visitor_;
  Dispatcher& editor_dispatcher_;

  // Stores the state of the component before it changes, for creating a command
  // to undo the change.
  std::optional<StateT> prev_state_;

  std::optional<StateT> base_state_;
};

template <typename T>
IsfComponentWidget<T>::IsfComponentWidget(ComponentHandle<T> component,
                                          Dispatcher& editor_dispatcher)
    : StatelessIsfComponentWidget<T>(component),
      visitor_(IsfInfo::GetState(component)),
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

  // Track the previous state for undo/redo.
  if (!prev_state_) {
    prev_state_ = IsfInfo::GetState(component);
  }

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
  // for notifying the component of the change & making it possible to undo the
  // change.
  if (visitor_.AnyFieldEdited()) {
    bool is_first_perform = true;
    CommandManager& command_manager = this->GetCommandManager();
    command_manager.template PerformCommand<NodeValueCommand<StateT>>(
        component->GetNode(), *prev_state_, IsfInfo::GetState(component),
        [this, component, is_first_perform](NodeHandle target,
                                            const StateT& state) mutable {
          if (!component) {
            return absl::NotFoundError("Component not found");
          }

          if (!is_first_perform) {
            // Only set the state if this is the not first perform. This
            // avoids an unnecessary copy of the state, because on the first
            // perform it's already correct. It only needs to be updated during
            // undo/redo actions.
            IsfInfo::GetState(component) = std::move(state);
          }

          InvokeOnIsfStateChanged();
          NotifyComponentStateChanged();

          is_first_perform = false;

          return absl::OkStatus();
        });

    prev_state_.reset();
  }
}

template <typename T>
Invocable<void()> IsfComponentWidget<T>::OnCloseButton() {
  // If this component comes from a base isf file then it can't be removed.
  if (StatelessIsfComponentWidget<T>::GetBaseComponentSource()) {
    return {};
  }

  return [this]() {
    ComponentHandle<T> component = this->GetComponent();
    NodeHandle node = component->GetNode();
    StateT& state = IsfInfo::GetState(component);

    CommandManager& command_manager = this->GetCommandManager();
    command_manager.PerformCommand<NodeValueCommand<std::optional<StateT>>>(
        node, state, std::nullopt,
        [](NodeHandle target,
           const std::optional<StateT>& state) -> absl::Status {
          if (!state) {
            target->RemoveComponent<T>();
            return absl::OkStatus();
          }

          return AddComponentWithStateSync<T>(target, *state).status();
        });
  };
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
