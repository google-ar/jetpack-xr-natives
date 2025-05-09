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

#ifndef THIRD_PARTY_IMPRESS_CORE_STATE_STATE_MACHINE_H_
#define THIRD_PARTY_IMPRESS_CORE_STATE_STATE_MACHINE_H_

#include "absl/status/status.h"
#include "absl/types/variant.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node.h"

namespace imp {
// The StateMachine helps manage components that represent State for the node.

// The StateMachine expects that all StateComponent types passed in are added to
// the node, otherwise its Setup function will fail. The initial state will be
// the InitialStateComponentType (e.g., the first State component of the
// templated parameters). During Setup, all other components besides the initial
// state component will be disabled, and the initial state will be enabled. As
// State changes, the active state will be enabled and all other components
// remain disabled. Each state component can handle a state change via its
// Component::OnActiveStatusChanged function. When the StateMachine component is
// removed, all State components will be disabled.
template <typename InitialStateComponentType, typename... StateComponentTypes>
class StateMachine : public Component {
 public:
  using StatesVariant =
      absl::variant<ComponentHandle<InitialStateComponentType>,
                    ComponentHandle<StateComponentTypes>...>;

  // Checks that all the states are valid components on the node and enables the
  // InitialStateComponentType.
  absl::Status Setup();
  // Disables all State components.
  void Cleanup();
  // Called by each state component to disable the current state and enable the
  // next state.
  template <typename NextStateType>
  void ChangeState();
  // Returns the current state.
  template <typename CurrentStateType>
  ComponentHandle<CurrentStateType> GetCurrentState() const;

 private:
  void EnableState(StatesVariant component, bool enable);
  StatesVariant current_state_ = ComponentHandle<InitialStateComponentType>();
};

template <typename InitialStateComponentType, typename... StateComponentTypes>
absl::Status
StateMachine<InitialStateComponentType, StateComponentTypes...>::Setup() {
  if (!GetNode()->template GetComponent<InitialStateComponentType>()) {
    return absl::FailedPreconditionError(
        absl::StrFormat("StateMachine's node is missing initial state type %s",
                        type_traits::kTypeName<InitialStateComponentType>));
  }
  // Ensure all other state components are valid and disabled before starting.
  if (!(GetNode()->template GetComponent<StateComponentTypes>() && ...)) {
    return absl::FailedPreconditionError(
        absl::StrFormat("StateMachine's node is missing expected state type %s",
                        type_traits::kTypeName<InitialStateComponentType>));
  }
  (EnableState(GetNode()->template GetComponent<StateComponentTypes>(), false),
   ...);
  ChangeState<InitialStateComponentType>();
  return absl::OkStatus();
}

template <typename InitialStateComponentType, typename... StateComponentTypes>
void StateMachine<InitialStateComponentType,
                  StateComponentTypes...>::Cleanup() {
  EnableState(GetNode()->template GetComponent<InitialStateComponentType>(),
              false);
  (EnableState(GetNode()->template GetComponent<StateComponentTypes>(), false),
   ...);
}

template <typename InitialStateComponentType, typename... StateComponentTypes>
template <typename NextStateType>
void StateMachine<InitialStateComponentType,
                  StateComponentTypes...>::ChangeState() {
  EnableState(current_state_, false);
  current_state_ = GetNode()->template GetComponent<NextStateType>();
  EnableState(current_state_, true);
}

template <typename InitialStateComponentType, typename... StateComponentTypes>
void StateMachine<InitialStateComponentType,
                  StateComponentTypes...>::EnableState(StatesVariant component,
                                                       bool enable) {
  absl::visit(
      [this, enable](auto&& state_component) {
        using CurrentType =
            typename std::decay_t<decltype(state_component)>::ComponentType;
        ComponentHandle<CurrentType> state_comp =
            GetNode()->template GetComponent<CurrentType>();
        if (state_comp) {
          state_comp->SetEnabled(enable);
        }
      },
      component);
}

template <typename InitialStateComponentType, typename... StateComponentTypes>
template <typename CurrentStateType>
ComponentHandle<CurrentStateType>
StateMachine<InitialStateComponentType,
             StateComponentTypes...>::GetCurrentState() const {
  ComponentHandle<CurrentStateType> current_state =
      GetNode()->template GetComponent<CurrentStateType>();
  if (!current_state) {
    return current_state;
  }
  return current_state->IsEnabled() ? current_state
                                    : ComponentHandle<CurrentStateType>();
}

// StateMachine that can be loaded in a .isf file as a Stateless component.
//
// It's guaranteed that this component will be setup after all of the states.
//
// Example usage:
//
// Code:
// static constexpr absl::string_view kUrl = "foo.BarStateMachine";
// using BarStateMachine = IsfStateMachine<kUrl, FirstState, SecondState>;
//
// Isf:
// components: {
//   [type.googleapis.com/imp.StatelessComponent] {
//     type: "foo.BarStateMachine"
// }
template <const absl::string_view& type_url, typename InitialStateComponentType,
          typename... StateComponentTypes>
class IsfStateMachine
    : public StateMachine<InitialStateComponentType, StateComponentTypes...> {
 public:
  using IsfInfo = StatelessIsfInfo<
      IsfStateMachine, type_url,
      IsfDependencies<InitialStateComponentType, StateComponentTypes...>>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_STATE_STATE_MACHINE_H_
