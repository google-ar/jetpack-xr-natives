/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_STATE_MACHINE_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_STATE_MACHINE_H_

#include <cstddef>
#include <optional>
#include <type_traits>
#include <variant>

#include "core/common/log.h"
#include "filament/libs/utils/include/utils/compiler.h"
#include "core/common/visit.h"

namespace svxr {

namespace scene_viewer_ux_details {

template <typename...>
struct TypeIndexInPack;

// Termination case
template <typename T, typename... Rest>
struct TypeIndexInPack<T, T, Rest...> : std::integral_constant<size_t, 0> {};

// Iteration case
template <typename T, typename First, typename... Rest>
struct TypeIndexInPack<T, First, Rest...>
    : std::integral_constant<size_t, 1 + TypeIndexInPack<T, Rest...>::value> {};

// Invalid case
template <typename...>
struct IsFirstInRest {
  static constexpr bool value = false;
};

// Termination case
template <typename T, typename... Rest>
struct IsFirstInRest<T, T, Rest...> {
  static constexpr bool value = true;
};

// Iteration case
template <typename T, typename First, typename... Rest>
struct IsFirstInRest<T, First, Rest...> {
  static constexpr bool value = IsFirstInRest<T, Rest...>::value;
};

}  // namespace scene_viewer_ux_details

// Represents a state machine
template <typename... States>
class StateMachine {
  static constexpr size_t kStateCount = sizeof...(States);
  static constexpr size_t kAnyStateSentinel = kStateCount + 1;

 public:
  using State = std::variant<States...>;
  using OptionalState = std::optional<State>;
  template <typename S>
  static constexpr size_t kIndexOf =
      scene_viewer_ux_details::TypeIndexInPack<S, States...>::value;
  template <typename S>
  static constexpr bool kIsValidState =
      scene_viewer_ux_details::IsFirstInRest<std::decay_t<S>, States...>::value;

  class Observer {
   public:
    virtual ~Observer() = default;
    virtual void OnStateChange(const StateMachine& machine,
                               const State& currentState,
                               const State& nextState) = 0;
  };

  // Default-constructing a state machine default-constructs the first state.
  StateMachine() = default;

  // StateMachine can construct with any valid state object.
  template <typename S, std::enable_if_t<kIsValidState<S>, int> = 0>
  explicit StateMachine(S&& initialState, Observer* observer = nullptr) noexcept
      : state_(std::forward<S>(initialState)), observer_(observer) {}

  explicit StateMachine(Observer* observer = nullptr) noexcept
      : state_(), observer_(observer) {}

  // Updates a state machine via a set of candidate functions provided as
  // arguments, e.g.:
  //
  // mMachine.UpdateWithAlternatives(
  //     [](StateA& s) -> OptionalState {...},
  //     [](StateB& s) -> OptionalState {...},
  //     ...
  //     [](StateN& s) -> OptionalState {...});
  //
  // The state fields are mutable.
  // Returning a default-constructed (i.e. empty) result means "remain in the
  // same state". Otherwise the result defines the next state for the machine.
  //
  // If set at construction time, Observer::OnStateChange() will be called on
  // each state change. Update proceeds through states until the machine
  // performs a step without changing state. Returning the same state type that
  // is passed in works the same as mutating fields and returning an empty
  // result.
  //
  // The first suitable candidate found in the argument list is the one the
  // compiler will select to call. Note that the alternative set must be
  // exhaustive (i.e. every possible state must be covered by the set of
  // alternatives).  This is enforced at compile time, but for convenience a
  // stub closure with parameter type 'auto&' as the last entry will work for
  // any state.
  template <typename... Alternatives>
  void UpdateWithAlternatives(Alternatives&&... alternatives);

  // Like UpdateWithAlternatives, but does not change state and returns the
  // result from the selected closure.
  template <typename... Alternatives>
  auto ApplyWithAlternatives(Alternatives&&... alternatives);

  // True if the current state type is S (requires S to be a valid State)
  template <typename S>
  bool InState() const;

  // Non-null if the current state type is S
  template <typename S>
  S* TryGet();

  // Non-null if the current state type is S
  template <typename S>
  const S* TryGet() const;

  // Non-fatal if the current state type is S
  template <typename S>
  S& Get();

  // Non-fatal if the current state type is S
  template <typename S>
  const S& Get() const;

 protected:
  // Directly setting the state should never be performed by clients; they use
  // UpdateWithAlternatives.
  template <typename S>
  void SetNextState(S&& next_explicit_state);
  void SetNextState(OptionalState nextState);

  template <typename Visitor>
  void UpdateWithVisitor(Visitor&& visitor);

  template <typename Visitor>
  auto ApplyWithVisitor(Visitor&& visitor);

 private:
  State state_ = {};
  Observer* observer_ = nullptr;
};

template <typename... States>
template <typename S>
bool StateMachine<States...>::InState() const {
  static_assert(kIsValidState<S>, "InState called with invalid state type");
  // todo: if the first state is not default constructible, account for nullopt
  return state_.index() == kIndexOf<S>;
}

template <typename... States>
template <typename S>
S* StateMachine<States...>::TryGet() {
  static_assert(kIsValidState<S>, "TryGet called with invalid state type");
  if (!absl::holds_alternative<S>(state_)) {
    return nullptr;
  }
  return &absl::get<S>(state_);
}

template <typename... States>
template <typename S>
const S* StateMachine<States...>::TryGet() const {
  static_assert(kIsValidState<S>, "TryGet called with invalid state type");
  if (!absl::holds_alternative<S>(state_)) {
    return nullptr;
  }
  return &absl::get<S>(state_);
}

template <typename... States>
template <typename S>
S& StateMachine<States...>::Get() {
  static_assert(kIsValidState<S>, "Get called with invalid state type");
  return absl::get<S>(state_);
}

template <typename... States>
template <typename S>
const S& StateMachine<States...>::Get() const {
  static_assert(kIsValidState<S>, "Get called with invalid state type");
  return absl::get<S>(state_);
}

template <typename... States>
template <typename S>
void StateMachine<States...>::SetNextState(S&& next_explicit_state) {
  static_assert(kIsValidState<S>,
                "SetNextState called with invalid state type");
  SetNextState(OptionalState{std::forward<S>(next_explicit_state)});
}

template <typename... States>
void StateMachine<States...>::SetNextState(OptionalState nextState) {
  if (!nextState) return;

  if (nextState->index() == state_.index()) {
    state_ = std::move(*nextState);
    return;
  }

  if (observer_) observer_->OnStateChange(*this, state_, *nextState);
  state_ = std::move(*nextState);
}

template <typename... States>
template <typename Visitor>
void StateMachine<States...>::UpdateWithVisitor(Visitor&& visitor) {
  OptionalState next_state;
  for (;;) {
    size_t beforeIndex = state_.index();
    next_state = std::visit(visitor, state_);
    if (UTILS_UNLIKELY(beforeIndex != state_.index()))
      IMP_LOG(imp::FATAL) << "Visitor directly changed state";
    if (next_state) SetNextState(std::move(next_state));
    if (beforeIndex == state_.index()) break;
  }
}

template <typename... States>
template <typename... Alternatives>
void StateMachine<States...>::UpdateWithAlternatives(
    Alternatives&&... alternatives) {
  UpdateWithVisitor(
      imp::MakeOverload{std::forward<Alternatives>(alternatives)...});
}

template <typename... States>
template <typename Visitor>
auto StateMachine<States...>::ApplyWithVisitor(Visitor&& visitor) {
  return std::visit(visitor, state_);
}

template <typename... States>
template <typename... Alternatives>
auto StateMachine<States...>::ApplyWithAlternatives(
    Alternatives&&... alternatives) {
  return ApplyWithVisitor(
      imp::MakeOverload{std::forward<Alternatives>(alternatives)...});
}

}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_STATE_MACHINE_H_
