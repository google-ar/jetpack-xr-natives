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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_COMPONENT_EDITOR_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_COMPONENT_EDITOR_HELPERS_H_

#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/config.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/scene_metadata.h"

namespace imp::editor {

// Add a component with the given state to the node.
//
// This is a blocking operation that will wait for the component to be added
// before returning.
//
// This is not recommended for general use, but is useful for adding components
// in the editor, where the component needs to be added synchronously in order
// to avoid subtle timing issues & bugs that can occur when adding components
// asynchronously.
template <typename ComponentT, typename StateT>
absl::StatusOr<ComponentHandle<ComponentT>> AddComponentWithStateSync(
    NodeHandle node, StateT state) {
  auto add_result = node->AddComponentWithState<ComponentT>(std::move(state));
  using AddResultT = decltype(add_result);
  if constexpr (std::is_same_v<AddResultT,
                               Future<ComponentHandle<ComponentT>>>) {
    while (!add_result.Ready()) {
      imp::Executor::CurrentExecutor()->Pump(/*drain =*/false);
    }
    return add_result.Get();
  } else {
    return absl::StatusOr<ComponentHandle<ComponentT>>(add_result);
  }
}

// Add a component to the node.
//
// This is a blocking operation that will wait for the component to be added
// before returning.
//
// This is not recommended for general use, but is useful for adding components
// in the editor, where the component needs to be added synchronously in order
// to avoid subtle timing issues & bugs that can occur when adding components
// asynchronously.
template <typename ComponentT>
absl::StatusOr<ComponentHandle<ComponentT>> AddComponentSync(NodeHandle node) {
  auto add_result = node->AddComponent<ComponentT>();
  using AddResultT = decltype(add_result);
  if constexpr (std::is_same_v<AddResultT,
                               Future<ComponentHandle<ComponentT>>>) {
    while (!add_result.Ready()) {
      imp::Executor::CurrentExecutor()->Pump(/*drain =*/false);
    }
    return add_result.Get();
  } else {
    return absl::StatusOr<ComponentHandle<ComponentT>>(add_result);
  }
}

// Updates a component with state and marks it as authored in SceneMetadata.
//
// If state is nullopt, then the component is removed and marked as unauthored.
template <typename ComponentT>
absl::Status UpdateComponentWithStateAuthoredSync(
    NodeHandle target,
    const std::optional<typename ComponentT::IsfInfo::StateT>& state) {
  absl::Status status = absl::OkStatus();
  bool is_authored = false;

  if (!state) {
    target->RemoveComponent<ComponentT>();
  } else {
    const absl::StatusOr<ComponentHandle<ComponentT>> result =
        AddComponentWithStateSync<ComponentT>(target, *state);
    status = result.status();
    is_authored = true;
  }

  ComponentHandle<SceneMetadata> scene_metadata =
      target->GetComponent<SceneMetadata>();

  if (scene_metadata) {
    scene_metadata->SetComponentAuthored(ComponentT::IsfInfo::kTypeUrlHash,
                                         is_authored);
  }

  return status;
}

// Updates a component and marks it as authored in SceneMetadata.
//
// If value is false, then the component is removed and marked as unauthored.
template <typename ComponentT>
absl::Status UpdateComponentAuthoredSync(NodeHandle target, bool value) {
  absl::Status status = absl::OkStatus();
  bool is_authored = false;

  if (!value) {
    target->RemoveComponent<ComponentT>();
  } else {
    const auto result = AddComponentSync<ComponentT>(target);
    status = result.status();
    is_authored = true;
  }

  ComponentHandle<SceneMetadata> scene_metadata =
      target->GetComponent<SceneMetadata>();

  if (scene_metadata) {
    scene_metadata->SetComponentAuthored(ComponentT::IsfInfo::kTypeUrlHash,
                                         is_authored);
  }

  return status;
}

// Invokes the OnIsfStateChanged method on the component.
//
// This is a blocking operation that will wait until any async operations
// triggered by OnIsfStateChanged are complete before returning.
//
// This is not recommended for general use, but is useful in the editor, where
// some operations must be performed synchronously in order to avoid subtle
// timing issues & bugs that can occur when doing asynchronous work.
template <typename ComponentT>
absl::Status InvokeOnIsfStateChangedSync(
    ComponentHandle<ComponentT> component) {
  // Helper used to deduce the return type of OnIsfStateChanged.
  auto on_isf_state_changed_invoker = [](auto component) -> auto {
    return component->OnIsfStateChanged();
  };
  using ResultT = decltype(on_isf_state_changed_invoker(component));

  if constexpr (std::is_same_v<ResultT, void>) {
    component->OnIsfStateChanged();
    return absl::OkStatus();
  } else if constexpr (std::is_same_v<ResultT, absl::Status>) {
    return component->OnIsfStateChanged();
  } else {
    static_assert(std::is_same_v<ResultT, Future<absl::Status>>);
    Future<absl::Status> result = component->OnIsfStateChanged();
#if IMP_PLATFORM(WASM)
    // On WASM, blocking the main thread in a spinloop prevents browser async
    // fetch callbacks from ever executing, which freezes the application.
    // Return early if the future is pending; callbacks attached to the future
    // will execute when the asset finishes loading asynchronously.
    if (!result.Ready()) {
      result.KeptBy(component);
      return absl::OkStatus();
    }
#else
    while (!result.Ready()) {
      imp::Executor::CurrentExecutor()->Pump(/*drain =*/false);
    }
#endif
    return result.Get();
  }
}

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_COMPONENT_HELPERS_H_
