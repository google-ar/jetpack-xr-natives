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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_H_

#include <utility>

#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/TransformManager.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_manager.h"
#include "core/ncsb/component_traits.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/groups_manager.h"  // IWYU pragma: keep
#include "core/ncsb/node_controller.h"

// This is designed to be the client-facing header to include for Node.
// It exports NodeHandle and the definition of the Node class for IWYU.
// IWYU pragma: begin_exports
#include "core/ncsb/node_children_iterator.h"
#include "core/ncsb/node_definition.h"
#include "core/ncsb/node_handle.h"
// IWYU pragma: end_exports

#include "core/view/base_view.h"

namespace imp {

inline BaseView& Node::GetView() const { return node_controller_->GetView(); }

inline filament::Engine* Node::GetEngine() const {
  return BaseView::GetSharedEngine();
}

inline filament::TransformManager& Node::GetTransformManager() const {
  return GetEngine()->getTransformManager();
}

inline ComponentManager& Node::GetComponentManager() const {
  return GetView().GetComponentManager();
}

inline Dispatcher& Node::GetDispatcher() const {
  return GetView().GetDispatcher();
}

template <typename T, typename... Args>
component_traits::AddResult<T, Args...> Node::AddComponent(Args&&... args) {
  return GetComponentManager().Add<T>(NodeHandle(*this),
                                      std::forward<Args>(args)...);
}

template <typename T, typename... Args>
component_traits::AddWithStateResult<T, Args...> Node::AddComponentWithState(
    typename T::IsfInfo::StateT state, Args&&... args) {
  return GetComponentManager().AddWithState<T>(
      NodeHandle(*this), std::move(state), std::forward<Args>(args)...);
}

template <typename T>
ComponentHandle<T> Node::GetComponent() const {
  return GetComponentManager().Get<T>(entity_);
}

template <typename T>
void Node::RemoveComponent() {
  GetComponentManager().Remove<T>(entity_);
}

template <typename T, typename... Args>
component_traits::AddResult<T, Args...> Node::GetOrAddComponent(
    Args&&... args) {
  return GetComponentManager().GetOrAdd<T>(NodeHandle(*this),
                                           std::forward<Args>(args)...);
}

template <typename T, typename... Args>
component_traits::AddWithStateResult<T, Args...>
Node::GetOrAddComponentWithState(typename T::IsfInfo::StateT state,
                                 Args&&... args) {
  return GetComponentManager().GetOrAddWithState<T>(
      NodeHandle(*this), std::move(state), std::forward<Args>(args)...);
}

template <typename EventType>
NodeHandle Node::Send(const EventType& event) {
  return GetDispatcher().Send(NodeHandle(*this),
                              std::forward<const EventType>(event));
}

template <typename Fn>
DispatcherConnection Node::Connect(Fn&& handler) {
  NodeHandle handle(*this);
  return Connect(std::forward<Fn>(handler), handle);
}

template <typename Owner, typename Fn>
DispatcherConnection Node::Connect(Fn&& handler, Owner owner) {
  NodeHandle handle(*this);
  return GetDispatcher().Connect(handle, std::forward<Fn>(handler),
                                 std::forward<Owner>(owner));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_NODE_H_
