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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_CONNECTION_OWNER_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_CONNECTION_OWNER_H_

#include <cstddef>

#include "absl/hash/hash.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_id.h"
#include "core/ncsb/node_handle.h"

namespace imp {

class Component;

// Used to connect a ComponentHandle<T>, Component*, NodeHandle, or void* to
// an event handler in the dispatcher.
//
// Allows implicit conversion for any of the above types.
//
// If the ConnectionOwner wraps a component, then the connection will
// automatically be disconnected from the dispatcher when the component is
// removed.
//
// If the ConnectionOwner wraps a node, then the connection will automatically
// be disconnected from the dispatcher when the node is destroyed.
//
// If the ConnectionOwner wraps a raw pointer, then the connection needs to
// be manually disconnected by calling Dispatcher::DisconnectAll(owner).
class ConnectionOwner {
 public:
  // Creates a ConnectionOwner from a raw component pointer.
  // This will cause the connection to automatically disconnect when the
  // component is removed.
  explicit ConnectionOwner(Component* component);

  // Creates a ConnectionOwner from a ComponentHandle<T>.
  // This will cause the connection to automatically disconnect when the
  // component is removed.
  template <typename T>
  explicit ConnectionOwner(ComponentHandle<T> component_handle);

  // Creates a ConnectionOwner from a NodeHandle.
  // This will cause the connection to automatically disconnect when the
  // node is destroyed.
  explicit ConnectionOwner(NodeHandle node);

  // Creates a ConnectionOwner from any raw pointer.
  // This will require the owner to be manually disconnected.
  explicit ConnectionOwner(void* pointer_owner);

  explicit ConnectionOwner(std::nullptr_t);

  // Creates an empty ConnectionOwner.
  ConnectionOwner();

  // Returns the owning pointer if the owner isn't a node or component.
  // Otherwise, returns nullptr.
  void* GetPointerOwner() const;

  // Returns the node if the owner is a node, or the node that the component
  // is attached to if the owner is a component.
  // Otherwise, returns an invalid handle.
  NodeHandle GetNode() const;

  // Returns the hash of the component if the owner is a component.
  // Otherwise, returns 0.
  ComponentId GetComponentId() const;

  // Returns true if this ConnectionOwner wraps a valid owner.
  bool IsValid() const;

  bool operator==(const ConnectionOwner& other) const;
  bool operator!=(const ConnectionOwner& other) const;

  template <typename H>
  friend H AbslHashValue(H h, const ConnectionOwner& owner) {
    return H::combine(std::move(h), owner.pointer_owner_, owner.node_,
                      owner.component_id_);
  }

 private:
  friend struct std::hash<ConnectionOwner>;

  void* pointer_owner_;
  NodeHandle node_;
  ComponentId component_id_;
};

template <typename T>
ConnectionOwner::ConnectionOwner(ComponentHandle<T> component_handle)
    : pointer_owner_(nullptr),
      node_(component_handle->GetNode()),
      component_id_(component_handle->GetComponentId()) {}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_DISPATCHER_CONNECTION_OWNER_H_
