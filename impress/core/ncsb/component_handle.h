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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_HANDLE_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_HANDLE_H_

#include <string>

#include "core/common/log.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "filament/libs/utils/include/utils/EntityManager.h"
#include "core/common/type_traits.h"
#include "core/ncsb/base_component_pool.h"
#include "core/ncsb/component_id.h"

namespace imp {

// Provides access to a Component with semantics that behave similarly to a
// weak pointer.
//
// The ComponentHandle remains valid until the Component is destroyed. The
// component can be destroyed by removing the component from the node, the node
// being destroyed, or if a new component of the same type is added to the node
// (which replaces the previous component).
//
// Note, if a component of type T is removed and then re-added to the same node,
// then the old ComponentHandles will be invalid as they were referencing
// the old component instance.
//
// ComponentHandle is cheap to copy and should be passed by value.
template <typename T>
class ComponentHandle {
 public:
  using ComponentType = T;
  explicit ComponentHandle(T& component);

  // Typecast operator that allows casting a ComponentHandle to a related type.
  //
  // Example:
  //
  // ComponentHandle<SubComponent> original_comp =
  //     node->AddComponent<SubComponent>();
  //
  // // Upcast the component to a base type.
  // ComponentHandle<SuperComponent> upcasted_comp =
  //     static_cast<ComponentHandle<SuperComponent>>(original_comp);
  //
  // // Downcast the component back to the original type.
  // ComponentHandle<SubComponent> downcasted_comp =
  //     static_cast<ComponentHandle<SubComponent>>(upcasted_comp);
  template <typename U>
  explicit operator ComponentHandle<U>() const;

  // Creates an invalid component handle.
  ComponentHandle() = default;

  const T& operator*() const noexcept;
  T& operator*() noexcept;
  const T* operator->() const noexcept;
  T* operator->() noexcept;

  bool operator==(const ComponentHandle<T>& other) const;
  bool operator!=(const ComponentHandle<T>& other) const;
  explicit operator bool() const noexcept;

  T* Get() const noexcept;

  // Returns the filament entity that this ComponentHandle is wrapping.
  // Do not use this API unless you understand the underlying details of
  // filament.
  utils::Entity GetEntity() const noexcept;

  bool IsValid() const;

  template <typename Sink>
  friend void AbslStringify(Sink& sink, const ComponentHandle<T>& component) {
    sink.Append(ToString(component));
  }

  // Necessary for absl hashing
  template <typename H>
  friend H AbslHashValue(H hash, const ComponentHandle<T>& handle) {
    return H::combine(std::move(hash), absl::HashOf(handle.component_));
  }

 private:
  void DCheckIsValid() const;

  utils::Entity entity_;
  BaseComponentPool* pool_ = nullptr;
  T* component_ = nullptr;
  ComponentKey key_;
};

template <typename T>
ComponentHandle<T>::ComponentHandle(T& component)
    : entity_(component.GetEntity()),
      pool_(&component.GetBaseComponentPool()),
      component_(&component),
      key_(component.GetComponentKey()) {}

template <typename T>
template <typename U>
ComponentHandle<T>::operator ComponentHandle<U>() const {
  static_assert(std::is_base_of_v<T, U> || std::is_base_of_v<U, T>,
                "Cannot convert ComponentHandle types, they are unrelated.");
  return ComponentHandle<U>(static_cast<U&>(*component_));
}

template <typename T>
const T& ComponentHandle<T>::operator*() const noexcept {
  DCheckIsValid();
  return *component_;
}

template <typename T>
T& ComponentHandle<T>::operator*() noexcept {
  DCheckIsValid();
  return *component_;
}

template <typename T>
const T* ComponentHandle<T>::operator->() const noexcept {
  DCheckIsValid();
  return component_;
}

template <typename T>
T* ComponentHandle<T>::operator->() noexcept {
  DCheckIsValid();
  return component_;
}

template <typename T>
ComponentHandle<T>::operator bool() const noexcept {
  return IsValid();
}

template <typename T>
bool ComponentHandle<T>::operator==(const ComponentHandle<T>& other) const {
  return component_ == other.component_;
}

template <typename T>
bool ComponentHandle<T>::operator!=(const ComponentHandle<T>& other) const {
  return component_ != other.component_;
}

template <typename T>
T* ComponentHandle<T>::Get() const noexcept {
  return component_;
}

template <typename T>
bool ComponentHandle<T>::IsValid() const {
  if (!component_) {
    return false;
  }

  if (key_) {
    // If the key isn't empty, then that means the PoolAllocator is enabled, so
    // we can check if the key is valid to determine if the component is valid.
    return pool_->IsKeyValid(key_);
  } else {
    // Fallback using a hash map lookup to determine if the component is valid.
    return pool_->TryGetRawComponentFromEntity(entity_) == component_;
  }
}

template <typename T>
void ComponentHandle<T>::DCheckIsValid() const {
  // This has a mild cost because the call to IsValid() will do a hashmap
  // lookup, which accumulates over large numbers of calls to ComponentHandle so
  // we don't do this check in opt builds.
#ifndef NDEBUG
  if (!IsValid()) {
    if (component_) {
      if (utils::EntityManager::get().isAlive(entity_)) {
        IMP_LOG(imp::FATAL)
            << "ComponentHandle is invalid, component was removed. Entity ="
            << entity_.getId();
      } else {
        IMP_LOG(imp::FATAL)
            << "ComponentHandle is invalid, entity was was destroyed. Entity ="
            << entity_.getId();
      }
    } else {
      IMP_LOG(imp::FATAL) << "ComponentHandle is null.";
    }
  }
#endif
}

template <typename T>
utils::Entity ComponentHandle<T>::GetEntity() const noexcept {
  return entity_;
}

template <typename T>
std::string ToString(const ComponentHandle<T>& handle) {
  if (!handle) {
    return absl::StrFormat("%s#INVALID", type_traits::GetTypeName<T>());
  } else {
    return absl::StrFormat("%s#%04x", type_traits::GetTypeName<T>(),
                           handle->GetEntity().getId());
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_HANDLE_H_
