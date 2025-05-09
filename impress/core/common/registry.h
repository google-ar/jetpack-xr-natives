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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_REGISTRY_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_REGISTRY_H_

#include <cassert>
#include <functional>
#include <memory>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "core/common/hash.h"
#include "core/common/type_traits.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

// A map of objects of any type registered based on its type.
//
// This class can be used to simplify dependency injection.  Rather than passing
// multiple pointers to various objects to the constructor of a class, a pointer
// to a Registry can be used and the individual object pointers can be extracted
// from the Registry.
//
// The Registry is the sole owner of the objects created/registered with it.
// It provides a raw-pointer to the object when requested.  The Registry will
// destroy all objects (in reverse order of creation/registration) when it
// itself is destroyed.
//
// This class is not thread-safe.
class Registry {
 public:
  Registry();
  ~Registry();

  void Clear();

  // Gets a reference to the object instance of type |T| if one had previously
  // been registered.
  //
  // Otherwise, creates an object of type |T| and registers it, using |Args| as
  // its constructor arguments. Then, returns a reference pointer to the newly
  // registered object.
  template <typename T, typename... Args>
  T& GetOrCreate(Args&&... args);

  // Registers an object of type |T| so that it can be looked up in the
  // Registry.
  //
  // If an object of type |T| had previously been registered, unregisters the
  // old object before registering the new one.
  template <typename T, typename Deleter = std::default_delete<T>>
  void Register(std::unique_ptr<T, Deleter> obj);

  // Registers an object of type |U| so that it can be looked up in the registry
  // as type |T|.
  //
  // Type |U| must be a subclass of type |T|. This is useful for registering an
  // object via its base type. Common in cases where there could be different
  // implementations of the same interface in the registry. For example, a mock
  // implementation.
  template <typename T, typename U>
  void Register(std::unique_ptr<U> obj);

  // Unregisters and destroys object of type |T| if one had been registered.
  // Otherwise, does nothing.
  template <typename T>
  void Unregister();

  // Gets a reference to the object instance of type |T| if one had previously
  // been registered.
  //
  // Otherwise, call Fn to create an object of type |T| and register it. Then,
  // returns a reference pointer to the newly registered object.
  //
  // Fn is only called if an object of type |T| wasn't already registered.
  // Fn must return one of the following types:
  //   - std::unique_ptr<T>
  //   - A unique_ptr to a subclass of T
  //   - A unique_ptr of T with a custom deleter.
  template <typename T, typename Fn>
  T& GetOrRegister(Fn fn);

  // Gets a reference to an object of type |T| that had been previously
  // registered with the registry.
  //
  // If no object of type |T| is registered, then returns a failure status with
  // the status code kUnavailable.
  template <typename T>
  absl::StatusOr<std::reference_wrapper<T>> Get();
  template <typename T>
  absl::StatusOr<std::reference_wrapper<const T>> Get() const;

 private:
  template <typename T>
  absl::Status GetErrorForUnregisteredObject() const;

  using Pointer = std::unique_ptr<void, std::function<void(void*)>>;
  using TypedPointer = std::pair<HashValue, Pointer>;
  using ObjectList = std::vector<TypedPointer>;

  // Store a raw pointer in the ObjectTable so that the lifetime of the object
  // can be more explicitly controlled (and for slightly better performance).
  using ObjectTable = tsl::robin_map<HashValue, void*>;

  ObjectList objects_;  // List of Objects in order of creation that is used to
                        // destroy them in reverse order.
  ObjectTable table_;   // Map of Objects and their HashValues for lookup.

  Registry(const Registry&) = delete;
  Registry& operator=(const Registry&) = delete;
};

template <typename T, typename... Args>
T& Registry::GetOrCreate(Args&&... args) {
  absl::StatusOr<std::reference_wrapper<T>> previously_registered = Get<T>();
  if (previously_registered.ok()) {
    return previously_registered.value().get();
  }

  auto unique_ptr = std::make_unique<T>(std::forward<Args>(args)...);
  T* ptr = unique_ptr.get();
  Register(std::move(unique_ptr));
  return *ptr;
}

template <typename T, typename Deleter /*= std::default_delete<T>*/>
void Registry::Register(std::unique_ptr<T, Deleter> obj) {
  Unregister<T>();

  assert(obj != nullptr);
  constexpr HashValue type = type_traits::kTypeHash<T>;

  Pointer ptr(obj.release(),
              [deleter = std::move(obj.get_deleter())](void* held_ptr) {
                // Capture deleter from the registered object in a type-erased
                // deleter and forward to it.
                deleter(static_cast<T*>(held_ptr));
              });

  table_.emplace(type, ptr.get());
  objects_.emplace_back(type, std::move(ptr));
}

template <typename T, typename U>
void Registry::Register(std::unique_ptr<U> obj) {
  static_assert(std::is_base_of_v<T, U>);
  Register(std::unique_ptr<T, void (*)(void*)>(
      obj.release(), +[](void* ptr) { delete static_cast<U*>(ptr); }));
}

template <typename T>
void Registry::Unregister() {
  constexpr HashValue type = type_traits::kTypeHash<T>;
  table_.erase(type);

  auto itr = std::remove_if(objects_.begin(), objects_.end(),
                            [](const TypedPointer& typed_pointer) {
                              return type == typed_pointer.first;
                            });
  if (itr != objects_.end()) {
    objects_.erase(itr);
  }
}

template <typename T, typename Fn>
T& Registry::GetOrRegister(Fn fn) {
  absl::StatusOr<std::reference_wrapper<T>> previously_registered = Get<T>();
  if (previously_registered.ok()) {
    return previously_registered.value().get();
  }

  auto unique_ptr = fn();
  T* ptr = unique_ptr.get();
  Register<T>(std::move(unique_ptr));
  return *ptr;
}

template <typename T>
absl::StatusOr<std::reference_wrapper<T>> Registry::Get() {
  auto iter = table_.find(type_traits::kTypeHash<T>);
  if (iter == table_.end()) {
    return GetErrorForUnregisteredObject<T>();
  }

  T* obj = static_cast<T*>(iter->second);
  return *obj;
}

template <typename T>
absl::StatusOr<std::reference_wrapper<const T>> Registry::Get() const {
  auto iter = table_.find(type_traits::kTypeHash<T>);
  if (iter == table_.end()) {
    return GetErrorForUnregisteredObject<T>();
  }

  T* obj = static_cast<T*>(iter->second);
  return *obj;
}

template <typename T>
absl::Status Registry::GetErrorForUnregisteredObject() const {
  return absl::NotFoundError(
      absl::StrFormat("No object of type %s is registered with the Registry.",
                      type_traits::kTypeName<T>));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_REGISTRY_H_
