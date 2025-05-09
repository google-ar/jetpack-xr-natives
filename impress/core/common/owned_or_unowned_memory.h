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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_OR_UNOWNED_MEMORY_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_OR_UNOWNED_MEMORY_H_

#include <memory>
#include <variant>

#include "core/common/log.h"
#include "core/common/platform_helpers.h"

namespace imp {

// OwnedOrUnownedMemory holds either a unique pointer(owned) a raw
// pointer(unowned). This can be useful for defining a collection of pointers
// when having a mixture of unique pointers and raw pointers. This also provides
// convenient access to the pointer and avoids boilerplate code. Please use this
// with caution as this could potentially obscure memory ownership when used
// extensively.
//
// OwnedOrUnownedMemory is not copyable and is move only.
template <typename T>
class OwnedOrUnownedMemory {
 public:
  OwnedOrUnownedMemory() : memory_(nullptr) {}

  // OwnedOrUnownedMemory is not copyable.
  OwnedOrUnownedMemory(const OwnedOrUnownedMemory<T>& other) = delete;
  // Move constructor. Transfers ownership.
  OwnedOrUnownedMemory(OwnedOrUnownedMemory<T>&& other)
      : memory_(std::move(other.memory_)) {}

  // OwnedOrUnownedMemory is not copyable.
  OwnedOrUnownedMemory<T>& operator=(const OwnedOrUnownedMemory<T>& other) =
      delete;
  // Move assignment operator. Transfers ownership.
  OwnedOrUnownedMemory<T>& operator=(OwnedOrUnownedMemory<T>&& other);

  explicit OwnedOrUnownedMemory(std::unique_ptr<T> memory)
      : memory_(std::move(memory)) {}
  explicit OwnedOrUnownedMemory(T* memory) : memory_(memory) {}

  // Accesses the memory as a reference. Asserts that the memory is valid.
  T& operator*() const noexcept;

  // Accesses the memory as a pointer. Asserts that the memory is valid.
  T* operator->() const noexcept;

  // Returns whether the memory its holding is valid.
  explicit operator bool() const noexcept;

  // Returns whether OwnedOrUnownedMemory owns the memory.
  // Please note that this will also return false if OwnedOrUnownedMemory holds
  // an empty unique_ptr.
  bool OwnsMemory() const;

  // Returns a raw pointer of the memory.
  T* Get() const;

  void Set(std::unique_ptr<T> memory);
  void Set(T* memory);

  // Resets the pointer.
  // If OwnedOrUnownedMemory holds a unique_ptr, std::unique_ptr::reset() will
  // be called.
  // If OwnedOrUnownedMemory holds a raw pointer, this simply sets the pointer
  // to be nullptr.
  // Get() will return nullptr after Reset() is called.
  void Reset();

  // If OwnedOrUnownedMemory holds a unique_ptr, moves the memory to a new
  // unique_ptr.
  // If OwnedOrUnownedMemory holds a raw pointer, this is no-op and an empty
  // unique_ptr will be returned.
  std::unique_ptr<T> Release();

 private:
  bool HoldsUniquePtr() const;

  std::variant<T*, std::unique_ptr<T>> memory_;
};

template <typename T>
OwnedOrUnownedMemory<T>& OwnedOrUnownedMemory<T>::operator=(
    OwnedOrUnownedMemory<T>&& other) {
  memory_ = std::move(other.memory_);
  return *this;
}

template <typename T>
T& OwnedOrUnownedMemory<T>::operator*() const noexcept {
  return *operator->();
}

template <typename T>
T* OwnedOrUnownedMemory<T>::operator->() const noexcept {
  T* memory = Get();

  if (!memory) {
    IMP_LOG(imp::FATAL) << "OwnedOrUnownedMemory: Deferencing invalid memory";
  }

  return memory;
}

template <typename T>
OwnedOrUnownedMemory<T>::operator bool() const noexcept {
  return Get();
}

template <typename T>
bool OwnedOrUnownedMemory<T>::OwnsMemory() const {
  return HoldsUniquePtr() && std::get<std::unique_ptr<T>>(memory_);
}

template <typename T>
T* OwnedOrUnownedMemory<T>::Get() const {
  if (!HoldsUniquePtr()) {
    return std::get<T*>(memory_);
  } else {
    return std::get<std::unique_ptr<T>>(memory_).get();
  }
}

template <typename T>
void OwnedOrUnownedMemory<T>::Set(std::unique_ptr<T> memory) {
  memory_ = std::move(memory);
}

template <typename T>
void OwnedOrUnownedMemory<T>::Set(T* memory) {
  memory_ = memory;
}

template <typename T>
void OwnedOrUnownedMemory<T>::Reset() {
  if (!HoldsUniquePtr()) {
    memory_ = nullptr;
  } else {
    std::get<std::unique_ptr<T>>(memory_).reset();
    memory_ = nullptr;
  }
}

template <typename T>
std::unique_ptr<T> OwnedOrUnownedMemory<T>::Release() {
  if (HoldsUniquePtr()) {
    auto moved_memory = std::move(std::get<std::unique_ptr<T>>(memory_));
    memory_ = nullptr;
    return moved_memory;
  } else {
    return std::unique_ptr<T>();
  }
}

template <typename T>
bool OwnedOrUnownedMemory<T>::HoldsUniquePtr() const {
  return std::holds_alternative<std::unique_ptr<T>>(memory_);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_OR_UNOWNED_MEMORY_H_
