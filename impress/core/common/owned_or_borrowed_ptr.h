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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_OR_BORROWED_PTR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_OR_BORROWED_PTR_H_

#include <cstdint>
#include <variant>

#include "core/common/owned_ptr.h"
#include "core/common/small_source_location.h"

namespace imp {

// OwnedOrBorrowedPtr holds either an OwnedPtr or a BorrowedPtr.
//
// This can be useful for defining a collection of pointers when having a
// mixture of owned pointers and borrowed pointers.
//
// This also provides convenient access to the pointer and avoids boilerplate
// code. Please use this with caution as this could potentially obscure memory
// ownership when used extensively because OwnedPtr and BorrowedPtr have
// different ownership semantics.
//
// For example, if you have an std::vector<OwnedOrBorrowedPtr<T>> and the
// collection includes both borrowed pointers and the owned pointer that owns
// the borrowed pointers, then when destroying the vector, you will need to
// ensure that the OwnedPtr is destroyed last.
template <typename T>
class OwnedOrBorrowedPtr {
 public:
  OwnedOrBorrowedPtr() = default;
  OwnedOrBorrowedPtr(OwnedPtr<T> owned_ptr);
  OwnedOrBorrowedPtr(BorrowedPtr<T> borrowed_ptr);

  T& operator*() const noexcept;
  T* operator->() const noexcept;

  explicit operator bool() const noexcept;

  // Returns true if holding an OwnedPtr.
  bool IsOwned() const;

  // Borrows the held object.
  //
  // If holding an OwnedPtr, this calls Borrow() on the OwnedPtr and returns the
  // BorrowedPtr<T>. If holding a BorrowedPtr, this returns a new
  // BorrowedPtr<T> associated with the new location.
  BorrowedPtr<T> Borrow(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // Returns the number of outstanding tracked BorrowedPtr<T> objects.
  //
  // If holding a BorrowedPtr, this number is inclusive of itself.
  uint16_t GetBorrowedCount() const;

  // Returns true if there are any outstanding BorrowedPtr<T> objects excluding
  // itself.
  bool HasOutstandingUsages() const;

 private:
  T* GetPointer() const;

  std::variant<BorrowedPtr<T>, OwnedPtr<T>> ptr_;
};

template <typename T>
OwnedOrBorrowedPtr<T>::OwnedOrBorrowedPtr(OwnedPtr<T> owned_ptr)
    : ptr_(std::move(owned_ptr)) {}

template <typename T>
OwnedOrBorrowedPtr<T>::OwnedOrBorrowedPtr(BorrowedPtr<T> borrowed_ptr)
    : ptr_(std::move(borrowed_ptr)) {}

template <typename T>
T& OwnedOrBorrowedPtr<T>::operator*() const noexcept {
  return *operator->();
}

template <typename T>
T* OwnedOrBorrowedPtr<T>::operator->() const noexcept {
  return GetPointer();
}

template <typename T>
OwnedOrBorrowedPtr<T>::operator bool() const noexcept {
  return GetPointer() != nullptr;
}

template <typename T>
bool OwnedOrBorrowedPtr<T>::IsOwned() const {
  return std::holds_alternative<OwnedPtr<T>>(ptr_) && GetPointer() != nullptr;
}

template <typename T>
T* OwnedOrBorrowedPtr<T>::GetPointer() const {
  return std::visit([](auto& ptr) -> T* { return ptr.operator->(); }, ptr_);
}

template <typename T>
BorrowedPtr<T> OwnedOrBorrowedPtr<T>::Borrow(SmallSourceLocation loc) const {
  if (!*this) {
    return nullptr;
  }

  if (std::holds_alternative<OwnedPtr<T>>(ptr_)) {
    return std::get<OwnedPtr<T>>(ptr_).Borrow(loc);
  } else {
    return std::get<BorrowedPtr<T>>(ptr_).WithNewLocation(loc);
  }
}

template <typename T>
uint16_t OwnedOrBorrowedPtr<T>::GetBorrowedCount() const {
  if (!*this) {
    return 0;
  }

  if (std::holds_alternative<OwnedPtr<T>>(ptr_)) {
    return std::get<OwnedPtr<T>>(ptr_).GetBorrowedCount();
  } else {
    return std::get<BorrowedPtr<T>>(ptr_).GetBorrowedCount();
  }
}

template <typename T>
bool OwnedOrBorrowedPtr<T>::HasOutstandingUsages() const {
  // If it's owned, we only need to check if the borrowed count is 0. If it's
  // borrowed, we need to incorporate the borrowed pointer itself (i.e. borrowed
  // count is 1).
  if (IsOwned()) {
    return GetBorrowedCount() > 0;
  } else {
    return GetBorrowedCount() > 1;
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_OR_BORROWED_PTR_H_
