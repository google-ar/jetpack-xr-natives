/*
 * Copyright 2025 Google LLC
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
#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_COPYABLE_PTR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_COPYABLE_PTR_H_

#include <cstddef>
#include <memory>

namespace imp {

// Provides unique_ptr like semantics, but is copyable and assignable.
//
// Copy and assignment will perform a copy of the contained value of type T. The
// copy uses the T's copy constructor / assignment operator.
//
// Useful if the owning class must have no user defined constructors but must be
// copyable: ie aggregate types.
//
//  The use case for this in impress is proto fields which must be
//  nullable, ie recursive message fields. These must be able to hold a value or
//  null, must be copyable, and must not have a user defined constructor.
//
// std::unique_ptr is recommended over this class otherwise.
//
// NOTE: Only the default deleter is supported.
template <typename T>
struct CopyablePtr : public std::unique_ptr<T> {
  using std::unique_ptr<T>::unique_ptr;  // inheriting constructors

  CopyablePtr(const CopyablePtr<T>& rhs)
      : std::unique_ptr<T>()  // Default construct the
                              // base unique_ptr
  {
    Copy(rhs);
  }

  CopyablePtr& operator=(const CopyablePtr<T>& rhs) {
    // Prevent self assignment.
    if (this != &rhs) {
      Copy(rhs);
    }
    return *this;
  }

  // Constructors for both CopyablePtr and unique_ptr are required.
  CopyablePtr(const std::unique_ptr<T>& rhs)
      : std::unique_ptr<T>()  // Default construct the
                              // base unique_ptr
  {
    Copy(rhs);
  }

  // Assignment operators for both CopyablePtr and unique_ptr are required.
  CopyablePtr& operator=(const std::unique_ptr<T>& rhs) {
    // Prevent self assignment.
    if (this != &rhs) {
      Copy(rhs);
    }
    return *this;
  }

  // Assign nullptr.
  //
  // Required to resolve ambiguity.
  CopyablePtr& operator=(std::nullptr_t) noexcept {
    this->reset();
    return *this;
  }

  CopyablePtr(CopyablePtr<T>&& rhs) noexcept
      : std::unique_ptr<T>(std::move(rhs)) {}

  CopyablePtr& operator=(CopyablePtr<T>&& rhs) noexcept {
    std::unique_ptr<T>::operator=(std::move(rhs));
    return *this;
  }

  CopyablePtr(std::unique_ptr<T>&& rhs) noexcept
      : std::unique_ptr<T>(std::move(rhs)) {}

  CopyablePtr& operator=(std::unique_ptr<T>&& rhs) noexcept {
    std::unique_ptr<T>::operator=(std::move(rhs));
    return *this;
  }

  // Copy the rhs into this using T's copy constructor.
  // If the copy constructor performs a deep copy, this will be a deep copy.
  inline void Copy(const std::unique_ptr<T>& rhs) {
    if (rhs) {
      this->reset(new T(*rhs));
    } else {
      this->reset(nullptr);
    }
  }
};

template <typename T, typename... Args>
CopyablePtr<T> MakeCopyablePtr(Args&&... args) {
  return CopyablePtr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_COPYABLE_PTR_H_
