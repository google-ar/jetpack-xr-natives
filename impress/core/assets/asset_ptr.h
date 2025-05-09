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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_ASSET_PTR_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_ASSET_PTR_H_

#include <cstddef>

#include "core/common/ref_counter.h"

namespace imp {

template <typename T>
class AssetCache;

// A reference counted pointer for an Imp Asset.
//
// The Imp AssetCache will automatically deallocate the asset when it is no
// longer being held onto by anything.
//
// This uses Impress's RefCounter utility to do the reference counting, but the
// actual memory management is done by AssetCache.
template <typename T>
class AssetPtr {
 public:
  AssetPtr() = default;

  // Accesses and de-references the stored pointer.
  const T& operator*() const noexcept;

  // Accesses stored pointer.
  const T* operator->() const noexcept;

  bool operator==(const AssetPtr<T>& other) const;
  bool operator!=(const AssetPtr<T>& other) const;
  bool operator==(std::nullptr_t) const;
  bool operator!=(std::nullptr_t) const;

  explicit operator bool() const noexcept;

  const T* Get() const noexcept;

  int GetUseCount() const noexcept;

  void Reset();

  template <typename H>
  friend H AbslHashValue(H h, const AssetPtr& ptr) {
    return H::combine(std::move(h), ptr.ptr_);
  }

 private:
  AssetPtr(T* ptr, RefCounter::Ref ref);

  const T* ptr_ = nullptr;
  RefCounter::Ref ref_;

  friend class AssetCache<T>;
  friend class AssetManager;
};

template <typename T>
AssetPtr<T>::AssetPtr(T* ptr, RefCounter::Ref ref) : ptr_(ptr), ref_(ref) {}

template <typename T>
const T& AssetPtr<T>::operator*() const noexcept {
  return *ptr_;
}

template <typename T>
const T* AssetPtr<T>::operator->() const noexcept {
  return ptr_;
}

template <typename T>
bool AssetPtr<T>::operator==(const AssetPtr<T>& other) const {
  return ptr_ == other.ptr_;
}

template <typename T>
bool AssetPtr<T>::operator!=(const AssetPtr<T>& other) const {
  return ptr_ != other.ptr_;
}

template <typename T>
bool AssetPtr<T>::operator==(std::nullptr_t) const {
  return ptr_ == nullptr;
}

template <typename T>
bool AssetPtr<T>::operator!=(std::nullptr_t) const {
  return ptr_ != nullptr;
}

template <typename T>
AssetPtr<T>::operator bool() const noexcept {
  return ptr_ != nullptr;
}

template <typename T>
const T* AssetPtr<T>::Get() const noexcept {
  return ptr_;
}

template <typename T>
int AssetPtr<T>::GetUseCount() const noexcept {
  return ref_.GetCount();
}

template <typename T>
void AssetPtr<T>::Reset() {
  ptr_ = nullptr;
  ref_ = RefCounter::Ref();
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_ASSET_PTR_H_
