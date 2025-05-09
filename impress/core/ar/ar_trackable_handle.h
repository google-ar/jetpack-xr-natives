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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_HANDLE_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_HANDLE_H_
#include <atomic>

#include "core/common/log.h"
#include "core/ar/ar_trackable_id.h"
#include "core/ar/ar_trackables_manager.h"
#include "core/common/platform_helpers.h"

namespace imp {
namespace ar {

// Provides safe access to the underlaying Ar trackable data.
template <typename T>
class ArTrackableHandle {
 public:
  using TrackableType = T;
  explicit ArTrackableHandle(ArTrackablesManager* trackables_manager,
                             ArTrackableId id);

  // Creates an invalid component handle.
  ArTrackableHandle() = default;

  T& operator*() const noexcept;
  T& operator*() noexcept;
  T* operator->() const noexcept;
  T* operator->() noexcept;

  bool operator==(const ArTrackableHandle<T>& other) const;
  bool operator!=(const ArTrackableHandle<T>& other) const;
  explicit operator bool() const noexcept;

  T* Get() const noexcept;
  ArTrackableId GetId() const { return trackable_id_; }
  bool IsValid() const;

  template <typename H>
  friend H AbslHashValue(H h, const ArTrackableHandle<T>& handle) {
    return H::combine(std::move(h), handle.trackable_id_);
  }

 private:
  friend class ArTrackablesManager;

  void AssertIsValid() const;

  ArTrackablesManager* trackables_manager_;
  ArTrackableId trackable_id_;
  static std::atomic_bool session_is_valid_;
};

template <typename T>
std::atomic_bool ArTrackableHandle<T>::session_is_valid_ = false;

template <typename T>
ArTrackableHandle<T>::ArTrackableHandle(ArTrackablesManager* trackables_manager,
                                        ArTrackableId id)
    : trackables_manager_(trackables_manager), trackable_id_(id) {}

template <typename T>
T& ArTrackableHandle<T>::operator*() const noexcept {
  return *operator->();
}

template <typename T>
T& ArTrackableHandle<T>::operator*() noexcept {
  return *operator->();
}

template <typename T>
T* ArTrackableHandle<T>::operator->() const noexcept {
  AssertIsValid();
  return Get();
}

template <typename T>
T* ArTrackableHandle<T>::operator->() noexcept {
  AssertIsValid();
  return Get();
}

template <typename T>
ArTrackableHandle<T>::operator bool() const noexcept {
  return IsValid();
}

template <typename T>
bool ArTrackableHandle<T>::operator==(const ArTrackableHandle<T>& other) const {
  return Get() == other.Get();
}

template <typename T>
bool ArTrackableHandle<T>::operator!=(const ArTrackableHandle<T>& other) const {
  return Get() != other.Get();
}

template <typename T>
T* ArTrackableHandle<T>::Get() const noexcept {
  if (!IsValid() || !trackable_id_.IsValid()) {
    return nullptr;
  }
  return trackables_manager_->GetTrackable<T>(trackable_id_);
}

template <typename T>
bool ArTrackableHandle<T>::IsValid() const {
  return session_is_valid_ && trackable_id_.IsValid() &&
         trackables_manager_->GetTrackable<T>(trackable_id_) != nullptr;
}

template <typename T>
void ArTrackableHandle<T>::AssertIsValid() const {
  if (!IsValid()) {
    IMP_LOG(imp::FATAL) << "ArTrackableHandle is invalid. Verify AR session is active.";
  }
}

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_TRACKABLE_HANDLE_H_
