/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_STORE_OWNED_PTR_STORE_H_
#define THIRD_PARTY_IMPRESS_CORE_STORE_OWNED_PTR_STORE_H_

#include <algorithm>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "core/common/owned_ptr.h"
#include "core/common/small_source_location.h"
#include "core/ncsb/update_id.h"
#include "core/ncsb/update_phase.h"
#include "core/ncsb/update_system.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// OwnedPtrStore is a utility that stores OwnedPtrs and automatically evicts
// them when they are no longer borrowed (BorrowedCount == 0).
//
// Ownership and Safety Model:
// In alignment with Google C++ Style guidelines favoring clear ownership
// semantics, OwnedPtrStore maintains exclusive ownership of the resources it
// manages (holding them via `OwnedPtr`). It distributes non-owning,
// safety-tracked references (`BorrowedPtr`) to clients.
//
// This design avoids the lifetime ambiguity of shared ownership (such as
// `std::shared_ptr`), while providing runtime protection against use-after-free
// bugs that raw pointers cannot offer. The store can safely decide when to
// destroy the memory by checking if `GetBorrowedCount() == 0`.
//
// Cache Lifecycle (Use-Based Retention):
// OwnedPtrStore functions as a resource cache where the retention policy is
// strictly use-based. It does not persistently retain resources once they are
// no longer in use.
//
// Eviction sweeps run automatically at the end of every frame (during
// `UpdatePhase::kEnd`). Any resource whose `BorrowedCount` drops to zero is
// immediately destroyed and evicted.
//
// Consequently:
// *   Storing a resource does NOT guarantee it will persist into subsequent
//     frames. It will only remain cached as long as at least one active
//     `BorrowedPtr<T>` is held externally.
// *   Retrieving a keyed resource in a later frame will return `nullptr` if all
//     borrows were released in a previous frame.
//
// Template parameters:
//   T: The type of object stored.
//   Deps: Updaters that this store depends on (for ordering within kEnd phase).
template <typename T, typename... Deps>
class OwnedPtrStore : public UpdateSystem::Updater<OwnedPtrStore<T, Deps...>> {
 public:
  using Base = UpdateSystem::Updater<OwnedPtrStore<T, Deps...>>;
  using UpdateDependencies = UpdateIds<Deps...>;
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEnd;

  explicit OwnedPtrStore(BaseView& view) : Base(view) {}

  // Stores an OwnedPtr, optionally associated with a key.
  //
  // If the key is already in use, the previously stored object is moved to the
  // anonymous store, allowing it to persist as long as it is borrowed.
  //
  // Returns a BorrowedPtr to the stored object.
  BorrowedPtr<T> Store(
      OwnedPtr<T> ptr, absl::string_view key = "",
      SmallSourceLocation loc = SmallSourceLocation::Current()) {
    BorrowedPtr<T> borrowed = ptr.Borrow(loc);
    if (key.empty()) {
      anonymous_store_.push_back(std::move(ptr));
    } else {
      auto it = keyed_store_.find(key);
      if (it != keyed_store_.end()) {
        anonymous_store_.push_back(std::move(it->second));
        it->second = std::move(ptr);
      } else {
        keyed_store_.emplace(key, std::move(ptr));
      }
    }
    return borrowed;
  }

  // Retrieves a BorrowedPtr to the object associated with the key.
  //
  // Returns nullptr if the key is empty or not found.
  BorrowedPtr<T> Retrieve(
      absl::string_view key = "",
      SmallSourceLocation loc = SmallSourceLocation::Current()) {
    if (key.empty()) {
      return nullptr;
    }

    auto it = keyed_store_.find(key);
    if (it != keyed_store_.end()) {
      return it->second.Borrow(loc);
    }
    return nullptr;
  }

  // Checks if a keyed resource exists in the store.
  bool Contains(absl::string_view key) const {
    if (key.empty()) {
      return false;
    }

    return keyed_store_.contains(key);
  }

  // Returns the total number of resources (both keyed and anonymous) in the
  // store.
  size_t GetSize() const {
    return keyed_store_.size() + anonymous_store_.size();
  }

  // Manually triggers eviction of unused resources.
  void ClearUnused() { EvictUnused(); }

  // Evicts any stored OwnedPtr where BorrowedCount is zero.
  void Update(const FrameTime& frame_time) override { EvictUnused(); }

 private:
  void EvictUnused() {
    for (auto it = keyed_store_.begin(); it != keyed_store_.end();) {
      if (!it->second.IsBorrowed()) {
        keyed_store_.erase(it++);
      } else {
        ++it;
      }
    }
    anonymous_store_.erase(
        std::remove_if(
            anonymous_store_.begin(), anonymous_store_.end(),
            [](const OwnedPtr<T>& ptr) { return !ptr.IsBorrowed(); }),
        anonymous_store_.end());
  }

  absl::flat_hash_map<std::string, OwnedPtr<T>> keyed_store_;
  std::vector<OwnedPtr<T>> anonymous_store_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_STORE_OWNED_PTR_STORE_H_
