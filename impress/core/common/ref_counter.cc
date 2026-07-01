// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/common/ref_counter.h"

#include <cstddef>
#include <memory>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/log/check.h"
#include "core/common/small_source_location.h"

namespace imp {

// Stores the location and count for outstanding references.
// Stored in a set of unique_ptr to provide pointer stability,
// allowing Ref to store a direct pointer to this tracker.
struct SourceLocationTracker {
  SmallSourceLocation loc;
  RefCounter::CounterType count;

  bool operator==(const SourceLocationTracker& other) const {
    return loc == other.loc;
  }
  template <typename H>
  friend H AbslHashValue(H h, const SourceLocationTracker& tracker) {
    return H::combine(std::move(h), tracker.loc);
  }

  // Transparent hash comparator for unique_ptr<SourceLocationTracker>
  // allowing lookups using SmallSourceLocation directly.
  struct Hash {
    using is_transparent = void;
    size_t operator()(const std::unique_ptr<SourceLocationTracker>& ptr) const {
      return absl::Hash<SmallSourceLocation>()(ptr->loc);
    }
    size_t operator()(const SmallSourceLocation& loc) const {
      return absl::Hash<SmallSourceLocation>()(loc);
    }
  };

  // Transparent equality comparator for unique_ptr<SourceLocationTracker>
  // allowing lookups using SmallSourceLocation directly.
  struct Eq {
    using is_transparent = void;
    bool operator()(const std::unique_ptr<SourceLocationTracker>& a,
                    const std::unique_ptr<SourceLocationTracker>& b) const {
      return a->loc == b->loc;
    }
    bool operator()(const std::unique_ptr<SourceLocationTracker>& a,
                    const SmallSourceLocation& b) const {
      return a->loc == b;
    }
    bool operator()(const SmallSourceLocation& a,
                    const std::unique_ptr<SourceLocationTracker>& b) const {
      return a == b->loc;
    }
  };
};

// Holds the state for tracked references. This is separated from RefCounter
// so that it can outlive the RefCounter if there are still outstanding
// Ref objects when the RefCounter is destroyed.
struct TrackedRefs {
  using SetType = absl::flat_hash_set<std::unique_ptr<SourceLocationTracker>,
                                      SourceLocationTracker::Hash,
                                      SourceLocationTracker::Eq>;

  SetType source_location_trackers;
  bool is_destroyed = false;
};

namespace {

// Returns the total number of tracked references by summing up the counts
// for all source locations.
RefCounter::CounterType GetTrackedRefsCount(const TrackedRefs& tracked_refs) {
  RefCounter::CounterType total = 0;
  for (const auto& tracker_ptr : tracked_refs.source_location_trackers) {
    total += tracker_ptr->count;
  }
  return total;
}

// Deletes the TrackedRefs object if the RefCounter has been destroyed
// and there are no more outstanding Ref objects.
void DeleteTrackedRefsIfNeeded(TrackedRefs* tracked_refs) {
  if (!tracked_refs->is_destroyed) {
    // Don't delete until the RefCounter is destroyed.
    return;
  }

  if (!tracked_refs->source_location_trackers.empty()) {
    // There are still outstanding Ref objects.
    return;
  }

  delete tracked_refs;
}

}  // namespace

absl::flat_hash_map<SmallSourceLocation, RefCounter::CounterType>
RefCounter::GetLocationToCount() const {
  absl::flat_hash_map<SmallSourceLocation, CounterType> result;
  if (!tracked_refs_) return result;
  for (const auto& tracker_ptr : tracked_refs_->source_location_trackers) {
    result[tracker_ptr->loc] = tracker_ptr->count;
  }
  return result;
}

RefCounter::Ref::Ref(SmallSourceLocation loc,
                     TrackedRefs* tracked_refs) noexcept
    : tracked_refs_(tracked_refs) {
  auto it = tracked_refs_->source_location_trackers.find(loc);
  if (it == tracked_refs_->source_location_trackers.end()) {
    auto [new_it, inserted] = tracked_refs_->source_location_trackers.insert(
        std::make_unique<SourceLocationTracker>(SourceLocationTracker{loc, 0}));
    it = new_it;
  }
  tracker_ptr_ = it->get();
  tracker_ptr_->count++;
}

RefCounter::Ref::Ref() noexcept : tracked_refs_(nullptr) {}

RefCounter::Ref::Ref(Ref&& rhs) noexcept
    : tracker_ptr_(rhs.tracker_ptr_), tracked_refs_(rhs.tracked_refs_) {
  // Count doesn't change.
  rhs.tracker_ptr_ = nullptr;
  rhs.tracked_refs_ = nullptr;
}

RefCounter::Ref& RefCounter::Ref::operator=(Ref&& rhs) noexcept {
  if (&rhs == this) {
    // This is a self-move. Do nothing.
    return *this;
  }

  DecrementCount();

  tracker_ptr_ = rhs.tracker_ptr_;
  if (tracked_refs_ && tracked_refs_ != rhs.tracked_refs_) {
    DeleteTrackedRefsIfNeeded(tracked_refs_);
  }
  tracked_refs_ = rhs.tracked_refs_;

  rhs.tracker_ptr_ = nullptr;
  rhs.tracked_refs_ = nullptr;

  return *this;
}

RefCounter::Ref::Ref(const Ref& other) noexcept
    : tracker_ptr_(other.tracker_ptr_), tracked_refs_(other.tracked_refs_) {
  if (tracker_ptr_) {
    tracker_ptr_->count++;
  }
}

RefCounter::Ref& RefCounter::Ref::operator=(const Ref& other) noexcept {
  // This check prevents any work being done in the case of a self-copy.
  if (tracked_refs_ != other.tracked_refs_ ||
      tracker_ptr_ != other.tracker_ptr_) {
    DecrementCount();

    tracker_ptr_ = other.tracker_ptr_;
    if (tracked_refs_ && tracked_refs_ != other.tracked_refs_) {
      DeleteTrackedRefsIfNeeded(tracked_refs_);
    }
    tracked_refs_ = other.tracked_refs_;

    if (tracker_ptr_) {
      tracker_ptr_->count++;
    }
  }

  return *this;
}

RefCounter::Ref::~Ref() noexcept {
  DecrementCount();
  if (tracked_refs_) {
    DeleteTrackedRefsIfNeeded(tracked_refs_);
    tracked_refs_ = nullptr;
  }
}

void RefCounter::Ref::DecrementCount() {
  if (tracker_ptr_) {
    tracker_ptr_->count--;
    if (tracker_ptr_->count == 0) {
      auto it = tracked_refs_->source_location_trackers.find(tracker_ptr_->loc);
      if (it != tracked_refs_->source_location_trackers.end()) {
        tracked_refs_->source_location_trackers.erase(it);
      }
    }
  }
}

RefCounter::CounterType RefCounter::Ref::GetCount() const {
  if (tracked_refs_) {
    return GetTrackedRefsCount(*tracked_refs_);
  }
  return 0;
}

SmallSourceLocation RefCounter::Ref::GetLocation() const {
  if (tracker_ptr_) {
    return tracker_ptr_->loc;
  }
  return {};
}

bool RefCounter::Ref::IsCounterDestroyed() const {
  if (tracked_refs_) {
    return tracked_refs_->is_destroyed;
  }

  // There is no counter, so it counts as destroyed.
  return true;
}

RefCounter::Ref RefCounter::Ref::WithNewLocation(
    SmallSourceLocation loc) const {
  return Ref(loc, tracked_refs_);
}

RefCounter::RefCounter() noexcept {}

RefCounter::~RefCounter() noexcept {
  if (tracked_refs_) {
    

    tracked_refs_->is_destroyed = true;

    // If there are no outstanding Ref objects, then the TrackedRefs object can
    // be deleted. Otherwise, this will be deleted when the last Ref object
    // goes out of scope.
    DeleteTrackedRefsIfNeeded(tracked_refs_);

    tracked_refs_ = nullptr;
  }
}

RefCounter::RefCounter(RefCounter&& rhs) noexcept
    : tracked_refs_(rhs.tracked_refs_) {
  rhs.tracked_refs_ = nullptr;
}

RefCounter& RefCounter::operator=(RefCounter&& rhs) noexcept {
  if (tracked_refs_ == rhs.tracked_refs_) {
    // This is a self-move. Do nothing.
    return *this;
  }

  // TODO: Check that rhs is not already destroyed.

  if (tracked_refs_) {
    tracked_refs_->is_destroyed = true;
    DeleteTrackedRefsIfNeeded(tracked_refs_);
  }

  tracked_refs_ = rhs.tracked_refs_;
  rhs.tracked_refs_ = nullptr;

  return *this;
}

RefCounter::Ref RefCounter::Retain(SmallSourceLocation loc) const {
  if (!tracked_refs_) {
    // Lazy allocation of the tracking structure.
    tracked_refs_ = new TrackedRefs();
  }
  return Ref(loc, tracked_refs_);
}

RefCounter::CounterType RefCounter::GetCount() const {
  if (!tracked_refs_) {
    return 0;
  }
  
  return GetTrackedRefsCount(*tracked_refs_);
}

}  // namespace imp
