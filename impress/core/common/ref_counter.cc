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

#include "absl/log/check.h"
#include "core/common/small_source_location.h"

namespace imp {

namespace {

RefCounter::CounterType GetTrackedRefsCount(
    const RefCounter::TrackedRefs& tracked_refs) {
  RefCounter::CounterType total = 0;
  for (const auto& [_, counter] : tracked_refs.locations_to_counts) {
    total += counter;
  }
  return total;
}

void DeleteTrackedRefsIfNeeded(RefCounter::TrackedRefs* tracked_refs) {
  if (!tracked_refs->is_destroyed) {
    // Don't delete until the RefCounter is destroyed.
    return;
  }

  if (!tracked_refs->locations_to_counts.empty()) {
    // There are still outstanding Ref objects.
    return;
  }

  delete tracked_refs;
}

}  // namespace

RefCounter::Ref::Ref(SmallSourceLocation loc,
                     TrackedRefs* tracked_refs) noexcept
    : loc_(loc), tracked_refs_(tracked_refs) {
  ++tracked_refs_->locations_to_counts[loc_];
}

RefCounter::Ref::Ref() noexcept : tracked_refs_(nullptr) {}

RefCounter::Ref::Ref(Ref&& rhs) noexcept
    : loc_(rhs.loc_), tracked_refs_(rhs.tracked_refs_) {
  // Count doesn't change.
  rhs.loc_ = {};
  rhs.tracked_refs_ = nullptr;
}

RefCounter::Ref& RefCounter::Ref::operator=(Ref&& rhs) noexcept {
  if (&rhs == this) {
    // This is a self-move. Do nothing.
    return *this;
  }

  DecrementCount();

  loc_ = rhs.loc_;
  if (tracked_refs_ && tracked_refs_ != rhs.tracked_refs_) {
    DeleteTrackedRefsIfNeeded(tracked_refs_);
  }
  tracked_refs_ = rhs.tracked_refs_;

  rhs.loc_ = {};
  rhs.tracked_refs_ = nullptr;

  return *this;
}

RefCounter::Ref::Ref(const Ref& other) noexcept
    : loc_(other.loc_), tracked_refs_(other.tracked_refs_) {
  if (tracked_refs_) {
    ++tracked_refs_->locations_to_counts[loc_];
  }
}

RefCounter::Ref& RefCounter::Ref::operator=(const Ref& other) noexcept {
  // This check prevents any work being done in the case of a self-copy.
  if (loc_ != other.loc_ || tracked_refs_ != other.tracked_refs_) {
    DecrementCount();

    loc_ = other.loc_;
    if (tracked_refs_ && tracked_refs_ != other.tracked_refs_) {
      DeleteTrackedRefsIfNeeded(tracked_refs_);
    }
    tracked_refs_ = other.tracked_refs_;

    if (tracked_refs_) {
      ++tracked_refs_->locations_to_counts[loc_];
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
  if (tracked_refs_) {
    CounterType& counter = tracked_refs_->locations_to_counts[loc_];
    --counter;
    if (counter == 0) {
      tracked_refs_->locations_to_counts.erase(loc_);
    }
  }
}

RefCounter::CounterType RefCounter::Ref::GetCount() const {
  if (tracked_refs_) {
    return GetTrackedRefsCount(*tracked_refs_);
  }
  return 0;
}

SmallSourceLocation RefCounter::Ref::GetLocation() const { return loc_; }

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

const RefCounter::TrackedRefs& RefCounter::GetTrackedRefs() const {
  if (!tracked_refs_) {
    // When tracked_refs_ is null, return a static empty TrackedRefs to avoid
    // heap allocation.
    static const TrackedRefs kEmptyTrackedRefs;
    return kEmptyTrackedRefs;
  }
  return *tracked_refs_;
}

}  // namespace imp
