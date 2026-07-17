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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_REF_COUNTER_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_REF_COUNTER_H_

#include <cstdint>

#include "absl/container/flat_hash_map.h"
#include "core/common/small_source_location.h"

namespace imp {

// Utility used for simple reference counting.
//
// RefCounter itself is a move-only type that provides Ref objects that
// increment the counter. Ref objects can be copied & moved, and they decrement
// the counter when they go out of scope.
//
// This is intended to be a lightweight alternative to using shared_ptr for
// reference counting. shared_ptr is slow to copy because it modifies the
// counter atomically for thread safety. RefCounter is fast, small & not thread
// safe.
//
// By making RefCounter move-only, the semantics of this type are meant
// to discourage shared ownership in keeping with the guidance in
// (broken link). It's meant to encourage usages where
// memory is owned in one location (which also owns the RefCounter) and
// external usages of that memory are tracked. This is particularly useful for
// caching mechanisms that want to know when memory is no longer in use and
// internally decide when to actually destroy the memory.
//
// Starts with a count of zero.
class RefCounter {
 public:
  using CounterType = uint16_t;
  struct TrackedRefs {
    // The key is the location from the Retain method, the value is the number
    // of Retain calls from that location.
    absl::flat_hash_map<SmallSourceLocation, CounterType> locations_to_counts;

    // True if the RefCounter has been destroyed. This is used to detect the
    // case where a Ref object outlives the RefCounter object.
    bool is_destroyed = false;
  };

  class Ref {
   public:
    Ref() noexcept;
    Ref(Ref&& rhs) noexcept;
    Ref& operator=(Ref&& rhs) noexcept;
    Ref(const Ref& other) noexcept;
    Ref& operator=(const Ref& other) noexcept;
    ~Ref() noexcept;

    // Returns the number of outstanding tracked Ref objects for the
    // RefCounter this Ref comes from, inclusive of this Ref.
    CounterType GetCount() const;

    // Returns the location that the Ref was retained from.
    SmallSourceLocation GetLocation() const;

    // Returns true if the RefCounter has been destroyed. This indicates that
    // the Ref object has outlived the RefCounter object.
    bool IsCounterDestroyed() const;

    // Returns a new Ref associated with the new location. The tracked count
    // will include the new ref.
    Ref WithNewLocation(
        SmallSourceLocation loc = SmallSourceLocation::Current()) const;

   private:
    Ref(SmallSourceLocation loc, TrackedRefs* tracked_refs) noexcept;

    void DecrementCount();

    SmallSourceLocation loc_;
    TrackedRefs* tracked_refs_ = nullptr;

    friend class RefCounter;
  };

  RefCounter() noexcept;
  ~RefCounter() noexcept;
  RefCounter(RefCounter&& rhs) noexcept;
  RefCounter& operator=(RefCounter&& rhs) noexcept;

  // RefCounter is move-only.
  RefCounter(const RefCounter& other) = delete;
  RefCounter& operator=(const RefCounter& other) = delete;

  // Creates a new Ref that contributes the count.
  //
  // When the ref goes out of scope, the count is automatically decremented.
  //
  // The ref can freely be copied and moved, and the count will be tracked
  // correctly.
  Ref Retain(SmallSourceLocation loc = SmallSourceLocation::Current()) const;

  // The number of Ref objects for this RefCounter that currently exist.
  //
  // The count starts at zero.
  CounterType GetCount() const;

  const TrackedRefs& GetTrackedRefs() const;

 private:
  // This pointer is allocated dynamically with new & delete. This is done so
  // that we can ensure that the TrackedRefs object is not destroyed until all
  // outstanding Ref objects have been destroyed to prevent Ref objects from
  // crashing if they outlive the RefCounter.
  //
  // This is done manually instead of using std::shared_ptr because we're
  // already tracking the reference count anyways & it avoids the extra overhead
  // for thread synchronization that shared_ptr would do.
  //
  // This requires the lifetime of tracked_refs_ to be managed carefully.
  mutable TrackedRefs* tracked_refs_ = nullptr;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_REF_COUNTER_H_
