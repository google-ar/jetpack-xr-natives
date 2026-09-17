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
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/base/no_destructor.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/hash/hash.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/synchronization/mutex.h"
#include "core/common/small_source_location.h"

namespace imp {

// Stores the location and count for outstanding references.
struct SourceLocationTracker {
  SmallSourceLocation loc;
  RefCounter::CounterType count;
  TrackedRefs* tracked_refs;

  bool operator==(const SourceLocationTracker& other) const {
    return loc == other.loc;
  }
  template <typename H>
  friend H AbslHashValue(H h, const SourceLocationTracker& tracker) {
    return H::combine(std::move(h), tracker.loc);
  }

  // Transparent hash comparator for SourceLocationTracker*
  // allowing lookups using SmallSourceLocation directly.
  struct Hash {
    using is_transparent = void;
    size_t operator()(const SourceLocationTracker* ptr) const {
      return absl::Hash<SmallSourceLocation>()(ptr->loc);
    }
    size_t operator()(const SmallSourceLocation& loc) const {
      return absl::Hash<SmallSourceLocation>()(loc);
    }
  };

  // Transparent equality comparator for SourceLocationTracker*
  // allowing lookups using SmallSourceLocation directly.
  struct Eq {
    using is_transparent = void;
    bool operator()(const SourceLocationTracker* a,
                    const SourceLocationTracker* b) const {
      return a->loc == b->loc;
    }
    bool operator()(const SourceLocationTracker* a,
                    const SmallSourceLocation& b) const {
      return a->loc == b;
    }
    bool operator()(const SmallSourceLocation& a,
                    const SourceLocationTracker* b) const {
      return a == b->loc;
    }
  };
};

// Holds the state for tracked references. This is separated from RefCounter
// so that it can outlive the RefCounter if there are still outstanding
// Ref objects when the RefCounter is destroyed.
struct TrackedRefs {
  using SetType =
      absl::flat_hash_set<SourceLocationTracker*, SourceLocationTracker::Hash,
                          SourceLocationTracker::Eq>;

  SetType source_location_trackers;
  bool is_destroyed = false;
};

namespace {

// GlobalTrackerPool is a thread-safe singleton that backs the thread-local
// TrackerAllocators. It owns the memory chunks and maintains a global free list
// to prevent memory leaks over the process lifetime.
//
// NOTE: If a thread exits without releasing its local resource, the global
// pool will not be able to redistribute the memory to other threads.
class GlobalTrackerPool {
 public:
  static GlobalTrackerPool& GetInstance() {
    // We intentionally use absl::NoDestructor to "leak" the pool at exit.
    // Tearing down the pool risks Use-After-Free crashes if any trailing
    // threads or static destructors still try to invoke ReleaseBatch().
    static absl::NoDestructor<GlobalTrackerPool> instance;
    return *instance;
  }

  // TrackerNode is a union of SourceLocationTracker and a pointer to the next
  // free node. This allows us to reuse the memory for SourceLocationTracker
  // objects and avoid memory allocation overhead.
  union TrackerNode {
    SourceLocationTracker tracker;
    TrackerNode* next_free;

    TrackerNode() {}
    ~TrackerNode() {}
  };

  // Holds available resources from the global pool.
  struct TrackerNodeResource {
    // The head and tail of the free list.
    TrackerNode* free_list_head = nullptr;
    TrackerNode* free_list_tail = nullptr;

    // The cursor to the next available node in the current batch.
    // This is guaranteed to be contiguous in memory, meaning we can use
    // pointer arithmetic to quickly find the next available node.
    TrackerNode* batch_cursor = nullptr;
    // The number of available nodes in the current batch.
    int available_count_in_batch = 0;
  };

  // The number of tracker nodes to allocate in each chunk.
  static constexpr int kBatchSize = 64;

  // Requests tracker node resources from the global pool.
  //
  // If there are no resources available, a new chunk of nodes will be
  // allocated.
  //
  // NOTE: The returned resource may contain a small number of available nodes
  // than kBatchSize.
  TrackerNodeResource RequestResource() {
    absl::MutexLock lock(mutex_);
    if (available_resources_.empty()) {
      // No resources available, allocate a new chunk.
      auto chunk = std::make_unique<TrackerNode[]>(kBatchSize);
      chunks_.push_back(std::move(chunk));

      return TrackerNodeResource{.free_list_head = nullptr,
                                 .free_list_tail = nullptr,
                                 .batch_cursor = chunks_.back().get(),
                                 .available_count_in_batch = kBatchSize};
    }

    TrackerNodeResource resource = available_resources_.back();
    available_resources_.pop_back();
    return resource;
  }

  // Releases tracker node resources back to the global pool.
  void ReleaseResource(TrackerNodeResource resource) {
    absl::MutexLock lock(mutex_);
    available_resources_.push_back(resource);
  }

 private:
  friend class absl::NoDestructor<GlobalTrackerPool>;
  GlobalTrackerPool() = default;

  absl::Mutex mutex_;

  // Available resources released from TrackerAllocators.
  std::vector<TrackerNodeResource> available_resources_ ABSL_GUARDED_BY(mutex_);
  // Owns the allocated chunks.
  std::vector<std::unique_ptr<TrackerNode[]>> chunks_ ABSL_GUARDED_BY(mutex_);
};

// TrackerAllocator is a thread-local singleton class that manages the
// allocation and reuse of SourceLocationTracker instances. It uses a free list
// to minimize memory allocation overhead and avoid locks on the fast path.
//
// Note: RefCounter is not designed to be thread safe (since each
// thread will only access their own RefCounter/OwnedPtr). The TrackerAllocator
// is thread-local so that multiple threads can allocate/deallocate trackers in
// parallel without locks.
class TrackerAllocator {
 public:
  using TrackerNode = GlobalTrackerPool::TrackerNode;

  static TrackerAllocator& GetInstance() {
    thread_local TrackerAllocator instance;
    return instance;
  }

  ~TrackerAllocator() {
    // Return all remaining local free nodes to the global pool on thread exit,
    // thereby ensuring another thread dropping references to this thread's
    // allocated memory won't UAF against deleted chunks.
    GlobalTrackerPool::GetInstance().ReleaseResource(
        std::move(local_resource_));
  }

  // Allocates a SourceLocationTracker. It reuses instances from the free list
  // if possible. If the free list is empty, it queries the GlobalTrackerPool.
  SourceLocationTracker* Allocate(SmallSourceLocation loc,
                                  TrackedRefs* tracked_refs) {
    TrackerNode* allocated_node = nullptr;
    if (local_resource_.available_count_in_batch == 0 &&
        !local_resource_.free_list_head) {
      // We've exhausted the free list and the current batch, query the global
      // pool for a new batch.
      local_resource_ = GlobalTrackerPool::GetInstance().RequestResource();
    }

    if (local_resource_.free_list_head) {
      // Check free list first for better cache locality and less batch
      // consumption.
      allocated_node = local_resource_.free_list_head;
      local_resource_.free_list_head = allocated_node->next_free;
    } else if (local_resource_.available_count_in_batch > 0) {
      allocated_node = local_resource_.batch_cursor;
      local_resource_.available_count_in_batch--;
      local_resource_.batch_cursor++;
    } else {
      // this should never happen.
      IMP_LOG(imp::FATAL) << "We've exhausted the free list and the current batch, but "
                    "failed to allocate a new batch.";
    }

    SourceLocationTracker* tracker = new (&allocated_node->tracker)
        SourceLocationTracker{loc, 0, tracked_refs};
    return tracker;
  }

  // Frees a SourceLocationTracker by returning it to the local free list for
  // reuse.
  void Free(SourceLocationTracker* tracker) {
    static_assert(std::is_trivially_destructible_v<SourceLocationTracker>,
                  "SourceLocationTracker must be trivially destructible so we "
                  "don't need to explicitly call its destructor in Free().");

    TrackerNode* node = reinterpret_cast<TrackerNode*>(tracker);
    node->next_free = local_resource_.free_list_head;
    local_resource_.free_list_head = node;
  }

 private:
  TrackerAllocator() = default;

  GlobalTrackerPool::TrackerNodeResource local_resource_;
};

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
                     TrackedRefs* tracked_refs) noexcept {
  auto it = tracked_refs->source_location_trackers.lazy_emplace(
      loc, [&](const auto& ctor) {
        ctor(TrackerAllocator::GetInstance().Allocate(loc, tracked_refs));
      });
  tracker_ptr_ = *it;
  tracker_ptr_->count++;
}

RefCounter::Ref::Ref() noexcept {}

RefCounter::Ref::Ref(Ref&& rhs) noexcept : tracker_ptr_(rhs.tracker_ptr_) {
  // Count doesn't change.
  rhs.tracker_ptr_ = nullptr;
}

RefCounter::Ref& RefCounter::Ref::operator=(Ref&& rhs) noexcept {
  if (&rhs == this) {
    // This is a self-move. Do nothing.
    return *this;
  }

  DecrementCount();

  tracker_ptr_ = rhs.tracker_ptr_;
  rhs.tracker_ptr_ = nullptr;

  return *this;
}

RefCounter::Ref::Ref(const Ref& other) noexcept
    : tracker_ptr_(other.tracker_ptr_) {
  if (tracker_ptr_) {
    tracker_ptr_->count++;
  }
}

RefCounter::Ref& RefCounter::Ref::operator=(const Ref& other) noexcept {
  // This check prevents any work being done in the case of a self-copy.
  if (tracker_ptr_ != other.tracker_ptr_) {
    DecrementCount();

    tracker_ptr_ = other.tracker_ptr_;
    if (tracker_ptr_) {
      tracker_ptr_->count++;
    }
  }

  return *this;
}

RefCounter::Ref::~Ref() noexcept { DecrementCount(); }

void RefCounter::Ref::DecrementCount() {
  if (tracker_ptr_) {
    TrackedRefs* tracked_refs = tracker_ptr_->tracked_refs;
    tracker_ptr_->count--;
    if (tracker_ptr_->count == 0) {
      auto it = tracked_refs->source_location_trackers.find(tracker_ptr_->loc);
      if (it != tracked_refs->source_location_trackers.end()) {
        tracked_refs->source_location_trackers.erase(it);
        TrackerAllocator::GetInstance().Free(tracker_ptr_);
      }
    }
    DeleteTrackedRefsIfNeeded(tracked_refs);
    tracker_ptr_ = nullptr;
  }
}

RefCounter::CounterType RefCounter::Ref::GetCount() const {
  if (tracker_ptr_) {
    return GetTrackedRefsCount(*tracker_ptr_->tracked_refs);
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
  if (tracker_ptr_) {
    return tracker_ptr_->tracked_refs->is_destroyed;
  }

  // There is no counter, so it counts as destroyed.
  return true;
}

RefCounter::Ref RefCounter::Ref::WithNewLocation(
    SmallSourceLocation loc) const {
  if (tracker_ptr_) {
    return Ref(loc, tracker_ptr_->tracked_refs);
  }
  return Ref();
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

bool RefCounter::IsBorrowed() const {
  if (!tracked_refs_) {
    return false;
  }
  
  return !tracked_refs_->source_location_trackers.empty();
}

}  // namespace imp
