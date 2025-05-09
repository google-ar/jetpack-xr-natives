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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_ALLOCATOR_POOL_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_ALLOCATOR_POOL_H_

#include <algorithm>
#include <cassert>
#include <memory>
#include <optional>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "core/common/invocable.h"
#include "core/view/utils/macros.h"

namespace imp {

// AllocatorPool is responsible for doling out allocated blocks of a
// one-dimensional, subdivisible, user-specified resource T. AllocatorPool is
// agnostic to the actual contents and layout of T. It merely divides it into
// sections based on calls to Alloc() by clients, and returns a Handle which
// points into this resource.
//
// For example, to allocate a section of T of size N, a client calls Alloc(N),
// which returns a Handle H. H.GetResource() returns T, H.GetOffset() points to
// a position in T, and H.GetSize() returns N. When H goes out of scope, the
// section of memory it points to within T is considered freed, and may be
// reassigned by another call to Alloc() later.
//
// However, there's a twist. AllocatorPool doesn't just manage only one instance
// of T. If T runs out of space for a desired allocation, AllocatorPool creates
// another, bigger instance of T, and so on. All instances of T coexist
// alongside each other, and further calls to Alloc() can return an allocation
// into any T owned by AllocatorPool. As Handles are released, AllocatorPool
// will automatically free these extra Ts whenever possible.
//
// To create an instance of AllocatorPool, call AllocatorPool::Create(). See the
// function documentation for details.
template <typename T>
class AllocatorPool {
 public:
  using ResourceFactoryFn = imp::Invocable<T(int capacity)>;

  // A handle which reserves an allocation into T.
  //
  // When Handle goes out of scope, its allocation is freed.
  //
  // Key public methods on this function are GetResource(), GetOffset(), and
  // GetSize().
  class IMP_WARN_UNUSED_RESULT Handle {
    friend class AllocatorPool;

   public:
    Handle(Handle&& rhs) noexcept;
    Handle& operator=(Handle&& rhs) noexcept;
    ~Handle();

    // The resource this handle points into.
    T& GetResource() noexcept;
    const T& GetResource() const noexcept;

    // Offset into T.
    int GetOffset() const noexcept;

    // Size of the window into T.
    int GetSize() const noexcept;

   private:
    Handle(AllocatorPool* pool, int index, int offset, int size);

    // Move-only class.
    Handle(const Handle&) = delete;
    Handle& operator=(const Handle&) = delete;

    AllocatorPool* pool_;  // null if moved
    int index_;            // index in pool
    int offset_;           // offset in buffer
    int size_;             // size of entry in buffer
  };

  // Create a AllocatorPool.
  //
  // Because handles record pointers into this AllocatorPool, we cannot allow
  // AllocatorPools to be allocated on the stack.
  //
  // initial_order refers to the max order of the buddy allocator used
  // internally by AllocatorPool. In simpler terms, AllocatorPool creates the
  // initial T with a capacity of 2^initial_order. If T is exhausted, the next T
  // allocated will be created with a max order of the first T + 1, and so on.
  //
  // ResourceFactoryFn is a function which accepts one argument, int capacity,
  // and returns T. This function must construct T with at least as much
  // capacity as specified. As Ts are exhausted, AllocatorPool will call this
  // function with increasing capacities.
  //
  // See the documentation of Alloc() below for an elaboration on what "size"
  // represents.
  static std::unique_ptr<AllocatorPool> Create(
      int initial_order, ResourceFactoryFn resource_factory);

  ~AllocatorPool();

  // Run function f(T&) on each T.
  template <typename Fn>
  void ForEachResource(Fn&& f);

  // Allocate a block of the specified size from a T.
  //
  // What "size" means for T is entirely up to the client. For example, if T is
  // a vertex buffer, one "unit" of size may be a single vertex, or it could be
  // an entire quad. Clients must take care to interpret this size correctly and
  // consistently.
  Handle Alloc(int size);

 private:
  class Allocator {
   public:
    Allocator(Allocator&& rhs) noexcept = default;
    Allocator& operator=(Allocator&& rhs) noexcept = default;
    Allocator(AllocatorPool* owner, int index, T&& resource, int order);

    T& GetResource() noexcept;
    int GetOrder() const noexcept;
    bool IsEmpty();

    std::optional<AllocatorPool::Handle> Alloc(int size);
    void Free(int offset, int capacity);

   private:
    AllocatorPool* owner_;
    int index_;
    T resource_;
    int order_;
    std::vector<bool> record_;
    std::optional<bool> is_empty_;

    // Move-only class.
    Allocator(const Allocator&) = delete;
    Allocator& operator=(const Allocator&) = delete;

    std::optional<AllocatorPool::Handle> TryAlloc(int size, int order,
                                                  int offset);
  };

  // TODO: Use a less memory-intensive data structure? Though, this
  // should be a bit vector.
  std::vector<Allocator> allocators_;
  ResourceFactoryFn resource_factory_fn_;

  AllocatorPool(int initial_order, ResourceFactoryFn resource_factory_fn);

  void NewAllocator(int order);

  // If we have more than one empty Allocator, delete the biggest ones.
  void Prune();
};

/*******************************************************************************
 * AllocatorPool */

template <typename T>
inline AllocatorPool<T>::AllocatorPool(int initial_order,
                                       ResourceFactoryFn resource_factory_fn)
    : resource_factory_fn_(std::move(resource_factory_fn)) {
  NewAllocator(initial_order);
}

template <typename T>
inline std::unique_ptr<AllocatorPool<T>> AllocatorPool<T>::Create(
    int initial_order, ResourceFactoryFn resource_factory) {
  return absl::WrapUnique<AllocatorPool>(
      new AllocatorPool(initial_order, std::move(resource_factory)));
}

template <typename T>
inline typename AllocatorPool<T>::Handle AllocatorPool<T>::Alloc(int size) {
  for (Allocator& allocator : allocators_) {
    std::optional<Handle> allocated = allocator.Alloc(size);
    if (allocated) {
      return std::move(*allocated);
    }
    // Try the next one...
  }
  // None of our allocators can fit this chonker. Allocate one greater than
  // the final max order, unless this thing is so big that it needs an entire
  // allocator to itself.
  
  int order = allocators_.back().GetOrder() + 1;
  while ((1 << order) < size) {
    order++;
  }

  NewAllocator(order);
  std::optional<Handle> result = allocators_.back().Alloc(size);
  CHECK(result.has_value());  // this better not be false
  return std::move(*result);
}

template <typename T>
inline void AllocatorPool<T>::NewAllocator(int order) {
  allocators_.emplace_back(this, allocators_.size(),
                           resource_factory_fn_(1 << order), order);
}

template <typename T>
inline void AllocatorPool<T>::Prune() {
  int num_empty = 0;
  for (auto it = allocators_.end() - 1; it >= allocators_.begin(); it--) {
    if (it->IsEmpty()) {
      num_empty++;
    } else {
      if (num_empty > 1) {
        // Leave just one empty Allocator on the end.
        allocators_.erase(it + 2, allocators_.end());
      }
      break;
    }
  }
}

/*******************************************************************************
 * Allocator */

template <typename T>
inline AllocatorPool<T>::Allocator::Allocator(AllocatorPool* owner, int index,
                                              T&& resource, int order)
    : owner_(owner),
      index_(index),
      resource_(std::move(resource)),
      order_(order),
      is_empty_(true) {
  
  
  
  record_.resize(1 << order_);
}

template <typename T>
inline AllocatorPool<T>::~AllocatorPool() {
  for (Allocator& allocator : allocators_) {
    
  }
}

template <typename T>
template <typename Fn>
inline void AllocatorPool<T>::ForEachResource(Fn&& f) {
  for (Allocator& allocator : allocators_) {
    f(allocator.GetResource());
  }
}

template <typename T>
inline int AllocatorPool<T>::Allocator::GetOrder() const noexcept {
  return order_;
}

template <typename T>
inline T& AllocatorPool<T>::Allocator::GetResource() noexcept {
  return resource_;
}

template <typename T>
inline bool AllocatorPool<T>::Allocator::IsEmpty() {
  if (!is_empty_.has_value()) {
    is_empty_ =
        std::find(record_.begin(), record_.end(), true) == record_.end();
  }
  return *is_empty_;
}

template <typename T>
inline std::optional<typename AllocatorPool<T>::Handle>
AllocatorPool<T>::Allocator::Alloc(int size) {
  if (size > (1 << order_)) {
    return std::nullopt;
  }
  return TryAlloc(size, order_, 0);
}

template <typename T>
inline std::optional<typename AllocatorPool<T>::Handle>
AllocatorPool<T>::Allocator::TryAlloc(int size, int order, int offset) {
  if (order == 0) {
    if (record_[offset]) {
      return std::nullopt;
    }
    record_[offset] = true;
    is_empty_ = false;
    return Handle(owner_, index_, offset, size);
  }

  int capacity = 1 << order;
  int half_capacity = capacity / 2;

  if (half_capacity < size) {
    // Try to store this node in all of our children.
    auto begin = record_.begin() + offset;
    auto end = begin + capacity;
    if (std::find(begin, end, true) != end) {
      return std::nullopt;
    }
    // We fit! Allocate and return handle.
    std::fill(begin, begin + size, true);
    is_empty_ = false;
    return Handle(owner_, index_, offset, size);
  }

  std::optional<Handle> left = TryAlloc(size, order - 1, offset);
  if (left) {
    return left;
  }
  return TryAlloc(size, order - 1, offset + half_capacity);
}

template <typename T>
inline void AllocatorPool<T>::Allocator::Free(int offset, int capacity) {
  auto begin = record_.begin() + offset;
  auto end = begin + capacity;
  LOG_IF(FATAL, std::find(begin, end, false) != end) << "Allocator double-free";
  std::fill(begin, end, false);
  is_empty_ = std::nullopt;
}

/*******************************************************************************
 * Handle */

template <typename T>
inline AllocatorPool<T>::Handle::Handle(AllocatorPool* pool, int index,
                                        int offset, int size)
    : pool_(pool), index_(index), offset_(offset), size_(size) {}

template <typename T>
inline AllocatorPool<T>::Handle::Handle(Handle&& rhs) noexcept
    : pool_(rhs.pool_),
      index_(rhs.index_),
      offset_(rhs.offset_),
      size_(rhs.size_) {
  rhs.pool_ = nullptr;
}

template <typename T>
inline typename AllocatorPool<T>::Handle& AllocatorPool<T>::Handle::operator=(
    Handle&& rhs) noexcept {
  pool_ = rhs.pool_;
  index_ = rhs.index_;
  offset_ = rhs.offset_;
  size_ = rhs.size_;
  rhs.pool_ = nullptr;
  return *this;
}

template <typename T>
inline AllocatorPool<T>::Handle::~Handle() {
  if (pool_) {
    pool_->allocators_[index_].Free(offset_, size_);
    pool_->Prune();
  }
}

template <typename T>
inline T& AllocatorPool<T>::Handle::GetResource() noexcept {
  return pool_->allocators_[index_].GetResource();
}

template <typename T>
inline const T& AllocatorPool<T>::Handle::GetResource() const noexcept {
  return pool_->allocators_[index_].GetResource();
}

template <typename T>
inline int AllocatorPool<T>::Handle::GetOffset() const noexcept {
  return offset_;
}

template <typename T>
inline int AllocatorPool<T>::Handle::GetSize() const noexcept {
  return size_;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_ALLOCATOR_POOL_H_
