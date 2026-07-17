// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_PAGED_POINTER_ARRAY_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_PAGED_POINTER_ARRAY_H_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <new>
#include <utility>
#include <vector>

#include "absl/base/optimization.h"
#include "absl/log/check.h"
#include "core/common/uniform_block_pool.h"

namespace imp {

// A paged sparse array of pointers of type T, optimized for mapping large,
// sparse ranges of integer keys (such as Entity IDs in an ECS) to pointers.
//
// This class is intended for use cases that require the O(1) lookup speed of a
// flat array but cannot afford its memory overhead due to the sparseness of the
// keys. By dividing the index space into fixed-size pages, memory is only
// allocated for active ranges of keys. This avoids the overhead of a massive
// contiguous allocation while also avoiding the per-node memory overhead and
// non-deterministic performance of a hash map.
//
// Typical use cases include:
// - Mapping entity identifiers to components or systems in an ECS.
// - Managing large, scattered sets of objects where lookups are
//   performance-critical.
// - Scenarios where data naturally clusters, allowing large empty spans to
//   consume no memory.
//
// Key Characteristics:
// - **O(1) Lookups and Writes**: Accessing an element requires a page lookup
//   (vector index) and an offset calculation, providing guaranteed
//   constant-time performance without hashing collisions.
// - **Memory Efficiency**: Memory is allocated in pages of size 2^Power on
//   demand. If a large block of indices is unused, the corresponding page is
//   never allocated, saving significant memory compared to a flat array.
// - **Automatic Compaction**: When all elements in a page become nullptr, the
//   page itself is deallocated, reducing the memory footprint at runtime.
// - **UniformBlockPool Integration**: Pages are allocated using the provided
//   imp::UniformBlockPool. This reduces churn from frequent allocation and
//   deallocation of pages on the heap by reusing memory. In particular, the
//   same UniformBlockPool can be used across many PagedPointerArray instances
//   to optimize memory usage.
// - **Minimal Footprint**: The class itself only stores a std::vector of
//   PageInfo structs (which hold the page pointer and a count of active
//   elements) and a reference to the resource.
//
// Power must be between 2 and 8.
// Default page size is 32 elements (Power = 5).
template <typename T, uint8_t Power = 5>
class PagedPointerArray {
 public:
  explicit PagedPointerArray(UniformBlockPool& block_pool);

  ~PagedPointerArray();

  PagedPointerArray(const PagedPointerArray&) = delete;
  PagedPointerArray& operator=(const PagedPointerArray&) = delete;

  PagedPointerArray(PagedPointerArray&& other) noexcept;
  PagedPointerArray& operator=(PagedPointerArray&& other) noexcept;

  // Returns the element at the given index, or nullptr if the index is not
  // set.
  T* Get(uint32_t index) const;

  // Sets the element at the given index.
  // If value is nullptr and the page becomes empty, the page is deallocated.
  void Set(uint32_t index, T* value);

  // Returns true if the set is empty.
  bool IsEmpty() const;

  // Returns the last non-null element in the set, or nullptr if the set is
  // empty.
  //
  // This is O(page_size) in the worst case.
  T* Back();

  // Returns the number of bytes required to store a page of elements.
  static constexpr size_t GetPageBytes();

  // Returns the number of elements that can be stored in a page.
  static constexpr size_t GetPageSize();

 private:
  struct Page {
    T* elements[1 << Power]{};
  };

  // Used for empty pages to avoid unnecessary branching checking for null.
  inline static Page kSentinelPage{};

  struct PageInfo {
    Page* page = &kSentinelPage;
    uint16_t count = 0;
  };

  static constexpr size_t kOffsetMask = (1 << Power) - 1;

  void DestroyPages();

  UniformBlockPool* block_pool_;
  std::vector<PageInfo> pages_;
  uint32_t active_pages_count_ = 0;
  uint32_t last_active_page_idx_ = 0;

  static_assert(Power >= 2, "Power must be at least 2");
  static_assert(Power <= 8, "Power must be at most 8 (max 256 elements)");

  // Ensure that UniformBlockPool alignment is sufficient for Page alignment.
  static_assert(alignof(Page) <= __STDCPP_DEFAULT_NEW_ALIGNMENT__);
};

template <typename T, uint8_t Power>
PagedPointerArray<T, Power>::PagedPointerArray(UniformBlockPool& block_pool)
    : block_pool_(&block_pool) {
  
}

template <typename T, uint8_t Power>
PagedPointerArray<T, Power>::~PagedPointerArray() {
  DestroyPages();
}

template <typename T, uint8_t Power>
PagedPointerArray<T, Power>::PagedPointerArray(
    PagedPointerArray&& other) noexcept
    : block_pool_(other.block_pool_),
      pages_(std::move(other.pages_)),
      active_pages_count_(other.active_pages_count_),
      last_active_page_idx_(other.last_active_page_idx_) {
  other.active_pages_count_ = 0;
  other.last_active_page_idx_ = 0;
}

template <typename T, uint8_t Power>
PagedPointerArray<T, Power>& PagedPointerArray<T, Power>::operator=(
    PagedPointerArray&& other) noexcept {
  if (this != &other) {
    DestroyPages();

    block_pool_ = other.block_pool_;
    pages_ = std::move(other.pages_);
    active_pages_count_ = other.active_pages_count_;
    last_active_page_idx_ = other.last_active_page_idx_;

    other.active_pages_count_ = 0;
    other.last_active_page_idx_ = 0;
  }
  return *this;
}

template <typename T, uint8_t Power>
T* PagedPointerArray<T, Power>::Get(uint32_t index) const {
  const size_t page_idx = index >> Power;
  const size_t offset = index & kOffsetMask;

  if (ABSL_PREDICT_FALSE(page_idx >= pages_.size())) {
    return nullptr;
  }

  return pages_[page_idx].page->elements[offset];
}

template <typename T, uint8_t Power>
void PagedPointerArray<T, Power>::Set(uint32_t index, T* value) {
  const size_t page_idx = index >> Power;
  const size_t offset = index & kOffsetMask;

  if (ABSL_PREDICT_FALSE(page_idx >= pages_.size())) {
    if (value == nullptr) {
      return;
    }
    pages_.resize(page_idx + 1);
  }

  PageInfo& info = pages_[page_idx];
  if (info.page == &kSentinelPage) {
    if (value == nullptr) {
      return;
    }
    void* mem = block_pool_->Allocate();
    info.page = new (mem) Page();
    active_pages_count_++;
    last_active_page_idx_ =
        std::max(last_active_page_idx_, static_cast<uint32_t>(page_idx));
  }

  T*& element = info.page->elements[offset];

  if (element == nullptr && value != nullptr) {
    info.count++;
    element = value;
  } else if (element != nullptr && value == nullptr) {
    info.count--;
    element = nullptr;

    if (info.count == 0) {
      info.page->~Page();
      block_pool_->Deallocate(info.page);
      info.page = &kSentinelPage;
      active_pages_count_--;

      if (page_idx == last_active_page_idx_) {
        while (last_active_page_idx_ > 0 &&
               pages_[last_active_page_idx_].page == &kSentinelPage) {
          last_active_page_idx_--;
        }
      }
    }
  } else {
    element = value;
  }
}

template <typename T, uint8_t Power>
bool PagedPointerArray<T, Power>::IsEmpty() const {
  return active_pages_count_ == 0;
}

template <typename T, uint8_t Power>
T* PagedPointerArray<T, Power>::Back() {
  if (active_pages_count_ == 0) {
    return nullptr;
  }

  PageInfo& page_info = pages_[last_active_page_idx_];
  for (int j = (1 << Power) - 1; j >= 0; --j) {
    T* element = page_info.page->elements[j];
    if (element) {
      return element;
    }
  }
  return nullptr;
}

template <typename T, uint8_t Power>
constexpr size_t PagedPointerArray<T, Power>::GetPageBytes() {
  return sizeof(Page);
}

template <typename T, uint8_t Power>
constexpr size_t PagedPointerArray<T, Power>::GetPageSize() {
  return 1 << Power;
}

template <typename T, uint8_t Power>
void PagedPointerArray<T, Power>::DestroyPages() {
  if (active_pages_count_ == 0) {
    return;
  }

  for (uint32_t i = 0; i <= last_active_page_idx_; ++i) {
    PageInfo& info = pages_[i];
    if (info.page != &kSentinelPage) {
      info.page->~Page();
      block_pool_->Deallocate(info.page);
    }
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_PAGED_POINTER_ARRAY_H_
