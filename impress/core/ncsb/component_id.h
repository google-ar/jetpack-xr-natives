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

#ifndef THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_ID_H_
#define THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_ID_H_

#include <array>
#include <atomic>
#include <cstdint>

#include "absl/base/attributes.h"
#include "core/common/base_pool_allocator.h"
#include "core/common/hash.h"
#include "core/ncsb/update_id.h"

namespace imp {

namespace imp_internal {

inline std::atomic<uint32_t>& GetComponentIdCounter() {
  // This is an atomic even though Impress APIs are all accessed from the
  // foreground thread to ensure that the ID is unique across multiple threads
  // that have their own Impress instances in the same process.
  static std::atomic<uint32_t> value = 0;
  return value;
}

}  // namespace imp_internal

// Used to refer to a component type at runtime.
using ComponentId = uint32_t;

// Returns the component ID for the given component type.
//
// This is an index that sequentially increases as the application runs for each
// unique component type.
//
// This is used to access component pools via a sparse set in O(1) time,
// avoiding the need for hash map lookups.
//
// Note: This ID is not stable across application runs. For serialization, the
// ISF proto state type urls are used instead.
template <typename T>
inline uint32_t GetComponentTypeId() {
  // This is a static variable that is initialized only once, the first time
  // this function is called for a particular type T. All subsequent calls for
  // the same type T will return the same value.
  static const uint32_t kId = imp_internal::GetComponentIdCounter()++;
  return kId;
}

// Key used to identify a component instance within a pool.
//
// Used to efficiently check if a ComponentHandle is valid.
using ComponentKey = PoolAllocatorKey;

template <typename... Components>
using ComponentIds ABSL_DEPRECATED("Use imp::UpdateIds<T> instead.") =
    UpdateIds<Components...>;

// Helper for representing a list of component ids at compile time to specify
// the order components are cleaned up in.
//
// Ex:
//   using CleanupDependencies = CleanupIds<Foo>;
//
// For more details:
//   third_party/impress/core/ncsb/component.h
//   third_party/impress/core/ncsb/component_test.cc
template <typename... Args>
struct CleanupIds {
  static constexpr std::array<HashValue, sizeof...(Args)> kHashes{
      type_traits::kTypeHash<Args>...};
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_NCSB_COMPONENT_ID_H_
