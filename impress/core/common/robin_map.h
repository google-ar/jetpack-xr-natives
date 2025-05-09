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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_ROBIN_MAP_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_ROBIN_MAP_H_

#include "absl/hash/hash.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp {

// Provide a wrap of tsl::robin_map that uses absl::Hash instead of std::hash.
template <class Key, class T, class Hash = absl::Hash<Key>,
          class KeyEqual = std::equal_to<Key>,
          class Allocator = std::allocator<std::pair<Key, T>>,
          bool kStoreHash = false,
          class GrowthPolicy = tsl::rh::power_of_two_growth_policy<2>>
using RobinMap =
    tsl::robin_map<Key, T, Hash, KeyEqual, Allocator, kStoreHash, GrowthPolicy>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_ROBIN_MAP_H_
