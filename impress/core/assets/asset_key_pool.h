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
#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_ASSET_KEY_POOL_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_ASSET_KEY_POOL_H_
#include <cstddef>
#include <vector>

#include "absl/container/flat_hash_set.h"

namespace imp::imp_internal {

// A pool of lookup keys for assets, AssetKeys.
//
// AssetKeys are stored in order (first in, first out).
// AssetKeys are unique in the pool, Insert() will not add a duplicate key.
// A fast Contains() method is supported.
class AssetKeyPool {
 public:
  AssetKeyPool() {}
  using AssetKey = size_t;
  using Iterator = typename std::vector<AssetKey>::iterator;

  // Iterators follow the same rules as std::vector iterators.
  Iterator Begin() { return ordered_keys_.begin(); }

  // Iterators follow the same rules as std::vector iterators.
  Iterator End() { return ordered_keys_.end(); }

  // Insert adds a key to the end of the pool, if it is not already present.
  // Does nothing if the key is already present.
  void Insert(AssetKey id) {
    if (searchable_keys_.contains(id)) {
      return;
    }
    searchable_keys_.insert(id);
    ordered_keys_.push_back(id);
  }

  // Removes a key from the pool.
  Iterator Erase(Iterator it) {
    searchable_keys_.erase(*it);
    return ordered_keys_.erase(it);
  }

  bool Contains(AssetKey id) const { return searchable_keys_.contains(id); }

  void Clear() {
    searchable_keys_.clear();
    ordered_keys_.clear();
  }

  size_t Size() const { return ordered_keys_.size(); }

 private:
  // Searchable set of keys.
  absl::flat_hash_set<AssetKey> searchable_keys_;
  // Ordered list of keys.
  std::vector<AssetKey> ordered_keys_;
};

}  // namespace imp::imp_internal

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_ASSET_KEY_POOL_H_
