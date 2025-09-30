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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSET_CACHE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSET_CACHE_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/hash/hash.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/assets/asset_key_pool.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/base_asset_cache.h"
#include "core/async/future.h"
#include "core/common/ref_counter.h"

namespace imp {

// Used to cache assets loaded by Imp's AssetManager.
//
// ClearUnused is called periodically to remove assets that are no longer being
// used. Assets only get destroyed if they are unused when ClearUnused is
// called. If an unused asset is loaded again between ClearUnused calls, the
// reference count will be incremented and the asset will not be destroyed.
//
// SetLruCacheCapacity can be used to retain a fixed number of assets even if
// they are not used.
//
// This gives us a centralized place to track what assets currently exist which
// allows AssetManager to re-use instead of re-loading assets if they are
// already currently in use. It also gives us a convenient place to check what
// assets are in use so that we can more easily investigate memory usage of
// assets.
template <typename T>
class AssetCache : public BaseAssetCache {
 public:
  ~AssetCache() override { Clear(); }

  absl::optional<Future<AssetPtr<T>>> Retrieve(absl::string_view asset_url) {
    size_t key = absl::Hash<absl::string_view>()(asset_url);

    // Check if the asset is already loaded or loading.
    auto asset_itr = assets_.find(key);
    if (asset_itr != assets_.end()) {
      AssetHolder& asset_holder = asset_itr->second;

      // The asset was already loaded, but failed. Remove the cache entry so it
      // can be tried again.
      if (asset_holder.raw_asset_future.Ready() &&
          !asset_holder.raw_asset_future.Get().ok()) {
        assets_.erase(asset_itr);
      } else {
        // return the cached entry.
        return ToAssetFuture(asset_holder.raw_asset_future,
                             asset_holder.ref_counter.get());
      }
    }

    // This asset isn't loaded or loading, so return nullopt.
    return absl::nullopt;
  }

  Future<AssetPtr<T>> Store(absl::string_view asset_url,
                            Future<std::unique_ptr<T>> raw_asset_future) {
    if (asset_url.empty()) {
      unnamed_assets_.push_back(
          AssetHolder{std::make_unique<RefCounter>(), raw_asset_future});
      AssetHolder& asset_holder = unnamed_assets_.back();
      return ToAssetFuture(asset_holder.raw_asset_future,
                           asset_holder.ref_counter.get());
    }
    size_t key = absl::Hash<absl::string_view>()(asset_url);

    AssetHolder& asset_holder =
        assets_
            .emplace(key, AssetHolder{std::make_unique<RefCounter>(),
                                      raw_asset_future})
            .first->second;

    return ToAssetFuture(asset_holder.raw_asset_future,
                         asset_holder.ref_counter.get());
  }

  void CancelInProgressLoad(absl::string_view asset_url) override {
    size_t key = absl::Hash<absl::string_view>()(asset_url);
    auto found = assets_.find(key);
    if (found != assets_.end()) {
      AssetHolder& asset_holder = found->second;
      // Does nothing if the future is already ready.
      asset_holder.raw_asset_future.Cancel();
      cancelled_count_++;
    }
  }

  void Clear() override {
    for (auto itr = assets_.begin(); itr != assets_.end(); itr++) {
      ClearAssetHolder(itr->second);
    }
    assets_.clear();
    for (AssetHolder& asset_holder : unnamed_assets_) {
      ClearAssetHolder(asset_holder);
    }
    unnamed_assets_.clear();
    unused_asset_keys_.Clear();
  }

  void ClearUnused() override {
    // If possible, make space in the pool of retained assets.
    FindReusedAssets();

    auto itr = assets_.begin();
    while (itr != assets_.end()) {
      auto copy_itr = itr++;
      if (IsUnused(copy_itr->second)) {
        // Unused assets are inserted into the pool of retained assets first.
        // It will be destroyed depending on the size of the pool in the order
        // it was inserted.
        unused_asset_keys_.Insert(copy_itr->first);
      }
    }

    // Insert may have raised the pool over capacity. Evict assets as needed.
    if (unused_asset_keys_.Size() > lru_capacity_) {
      EvictAssets();
    }
    auto itr1 = unnamed_assets_.begin();
    while (itr1 != unnamed_assets_.end()) {
      if (IsUnused(*itr1)) {
        ClearAssetHolder(*itr1);
        itr1 = unnamed_assets_.erase(itr1);
        continue;
      }
      itr1++;
    }
  }

  int GetAssetCount() const override {
    return assets_.size() + unnamed_assets_.size();
  }
  // Returns the number of assets that have been destroyed.
  int64_t GetDestroyedCount() const { return destroyed_count_; }

  // Returns the number of loads were cancelled before completion.
  int64_t GetCancelledCount() const { return cancelled_count_; }

  // Retains a fixed number of assets even if they are not used.
  // Assets will be destroyed using a LRU strategy.  Order for the LRU strategy
  // is determined by the ClearUnused() function.
  void SetLruCacheCapacity(int64_t lru_capacity) {
    lru_capacity_ = lru_capacity;
  }
  int64_t GetLruCacheCapacity() const { return lru_capacity_; }

 private:
  struct AssetHolder {
    std::unique_ptr<RefCounter> ref_counter;
    Future<std::unique_ptr<T>> raw_asset_future;
  };

  void ClearAssetHolder(AssetHolder& asset_holder) {
    Future<std::unique_ptr<T>>& raw_asset_future =
        asset_holder.raw_asset_future;

    if (raw_asset_future.Ready()) {
      // Move the asset out of the future to guarantee that it is
      // destroyed.
      //
      // This addresses an edge case where something is holding onto a Future
      // to an asset past the lifetime of the asset cache. This forces the
      // asset to be destroyed preventing it from outliving the filament
      // engine and potentially causing a double-deletion when filament tries
      // to free asset resources.
      absl::StatusOr<std::unique_ptr<T>> asset = raw_asset_future.Move();
      destroyed_count_++;
    } else {
      raw_asset_future.Cancel();
      cancelled_count_++;
    }
  }

  bool IsUnused(const AssetHolder& asset_holder) const {
    if (!asset_holder.raw_asset_future.IsUnique()) {
      // The asset's future is referenced outside of the cache.
      return false;
    }
    if (!asset_holder.raw_asset_future.Ready()) {
      // Asset is still loading, but nothing is waiting on the result.
      return true;
    }
    if (!asset_holder.raw_asset_future.Get().ok()) {
      // Error loading the asset, but nothing is waiting on the result.
      return true;
    }
    // The asset finished loading, return true if there are no external
    // references to it.
    return asset_holder.ref_counter->GetCount() == 0;
  }

  void FindReusedAssets() {
    for (auto retained_iter = unused_asset_keys_.Begin();
         retained_iter != unused_asset_keys_.End();) {
      size_t assetkey = *retained_iter;
      typename absl::flat_hash_map<size_t, AssetHolder>::iterator
          key_vs_holder_itr = assets_.find(assetkey);
      if (key_vs_holder_itr != assets_.end() &&
          IsUnused(key_vs_holder_itr->second)) {
        // The asset is still unused.
        ++retained_iter;
      } else {
        // Remove asset from LRU, either it no longer exists, or it was re-used.
        retained_iter = unused_asset_keys_.Erase(retained_iter);
      }
    }
  }

  // Shrinks the pool of retained assets to lru_capacity_.
  void EvictAssets() {
    int64_t space_remaining = lru_capacity_ - unused_asset_keys_.Size();
    for (auto retained_iter = unused_asset_keys_.Begin();
         space_remaining < 0 && retained_iter != unused_asset_keys_.End();) {
      size_t assetkey = *retained_iter;
      typename absl::flat_hash_map<size_t, AssetHolder>::iterator
          key_vs_holder_itr = assets_.find(assetkey);

      if (key_vs_holder_itr != assets_.end() &&
          IsUnused(key_vs_holder_itr->second)) {
        // Remove asset from retained asset pool and destroy it because the
        // cache is over capacity.
        ClearAssetHolder(key_vs_holder_itr->second);
        assets_.erase(key_vs_holder_itr);
        retained_iter = unused_asset_keys_.Erase(retained_iter);
        space_remaining++;
      } else {
        ++retained_iter;
      }
    }
  }

  Future<AssetPtr<T>> ToAssetFuture(
      Future<std::unique_ptr<T>>& raw_asset_future, RefCounter* ref_counter) {
    return raw_asset_future.Then(
        [ref_counter](const std::unique_ptr<T>& raw_asset) {
          return AssetPtr<T>(raw_asset.get(), ref_counter->Retain());
        });
  }

  // Contains assets that are currently loading as well as fully loaded
  // assets.
  absl::flat_hash_map<size_t, AssetHolder> assets_;
  // Assets that come with no cache key.
  // They are here just because we need asset cache to do the memory management.
  // They are not supposed to be retrieved from this cache.
  std::vector<AssetHolder> unnamed_assets_;

  // The number of unused assets to retain.
  int64_t lru_capacity_ = 0;
  // An ordered collection of keys to assets which are currently retained in
  // memory, but not actively in use. Order in the pool is determined by when
  // ClearUnused found that that asset has no references.
  // Only the lookup key is stored, the asset remains in the assets_ map.
  imp_internal::AssetKeyPool unused_asset_keys_;
  // The number of assets that have been released from the cache.
  int64_t destroyed_count_ = 0;
  int64_t cancelled_count_ = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSET_CACHE_H_
