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
#include "core/assets/asset_ptr.h"
#include "core/assets/base_asset_cache.h"
#include "core/async/future.h"
#include "core/common/ref_counter.h"

namespace imp {

// Used to cache assets loaded by Imp's AssetManager.
//
// ClearUnused is called each frame to remove assets that are no longer being
// used. If an asset's use count drops to zero but is then accessed again before
// the end of the frame, then it isn't dropped from the cache.
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
  }

  void ClearUnused() override {
    auto itr = assets_.begin();
    while (itr != assets_.end()) {
      auto copy_itr = itr++;
      if (ClearAssetHolderIfUnused(copy_itr->second)) {
        assets_.erase(copy_itr);
        continue;
      }
    }
    auto itr1 = unnamed_assets_.begin();
    while (itr1 != unnamed_assets_.end()) {
      if (ClearAssetHolderIfUnused(*itr1)) {
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

  // Returns true if the AssetHolder needs to be removed.
  bool ClearAssetHolderIfUnused(AssetHolder& asset_holder) {
    Future<std::unique_ptr<T>>& raw_asset_future =
        asset_holder.raw_asset_future;

    // A user could be holding onto the future instead of the AssetPtr, so
    // don't consider an asset unused unless both the future and the asset
    // aren't held.
    if (raw_asset_future.IsUnique()) {
      if (raw_asset_future.Ready()) {
        // If the asset has finished, and it either failed or nothing is
        // using it, then we can remove it.
        if (!raw_asset_future.Get().ok() ||
            asset_holder.ref_counter->GetCount() == 0) {
          // Move the asset out of the future to guarantee that it is
          // destroyed.
          absl::StatusOr<std::unique_ptr<T>> asset = raw_asset_future.Move();
          destroyed_count_++;
          return true;
        }
      } else {
        // The asset is currently in the process of loading, but nothing is
        // referencing it anymore so remove it.
        asset_holder.raw_asset_future.Cancel();
        cancelled_count_++;
        return true;
      }
    }
    return false;
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

  // The number of assets that have been released from the cache.
  int64_t destroyed_count_ = 0;
  int64_t cancelled_count_ = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSET_CACHE_H_
