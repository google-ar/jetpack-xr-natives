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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_GLTF_ASSET_LOADER_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_GLTF_ASSET_LOADER_H_

#include <cstddef>
#include <deque>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "mediapipe/framework/deps/clock.h"
#include "absl/time/time.h"
#include "core/assets/gltf/gltf_asset.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/context.h"
#include "core/loader/loader.h"
#include "core/loader/loader_creator.h"
#include "core/material_library/material_package.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_state.proto.imp.h"
#include "core/view/utils/string_map.h"

namespace imp {

namespace animation {
class GltfAnimation;
}  // namespace animation

// Used by AssetManager to manage the loading of an individual GltfAsset.
class GltfAssetLoader {
 public:
  // Creates a GltfAsset from a Resource.
  // Invokes callback when the GltfAsset IsFullyLoaded.
  // This is transferring ownership, the caller of Create is expected to
  // delete the instantiated GltfAsset.
  //
  // Note: asset_id may not match the real id passed into the AssetManager.
  // real_asset_id is the id passed into the AssetManager to be passed through
  // GltfAsset::LoadEvent.
  Future<std::unique_ptr<GltfAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future,
      resources::ResourceManager* resource_manager,
      GltfAsset::LoadOptions options, mediapipe::Clock* clock = nullptr);

  // Sets the sandboxed loader creator to use when loading GLTFs.
  // This must be called prior to loading any GLTFs.
  // In a select statement guarded on if_android, clients should include
  // imp/loader:loader_in_sandbox and then call:
  // #if IMP_PLATFORM(ANDROID)
  // SetSandboxedGltfLoader(std::make_unique<LoaderInSandbox>());
  // #endif
  void SetSandboxedGltfLoader(
      std::unique_ptr<loader::LoaderCreator> sandboxed_gltf_loader_creator);

 private:
  using LoadAssetFn =
      std::function<Future<resources::Resource>(absl::string_view)>;

  class LoadInProgress {
   public:
    // Make sure to hold onto the resource so it doesn't get cleaned up.
    LoadInProgress(const Context& context,
                   std::unique_ptr<loader::Loader> loader,
                   resources::Resource resource);

    // Loads the resource data, should be called on a background thread.
    Future<absl::Status> Load(
        LoadAssetFn load_missing_asset, mediapipe::Clock* clock,
        std::shared_ptr<LoadInProgress>& load_in_progress);

    // Creates the actual GltfAsset.
    // This must be called on the main thread, and Load must have already been
    // called.
    Future<std::unique_ptr<GltfAsset>> CreateGltfAsset(
        BaseView* view, std::shared_ptr<LoadInProgress>& load_in_progress,
        GltfState::ColliderMode collider_mode =
            GltfState::ColliderMode::GLTF_COLLIDER_BOUNDS_PER_MESH_DEFAULT);

    absl::Status LoadAnimations();

    // Information stored about the load in progress.
    absl::Time start_parse_time_ = absl::InfinitePast();
    absl::Time start_download_deps_time_ = absl::InfinitePast();
    absl::Time end_download_deps_time_ = absl::InfinitePast();
    mutable absl::Mutex num_bytes_downloaded_mutex_;
    size_t num_bytes_downloaded_ ABSL_GUARDED_BY(num_bytes_downloaded_mutex_);

   private:
    const Context& context_;
    std::unique_ptr<loader::Loader> loader_;
    resources::Resource resource_;
    std::deque<resources::Resource> missing_asset_resources_;
    std::vector<std::unique_ptr<animation::GltfAnimation>> loaded_animations_;

    Future<absl::Status> LoadHelper(
        GltfAssetLoader::LoadAssetFn load_missing_asset,
        std::shared_ptr<LoadInProgress> load_in_progress);
  };

  std::unique_ptr<loader::LoaderCreator> sandboxed_gltf_loader_creator_;

  // Manages the loading and storage of the glTF materials.
  std::unique_ptr<MaterialPackage> material_package_;
  std::unique_ptr<MaterialPackage> lite_material_package_;
  StringMap<std::unique_ptr<MaterialPackage>> custom_material_packages_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_GLTF_ASSET_LOADER_H_
