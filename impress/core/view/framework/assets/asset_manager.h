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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_ASSET_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_ASSET_MANAGER_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <utility>

#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/assets/asset_cache.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/base_asset_cache.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/async/future_group.h"
#include "core/common/hash.h"
#include "core/config.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/loader/loader_creator.h"
#include "core/media/media_asset.h"
#include "core/render/image_asset.h"
#include "core/resources/resource_manager.h"
#include "core/resources/url_loader.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_asset_loader.h"
#include "core/view/framework/assets/material_asset.h"
#include "core/view/framework/assets/proto_asset.h"
#include "core/view/utils/asset.h"
#include "robin_map/include/tsl/robin_map.h"

#if IMP_PLATFORM(ANDROID)
#include <vector>

#include "core/async/executor.h"
#include "core/async/future_interrupter.h"
#include "core/common/invocable.h"
#include "core/view/platforms/android/wrappers/input_stream.h"
#endif

#if IMP_RUNTIME(DEV)
#include "absl/container/btree_set.h"
#endif

namespace imp {

using ::imp::media::MediaAsset;

// AssetManager is used to load and assets for use with Impress.
//
// The AssetManager supports loading loading both local and remote assets
// cross-platform. Typically, this is done by passing an an AssetDefinition
// generated using the build rule imp_assets.
//
// Assets are immutable, reference counted, and cached in the AssetManager.
//
// Once loading an asset has begun, subsequent calls for the same asset will
// re-use the cached asset. At the end of each frame, unused assets are
// destroyed.
//
// AssetManager provides several methods for loading standard Impress asset
// types (i.e. ImageAsset, MaterialAsset).
//
// In addition, custom asset types can be created and loaded by doing the
// following:
//
// First, making a custom asset type that specifies a static Load method:
//
// class ExampleAsset {
//   public:
//     static Future<std::unique_ptr<ExampleAsset>> Load(
//       imp::BaseView* view, absl::string_view asset_url,
//       Future<imp::resources::Resource> resource_future);
// };
//
// Then, use AssetManager::LoadAsset<AssetT> to load it:
//
// LoadAsset<ExampleAsset>(example_assets::kFooBinary);
//
// Additionally, arbitrary parameters can be passed to Load through
// AssetManager::LoadAsset<AssetT>.
//
// class ExampleAsset {
//   public:
//     static Future<std::unique_ptr<ExampleAsset>> Load(
//       imp::BaseView* view, absl::string_view asset_url,
//       Future<imp::resources::Resource> resource_future,
//       int bar,
//       bool bazz);
// };
//
// LoadAsset<ExampleAsset>(example_assets::kFooBinary, 20, true);
//
//
// See imp/samples/simple/BUILD for an example.
// See impel/resources/resources.bzl for reference.
class AssetManager {
 public:
  explicit AssetManager(BaseView* view);

  // LoadModel and LoadGltfAsset will use the asset managers default load
  // options if none are provided.
  void SetDefaultLoadOptions(GltfAsset::LoadOptions load_options);
  const GltfAsset::LoadOptions& GetDefaultLoadOptions();

  // Loads a gLTF model into a Node asynchronously from an asset definition.
  //
  // The returned Node will have children that map to the gLTF hierarchy.
  // The node will also have a GltfRenderer attached to it that can be used
  // to access information about the model.
  //
  // The model may be cached on subsequent calls, in which case the callback
  // will be invoked immediately.
  //
  // If the model is unable to be loaded, then the resulting NodeHandle is
  // invalid.
  Future<NodeHandle> LoadModel(
      const AssetDefinition& asset_definition,
      absl::optional<GltfAsset::LoadOptions> options = absl::nullopt);
  Future<NodeHandle> LoadModel(
      absl::string_view asset_url,
      absl::optional<GltfAsset::LoadOptions> options = absl::nullopt);
  Future<NodeHandle> LoadModel(
      absl::Cord contents, absl::string_view asset_url,
      absl::optional<GltfAsset::LoadOptions> options = absl::nullopt);

  Future<AssetPtr<GltfAsset>> LoadGltfAsset(
      const AssetDefinition& asset_definition,
      absl::optional<GltfAsset::LoadOptions> options = absl::nullopt);

  Future<AssetPtr<GltfAsset>> LoadGltfAsset(
      absl::string_view asset_url,
      absl::optional<GltfAsset::LoadOptions> options = absl::nullopt);

  Future<AssetPtr<GltfAsset>> LoadGltfAsset(
      absl::Cord contents, absl::string_view asset_url,
      absl::optional<GltfAsset::LoadOptions> options = absl::nullopt);

#if IMP_PLATFORM(ANDROID)
  Future<AssetPtr<GltfAsset>> LoadGltfAsset(
      std::unique_ptr<InputStream> input_stream, absl::string_view asset_url,
      absl::optional<GltfAsset::LoadOptions> options = absl::nullopt);
#endif

  Future<AssetPtr<MediaAsset>> LoadMedia(
      const AssetDefinition& asset_definition);
  Future<AssetPtr<MediaAsset>> LoadMedia(absl::string_view asset_url);

  // Loads a MaterialAsset and caches it in the AssetManager.
  //
  // MaterialAssets can be used with the MaterialFactory to create materials
  // that can be rendered with a MeshRenderer or GltfMesh.
  // MaterialPreCompileOptions can be specified so that certain material
  // variants are pre-compiled before the MaterialAsset future becomes ready.
  // If the MaterialPreCompileOptions contain material constants, those can be
  // used to compile custom variants of the same material and the variants will
  // be cached separately from each other.
  //
  // This is a convenience method for AssetManager::LoadAsset<MaterialAsset>.
  Future<AssetPtr<MaterialAsset>> LoadMaterial(
      const AssetDefinition& asset_definition,
      std::optional<MaterialPreCompileOptions> material_pre_compile_options =
          std::nullopt);
  Future<AssetPtr<MaterialAsset>> LoadMaterial(
      absl::string_view asset_url,
      std::optional<MaterialPreCompileOptions> material_pre_compile_options =
          std::nullopt);

  // Loads an ImageAsset and caches it in the AssetManager.
  //
  // ImageAssets can be used with the TextureFactory to create textures that can
  // be rendered with a material.
  //
  // This is a convenience method for AssetManager::LoadAsset<ImageAsset>.
  Future<AssetPtr<ImageAsset>> LoadImage(
      const AssetDefinition& asset_definition);
  Future<AssetPtr<ImageAsset>> LoadImage(absl::string_view asset_url);
  Future<AssetPtr<ImageAsset>> LoadImage(absl::Cord contents,
                                         absl::string_view asset_url);

  // Loads an ImageBasedLightingAsset and caches it in the AssetManager.
  //
  // ImageBasedLightingAsset can be used with the ImageBasedLightingFactory to
  // create ImageBasedLighting that can be used for indirect lighting in
  // LightingManager.
  //
  // This is a convenience method for
  // AssetManager::LoadAsset<ImageBasedLightingAsset>.
  Future<AssetPtr<ImageBasedLightingAsset>> LoadImageBasedLighting(
      const AssetDefinition& asset_definition);
  Future<AssetPtr<ImageBasedLightingAsset>> LoadImageBasedLighting(
      absl::string_view asset_url);

  // Loads a ProtoAsset wrapping a protocol buffer of type T and caches it in
  // the AssetManager.
  //
  // ProtoAsset supports both impress protos ((broken link))
  // and standard protos.
  //
  // This is a convenience method for AssetManager::LoadAsset<ProtoAsset<T>>.
  //
  // The retain_resource parameter (default false) determines if the ProtoAsset
  // will retain a reference to the underlying resource after being loaded. This
  // is needed only if the proto contains string_view, Any, or Cord types since
  // the proto loader uses AppendCordFromExternal(cord) to avoid extra copying.
  template <typename T>
  Future<AssetPtr<ProtoAsset<T>>> LoadProto(
      const AssetDefinition& asset_definition, bool retain_resource = false);
  template <typename T>
  Future<AssetPtr<ProtoAsset<T>>> LoadProto(absl::string_view asset_url,
                                            bool retain_resource = false);

  // Loads an asset of type AssetT and caches it in the AssetManager from an
  // AssetDefinition generated by the imp_assets build rule.
  template <typename AssetT, typename... Args>
  Future<AssetPtr<AssetT>> LoadAsset(const AssetDefinition& asset_definition,
                                     Args&&... args);

  // Loads an asset of type AssetT from an AssetDefinition generated by the
  // imp_assets build rule, and caches it, using an explicit cache key, in the
  // AssetManager.
  template <typename AssetT, typename... Args>
  Future<AssetPtr<AssetT>> LoadAsset(const AssetDefinition& asset_definition,
                                     absl::string_view asset_cache_key,
                                     Args&&... args);

  // Loads an asset of type AssetT and caches it in the AssetManager from a url.
  //
  // The url could be remote (http), or the path to an asset in the imp_assets
  // rule that has previously been registered.
  template <typename AssetT, typename... Args>
  Future<AssetPtr<AssetT>> LoadAsset(absl::string_view asset_url,
                                     Args&&... args);

  // Loads an asset of type AssetT and caches it using an explicit cache key in
  // the AssetManager from a url.
  //
  // The url could be remote (http), or the path to an asset in the imp_assets
  // rule that has previously been registered.
  template <typename AssetT, typename... Args>
  Future<AssetPtr<AssetT>> LoadAsset(absl::string_view asset_url,
                                     absl::string_view asset_cache_key,
                                     Args&&... args);

  // Loads an asset of type AssetT and caches it in the AssetManager directly
  // from an absl::Cord.
  //
  // The asset_url passed in is used to identify the asset for caching purposes
  // when re-using the asset.
  template <typename AssetT, typename... Args>
  Future<AssetPtr<AssetT>> LoadAsset(absl::Cord contents,
                                     absl::string_view asset_url,
                                     Args&&... args);

#if IMP_PLATFORM(ANDROID)
  // Loads an asset of type AssetT directly
  // from an InputStream (JavaWrapper), and caches it using an explicit cache
  // key in the AssetManager for re-use.
  template <typename AssetT, typename... Args>
  Future<AssetPtr<AssetT>> LoadAsset(std::unique_ptr<InputStream> input_stream,
                                     absl::string_view asset_cache_key,
                                     Args&&... args);
#endif

  // Asynchronously loads a raw resource.
  Future<resources::Resource> LoadResource(
      const AssetDefinition& asset_definition,
      std::optional<FutureGroup> future_group = std::nullopt);
  Future<resources::Resource> LoadResource(
      absl::string_view asset_url,
      std::optional<FutureGroup> future_group = std::nullopt);

  // Returns the loading progress of pending downloads as a fraction, with
  // download_baseline establishing 0%.
  float GetDownloadProgress(size_t download_baseline);

  // Returns the loading progress of the indicated url as a fraction of its
  // entire requested size. Returns 0 if the url isn't found or the size of the
  // entire url is unknown.
  float GetDownloadProgress(std::string_view asset_url);

  // Returns the number of bytes downloaded so far by the session.
  size_t GetDownloadedSize();

  // Gets rid of all the data owned by the asset manager.
  void Cleanup();

  // Destroys all assets cached by the AssetManager that aren't currently
  // referenced anywhere. This is done automatically once per frame.
  void ClearUnused();

  // Returns the total number of assets of all types(both in-flight and finished
  // loading) held by the AssetManager.
  int GetAssetCount() const;

  // Returns the total number of assets of a specific type held by the
  // AssetManager.  Includes assets that are currently loading.
  template <typename AssetT>
  int GetResidentCount() const;

  // Returns the count of asset loads which were cancelled before completion for
  // a specific type of asset.
  template <typename AssetT>
  int GetCancelledCount() const;

  // Returns the count of assets of a specific type that have been released from
  // the cache.
  template <typename AssetT>
  int GetDestroyedCount() const;

  // Immediately cancels loading for the asset passed in for any asset type. If
  // the asset isn't currently loading (either it's cached or not started), then
  // this does nothing.
  void CancelLoad(const AssetDefinition& asset_definition);
  void CancelLoad(absl::string_view asset_url);

  // Sets the url loader to be used for loading from the network. If this is not
  // called, the default url loader will be used.
  void SetUrlLoader(std::unique_ptr<resources::UrlLoader> url_loader);

  // Sets a config globally for the url loader. The config will be applied to
  // all urls handled by the url loader if relevant.
  void SetUrlLoaderConfig(resources::UrlLoader::Config config);

  // Moves the Url Loader out of the asset manager and returns it.
  std::unique_ptr<resources::UrlLoader> MoveUrlLoader();

  // Sets the sandboxed loader creator to use when loading GLTFs.
  // This must be called prior to loading any GLTFs.
  // In a select statement guarded on if_android, clients should include
  // imp/loader:loader_in_sandbox and then call:
  // #if IMP_PLATFORM(ANDROID)
  // SetSandboxedGltfLoader(std::make_unique<LoaderInSandbox>());
  // #endif
  void SetSandboxedGltfLoader(
      std::unique_ptr<loader::LoaderCreator> sandboxed_gltf_loader_creator);

#if IMP_RUNTIME(DEV)
  const absl::btree_set<std::string>& GetRegisteredResources() const {
    return resource_manager_.GetRegisteredResources();
  }
#endif

 private:
  template <typename AssetT>
  AssetCache<AssetT>* GetAssetCache() const;

  // Implementation details shared by all versions of LoadAsset.
  // Fn should be a functor with the signature Future<resources::Resource>().
  template <typename AssetT, typename Fn, typename... Args>
  Future<AssetPtr<AssetT>> LoadAssetImpl(Fn load_resource_fn,
                                         absl::string_view asset_url,
                                         absl::string_view asset_cache_key,
                                         Args&&... args);

  // The view that owns the AssetManager.
  BaseView* view_;

  // The resource manager that handles loading the raw data of the underlying
  // resources.
  resources::ResourceManager resource_manager_;

  // Stores a mapping of asset type to asset cache. Used to track both loading
  // and loaded assets.
  tsl::robin_map<HashValue, std::unique_ptr<BaseAssetCache>> caches_;

  // Used to assist with the loading of GltfAssets.
  GltfAsset::LoadOptions default_load_options_;
  GltfAssetLoader gltf_asset_loader_;
};

template <typename T>
Future<AssetPtr<ProtoAsset<T>>> AssetManager::LoadProto(
    const AssetDefinition& asset_definition, bool retain_resource) {
  return LoadAsset<ProtoAsset<T>>(asset_definition, retain_resource);
}

template <typename T>
Future<AssetPtr<ProtoAsset<T>>> AssetManager::LoadProto(
    absl::string_view asset_url, bool retain_resource) {
  return LoadAsset<ProtoAsset<T>>(asset_url, retain_resource);
}

template <typename AssetT, typename... Args>
Future<AssetPtr<AssetT>> AssetManager::LoadAsset(
    const AssetDefinition& asset_definition, Args&&... args) {
  return LoadAssetImpl<AssetT>(
      [this, &asset_definition]() { return LoadResource(asset_definition); },
      asset_definition.GetUrl(), asset_definition.GetUrl(),
      std::forward<Args>(args)...);
}

template <typename AssetT, typename... Args>
Future<AssetPtr<AssetT>> AssetManager::LoadAsset(
    const AssetDefinition& asset_definition, absl::string_view asset_cache_key,
    Args&&... args) {
  return LoadAssetImpl<AssetT>(
      [this, &asset_definition]() { return LoadResource(asset_definition); },
      asset_definition.GetUrl(), asset_cache_key, std::forward<Args>(args)...);
}

template <typename AssetT, typename... Args>
Future<AssetPtr<AssetT>> AssetManager::LoadAsset(absl::string_view asset_url,
                                                 Args&&... args) {
  return LoadAssetImpl<AssetT>(
      [this, asset_url]() { return LoadResource(asset_url); }, asset_url,
      asset_url, std::forward<Args>(args)...);
}

template <typename AssetT, typename... Args>
Future<AssetPtr<AssetT>> AssetManager::LoadAsset(
    absl::string_view asset_url, absl::string_view asset_cache_key,
    Args&&... args) {
  return LoadAssetImpl<AssetT>(
      [this, asset_url]() { return LoadResource(asset_url); }, asset_url,
      asset_cache_key, std::forward<Args>(args)...);
}

template <typename AssetT, typename... Args>
Future<AssetPtr<AssetT>> AssetManager::LoadAsset(absl::Cord contents,
                                                 absl::string_view asset_url,
                                                 Args&&... args) {
  Future<resources::Resource> resource_future(
      resources::Resource(std::move(contents)));

  return LoadAssetImpl<AssetT>(
      [this, &resource_future]() { return resource_future; }, asset_url,
      asset_url, std::forward<Args>(args)...);
}

#if IMP_PLATFORM(ANDROID)
template <typename AssetT, typename... Args>
Future<AssetPtr<AssetT>> AssetManager::LoadAsset(
    std::unique_ptr<InputStream> input_stream,
    absl::string_view asset_cache_key, Args&&... args) {
  // Defines a function that reads the input stream and returns the cord.
  imp::Invocable<Future<resources::Resource>()> load_bytes_fn =
      [input_stream_ptr = input_stream.get(),
       context = view_->GetContext()]() -> Future<resources::Resource> {
    FutureInterrupter interrupter;

    // Reads the bytes data into a cord in a background thread.
    Future<absl::Cord> future = Future<absl::Cord>::Schedule(
        [input_stream_ptr, context, interrupter]() {
          return input_stream_ptr->BlockingReadFromJavaInputStream(
              context.GetJniEnv(), /*string_uri=(trivial)*/ "",
              /*content_length=(trivial)*/ -1,
              /*download_progress_info=*/nullptr,
              std::vector<FutureInterrupter>{interrupter});
        },
        {.executor = Executor::Type::kBackground});

    return future.Then(
        [interrupter](absl::Cord cord) mutable {
          cord.Flatten();
          return interrupter.MakeInterruptible(Future<resources::Resource>(
              resources::Resource(std::move(cord))));
        },
        Executor::Type::kBackground);
  };

  Future<AssetPtr<AssetT>> asset_future =
      LoadAssetImpl<AssetT>(std::move(load_bytes_fn), asset_cache_key,
                            asset_cache_key, std::forward<Args>(args)...);

  asset_future.DependsOn(std::move(input_stream));

  return asset_future;
}
#endif

template <typename AssetT, typename Fn, typename... Args>
Future<AssetPtr<AssetT>> AssetManager::LoadAssetImpl(
    Fn load_resource_fn, absl::string_view asset_url,
    absl::string_view asset_cache_key, Args&&... args) {
  // Find or create the asset cache.
  AssetCache<AssetT>* cache = nullptr;
  auto itr = caches_.find(type_traits::kTypeHash<AssetT>);
  if (itr != caches_.end()) {
    cache = static_cast<AssetCache<AssetT>*>(itr.value().get());
  } else {
    auto unique_cache = std::make_unique<AssetCache<AssetT>>();
    cache = unique_cache.get();
    caches_[type_traits::kTypeHash<AssetT>] = std::move(unique_cache);
  }

  // Attempt to retrieve the asset.
  // If the asset has finished loading, this will be a ready future containing
  // the asset. If the asset in the process of loading, this will be an unready
  // future. Otherwise, nullopt.
  if (!asset_cache_key.empty()) {
    absl::optional<Future<AssetPtr<AssetT>>> asset =
        cache->Retrieve(asset_cache_key);
    if (asset.has_value()) {
      return *asset;
    }
  }

  Future<resources::Resource> resource_future = load_resource_fn();
  Future<std::unique_ptr<AssetT>> asset_future = AssetT::Load(
      view_, asset_url, resource_future, std::forward<Args>(args)...);

  // Even if asset_cache_key is empty, we still need cache to store it because
  // cache is needed for memory management.
  return cache->Store(asset_cache_key, asset_future);
}
template <typename AssetT>
AssetCache<AssetT>* AssetManager::GetAssetCache() const {
  auto itr = caches_.find(type_traits::kTypeHash<AssetT>);
  if (itr == caches_.end()) {
    return nullptr;
  }
  return static_cast<AssetCache<AssetT>*>(itr.value().get());
}

template <typename AssetT>
int AssetManager::GetResidentCount() const {
  AssetCache<AssetT>* cache = GetAssetCache<AssetT>();
  if (cache == nullptr) {
    return 0;
  }
  return cache->GetAssetCount();
}

template <typename AssetT>
int AssetManager::GetCancelledCount() const {
  AssetCache<AssetT>* cache = GetAssetCache<AssetT>();
  if (cache == nullptr) {
    return 0;
  }
  return cache->GetCancelledCount();
}

template <typename AssetT>
int AssetManager::GetDestroyedCount() const {
  AssetCache<AssetT>* cache = GetAssetCache<AssetT>();
  if (cache == nullptr) {
    return 0;
  }
  return cache->GetDestroyedCount();
}
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_ASSET_MANAGER_H_
