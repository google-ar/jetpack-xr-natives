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

#include "core/view/framework/assets/asset_manager.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/base_asset_cache.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_group.h"
#include "core/common/string_helpers.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/loader/loader_creator.h"
#include "core/ncsb/component_handle.h"
#include "core/render/image_asset.h"
#include "core/resources/resource_manager.h"
#include "core/resources/url_loader.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/assets/gltf_asset_loader.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/assets/material_asset.h"
#include "core/view/utils/asset.h"

#if IMP_PLATFORM(ANDROID)
#include "core/common/registry.h"
#include "core/view/platforms/android/wrappers/input_stream.h"
#include "core/view/platforms/xr_android/xr_color_space_helper.h"
#endif

namespace imp {

namespace {

constexpr absl::string_view kDataUriPrefix = "data:";
constexpr absl::string_view kDataUriImageHeaderPrefix = "data:image/";
constexpr absl::string_view kBase64Encoded = "base64,";
std::vector<std::string>* kSupportedImageTypes =
    new std::vector<std::string>({"jpeg;", "png;"});

// Returns the length of the image uri prefix or -1 if it's not supported
int GetImageUriPrefixLength(absl::string_view uri) {
  for (const auto& image_type : *kSupportedImageTypes) {
    std::string uri_prefix =
        absl::StrCat(kDataUriImageHeaderPrefix, image_type);
    if (absl::StartsWith(uri, uri_prefix)) {
      return uri_prefix.length();
    }
  }
  return -1;
}

bool IsDataUri(absl::string_view uri) {
  return absl::StartsWith(uri, kDataUriPrefix);
}

bool IsSupportedDataUri(absl::string_view uri) {
  return GetImageUriPrefixLength(uri) > 0;
}

absl::Cord ExtractDataFromImageUri(absl::string_view uri) {
  int prefix_length = GetImageUriPrefixLength(uri);
  if (prefix_length <= 0) {
    return absl::Cord();
  }

  std::string data_uri;
  if (absl::StartsWith(uri.substr(prefix_length), kBase64Encoded)) {
    data_uri = std::string(uri.substr(prefix_length + kBase64Encoded.length()));
  } else {
    data_uri = std::string(uri.substr(prefix_length));
  }
  std::string dest;
  DeserializeBase64(data_uri, &dest);
  absl::Cord data(dest);
  data.Flatten();
  return data;
}

std::string GenerateMaterialConstantsCacheKey(
    absl::string_view asset_url,
    std::vector<imp::MaterialPreCompileConstant> constants) {
  std::sort(
      constants.begin(), constants.end(),
      [](const imp::MaterialPreCompileConstant& a,
         const imp::MaterialPreCompileConstant& b) { return a.name < b.name; });
  std::string key(asset_url);
  for (const auto& constant : constants) {
    switch (constant.value.index()) {
      case MaterialPreCompileConstant::kValue_IntValue:
        absl::StrAppend(&key, constant.name, *constant.int_value());
        break;
      case MaterialPreCompileConstant::kValue_FloatValue:
        absl::StrAppend(&key, constant.name, *constant.float_value());
        break;
      case MaterialPreCompileConstant::kValue_BoolValue:
        absl::StrAppend(&key, constant.name, *constant.bool_value());
        break;
    }
  }
  return key;
}

}  // namespace

using resources::Resource;

AssetManager::AssetManager(BaseView* view)
    : view_(view), resource_manager_(view->GetContext()) {}

void AssetManager::SetDefaultLoadOptions(GltfAsset::LoadOptions load_options) {
  default_load_options_ = std::move(load_options);
}

const GltfAsset::LoadOptions& AssetManager::GetDefaultLoadOptions() {
  return default_load_options_;
}

Future<NodeHandle> AssetManager::LoadModel(
    const AssetDefinition& asset_definition,
    absl::optional<GltfAsset::LoadOptions> options) {
  return LoadModel(asset_definition.GetUrl(), std::move(options));
}

Future<NodeHandle> AssetManager::LoadModel(
    absl::string_view asset_url,
    absl::optional<GltfAsset::LoadOptions> options) {
  IMP_TRACE();
  NodeHandle node = view_->CreateNode();
  return node->AddComponent<GltfRenderer>(asset_url, std::move(options))
      .Then([node, this](
                const absl::StatusOr<ComponentHandle<GltfRenderer>>& statusor)
                -> absl::StatusOr<NodeHandle> {
        IMP_TRACE_BLOCK("Then");
        if (!statusor.ok()) {
          view_->DestroyNode(node);
          return statusor.status();
        }
        return node;
      });
}

Future<NodeHandle> AssetManager::LoadModel(
    absl::Cord contents, absl::string_view asset_url,
    absl::optional<GltfAsset::LoadOptions> options) {
  IMP_TRACE();
  NodeHandle node = view_->CreateNode();
  return node
      ->AddComponent<GltfRenderer>(std::move(contents), asset_url,
                                   std::move(options))
      .Then([node, this](
                const absl::StatusOr<ComponentHandle<GltfRenderer>>& statusor)
                -> absl::StatusOr<NodeHandle> {
        IMP_TRACE_BLOCK("Then");
        if (!statusor.ok()) {
          view_->DestroyNode(node);
          return statusor.status();
        }
        return node;
      });
}

Future<AssetPtr<GltfAsset>> AssetManager::LoadGltfAsset(
    const AssetDefinition& asset_definition,
    absl::optional<GltfAsset::LoadOptions> options) {
  return LoadAsset<GltfAsset>(asset_definition, &gltf_asset_loader_,
                              options ? *options : GetDefaultLoadOptions());
}

Future<AssetPtr<GltfAsset>> AssetManager::LoadGltfAsset(
    absl::string_view asset_url,
    absl::optional<GltfAsset::LoadOptions> options) {
  return LoadAsset<GltfAsset>(asset_url, &gltf_asset_loader_,
                              options ? *options : GetDefaultLoadOptions());
}

Future<AssetPtr<GltfAsset>> AssetManager::LoadGltfAsset(
    absl::Cord contents, absl::string_view asset_url,
    absl::optional<GltfAsset::LoadOptions> options) {
  return LoadAsset<GltfAsset>(contents, asset_url, &gltf_asset_loader_,
                              options ? *options : GetDefaultLoadOptions());
}

#if IMP_PLATFORM(ANDROID)
Future<AssetPtr<GltfAsset>> AssetManager::LoadGltfAsset(
    std::unique_ptr<InputStream> input_stream, absl::string_view asset_url,
    absl::optional<GltfAsset::LoadOptions> options) {
  return LoadAsset<GltfAsset>(std::move(input_stream), asset_url,
                              &gltf_asset_loader_,
                              options ? *options : GetDefaultLoadOptions());
}
#endif

Future<AssetPtr<MediaAsset>> AssetManager::LoadMedia(
    const AssetDefinition& asset_definition) {
  return LoadAsset<MediaAsset>(asset_definition);
}

Future<AssetPtr<MediaAsset>> AssetManager::LoadMedia(
    absl::string_view asset_url) {
  return LoadAsset<MediaAsset>(asset_url);
}

Future<AssetPtr<MaterialAsset>> AssetManager::LoadMaterial(
    const AssetDefinition& asset_definition,
    std::optional<MaterialPreCompileOptions> material_pre_compile_options) {
  if (material_pre_compile_options) {
    if (!material_pre_compile_options->constants.empty()) {
      std::string cache_key = GenerateMaterialConstantsCacheKey(
          asset_definition.GetUrl(), material_pre_compile_options->constants);
      return LoadAsset<MaterialAsset>(asset_definition,
                                      absl::string_view(cache_key),
                                      *material_pre_compile_options);
    }
    return LoadAsset<MaterialAsset>(asset_definition,
                                    *material_pre_compile_options);
  }
  return LoadAsset<MaterialAsset>(asset_definition);
}

Future<AssetPtr<MaterialAsset>> AssetManager::LoadMaterial(
    absl::string_view asset_url,
    std::optional<MaterialPreCompileOptions> material_pre_compile_options) {
#if IMP_PLATFORM(ANDROID)
  // Add color space to the pre-compile options.
  auto color_space_helper = view_->GetRegistry().Get<XrColorSpaceHelper>();
  if (color_space_helper.ok()) {
    if (!material_pre_compile_options) {
      material_pre_compile_options.emplace();
    }
    material_pre_compile_options->constants.push_back(
        XrColorSpaceHelper::GetColorSpacePrecompileConstant());
  }
#endif

  if (material_pre_compile_options) {
    if (!material_pre_compile_options->constants.empty()) {
      std::string cache_key = GenerateMaterialConstantsCacheKey(
          asset_url, material_pre_compile_options->constants);
      return LoadAsset<MaterialAsset>(asset_url, absl::string_view(cache_key),
                                      *material_pre_compile_options);
    }
    return LoadAsset<MaterialAsset>(asset_url, *material_pre_compile_options);
  }
  return LoadAsset<MaterialAsset>(asset_url);
}

Future<AssetPtr<ImageAsset>> AssetManager::LoadImage(
    const AssetDefinition& asset_definition) {
  if (IsDataUri(asset_definition.GetUrl())) {
    if (IsSupportedDataUri(asset_definition.GetUrl())) {
      return LoadImage(ExtractDataFromImageUri(asset_definition.GetUrl()),
                       asset_definition.GetUrl());
    }
    return Future<AssetPtr<ImageAsset>>(
        absl::InternalError("Data URI is not supported"));
  }
  return LoadAsset<ImageAsset>(asset_definition);
}

Future<AssetPtr<ImageAsset>> AssetManager::LoadImage(
    absl::string_view asset_url) {
  if (IsDataUri(asset_url)) {
    if (IsSupportedDataUri(asset_url)) {
      return LoadImage(ExtractDataFromImageUri(asset_url), asset_url);
    }
    return Future<AssetPtr<ImageAsset>>(
        absl::InternalError("Data URI is not supported"));
  }
  return LoadAsset<ImageAsset>(asset_url);
}

Future<AssetPtr<ImageAsset>> AssetManager::LoadImage(
    absl::Cord contents, absl::string_view asset_url) {
  std::optional<absl::string_view> flattened_string = contents.TryFlat();
  if (flattened_string) {
    return LoadAsset<ImageAsset>(std::move(contents), asset_url);
  } else {
    return Future<absl::Cord>::Schedule(
               [contents = std::move(contents)]() mutable {
                 contents.Flatten();
                 return std::move(contents);
               },
               Executor::Type::kBackground)
        .Then([this, url = std::string(asset_url)](absl::Cord resource_cord) {
          return LoadAsset<ImageAsset>(std::move(resource_cord), url);
        });
  }
}

Future<AssetPtr<ImageBasedLightingAsset>> AssetManager::LoadImageBasedLighting(
    const AssetDefinition& asset_definition) {
  return LoadAsset<ImageBasedLightingAsset>(asset_definition);
}

Future<AssetPtr<ImageBasedLightingAsset>> AssetManager::LoadImageBasedLighting(
    absl::string_view asset_url) {
  return LoadAsset<ImageBasedLightingAsset>(asset_url);
}

float AssetManager::GetDownloadProgress(size_t download_baseline) {
  return resource_manager_.GetUrlLoaderProgress(download_baseline);
}

float AssetManager::GetDownloadProgress(std::string_view asset_url) {
  return resource_manager_.GetUrlLoaderProgress(asset_url);
}

size_t AssetManager::GetDownloadedSize() {
  return resource_manager_.GetUrlLoaderDownloadedSize();
}

void AssetManager::Cleanup() {
  for (auto& pair : caches_) {
    pair.second->Clear();
  }

  resource_manager_.Cleanup();
}

void AssetManager::ClearUnused() {
  IMP_TRACE();
  // Each frame, clears loaded assets if nothing is using them. We do this once
  // per-frame so that if something is dropped and then re-used within the same
  // frame we don't reload it.
  for (auto& pair : caches_) {
    pair.second->ClearUnused();
  }
}

void AssetManager::CancelLoad(const AssetDefinition& asset_definition) {
  CancelLoad(asset_definition.GetUrl());
}

void AssetManager::CancelLoad(absl::string_view asset_url) {
  for (auto& pair : caches_) {
    pair.second->CancelInProgressLoad(asset_url);
  }
}

int AssetManager::GetAssetCount() const {
  int total_count = 0;
  for (auto& pair : caches_) {
    total_count += pair.second->GetAssetCount();
  }
  return total_count;
}

Future<Resource> AssetManager::LoadResource(
    const AssetDefinition& asset_definition,
    std::optional<FutureGroup> future_group) {
  return resource_manager_.Load(asset_definition, future_group);
}

Future<resources::Resource> AssetManager::LoadResource(
    absl::string_view asset_url, std::optional<FutureGroup> future_group) {
  if (asset_url.empty()) {
    return Future<Resource>(absl::InternalError("Asset url is empty."));
  }
  return resource_manager_.Load(asset_url, future_group);
}

void AssetManager::SetUrlLoader(
    std::unique_ptr<resources::UrlLoader> url_loader) {
  resource_manager_.SetUrlLoader(std::move(url_loader));
}

void AssetManager::SetUrlLoaderConfig(resources::UrlLoader::Config config) {
  resource_manager_.SetUrlLoaderConfig(std::move(config));
}

std::unique_ptr<resources::UrlLoader> AssetManager::MoveUrlLoader() {
  return resource_manager_.MoveUrlLoader();
}

void AssetManager::SetSandboxedGltfLoader(
    std::unique_ptr<loader::LoaderCreator> sandboxed_gltf_loader_creator) {
  gltf_asset_loader_.SetSandboxedGltfLoader(
      std::move(sandboxed_gltf_loader_creator));
}

}  // namespace imp
