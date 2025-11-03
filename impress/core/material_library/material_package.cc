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

#include "core/material_library/material_package.h"

#include <algorithm>
#include <cassert>
#include <iterator>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/assets/material/material_helpers.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/robin_map.h"
#include "core/common/zip_helpers.h"
#include "core/config.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
#include "mediapipe/framework/port/status_macros.h"

#if IMP_PLATFORM(ANDROID)
#include "core/common/registry.h"
#include "core/view/platforms/xr_android/xr_color_space_helper.h"
#endif

namespace imp {
namespace {
using BufferCache = RobinMap<GenericMaterialSpec, BufferAccess>;
using MaterialCache = MaterialPackage::MaterialCache;

constexpr absl::string_view kDepthClearMaterial = "depth_clear.cmat";

std::string GetConcreteMaterialName(GenericMaterialSpec spec) {
  if (spec.GetDepthClearMaterial() ==
      schemas::GenericMaterialDepthClearMaterial::Enabled) {
    return std::string(kDepthClearMaterial);
  }
  auto lighting_model_string_view = spec.DescribeLightingModel();
  auto blend_mode_string_view = spec.DescribeBlendMode();
  auto double_sided_mode_string_view = spec.DescribeDoubleSidedMode();
  return absl::StrFormat("%s_%s_%s.cmat", lighting_model_string_view,
                         blend_mode_string_view, double_sided_mode_string_view);
}

// Returns a map of the requested materials that are already loaded in the
// cache.  Removes all cache hits from the list of requests.
MaterialCache GetMaterialsFromCache(
    const MaterialCache& materials_by_params,
    absl::flat_hash_set<GenericMaterialSpec>& requested_materials) {
  MaterialCache result;
  result.reserve(requested_materials.size());
  auto request_itr = requested_materials.begin();
  while (request_itr != requested_materials.end()) {
    auto cache_itr = materials_by_params.find(*request_itr);
    if (cache_itr != materials_by_params.end()) {
      // The material is already loaded, add it to the result and remove it from
      // the list of names we need to load.
      result.emplace(*request_itr, cache_itr->second);
      requested_materials.erase(request_itr++);
    } else {
      // The material isn't loaded yet, leave it in the list of names we need to
      // load.
      request_itr++;
    }
  }
  return result;
}

filament::Material* LoadMaterial(
    BaseView& view, filament::Engine* engine, GenericMaterialSpec key,
    const BufferAccess& buffer,
    const MaterialPreCompileOptions& material_pre_compile_options) {
  filament::Material* material = filament::Material::Builder()
                                     .package(buffer.Data(), buffer.Size())
                                     .build(*engine);

  if (auto* serializer = view.GetSplitEngineSerializer()) {
    serializer->AddMaterial(material, buffer, material_pre_compile_options);
  }
  return material;
}

absl::StatusOr<BufferCache> GetMaterialBuffersFromZip(
    resources::Resource materials_zip_resource,
    const absl::flat_hash_set<GenericMaterialSpec> requests) {
  // First, unzip the buffers from the zip file for the requested
  // materials to load on a background thread.
  std::set<std::string> request_filenames;
  std::transform(requests.begin(), requests.end(),
                 std::inserter(request_filenames, request_filenames.end()),
                 [](const GenericMaterialSpec& request_key) {
                   return GetConcreteMaterialName(request_key);
                 });

  MP_ASSIGN_OR_RETURN(
      std::vector<ZipFile> zip_files,
      GetFilesFromZip(materials_zip_resource.GetData(), request_filenames));

  BufferCache buffer_by_params;
  for (auto request_params : requests) {
    auto zip_file_itr = std::find_if(
        zip_files.begin(), zip_files.end(),
        [&request_params](const ZipFile& zip_file) {
          return zip_file.filename == GetConcreteMaterialName(request_params);
        });

    
    buffer_by_params.emplace(request_params, std::move(zip_file_itr->access));
  }

  return buffer_by_params;
}

}  // namespace

MaterialPackage::MaterialPackage(
    Future<resources::Resource> materials_zip,
    std::optional<MaterialPreCompileOptions> material_pre_compile_options)
    : materials_zip_(materials_zip),
      material_pre_compile_options_(material_pre_compile_options) {}

MaterialPackage::~MaterialPackage() {
  // Make sure to destroy all the loaded materials.
  for (auto& pair : cached_material_by_params_) {
    engine_->destroy(pair.second);
  }
  cached_material_by_params_.clear();
}

Future<MaterialCache> MaterialPackage::GetOrLoadMaterials(
    BaseView& view, filament::Engine* engine,
    absl::flat_hash_set<GenericMaterialSpec> requested_materials) {
  MaterialCache cache_hits;

  // Lazily assign the engine.
  // This is done here instead of the constructor because the engine isn't
  // available when the MaterialPackage is created in
  // loader_in_process_embedded.
  if (!engine_) {
    engine_ = engine;
  } else {
    assert(engine_ == engine);
  }
  assert(engine_);

  // Check requests against cache.  If there is a cache hit, copy it to results
  // and remove it from requests.
  cache_hits =
      GetMaterialsFromCache(cached_material_by_params_, requested_materials);

  // All requested materials are already loaded, return the result.
  if (requested_materials.empty()) {
    return Future<MaterialCache>(std::move(cache_hits));
  }

  // In Split Engine Mode, we don't need to load the materials. They are loaded
  // on the backend renderer side instead.
  if (view.GetSplitEngineSerializer()) {
    return Future<MaterialCache>(std::move(cache_hits));
  }

#if IMP_PLATFORM(ANDROID)
  // Add color space to the pre-compile options.
  auto color_space_helper = view.GetRegistry().Get<XrColorSpaceHelper>();
  if (color_space_helper.ok()) {
    if (!material_pre_compile_options_) {
      material_pre_compile_options_.emplace();
    }
    material_pre_compile_options_->constants.push_back(
        XrColorSpaceHelper::GetColorSpacePrecompileConstant());
  }
#endif

  // Attempt to load any requested materials from the zip file that haven't
  // already been loaded.
  return materials_zip_
      .Then(
          [request_keys = std::move(requested_materials)](
              resources::Resource materials_zip_resource)
              -> absl::StatusOr<BufferCache> {
            return GetMaterialBuffersFromZip(materials_zip_resource,
                                             request_keys);
          },
          {.executor = Executor::Type::kBackground})
      .Then(
          [this, &view, result = std::move(cache_hits)](
              const BufferCache& buffer_by_params) mutable
              -> Future<MaterialCache> {
            Future<absl::Status> pre_compile_future =
                Future<absl::Status>(absl::OkStatus());
            // If parallel shader compilation is disabled, materials can't be
            // pre-compiled in a separate thread, so there's no purpose in doing
            // so.
            bool should_pre_compile =
                !view.GetEngineConfig().disableParallelShaderCompile;

            // Then, on the foreground thread actually load the requested
            // materials into filament.
            std::vector<Future<absl::Status>> pre_compile_futures;
            for (auto& key_and_buffer : buffer_by_params) {
              // Look up the material in the cache again, in case it was already
              // loaded in-between when the cache was checked and the buffers
              // were loaded.
              auto cached_material_itr =
                  cached_material_by_params_.find(key_and_buffer.first);
              if (cached_material_itr != cached_material_by_params_.end()) {
                // The material is already loaded, add it to the result and
                // move on to the next material.
                result.emplace(key_and_buffer.first,
                               cached_material_itr->second);
                continue;
              }

              filament::Material* loaded_material = LoadMaterial(
                  view, engine_, key_and_buffer.first, key_and_buffer.second,
                  material_pre_compile_options_.value_or(
                      MaterialPreCompileOptions()));

              // return any loaded material
              result.emplace(key_and_buffer.first, loaded_material);

              // cache the materials for reuse.
              cached_material_by_params_.emplace(key_and_buffer.first,
                                                 loaded_material);
              bool is_supported = engine_->getActiveFeatureLevel() >=
                                  loaded_material->getFeatureLevel();
              if (material_pre_compile_options_ && should_pre_compile &&
                  is_supported) {
                pre_compile_futures.push_back(
                    material_helpers::PreCompileMaterial(
                        loaded_material, *material_pre_compile_options_));
              }
            }

            return Future<absl::Status>::CombineList(
                       std::move(pre_compile_futures))
                .Then([result = std::move(result)]() mutable -> MaterialCache {
                  return result;
                });
          });
}

Future<resources::Resource> MaterialPackage::GetMaterialsZipFuture() {
  return materials_zip_;
}

}  // namespace imp
