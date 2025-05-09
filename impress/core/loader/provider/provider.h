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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_PROVIDER_H_

#include <memory>
#include <string>
#include <vector>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/details/gltf_provider.h"
#include "core/loader/provider/details/usdz_provider.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"

namespace imp::loader {
namespace details {
// Forward declarations.
struct LoadedGltf;
struct LoaderState;
struct LoadedSfb;
struct LoadedZip;
struct ParsedGltf;

// Used to access private state in tests.
class ProviderGltfTestHelpers;
class ProviderTestAccess;

}  // namespace details

// Wrapper class for the parser phase of loading
class Provider {
 public:
  // Tests and tools may use whatever GltfProvder they like, but BUILD rules for
  // production usage pack in a single implementation, provided by this method.
  static std::unique_ptr<details::GltfProvider> CreateDefaultGltfProvider();

  static std::unique_ptr<details::UsdzProvider> CreateDefaultUsdzProvider();

  // Loader is created via factory function.
  static absl::StatusOr<std::unique_ptr<Provider>> Create(
      absl::string_view path, BufferAccess&& access, LoaderOptions options,
      std::unique_ptr<details::GltfProvider> gltf_provider = nullptr,
      std::unique_ptr<details::UsdzProvider> usdz_provider = nullptr);
  ~Provider();

  // Add the contents of a requested resource.  'path' is a context-free (i.e.
  // not asset-relative) path returned as the out-param of TryLoad.  Passing in
  // other resources will return an error.
  OptionalError AddMissingResource(absl::string_view path,
                                   BufferAccess&& access);

  // Add the contents of a resource via asset-relative path (i.e. "./foo.bin").
  // This method can be called before TryLoad().
  void AddResource(absl::string_view name, BufferAccess&& access);

  // Client check to see if the asset is loaded.
  bool Loaded() const;
  // Fire-and-forget load mechanism.  Will fail to resolve missing resources in
  // contexts without filesystem access.
  OptionalError Load();
  // Iterative load mechanism.  Will attempt to load given the currently loaded
  // resources; any missing assets which are required to load will have their
  // paths appear in out_missing_resource_paths.  If load is complete,
  // out_completed will be set to true, otherwise false.
  OptionalError TryLoad(std::vector<std::string>* out_missing_resource_paths,
                        bool* out_completed);

  // Gets the results of the load.
  OptionalError GetLoadedModel(
      FlatBufferAccess<schemas::LoadedModel>* out_loaded_model);

  const details::provider_gltf::ParsedGltf* GetParsedGltf() const;

 private:
  Provider(absl::string_view directory, absl::string_view basename,
           absl::string_view extension, BufferAccess&& primary_resource,
           LoaderOptions options,
           std::unique_ptr<details::GltfProvider> gltf_provider,
           std::unique_ptr<details::UsdzProvider> usdz_provider);

  std::unique_ptr<details::LoaderState> state_;
  std::unique_ptr<details::GltfProvider> gltf_provider_;
  std::unique_ptr<details::UsdzProvider> usdz_provider_;
  std::unique_ptr<details::LoadedZip> loaded_zip_;
  std::optional<FlatBufferAccess<schemas::LoadedModel>> loaded_model_;

  // Test access.
  friend class details::ProviderGltfTestHelpers;
  friend class details::ProviderTestAccess;
};

}  // namespace imp::loader

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_PROVIDER_H_
