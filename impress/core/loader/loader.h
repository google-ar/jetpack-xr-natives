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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/animation/gltf_animation.h"
#include "core/async/future.h"
#include "core/async/future_group.h"
#include "core/common/buffer_access.h"
#include "core/loader/loader_options.h"
#include "core/model/model_data.h"

namespace imp::loader {

// Wrapper class for a loading
class Loader {
 public:
  virtual ~Loader() = default;

  // Add the contents of a requested resource.  'path' is a context-free (i.e.
  // not asset-relative) path returned as the out-param of TryLoad.  Passing in
  // other resources will return an error.
  virtual absl::Status AddMissingResource(absl::string_view path,
                                          BufferAccess&& access) = 0;

  // Add the contents of a resource via asset-relative path (i.e. "./foo.bin").
  // This method can be called before TryLoad().
  virtual absl::Status AddResource(absl::string_view path,
                                   BufferAccess&& access) = 0;

  // Client check to see if the asset is loaded.
  virtual bool Loaded() const = 0;
  // Fire-and-forget load mechanism.  Will fail to resolve missing resources in
  // contexts without filesystem access.
  virtual bool IsFullyLoaded() const = 0;
  // Block until a loader's resources are fully processed
  virtual absl::Status Flush(filament::Engine* engine) = 0;

  // The `callback` parameter is not actually a callback, but should rather be
  // thought of as a Holdable<LoaderInProgress>. If the future chain in
  // gltf_asset_loader is cancelled, the load_in_progress object gets
  // potentially destroyed before the image loading can finish, causing a crash.
  // By retaining the object inside the passed in callback, the image loading
  // future can manage the lifetime of the object until it is finished. See the
  // following for additional information.
  // (broken link)
  virtual Future<absl::Status> Load(
      std::function<void()>&& callback,
      std::optional<FutureGroup> future_group = std::nullopt) = 0;
  // Iterative load mechanism.  Will attempt to load given the currently loaded
  // resources; any missing assets which are required to load will have their
  // paths appear in out_missing_resource_paths.
  virtual Future<absl::Status> TryLoad(
      std::vector<std::string>* out_missing_resource_paths,
      std::function<void()>&& callback,
      std::optional<FutureGroup> future_group = std::nullopt) = 0;

  // Instantiation.
  virtual Future<std::unique_ptr<model::ModelData>> CreateModel(
      filament::Engine* engine, std::optional<FutureGroup> future_group) = 0;
  virtual Future<std::unique_ptr<model::ModelData>> CreateModel(
      filament::Engine* engine, std::function<void()>&& callback,
      std::optional<FutureGroup> future_group) = 0;
  Future<std::unique_ptr<model::ModelData>> CreateModel(
      filament::Engine* engine);
  Future<std::unique_ptr<model::ModelData>> CreateModel(
      filament::Engine* engine, std::function<void()>&& callback);

  virtual void WhenFullyLoaded(std::function<void()>&& callback) = 0;
  virtual void RemoveWhenFullyLoadedCallback() = 0;

  virtual absl::StatusOr<std::unique_ptr<animation::GltfAnimation>>
  CreateAnimation(size_t animation_index) = 0;
  virtual absl::StatusOr<std::unique_ptr<animation::GltfAnimation>>
  CreateAnimation(absl::string_view name) = 0;

  // Queries.
  virtual std::vector<absl::string_view> GetAnimationNames() const = 0;
  virtual absl::string_view GetName() const = 0;

  // Queries the host's filament engine for texture format availability, so that
  // the loader can know which compressed formats it can transcode into.
  static LoaderOptions::TextureTranscodeCompressionType
  GetTextureTranscodeCompressionType(filament::Engine& engine);
};

}  // namespace imp::loader

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_LOADER_H_
