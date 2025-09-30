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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_CREATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_CREATOR_H_

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/animation/gltf_animation.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/image/image_contents.h"
#include "core/loader/creator/model_creator.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/material_package.h"
#include "core/model/model_data.h"
#include "core/view/base_view.h"

namespace imp::loader::details {

// Creates filament types from a loaded model.
class Creator {
 public:
  using LoadedModelAccess = FlatBufferAccess<schemas::LoadedModel>;

  explicit Creator(BaseView& view, MaterialPackage* material_package,
                   LoadedModelAccess&& access);

  ~Creator();

  // Instantiation.
  Future<std::unique_ptr<model::ModelData>> CreateModel(
      filament::Engine* engine, LoaderOptions loader_options = {},
      std::optional<absl::string_view> name = std::nullopt);
  absl::StatusOr<std::unique_ptr<animation::GltfAnimation>> CreateAnimation(
      absl::string_view name);
  absl::StatusOr<std::unique_ptr<animation::GltfAnimation>> CreateAnimation(
      size_t animation_index);
  Future<absl::Status> LoadImages(const imp::Context& context,
                                  std::function<void()> callback);

  // Queries.
  std::vector<absl::string_view> GetAnimationNames() const;

  bool IsFullyLoaded() const;
  absl::Status BlockUntilLoaded(filament::Engine* engine);
  void WhenFullyLoaded(std::function<void()>&& cb) const;

  // Removes the callback passes into WhenFullyLoaded or CreateModel.
  void RemoveWhenFullyLoadedCallback();

 private:
  BaseView& view_;
  MaterialPackage* material_package_;
  LoadedModelAccess access_;
  std::vector<std::unique_ptr<image::ImageContents>> images_;
  std::unique_ptr<ModelCreator> model_creator_;
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_CREATOR_H_
