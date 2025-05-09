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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_USDZ_CONSUMER_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_USDZ_CONSUMER_H_

#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/robin_map.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/loader/provider/usdz/mesh_material.h"
#include "third_party/tinyusdz/src/prim-types.hh"
#include "third_party/tinyusdz/src/stage.hh"

namespace imp::loader::details::provider_usdz {

class UsdzConsumer : protected imp::loader::details::LoadedModelBuilder {
 public:
  UsdzConsumer(const tinyusdz::Stage &stage,
               RobinMap<std::string, BufferAccess> &&encoded_image_from_path)
      : stage_(stage),
        encoded_image_from_path_(std::move(encoded_image_from_path)) {}
  ~UsdzConsumer() = default;

  // Attempts to build and return a serialized flatbuffer object containing the
  // transcoded contents of the USDZ file.
  absl::StatusOr<FlatBufferAccess<schemas::LoadedModel>> BuildLoadedModel();

 protected:
  absl::StatusOr<MaterialId> GetMaterial(const tinyusdz::Path &material_path);

 private:
  absl::StatusOr<SamplerId> EnsureSampler(
      const SurfaceTexture &surface_texture);
  absl::StatusOr<TextureId> EnsureTexture(const SurfaceTexture &surface_texture,
                                          bool expect_srgb);

  absl::Status ExtractEntity(const tinyusdz::Prim &prim, BoneId bone);
  absl::Status ExtractMaterial(const tinyusdz::Prim &prim,
                               uint16_t material_index);

  absl::Status AddPointSampler();

  absl::Status CollectResources(const uint8_t *addr, size_t length);

  const tinyusdz::Stage &stage_;
  RobinMap<std::string, MaterialId> material_from_path_;
  RobinMap<std::string, BufferAccess> encoded_image_from_path_;
  uint16_t next_sampler_id_ = 0;
  uint16_t next_texture_id_ = 0;
  uint16_t point_sampler_lookup_index_ = 0;
};

}  // namespace imp::loader::details::provider_usdz

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_USDZ_USDZ_CONSUMER_H_
