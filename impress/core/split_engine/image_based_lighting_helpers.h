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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_IMAGE_BASED_LIGHTING_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_IMAGE_BASED_LIGHTING_HELPERS_H_

#include <cstdint>

#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/lighting/image_based_lighting_types.h"
#include "split_engine/schemas/split_engine_data_generated.h"
namespace imp::split_engine {

// Packs an ImageBasedLightingAsset into its flatbuffer equivalent.
flatbuffers::Offset<android_xr::schemas::ImageBasedLightingAsset>
PackImageBasedLightingAsset(
    flatbuffers::FlatBufferBuilder* fbb, uint64_t id,
    const SphericalHarmonics& spherical_harmonics,
    const ImageBasedLightingAssetCubemapImages& cubemap_images);

// Unpacks a CubemapLevelImageContents from its flatbuffer equivalent.
absl::StatusOr<CubemapLevelImageContents> UnpackCubemapLevelImageContents(
    const android_xr::schemas::CubemapLevelImageContents*
        cubemap_level_image_contents);

}  // namespace imp::split_engine
#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_IMAGE_BASED_LIGHTING_HELPERS_H_
