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

#ifndef THIRD_PARTY_IMPRESS_CORE_LIGHTING_IMAGE_BASED_LIGHTING_TYPES_H_
#define THIRD_PARTY_IMPRESS_CORE_LIGHTING_IMAGE_BASED_LIGHTING_TYPES_H_
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "filament/filament/include/filament/Texture.h"
#include "core/image/image_contents.h"
#include "core/math/vec.h"
namespace imp {

// Spherical harmonics coefficients and number of bands. Used to approximate
// radiance of an Image-Based Lighting (IBL) cubemap. See
// (broken link)
// for an explanation of how Spherical Harmonics are used to approximate IBL
// radiance.
struct SphericalHarmonics {
  std::vector<float3> coefficients;
  uint8_t num_bands;
};

// The dimensions and face offsets of a single mipmap level of a cubemap.
struct CubemapLevel {
  filament::Texture::FaceOffsets face_offsets;
  uint32_t face_size;
};

// The dimensions and face offsets of a single mipmap level of a cubemap, and
// the corresponding stitched image of the cubemap level.
struct CubemapLevelImageContents {
  CubemapLevel cubemap_level;
  std::unique_ptr<image::ImageContents> stitched_face_image;
};

// The cubemap images required to construct an ImageBasedLightingAsset.
struct ImageBasedLightingAssetCubemapImages {
  std::vector<CubemapLevelImageContents> ibl_cubemap_images;
  std::optional<CubemapLevelImageContents> skybox_cubemap_images;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_LIGHTING_IMAGE_BASED_LIGHTING_TYPES_H_
