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

#include "core/split_engine/image_based_lighting_helpers.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/filament/include/filament/Texture.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "core/image/image_contents.h"
#include "core/lighting/image_based_lighting_types.h"
#include "core/math/vec.h"
#include "split_engine/schemas/split_engine_data_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

namespace {
// Packs a CubemapLevelImageContents into its flatbuffer equivalent.
flatbuffers::Offset<android_xr::schemas::CubemapLevelImageContents>
PackCubemapImageLevelContents(
    flatbuffers::FlatBufferBuilder& fbb,
    const CubemapLevelImageContents& cubemap_level_image_contents) {
  filament::Texture::FaceOffsets face_offsets =
      cubemap_level_image_contents.cubemap_level.face_offsets;
  flatbuffers::Offset<android_xr::schemas::FaceOffsets> face_offset_offset =
      android_xr::schemas::CreateFaceOffsets(
          fbb, face_offsets.px, face_offsets.nx, face_offsets.py,
          face_offsets.ny, face_offsets.pz, face_offsets.nz);
  flatbuffers::Offset<android_xr::schemas::CubemapLevel> cubemap_level_offset =
      android_xr::schemas::CreateCubemapLevel(
          fbb, face_offset_offset,
          cubemap_level_image_contents.cubemap_level.face_size);
  flatbuffers::Offset<android_xr::schemas::ImageContents>
      stitched_face_image_offset = android_xr::schemas::CreateImageContents(
          fbb, cubemap_level_image_contents.stitched_face_image->GetWidth(),
          cubemap_level_image_contents.stitched_face_image->GetHeight(),
          fbb.CreateVector(
              cubemap_level_image_contents.stitched_face_image->GetData(),
              cubemap_level_image_contents.stitched_face_image->GetSize()));
  return android_xr::schemas::CreateCubemapLevelImageContents(
      fbb, cubemap_level_offset, stitched_face_image_offset);
}
}  // namespace

flatbuffers::Offset<android_xr::schemas::ImageBasedLightingAsset>
PackImageBasedLightingAsset(
    flatbuffers::FlatBufferBuilder* fbb, std::uint64_t id,
    const SphericalHarmonics& spherical_harmonics,
    const ImageBasedLightingAssetCubemapImages& cubemap_images) {
  // Pack IBL cubemap images.
  std::vector<
      flatbuffers::Offset<android_xr::schemas::CubemapLevelImageContents>>
      cubemap_images_offset;
  cubemap_images_offset.reserve(cubemap_images.ibl_cubemap_images.size());
  for (const CubemapLevelImageContents& ibl_cubemap_image :
       cubemap_images.ibl_cubemap_images) {
    cubemap_images_offset.push_back(
        PackCubemapImageLevelContents(*fbb, ibl_cubemap_image));
  }

  // Pack spherical harmonics.
  std::vector<android_xr::schemas::Float3> coefficients;
  coefficients.reserve(spherical_harmonics.coefficients.size());
  for (const float3& coefficient : spherical_harmonics.coefficients) {
    coefficients.push_back({coefficient.x, coefficient.y, coefficient.z});
  }
  flatbuffers::Offset<android_xr::schemas::SphericalHarmonics>
      spherical_harmonics_offset =
          android_xr::schemas::CreateSphericalHarmonics(
              *fbb, fbb->CreateVectorOfStructs(coefficients),
              spherical_harmonics.num_bands);

  // Pack skybox cubemap images.
  std::optional<
      flatbuffers::Offset<android_xr::schemas::CubemapLevelImageContents>>
      skybox_cubemap_images_offset = std::nullopt;
  if (cubemap_images.skybox_cubemap_images.has_value()) {
    skybox_cubemap_images_offset = PackCubemapImageLevelContents(
        *fbb, cubemap_images.skybox_cubemap_images.value());
  }
  flatbuffers::Offset<flatbuffers::Vector<
      flatbuffers::Offset<android_xr::schemas::CubemapLevelImageContents>>>
      vector = fbb->CreateVector(cubemap_images_offset);

  // Build ImageBasedLightingAsset.
  android_xr::schemas::ImageBasedLightingAssetBuilder ibl_asset_builder(*fbb);
  ibl_asset_builder.add_id(id);
  ibl_asset_builder.add_ibl_cubemap_level_image_contents(vector);
  if (skybox_cubemap_images_offset) {
    ibl_asset_builder.add_skybox_cubemap_level_image_contents(
        skybox_cubemap_images_offset.value());
  }
  ibl_asset_builder.add_spherical_harmonics(spherical_harmonics_offset);
  return ibl_asset_builder.Finish();
}

absl::StatusOr<CubemapLevelImageContents> UnpackCubemapLevelImageContents(
    const android_xr::schemas::CubemapLevelImageContents*
        cubemap_level_image_contents) {
  // Unpack CubemapLevel.
  if (!cubemap_level_image_contents->cubemap_level()) {
    return absl::InvalidArgumentError("cubemap_level was null.");
  }
  const android_xr::schemas::CubemapLevel* cubemap_level_schema =
      cubemap_level_image_contents->cubemap_level();
  if (!cubemap_level_schema->face_offsets()) {
    return absl::InvalidArgumentError("face_offsets was null.");
  }
  const android_xr::schemas::FaceOffsets* face_offsets_schema =
      cubemap_level_schema->face_offsets();
  filament::Texture::FaceOffsets face_offsets;
  face_offsets.nx = face_offsets_schema->nx();
  face_offsets.ny = face_offsets_schema->ny();
  face_offsets.nz = face_offsets_schema->nz();
  face_offsets.px = face_offsets_schema->px();
  face_offsets.py = face_offsets_schema->py();
  face_offsets.pz = face_offsets_schema->pz();
  const CubemapLevel cubemap_level{
      .face_offsets = face_offsets,
      .face_size = cubemap_level_schema->face_size()};

  // Unpack CubemapLevelImageContents.
  if (!cubemap_level_image_contents->stitched_face_image()) {
    return absl::InvalidArgumentError("stitched_face_image was null.");
  }
  const android_xr::schemas::ImageContents& stitched_face_image_schema =
      *cubemap_level_image_contents->stitched_face_image();
  if (!stitched_face_image_schema.memory()) {
    return absl::InvalidArgumentError("stitched_face_image memory was null.");
  }
  std::vector<uint8_t> memory(stitched_face_image_schema.memory()->begin(),
                              stitched_face_image_schema.memory()->end());

  MP_ASSIGN_OR_RETURN(std::unique_ptr<image::ImageContents> stitched_face_image,
                   image::ImageContents::CreatePreStitchedImage(
                       stitched_face_image_schema.width(),
                       stitched_face_image_schema.height(), std::move(memory)));

  return CubemapLevelImageContents{
      .cubemap_level = cubemap_level,
      .stitched_face_image = std::move(stitched_face_image)};
}

}  // namespace imp::split_engine
