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

#ifndef THIRD_PARTY_IMPRESS_CORE_LIGHTING_IMAGE_BASED_LIGHTING_ASSET_IBLPREFILTER_LOADER_H_
#define THIRD_PARTY_IMPRESS_CORE_LIGHTING_IMAGE_BASED_LIGHTING_ASSET_IBLPREFILTER_LOADER_H_

#include <memory>
#include <optional>

#include "core/lighting/image_based_lighting_asset.h"
#include "core/math/vec.h"

namespace imp {

// A CustomLoader for loading HDR images and convert them into
// ImageBasedLightingAsset.
// Internally, it decodes the image with filament ImageDecoder and then converts
// the texture into cubemaps (one for skybox and one for lighting) with
// IblPrefilter.
// Please note that ImageBasedLightingAsset loaded with this loader will not
// contain any spherical harmonics data.
class IblPrefilterLoader : public ImageBasedLightingAsset::CustomLoader {
 public:
  // Default output size is 256 x 256 if unspecified.
  explicit IblPrefilterLoader(
      std::optional<int2> output_texture_size = std::nullopt);

  Future<std::unique_ptr<ImageBasedLightingAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future) override;

 private:
  // Default output size is 256 x 256 if unspecified.
  std::optional<int2> output_texture_size_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_LIGHTING_IMAGE_BASED_LIGHTING_ASSET_IBLPREFILTER_LOADER_H_
