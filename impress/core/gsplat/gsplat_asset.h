/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_GSPLAT_GSPLAT_ASSET_H_
#define THIRD_PARTY_IMPRESS_CORE_GSPLAT_GSPLAT_ASSET_H_

#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/gsplat/gsplat.pb.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp {
// Contains all the data needed to render a gsplat scene.
class GSplatAsset {
 public:
#if IMP_MATERIAL_API(VULKAN)
  // TODO: b//440182329 - 96-bit pixel formats are unsupported on Vulkan
  // currently.
  using Cov3dDataType = imp::uint4;
#else
  using Cov3dDataType = imp::uint3;
#endif

  enum class LoadFormat {
    // Unknown format. Will attempt to infer from the file extension.
    kUnknown,
    // .pb containing GSplatRequiredData proto.
    kRequiredData,
    // .pb containing GSplatPointCloudData proto.
    kFullData,
    // .ply file in the original uncompressed format from the 3DGS paper.
    kPly,
  };

  // Constructs the GSplatAsset and creates the data textures that will be used
  // to render the scene.
  // Note: this constructor takes ownership of the data provided.
  GSplatAsset(BaseView& view, float3& bounds_min, float3& bounds_max,
              std::vector<filament::math::half4>& positions,
              std::vector<Cov3dDataType>& cov3ds, std::vector<uint32_t>& colors,
              std::string url = "");

  // Load a Gsplat from a given ply or proto file located at the url specified.
  // Note: Use `format` to specify the file type. If `format` is kUnknown, this
  // function uses file extensions to identify what type of file is being
  // loaded, inference only supports .ply or .pb GSPlatPointCloudData protos.
  static Future<std::unique_ptr<GSplatAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future,
      LoadFormat format = LoadFormat::kUnknown);

  // Load a Gsplat from a given GSplatPointCloudData proto.
  static Future<std::unique_ptr<GSplatAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future,
      const GSplatPointCloudData& splat_data);

  // Get an impress RGBA16F texture that contains the position data for each
  // gaussian.
  BorrowedTexturePtr position_data_texture(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const {
    return position_data_texture_.Borrow(loc);
  };

  // Get an impress RG16F texture that contains the cov3d data for each
  // gaussian. There are 6 total Cov3d values per gaussian, so each gaussian
  // takes up 3 consecutive texels in the texture.
  BorrowedTexturePtr cov3d_data_texture(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const {
    return cov3d_data_texture_.Borrow(loc);
  };

  // Get an impress RGBA8 texture that contains the band 0 color data and
  // opacity for each gaussian.
  BorrowedTexturePtr color_data_texture(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const {
    return color_data_texture_.Borrow(loc);
  };

  int count() const { return count_; }
  std::shared_ptr<const std::vector<filament::math::half4>> positions() const {
    return positions_;
  }
  const float3& bounds_min() const { return bounds_min_; }
  const float3& bounds_max() const { return bounds_max_; }
  absl::string_view url() const { return url_; }

 private:
  // Helper function that creates a data texture containing data from the raw
  // gsplat data.
  void CreateDataTexture(BaseView& view, uint32_t texture_width,
                         uint32_t texture_height,
                         std::vector<Cov3dDataType>& cov3ds,
                         std::vector<uint32_t>& colors);

  // Number of 3d gaussians in the gsplat scene.
  int count_ = 0;
  // Position of each 3d gaussian. We use a shared_ptr to have thread safe
  // access to the position data during sorting, which happens on the background
  // thread.
  std::shared_ptr<std::vector<filament::math::half4>> positions_;
  // The min bounds of the center of the 3d gaussians of the gsplat scene.
  float3 bounds_min_ = float3(std::numeric_limits<float>::max());
  // The max bounds of the center of the 3d gaussians of the gsplat scene.
  float3 bounds_max_ = float3(std::numeric_limits<float>::lowest());
  // The url of the source data for this gsplat.
  std::string url_ = "";
  // The texture containing the position data needed to render the gsplat.
  OwnedTexturePtr position_data_texture_;
  // The texture containing the cov3d data needed to render the gsplat.
  OwnedTexturePtr cov3d_data_texture_;
  // The texture containing the color data needed to render the gsplat.
  OwnedTexturePtr color_data_texture_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GSPLAT_GSPLAT_ASSET_H_
