// Copyright 2025 Google LLC
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

#include "core/gsplat/gsplat_asset.h"

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/libs/math/include/math/TMatHelpers.h"
#include "filament/libs/math/include/math/vec3.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/config.h"
#include "core/gsplat/conversion.h"
#include "core/gsplat/encoding.h"
#include "core/gsplat/gsplat.pb.h"
#include "core/image/borrowed_image_content.h"
#include "core/image/owned_image_content.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_options.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp {
namespace {
using ::filament::backend::PixelDataFormat;
using ::filament::backend::TextureFormat;
using imp::GSplatPointCloudData;
using imp::Range3D;

#if IMP_MATERIAL_API(VULKAN)
// TODO: b//440182329 - RGB32UI is unsupported on Vulkan currently. Remove this
// and use RGB32UI once the bug is fixed.
constexpr TextureFormat kCov3dsTexture = TextureFormat::RGBA32UI;
constexpr PixelDataFormat kCov3dsPixelData = PixelDataFormat::RGBA_INTEGER;
#else
constexpr TextureFormat kCov3dsTexture = TextureFormat::RGB32UI;
constexpr PixelDataFormat kCov3dsPixelData = PixelDataFormat::RGB_INTEGER;
#endif

constexpr absl::string_view kGSplatPbFormat = "pb";
constexpr absl::string_view kGSplatPlyFormat = "ply";

// An impress friendly range struct containing the min and max.
struct RangeFloat3 {
  float3 min;
  float3 max;
};

RangeFloat3 GetRangeMinMax(const Range3D& range) {
  return {
      {range.range_x().min(), range.range_y().min(), range.range_z().min()},
      {range.range_x().max(), range.range_y().max(), range.range_z().max()}};
}

// Compute 3d covariance matrix, from eq(6) in 3DGS paper
// (broken link)
mat3 ComputeCov3d(const float4& q, const float3& s) {
  // clang-format off
  mat3 rotation_matrix = mat3(
      // Column 1
      1.0 - 2.0 * (q.y * q.y + q.z * q.z),
      2.0 * (q.x * q.y - q.w * q.z),
      2.0 * (q.x * q.z + q.w * q.y),
      // Column 2
      2.0 * (q.x * q.y + q.w * q.z),
      1.0 - 2.0 * (q.x * q.x + q.z * q.z),
      2.0 * (q.y * q.z - q.w * q.x),
      // Column 3
      2.0 * (q.x * q.z - q.w * q.y),
      2.0 * (q.y * q.z + q.w * q.x),
      1.0 - 2.0 * (q.x * q.x + q.y * q.y)
  );
  // clang-format on
  mat3 scale_matrix = mat3(s.x, 0, 0, 0, s.y, 0, 0, 0, s.z);
  mat3 cov3d = scale_matrix * rotation_matrix;
  return transpose(cov3d) * cov3d;
}

struct ProcessedData {
  static absl::StatusOr<ProcessedData> Process(
      const GSplatPointCloudData& gsplat_data,
      size_t max_gsplat_count = std::numeric_limits<size_t>::max()) {
    // Get the total number of gaussians.
    int count = 0;
    for (const auto& chunk : gsplat_data.required_data().chunk_data()) {
      count += chunk.positions_data_size();
    }
    if (count > max_gsplat_count) {
      return absl::InternalError(absl::StrCat(
          "GSplat scene has ", count,
          " points, which exceeds max gsplat count of ", max_gsplat_count));
    }

    ProcessedData processed_data;
    processed_data.positions.reserve(count);
    processed_data.cov3ds.reserve(count);
    processed_data.colors.reserve(count);
    for (const auto& chunk : gsplat_data.required_data().chunk_data()) {
      RangeFloat3 position_range = GetRangeMinMax(chunk.position_range());
      RangeFloat3 scale_range = GetRangeMinMax(chunk.scale_range());
      for (int i = 0; i < chunk.positions_data_size(); ++i) {
        const float3 position = lerp(position_range.min, position_range.max,
                                     Norm11ToFloat3(chunk.positions_data(i)));
        processed_data.positions.emplace_back(float4(position, 1.0f));
        // Update min and max bounds
        processed_data.bounds_min = min(processed_data.bounds_min, position);
        processed_data.bounds_max = max(processed_data.bounds_max, position);

        processed_data.colors.emplace_back(chunk.colors_data(i));

        const float3 scale = lerp(scale_range.min, scale_range.max,
                                  Norm11ToFloat3(chunk.scales_data(i)));
        const quatf rotation = Smallest3Norm10ToQuat(chunk.rotations_data(i));
        const mat3 cov3d = ComputeCov3d(rotation.xyzw, scale);
        GSplatAsset::Cov3dDataType cov3d_data;
        cov3d_data.x = PackHalf2x16(cov3d[0][0], cov3d[0][1]);
        cov3d_data.y = PackHalf2x16(cov3d[0][2], cov3d[1][1]);
        cov3d_data.z = PackHalf2x16(cov3d[1][2], cov3d[2][2]);
        processed_data.cov3ds.emplace_back(cov3d_data);
      }
    }
    return processed_data;
  }

  static absl::StatusOr<ProcessedData> Process(
      const GSplatRuntimeData& runtime_data,
      size_t max_gsplat_count = std::numeric_limits<size_t>::max()) {
    int count = runtime_data.positions.size();
    if (count > max_gsplat_count) {
      return absl::InternalError(absl::StrCat(
          "GSplat scene has ", count,
          " points, which exceeds max gsplat count of ", max_gsplat_count));
    }

    ProcessedData processed_data;
    processed_data.positions.reserve(count);
    processed_data.cov3ds.reserve(count);
    processed_data.colors.reserve(count);
    for (int x = 0; x < count; ++x) {
      const float3 position = runtime_data.positions[x];
      processed_data.positions.emplace_back(float4(position, 1.0f));
      // Update min and max bounds
      processed_data.bounds_min = min(processed_data.bounds_min, position);
      processed_data.bounds_max = max(processed_data.bounds_max, position);

      processed_data.colors.emplace_back(Float4ToNorm8(runtime_data.colors[x]));

      const float3 scale = runtime_data.scales[x];
      const quatf rotation = runtime_data.rotations[x];
      const mat3 cov3d = ComputeCov3d(rotation.xyzw, scale);
      GSplatAsset::Cov3dDataType cov3d_data;
      cov3d_data.x = PackHalf2x16(cov3d[0][0], cov3d[0][1]);
      cov3d_data.y = PackHalf2x16(cov3d[0][2], cov3d[1][1]);
      cov3d_data.z = PackHalf2x16(cov3d[1][2], cov3d[2][2]);
      processed_data.cov3ds.emplace_back(cov3d_data);
    }
    return processed_data;
  }

  float3 bounds_min = float3(std::numeric_limits<float>::max());
  float3 bounds_max = float3(std::numeric_limits<float>::lowest());
  std::vector<filament::math::half4> positions;
  std::vector<GSplatAsset::Cov3dDataType> cov3ds;
  std::vector<uint32_t> colors;
};

GSplatAsset::LoadFormat InferFormat(absl::string_view asset_url) {
  if (asset_url.ends_with(kGSplatPbFormat)) {
    return GSplatAsset::LoadFormat::kFullData;
  }
  if (asset_url.ends_with(kGSplatPlyFormat)) {
    return GSplatAsset::LoadFormat::kPly;
  }
  return GSplatAsset::LoadFormat::kUnknown;
}

}  // namespace

GSplatAsset::GSplatAsset(BaseView& view, float3& bounds_min, float3& bounds_max,
                         std::vector<filament::math::half4>& positions,
                         std::vector<GSplatAsset::Cov3dDataType>& cov3ds,
                         std::vector<uint32_t>& colors, std::string url)
    : count_(static_cast<int>(positions.size())),
      positions_(std::make_shared<std::vector<filament::math::half4>>(
          std::move(positions))),
      bounds_min_(bounds_min),
      bounds_max_(bounds_max),
      url_(std::move(url)) {
  const uint2 texture_size = CalculateTextureSize(count_);
  const uint32_t pixel_count = texture_size.x * texture_size.y;
  //  Before creating textures, fill these vectors to the reserved size.
  //  This must include any extra pixels introduced when rounding
  //  the texture width to a power of 2. This is vital because accessing
  //  vectors out of their bounds leads to undefined behavior, even if memory
  //  has been reserved beforehand.
  positions_->resize(pixel_count, 0);
  cov3ds.resize(pixel_count, 0);
  colors.resize(pixel_count, 0);
  CreateDataTexture(view, texture_size.x, texture_size.y, cov3ds, colors);
}

Future<std::unique_ptr<GSplatAsset>> GSplatAsset::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future, LoadFormat format) {
  const size_t max_texture_size = filament::Texture::getMaxTextureSize(
      *view->GetSharedEngine(), filament::Texture::Sampler::SAMPLER_2D);
  const size_t max_gsplat_count = max_texture_size * max_texture_size;

  return resource_future
      .Then(
          [asset_url = std::string(asset_url), max_gsplat_count,
           format](resources::Resource resource) {
            absl::string_view resource_data = resource.GetData().StringView();
            const LoadFormat current_format = format == LoadFormat::kUnknown
                                                  ? InferFormat(asset_url)
                                                  : format;

            switch (current_format) {
              case LoadFormat::kFullData: {
                GSplatPointCloudData gsplat_data;
                if (!gsplat_data.ParseFromString(resource_data)) {
                  return Future<ProcessedData>(absl::InternalError(
                      "Failed to parse to GSplatPointCloudData."));
                }
                return Future<ProcessedData>(
                    ProcessedData::Process(gsplat_data, max_gsplat_count));
              }
              case LoadFormat::kRequiredData: {
                GSplatRequiredData required_data;
                if (!required_data.ParseFromString(resource_data)) {
                  return Future<ProcessedData>(absl::InternalError(
                      "Failed to parse to GSplatRequiredData."));
                }
                GSplatPointCloudData gsplat_data;
                *gsplat_data.mutable_required_data() = std::move(required_data);
                return Future<ProcessedData>(
                    ProcessedData::Process(gsplat_data, max_gsplat_count));
              }
              case LoadFormat::kPly: {
                absl::StatusOr<GSplatRuntimeData> runtime_data =
                    GSplatPlyToRuntimeData(resource_data,
                                           /*should_load_optional_data=*/false);
                if (!runtime_data.ok()) {
                  return Future<ProcessedData>(runtime_data.status());
                }
                return Future<ProcessedData>(
                    ProcessedData::Process(*runtime_data, max_gsplat_count));
              }
              default:
                return Future<ProcessedData>(
                    absl::InternalError("Unknown gsplat format."));
            }
          },
          Executor::Type::kBackground)
      .Then([view, asset_url = std::string(asset_url)](
                absl::StatusOr<ProcessedData> constructor_data) {
        if (!constructor_data.ok()) {
          return Future<std::unique_ptr<GSplatAsset>>(
              constructor_data.status());
        }
        return Future<std::unique_ptr<GSplatAsset>>(
            std::make_unique<GSplatAsset>(
                *view, constructor_data->bounds_min,
                constructor_data->bounds_max, constructor_data->positions,
                constructor_data->cov3ds, constructor_data->colors, asset_url));
      });
};

Future<std::unique_ptr<GSplatAsset>> GSplatAsset::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future,
    const GSplatPointCloudData& gsplat_data) {
  const size_t max_texture_size = filament::Texture::getMaxTextureSize(
      *view->GetSharedEngine(), filament::Texture::Sampler::SAMPLER_2D);
  const size_t max_gsplat_count = max_texture_size * max_texture_size;

  return Future<ProcessedData>::Schedule(
             [gsplat_data, max_gsplat_count]() {
               return Future<ProcessedData>(
                   ProcessedData::Process(gsplat_data, max_gsplat_count));
             },
             Executor::Type::kBackground)
      .Then([view, asset_url =
                       std::string(asset_url)](ProcessedData constructor_data) {
        return std::make_unique<GSplatAsset>(
            *view, constructor_data.bounds_min, constructor_data.bounds_max,
            constructor_data.positions, constructor_data.cov3ds,
            constructor_data.colors, std::string(asset_url));
      });
};

void GSplatAsset::CreateDataTexture(
    BaseView& view, uint32_t texture_width, uint32_t texture_height,
    std::vector<GSplatAsset::Cov3dDataType>& cov3ds,
    std::vector<uint32_t>& colors) {
  const TextureSamplerOptions texture_sampler_options = {
      .mag_filter = TextureSamplerOptions::MagFilter::NEAREST,
      .min_filter = TextureSamplerOptions::MinFilter::NEAREST};

  image::BorrowedImageContents<filament::math::half4> position_image_content(
      texture_width, texture_height, *positions_,
      filament::backend::TextureFormat::RGBA16F,
      filament::backend::PixelDataFormat::RGBA,
      filament::backend::PixelDataType::HALF, [positions = positions_]() {});

  position_data_texture_ = view.GetTextureFactory().CreateTexture(
      position_image_content, TextureGenerationOptions(),
      texture_sampler_options, "position_texture");

  std::unique_ptr<std::vector<GSplatAsset::Cov3dDataType>> cov3ds_data =
      std::make_unique<std::vector<GSplatAsset::Cov3dDataType>>(
          std::move(cov3ds));
  image::OwnedImageContents<GSplatAsset::Cov3dDataType> cov3d_image_content(
      texture_width, texture_height, std::move(cov3ds_data), kCov3dsTexture,
      kCov3dsPixelData, filament::backend::PixelDataType::UINT);

  cov3d_data_texture_ = view.GetTextureFactory().CreateTexture(
      cov3d_image_content, TextureGenerationOptions(), texture_sampler_options,
      "cov3d_texture");

  std::unique_ptr<std::vector<uint32_t>> colors_data =
      std::make_unique<std::vector<uint32_t>>(std::move(colors));
  image::OwnedImageContents<uint32_t> color_image_content(
      texture_width, texture_height, std::move(colors_data),
      filament::backend::TextureFormat::RGBA8,
      filament::backend::PixelDataFormat::RGBA,
      filament::backend::PixelDataType::UBYTE);

  color_data_texture_ = view.GetTextureFactory().CreateTexture(
      color_image_content, TextureGenerationOptions(), texture_sampler_options,
      "color_texture");
}
}  // namespace imp
