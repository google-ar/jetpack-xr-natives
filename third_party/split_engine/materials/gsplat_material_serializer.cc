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

#include "split_engine/materials/gsplat_material_serializer.h"

#include <sys/stat.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>

#include "absl/base/no_destructor.h"
#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/owned_ptr.h"
#include "core/gsplat/gsplat_asset.h"
#include "core/gsplat/gsplat_material_params.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/node.h"
#include "core/render/texture.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"
#include "split_engine/schemas/split_engine_render_passes_generated.h"

namespace android_xr {
namespace {

flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
ToTextureParameter(flatbuffers::FlatBufferBuilder& fbb,
                   imp::split_engine::BuiltInTextureParameterCreator&
                       texture_parameter_creator,
                   const imp::OwnedOrBorrowedTexturePtr& texture) {
  if (!texture) {
    return flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>();
  }
  return texture_parameter_creator.Create(fbb, texture.Borrow());
}

flatbuffers::Offset<android_xr::schemas::ProjectionQuad> ToProjectionQuadSchema(
    flatbuffers::FlatBufferBuilder& fbb,
    const std::optional<imp::TexturePipelineRendererProjectionQuad>&
        projection_quad) {
  if (projection_quad.has_value()) {
    android_xr::schemas::Float2 size =
        imp::split_engine::Pack(projection_quad->size);
    android_xr::schemas::Float3 center =
        imp::split_engine::Pack(projection_quad->center);
    android_xr::schemas::Quatf rotation =
        imp::split_engine::Pack(projection_quad->rotation);

    return android_xr::schemas::CreateProjectionQuad(fbb, &size, &center,
                                                     &rotation);
  } else {
    return {};
  }
}

bool IsParameterSupported(absl::string_view parameter_name) {
  static const absl::NoDestructor<absl::flat_hash_set<absl::string_view>>
      kValues({
          imp::kColorDataTextureParameter,
          imp::kCov3dDataTextureParameter,
          imp::kMagicWindowFromUserWorldMatrixParameter,
          imp::kMagicWindowOffscreenResolutionParameter,
          imp::kMainViewResolutionParameter,
          imp::kMaxScreenSizeParameter,
          imp::kMinScreenSizeParameter,
          imp::kOpacityScaleParameter,
          imp::kPositionDataTextureParameter,
          imp::kSortedIndicesParameter,
          imp::kSplatDataPrecomputedParameter,
          imp::kSplatScaleParameter,
          imp::kWindowDimensionInMagicWindowParameter,
      });
  return kValues->contains(parameter_name);
}

void UnsupportedParameterError(absl::string_view parameter_name) {
  IMP_LOG(imp::DFATAL) << (IsParameterSupported(parameter_name)
                      ? "Type mismatch for parameter: "
                      : "No such parameter: ")
              << parameter_name;
}

bool TextureSamplersAreSame(const filament::TextureSampler& lhs,
                            const filament::TextureSampler& rhs) {
  return lhs.getSamplerParams() == rhs.getSamplerParams() &&
         lhs.getWrapModeS() == rhs.getWrapModeS() &&
         lhs.getWrapModeT() == rhs.getWrapModeT() &&
         lhs.getWrapModeR() == rhs.getWrapModeR() &&
         lhs.getAnisotropy() == rhs.getAnisotropy() &&
         lhs.getCompareMode() == rhs.getCompareMode() &&
         lhs.getCompareFunc() == rhs.getCompareFunc();
}
}  // namespace

imp::Future<imp::OwnedPtr<GsplatMaterialSerializer>>
GsplatMaterialSerializer::Create(
    imp::NodeHandle gsplat_node, imp::AssetPtr<imp::GSplatAsset> gsplat_asset,
    android_xr::schemas::GsplatMode material_mode,
    bool use_triangles_for_splats,
    imp::BorrowedTexturePtr precomputed_data_texture,
    absl::string_view render_group,
    std::optional<imp::uint2> magic_window_offscreen_resolution) {
  imp::BaseView& view = gsplat_node->GetView();
  uint32_t gsplat_renderer_entity_id = gsplat_node.GetEntity().getId();
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  auto use_triangles_bool = android_xr::schemas::Bool(use_triangles_for_splats);
  auto has_precomputed_texture_bool =
      android_xr::schemas::Bool(precomputed_data_texture != nullptr);

  std::optional<android_xr::schemas::Uint2>
      magic_window_offscreen_texture_resolution =
          magic_window_offscreen_resolution.has_value()
              ? std::make_optional(
                    imp::split_engine::Pack(*magic_window_offscreen_resolution))
              : std::nullopt;
  flatbuffers::Offset<flatbuffers::String> render_group_name_offset =
      !render_group.empty() ? fbb->CreateString(render_group)
                            : flatbuffers::Offset<flatbuffers::String>{};

  schemas::GsplatModeSpec mode_spec_type = schemas::GsplatModeSpec::NONE;
  flatbuffers::Offset<void> mode_spec_offset;

  if (material_mode == schemas::GsplatMode::GSPLAT) {
    mode_spec_type = schemas::GsplatModeSpec::GsplatSpec;
    mode_spec_offset =
        schemas::CreateGsplatSpec(*fbb, render_group_name_offset).Union();
  } else if (material_mode == schemas::GsplatMode::MAGIC_WINDOW) {
    mode_spec_type = schemas::GsplatModeSpec::MagicWindowSpec;
    mode_spec_offset = schemas::CreateMagicWindowSpec(
                           *fbb,
                           imp::split_engine::PointerFromOptional(
                               magic_window_offscreen_texture_resolution),
                           render_group_name_offset)
                           .Union();
  }

  flatbuffers::Offset<schemas::BuiltInMaterialGsplatSpec> spec_offset =
      schemas::CreateBuiltInMaterialGsplatSpec(
          *fbb, material_mode, gsplat_renderer_entity_id, mode_spec_type,
          mode_spec_offset, &use_triangles_bool, &has_precomputed_texture_bool);
  return RequestBuiltInMaterial(
             view, std::move(fbb),
             schemas::BuiltInMaterialSpec::BuiltInMaterialGsplatSpec,
             spec_offset.Union())
      .Then([&view, gsplat_asset, material_mode, precomputed_data_texture,
             magic_window_offscreen_texture_resolution](
                imp::OwnedMaterialPtr material) mutable {
        return imp::OwnedPtr<GsplatMaterialSerializer>(
            new GsplatMaterialSerializer(
                view, std::move(material), gsplat_asset, material_mode,
                precomputed_data_texture,
                magic_window_offscreen_texture_resolution));
      });
}

GsplatMaterialSerializer::GsplatMaterialSerializer(
    imp::BaseView& view, imp::OwnedMaterialPtr material,
    imp::AssetPtr<imp::GSplatAsset> gsplat_asset,
    android_xr::schemas::GsplatMode material_mode,
    imp::BorrowedTexturePtr precomputed_data_texture,
    std::optional<android_xr::schemas::Uint2> magic_window_offscreen_resolution)
    : SplitEngineBuiltinMaterial(
          view,
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialGsplatParameters,
          std::move(material)),
      gsplat_asset_(gsplat_asset),
      material_mode_(material_mode),
      precomputed_data_texture_(precomputed_data_texture),
      magic_window_offscreen_resolution_(magic_window_offscreen_resolution) {
  SetPrecomputeTextures(gsplat_asset);
}

GsplatMaterialSerializer::~GsplatMaterialSerializer() { Cleanup(); }

flatbuffers::Offset<void> GsplatMaterialSerializer::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      precomputed_data_texture = ToTextureParameter(
          fbb, texture_parameter_creator, precomputed_data_texture_);
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      position_data_texture = ToTextureParameter(fbb, texture_parameter_creator,
                                                 position_data_texture_);
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      cov3d_data_texture = ToTextureParameter(fbb, texture_parameter_creator,
                                              cov3d_data_texture_);
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      color_data_texture = ToTextureParameter(fbb, texture_parameter_creator,
                                              color_data_texture_);
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      sorted_indices_texture = ToTextureParameter(
          fbb, texture_parameter_creator, sorted_indices_texture_);
  schemas::GsplatModeParameters mode_parameters_type =
      schemas::GsplatModeParameters::NONE;
  flatbuffers::Offset<void> mode_parameters_offset;

  if (material_mode_ == schemas::GsplatMode::GSPLAT) {
    mode_parameters_type = schemas::GsplatModeParameters::GsplatParameters;
    mode_parameters_offset =
        schemas::CreateGsplatParameters(
            fbb, imp::split_engine::PointerFromOptional(opacity_scale_),
            imp::split_engine::PointerFromOptional(min_screen_size_),
            imp::split_engine::PointerFromOptional(max_screen_size_),
            position_data_texture, cov3d_data_texture, color_data_texture,
            sorted_indices_texture,
            imp::split_engine::PointerFromOptional(splat_scale_),
            ToProjectionQuadSchema(fbb, magic_window_projection_quad_),
            imp::split_engine::PointerFromOptional(
                magic_window_offscreen_resolution_),
            precomputed_data_texture,
            imp::split_engine::PointerFromOptional(view_resolution_))
            .Union();
  } else if (material_mode_ == schemas::GsplatMode::MAGIC_WINDOW) {
    mode_parameters_type = schemas::GsplatModeParameters::MagicWindowParameters;
    mode_parameters_offset =
        schemas::CreateMagicWindowParameters(
            fbb, ToProjectionQuadSchema(fbb, magic_window_projection_quad_),
            imp::split_engine::PointerFromOptional(
                magic_window_offscreen_resolution_))
            .Union();
  }

  return schemas::CreateBuiltInMaterialGsplatParameters(
             fbb, mode_parameters_type, mode_parameters_offset)
      .Union();
}

bool GsplatMaterialSerializer::HasParameter(absl::string_view parameter_name) {
  return IsParameterSupported(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            float value) {
  if (parameter_name == imp::kOpacityScaleParameter) {
    SetOpacityScale(value);
  } else if (parameter_name == imp::kSplatScaleParameter) {
    SetSplatScale(value);
  } else {
    UnsupportedParameterError(parameter_name);
  }
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::float2 value) {
  if (parameter_name == imp::kMinScreenSizeParameter) {
    SetMinScreenSize(value);
  } else if (parameter_name == imp::kMaxScreenSizeParameter) {
    SetMaxScreenSize(value);
  } else {
    UnsupportedParameterError(parameter_name);
  }
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::uint2 value) {
  if (parameter_name == imp::kMagicWindowOffscreenResolutionParameter) {
    SetMagicWindowOffscreenResolution(value);
  } else if (parameter_name == imp::kMainViewResolutionParameter) {
    SetViewResolution(value);
  } else {
    UnsupportedParameterError(parameter_name);
  }
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::mat4f value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            bool value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::bool2 value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::bool3 value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::bool4 value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::float3 value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::float4 value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            int value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::int2 value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::int3 value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::int4 value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            uint value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::uint3 value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::uint4 value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            imp::mat3f value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            absl::Span<const bool> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::bool2> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::bool3> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::bool4> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            absl::Span<const float> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::float2> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::float3> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::float4> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            absl::Span<const int> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            absl::Span<const imp::int2> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            absl::Span<const imp::int3> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            absl::Span<const imp::int4> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            absl::Span<const uint> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::uint2> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::uint3> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::uint4> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::mat3f> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, absl::Span<const imp::mat4f> value) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            filament::RgbaType type,
                                            filament::math::float4 color) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(absl::string_view parameter_name,
                                            filament::RgbType type,
                                            filament::math::float3 color) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, const imp::Texture* texture,
    std::optional<filament::TextureSampler> sampler_override) {
  UnsupportedParameterError(parameter_name);
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, imp::TexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  IMP_LOG(imp::DFATAL) << "SetParameter with TexturePtr is not supported. Use "
                 "OwnedTexturePtr or BorrowedTexturePtr instead.";
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, imp::OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  if (!texture) {
    IMP_LOG(imp::DFATAL) << "Texture is null for parameter: " << parameter_name;
    return;
  }
  if (sampler_override.has_value() &&
      !TextureSamplersAreSame(sampler_override.value(),
                              texture->GetSampler())) {
    IMP_LOG(imp::DFATAL) << "Sampler override is not supported for parameter: "
                << parameter_name;
  }
  if (parameter_name == imp::kSplatDataPrecomputedParameter) {
    precomputed_data_texture_ = std::move(texture);
  } else if (parameter_name == imp::kSortedIndicesParameter) {
    IMP_LOG(imp::DFATAL) << "Sorted indices texture passed as an OwnedTexturePtr it "
                   "must be a BorrowedTexturePtr.";
  } else if (parameter_name == imp::kPositionDataTextureParameter) {
    position_data_texture_ = std::move(texture);
  } else if (parameter_name == imp::kCov3dDataTextureParameter) {
    cov3d_data_texture_ = std::move(texture);
  } else if (parameter_name == imp::kColorDataTextureParameter) {
    color_data_texture_ = std::move(texture);
  } else {
    UnsupportedParameterError(parameter_name);
  }
  MarkParametersDirty();
}

void GsplatMaterialSerializer::SetParameter(
    absl::string_view parameter_name, imp::BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  if (!texture) {
    IMP_LOG(imp::DFATAL) << "Texture is null for parameter: " << parameter_name;
    return;
  }
  if (sampler_override.has_value() &&
      !TextureSamplersAreSame(sampler_override.value(),
                              texture->GetSampler())) {
    IMP_LOG(imp::DFATAL) << "Sampler override is not supported for parameter: "
                << parameter_name;
  }
  if (parameter_name == imp::kSplatDataPrecomputedParameter) {
    precomputed_data_texture_ = std::move(texture);
  } else if (parameter_name == imp::kSortedIndicesParameter) {
    sorted_indices_texture_ = std::move(texture);
  } else if (parameter_name == imp::kPositionDataTextureParameter) {
    position_data_texture_ = std::move(texture);
  } else if (parameter_name == imp::kCov3dDataTextureParameter) {
    cov3d_data_texture_ = std::move(texture);
  } else if (parameter_name == imp::kColorDataTextureParameter) {
    color_data_texture_ = std::move(texture);
  } else {
    UnsupportedParameterError(parameter_name);
  }
  MarkParametersDirty();
}

}  // namespace android_xr
