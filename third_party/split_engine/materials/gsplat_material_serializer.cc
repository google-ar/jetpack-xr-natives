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
#include <utility>

#include "absl/memory/memory.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/gsplat/gsplat_asset.h"
#include "core/ncsb/node.h"
#include "core/render/texture.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

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
}  // namespace

imp::Future<std::unique_ptr<GsplatMaterialSerializer>>
GsplatMaterialSerializer::Create(
    imp::NodeHandle gsplat_node, imp::AssetPtr<imp::GSplatAsset> gsplat_asset,
    android_xr::schemas::GsplatMode material_mode,
    bool use_triangles_for_splats,
    imp::BorrowedTexturePtr precomputed_data_texture) {
  imp::BaseView& view = gsplat_node->GetView();
  uint32_t gsplat_renderer_entity_id = gsplat_node.GetEntity().getId();
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  auto use_triangles_bool = android_xr::schemas::Bool(use_triangles_for_splats);
  auto has_precomputed_texture_bool =
      android_xr::schemas::Bool(precomputed_data_texture != nullptr);
  flatbuffers::Offset<schemas::BuiltInMaterialGsplatSpec> spec_offset =
      schemas::CreateBuiltInMaterialGsplatSpec(
          *fbb, material_mode, gsplat_renderer_entity_id, &use_triangles_bool,
          &has_precomputed_texture_bool);
  return RequestBuiltInMaterial(
             view, std::move(fbb),
             schemas::BuiltInMaterialSpec::BuiltInMaterialGsplatSpec,
             spec_offset.Union())
      .Then([&view, gsplat_asset, precomputed_data_texture](
                imp::split_engine::PlaceholderOrBuiltInMaterialPtr
                    material) mutable {
        return absl::WrapUnique(new GsplatMaterialSerializer(
            view, std::move(material), gsplat_asset, precomputed_data_texture));
      });
}

GsplatMaterialSerializer::GsplatMaterialSerializer(
    imp::BaseView& view,
    imp::split_engine::PlaceholderOrBuiltInMaterialPtr material,
    imp::AssetPtr<imp::GSplatAsset> gsplat_asset,
    imp::BorrowedTexturePtr precomputed_data_texture)
    : SplitEngineBuiltinMaterial(
          view,
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialGsplatParameters,
          std::move(material)),
      view_(view),
      gsplat_asset_(gsplat_asset),
      precomputed_data_texture_(precomputed_data_texture) {
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
  return schemas::CreateBuiltInMaterialGsplatParameters(
             fbb,  // parameters are in same order as in schema
             imp::split_engine::PointerFromOptional(opacity_scale_),
             imp::split_engine::PointerFromOptional(min_screen_size_),
             imp::split_engine::PointerFromOptional(max_screen_size_),
             imp::split_engine::PointerFromOptional(
                 window_dimension_in_magic_window_),
             imp::split_engine::PointerFromOptional(
                 magic_window_from_user_world_matrix_),
             precomputed_data_texture, position_data_texture,
             cov3d_data_texture, color_data_texture, sorted_indices_texture,
             imp::split_engine::PointerFromOptional(splat_scale_))
      .Union();
}

}  // namespace android_xr
