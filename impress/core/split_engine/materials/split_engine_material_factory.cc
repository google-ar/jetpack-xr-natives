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

#include "core/split_engine/materials/split_engine_material_factory.h"

#include <cstdint>
#include <memory>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_package.h"
#include "core/split_engine/materials/builtin/builtin_generic_material.h"
#include "core/split_engine/materials/builtin/builtin_jxr_media_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_svxr_footprint_material.h"
#include "core/split_engine/materials/builtin/builtin_svxr_plane_material.h"
#include "core/split_engine/materials/builtin/builtin_texture_external_material.h"
#include "core/split_engine/materials/builtin/builtin_vignette_material.h"
#include "core/split_engine/materials/builtin/builtin_water_material.h"
#include "core/split_engine/materials/builtin/builtin_youtube_stereo_player_material.h"
#include "core/split_engine/materials/builtin/photosxr/builtin_photos_texture_3d_material.h"
#include "core/split_engine/materials/builtin_material_creator_helper.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "core/view/framework/generic_materials.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

SplitEngineMaterialFactory::SplitEngineMaterialFactory(BaseView& view)
    : view_(view) {
  const GltfAsset::LoadOptions& load_options =
      view_.GetAssetManager().GetDefaultLoadOptions();
  material_package_ = std::make_unique<MaterialPackage>(
      view_.GetAssetManager().LoadResource(
          (load_options.materials_url_override.has_value()
               ? *load_options.materials_url_override
               : materials::kCompiledImpDefaultGltfMaterialsZip.GetUrl())),
      GltfAsset::kDefaultMaterialPreCompileOptions);
}

Future<BuiltInMaterialPtr> SplitEngineMaterialFactory::HandleCreateRequest(
    BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterialRequest& request) {
  switch (request.data_type()) {
    case android_xr::schemas::BuiltInMaterialSpec::GenericMaterialSpec:
      return CreateBuiltInGenericMaterial(
          *request.data_as<android_xr::schemas::GenericMaterialSpec>());
    default:
      // All other built-in types can be created statically.
      return HandleCreateRequest(view_, bridge_id, request);
  }
}

Future<BuiltInMaterialPtr> SplitEngineMaterialFactory::HandleCreateRequest(
    BaseView& view, BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterialRequest& request) {
  switch (request.data_type()) {
    case android_xr::schemas::BuiltInMaterialSpec::GenericMaterialSpec:
      return Future<BuiltInMaterialPtr>(absl::InternalError(
          "Generic materials cannot be created statically."));
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial5cf26af8:
      return BuiltInWaterMaterial::Create(
          view, bridge_id,
          *request.data_as<android_xr::schemas::BuiltInMaterial5cf26af8>());
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialE3ca0ab9:
      return BuiltInVignetteMaterial::Create(
          view, bridge_id,
          *request.data_as<android_xr::schemas::BuiltInMaterialE3ca0ab9>());
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialD1750064:
      return BuiltInPhotosTexture3dMaterial::Create(
          view, bridge_id,
          *request.data_as<android_xr::schemas::BuiltInMaterialD1750064>());
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialEb117dd9:
      return BuiltInYouTubeStereoPlayerMaterial::Create(
          view, bridge_id,
          *request.data_as<android_xr::schemas::BuiltInMaterialEb117dd9>());
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial1b616c8a:
      return BuiltInJxrMediaMaterial::Create(
          view, bridge_id,
          *request.data_as<android_xr::schemas::BuiltInMaterial1b616c8a>());
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial0d0cb9aa:
      return BuiltInSVXRFootprintMaterial::Create(
          view, bridge_id,
          *request.data_as<android_xr::schemas::BuiltInMaterial0d0cb9aa>());
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialbd7fe08c:
      return BuiltInSVXRPlaneMaterial::Create(
          view, bridge_id,
          *request.data_as<android_xr::schemas::BuiltInMaterialbd7fe08c>());
    case android_xr::schemas::BuiltInMaterialSpec::
        BuiltInMaterialTextureExternal:
      return BuiltInTextureExternalMaterial::Create(
          view, bridge_id,
          *request
               .data_as<android_xr::schemas::BuiltInMaterialTextureExternal>());
    default:
      return Future<BuiltInMaterialPtr>(
          absl::InvalidArgumentError(absl::StrFormat(
              "Unsupported material type: %d", request.data_type())));
  }
}

Future<BuiltInMaterialPtr>
SplitEngineMaterialFactory::CreateBuiltInGenericMaterial(
    const android_xr::schemas::GenericMaterialSpec& schema) {
  GenericMaterialSpec spec = FromFlatbuffer(schema);

  absl::flat_hash_set<GenericMaterialSpec> requested_materials;
  requested_materials.insert(spec);

  return material_package_
      ->GetOrLoadMaterials(view_, view_.GetSharedEngine(), requested_materials)
      .Then([this, spec](MaterialPackage::MaterialCache materials_by_params) {
        return BuiltInGenericMaterial::Create(view_, spec, materials_by_params);
      });
}

std::vector<Future<BuiltInMaterialPtr>>
SplitEngineMaterialFactory::CreateBuiltInCustomMaterialWithDefaultParams() {
  // Bridge IDs don't matter here - we just want to make sure AssetManager
  // actually loads the material. Same for the material instance IDs.
  const BridgeId kFakeBridgeId = 1;
  const uint64_t kFakeMaterialInstanceId = 0;

  std::vector<Future<BuiltInMaterialPtr>> builtin_material_futures;
  for (auto spec_type : CreateBuiltInCustomMaterialSpecList()) {
    flatbuffers::FlatBufferBuilder builder;
    flatbuffers::Offset<android_xr::schemas::BuiltInMaterialRequest>
        fbb_request = android_xr::schemas::CreateBuiltInMaterialRequest(
            builder, kFakeMaterialInstanceId, spec_type,
            CreateBuiltInMaterialSpecWithDefaultParameters(builder, spec_type));
    builder.Finish(fbb_request);

    const android_xr::schemas::BuiltInMaterialRequest* request =
        flatbuffers::GetRoot<android_xr::schemas::BuiltInMaterialRequest>(
            builder.GetBufferPointer());

    IMP_LOG(imp::INFO) << "Preloading material: " << absl::StrCat(spec_type);
    builtin_material_futures.push_back(
        HandleCreateRequest(view_, kFakeBridgeId, *request));
  }
  return builtin_material_futures;
}

}  // namespace imp::split_engine
