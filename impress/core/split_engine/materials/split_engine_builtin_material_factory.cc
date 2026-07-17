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

#include "core/split_engine/materials/split_engine_builtin_material_factory.h"

#include <cstdint>
#include <memory>
#include <optional>
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
#include "core/material_library/generic_materials.h"
#include "core/material_library/material_package.h"
#include "core/split_engine/materials/builtin/builtin_generic_spec_helpers.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_material_registry.h"
#include "core/split_engine/materials/builtin_material_creator_helper.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_asset.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

SplitEngineBuiltinMaterialFactory::SplitEngineBuiltinMaterialFactory(
    BaseView& view)
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

Future<BuiltInMaterialPtr>
SplitEngineBuiltinMaterialFactory::HandleCreateRequest(
    BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterialRequest& request) {
  switch (request.data_type()) {
    case android_xr::schemas::BuiltInMaterialSpec::GenericMaterialSpec:
      return CreateBuiltInGenericMaterial(request);
    default:
      // All other built-in types can be created statically.
      return HandleCreateRequest(view_, bridge_id, request);
  }
}

Future<BuiltInMaterialPtr>
SplitEngineBuiltinMaterialFactory::HandleCreateRequest(
    BaseView& view, BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterialRequest& request) {
  IMP_LOG(imp::INFO) << "SplitEngineMaterialFactory: Creating material for type: "
            << android_xr::schemas::EnumNameBuiltInMaterialSpec(
                   request.data_type());

  switch (request.data_type()) {
    case android_xr::schemas::BuiltInMaterialSpec::GenericMaterialSpec:
      return Future<BuiltInMaterialPtr>(absl::InternalError(
          "Generic materials cannot be created statically."));
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial5cf26af8:
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialE3ca0ab9:
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialD1750064:
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialEb117dd9:
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial1b616c8a:
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial0d0cb9aa:
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialbd7fe08c:
    case android_xr::schemas::BuiltInMaterialSpec::
        BuiltInMaterialTextureExternal:
    case android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialGsplatSpec: {
      absl::StatusOr<BuiltinMaterialRegistry::BuiltinMaterialCreator&> creator =
          BuiltinMaterialRegistry::Get(request.data_type());
      if (!creator.ok()) {
        return Future<BuiltInMaterialPtr>(creator.status());
      }
      return (*creator)(view, bridge_id, request, std::nullopt);
    }

    default:
      return Future<BuiltInMaterialPtr>(
          absl::InvalidArgumentError(absl::StrFormat(
              "Unsupported material type: %d", request.data_type())));
  }
}

Future<BuiltInMaterialPtr>
SplitEngineBuiltinMaterialFactory::CreateBuiltInGenericMaterial(
    const android_xr::schemas::BuiltInMaterialRequest& request) {
  absl::StatusOr<GenericMaterialSpec> spec =
      Unpack(*request.data_as_GenericMaterialSpec());
  if (!spec.ok()) {
    return Future<BuiltInMaterialPtr>(spec.status());
  }

  absl::flat_hash_set<GenericMaterialSpec> requested_materials;
  requested_materials.insert(*spec);

  return material_package_
      ->GetOrLoadMaterials(view_, view_.GetSharedEngine(), requested_materials)
      .Then([this, spec, &request](MaterialPackage::MaterialCache cache)
                -> Future<BuiltInMaterialPtr> {
        absl::StatusOr<BuiltinMaterialRegistry::BuiltinMaterialCreator&>
            creator = BuiltinMaterialRegistry::Get(
                android_xr::schemas::BuiltInMaterialSpec::GenericMaterialSpec);
        if (!creator.ok()) {
          return Future<BuiltInMaterialPtr>(creator.status());
        }
        // Bridge ID is not used for generic materials.
        return (*creator)(view_, /*bridge_id=*/0, request, cache);
      });
}

std::vector<Future<BuiltInMaterialPtr>> SplitEngineBuiltinMaterialFactory::
    CreateBuiltInCustomMaterialWithDefaultParams() {
  // Bridge IDs don't matter here - we just want to make sure AssetManager
  // actually loads the material. Same for the material instance IDs.
  const BridgeId kFakeBridgeId = 1;
  const uint64_t kFakeMaterialInstanceId = 0;

  std::vector<Future<BuiltInMaterialPtr>> builtin_material_futures;
  for (android_xr::schemas::BuiltInMaterialSpec spec_type :
       android_xr::schemas::EnumValuesBuiltInMaterialSpec()) {
    // Skip NONE as it is not a actionable spec type.
    // Skip the generic and gsplat material specs, as they are not custom
    // materials.
    if (spec_type == android_xr::schemas::BuiltInMaterialSpec::NONE ||
        spec_type ==
            android_xr::schemas::BuiltInMaterialSpec::GenericMaterialSpec ||
        spec_type == android_xr::schemas::BuiltInMaterialSpec::
                         BuiltInMaterialGsplatSpec) {
      continue;
    }

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
