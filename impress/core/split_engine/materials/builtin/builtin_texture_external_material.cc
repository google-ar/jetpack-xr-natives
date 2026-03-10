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
#include "core/split_engine/materials/builtin/builtin_texture_external_material.h"

#include <functional>
#include <optional>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "flatbuffers/verifier.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/material_library/material_package.h"
#include "core/materials/material.h"
#include "core/ncsb/update_system.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_custom_material.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_material_registry.h"
#include "core/split_engine/materials/builtin/builtin_texture_external_material_assets.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

namespace {

// The name of the texture parameter in the material.
static constexpr absl::string_view kTextureParameter = "texture";

}  // namespace

Future<BuiltInMaterialPtr> BuiltInTextureExternalMaterial::Create(
    BaseView& view, BridgeId bridge_id,
    const android_xr::schemas::BuiltInMaterialTextureExternal& spec) {
  Future<AssetPtr<MaterialAsset>> future = view.GetAssetManager().LoadMaterial(
      split_engine::kBuiltinTextureExternalMatCmat);
  return future.Then([&view, bridge_id](AssetPtr<MaterialAsset> material_asset)
                         -> BuiltInMaterialPtr {
    return absl::WrapUnique(new BuiltInTextureExternalMaterial(
        view, bridge_id,
        view.GetMaterialFactory().CreateMaterial(material_asset)));
  });
}

BuiltInTextureExternalMaterial::BuiltInTextureExternalMaterial(
    BaseView& view, BridgeId bridge_id, OwnedMaterialPtr material)
    : BuiltInCustomMaterial(bridge_id, std::move(material)), view_(view) {}

BuiltInMaterialPtr BuiltInTextureExternalMaterial::Duplicate() const {
  return absl::WrapUnique(new BuiltInTextureExternalMaterial(
      view_, GetBridgeId(),
      view_.GetMaterialFactory().WrapMaterial(
          filament::MaterialInstance::duplicate(
              GetMaterial()->GetFilamentMaterialInstance()))));
}

absl::Status BuiltInTextureExternalMaterial::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    BuiltInMaterialTextureExternalParameters) {
    return absl::InvalidArgumentError(
        "This material requires BuiltInMaterialTextureExternalParameters");
  }

  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialTextureExternalParameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  const android_xr::schemas::BuiltInMaterialTextureExternalParameters* schema =
      parameters.data_as<
          android_xr::schemas::BuiltInMaterialTextureExternalParameters>();
  if (schema->texture() && GetMaterial()->HasParameter(kTextureParameter)) {
    const BorrowedTexturePtr texture = texture_borrower(
        schema->texture()->texture_id(), SmallSourceLocation::Current());
    if (!texture) {
      return absl::NotFoundError(absl::StrFormat(
          "Texture not found: %d", schema->texture()->texture_id()));
    }
    GetMaterial()->SetParameter(kTextureParameter, texture,
                                ConvertSampler(schema->texture()->sampler()));
  }
  return absl::OkStatus();
}

// Registers the built-in material factory.
const bool kRegisterMaterial = BuiltinMaterialRegistry::RegisterOrDie(
    android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialTextureExternal,
    [](BaseView& view, BridgeId bridge_id,
       const android_xr::schemas::BuiltInMaterialRequest& request,
       std::optional<
           std::reference_wrapper<const MaterialPackage::MaterialCache>>
           cache) -> Future<BuiltInMaterialPtr> {
      const android_xr::schemas::BuiltInMaterialTextureExternal* spec =
          request.data_as_BuiltInMaterialTextureExternal();
      if (spec == nullptr) {
        return Future<BuiltInMaterialPtr>(absl::InvalidArgumentError(
            "Failed to get BuiltInTextureExternalMaterial spec from the "
            "request."));
      }
      return BuiltInTextureExternalMaterial::Create(view, bridge_id, *spec);
    });

}  // namespace imp::split_engine
