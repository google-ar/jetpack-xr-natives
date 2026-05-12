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

#include "core/split_engine/materials/builtin/builtin_generic_material.h"

#include <functional>
#include <optional>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "flatbuffers/verifier.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/material_library/generic_material_impl.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_package.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_generic_spec_helpers.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/builtin/builtin_material_registry.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

Future<BuiltInMaterialPtr> BuiltInGenericMaterial::Create(
    BaseView& view, const GenericMaterialSpec& spec,
    const MaterialPackage::MaterialCache& materials) {
  return GenericMaterialImpl::Create(view, spec, materials)
      .Then([](GenericMaterialPtr generic_material) -> BuiltInMaterialPtr {
        return absl::WrapUnique(
            new BuiltInGenericMaterial(std::move(generic_material)));
      });
}

BuiltInGenericMaterial::BuiltInGenericMaterial(
    GenericMaterialPtr generic_material)
    : generic_material_(std::move(generic_material)) {}

BuiltInMaterialPtr BuiltInGenericMaterial::Duplicate() const {
  return absl::WrapUnique(
      new BuiltInGenericMaterial(generic_material_->Duplicate()));
}

absl::Status BuiltInGenericMaterial::SetParameters(
    flatbuffers::Verifier& verifier,
    const android_xr::schemas::BuiltInMaterialInstanceParameters& parameters,
    const TextureBorrower& texture_borrower) {
  if (parameters.data_type() != android_xr::schemas::BuiltInMaterialParameters::
                                    GenericMaterialParameters) {
    return absl::InvalidArgumentError(
        "This material requires GenericMaterialParameters");
  }
  if (!VerifyBuiltInMaterialParameters(
          verifier, parameters.data(),
          android_xr::schemas::BuiltInMaterialParameters::
              GenericMaterialParameters)) {
    return absl::InvalidArgumentError("Invalid parameters");
  }

  const android_xr::schemas::GenericMaterialParameters*
      generic_parameters_schema = generic_parameters_schema =
          parameters.data_as<android_xr::schemas::GenericMaterialParameters>();
  GenericMaterialParameters generic_material_parameters =
      GenericMaterialParameters::FromFlatbuffer(*generic_parameters_schema);
  return generic_material_->AssignTexturesAndParams(generic_material_parameters,
                                                    texture_borrower);
}

BorrowedMaterialPtr BuiltInGenericMaterial::GetMaterialInternal(
    SmallSourceLocation loc) const {
  return generic_material_->GetMaterial(loc);
}

// Registers the built-in material factory.
const bool kRegisterMaterial = BuiltinMaterialRegistry::RegisterOrDie(
    android_xr::schemas::BuiltInMaterialSpec::GenericMaterialSpec,
    [](BaseView& view, BridgeId bridge_id,
       const android_xr::schemas::BuiltInMaterialRequest& request,
       std::optional<
           std::reference_wrapper<const MaterialPackage::MaterialCache>>
           cache) -> Future<BuiltInMaterialPtr> {
      const android_xr::schemas::GenericMaterialSpec* spec =
          request.data_as_GenericMaterialSpec();
      if (spec == nullptr) {
        return Future<BuiltInMaterialPtr>(absl::InvalidArgumentError(
            "Failed to get the GenericMaterialSpec from the request."));
      }
      absl::StatusOr<GenericMaterialSpec> unpacked_spec = Unpack(*spec);
      if (!unpacked_spec.ok()) {
        return Future<BuiltInMaterialPtr>(unpacked_spec.status());
      }
      return BuiltInGenericMaterial::Create(view, *unpacked_spec, *cache);
    });

}  // namespace imp::split_engine
