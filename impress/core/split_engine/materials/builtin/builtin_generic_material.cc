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

#include <cstddef>
#include <optional>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/verifier.h"
#include "core/async/future.h"
#include "core/common/small_source_location.h"
#include "core/material_library/flatbuffer_utils.h"
#include "core/material_library/generic_material_impl.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/material_package.h"
#include "core/material_library/material_param_value.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/materials/material.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

namespace {

template <typename EnumA, typename EnumB>
constexpr bool DoEnumsMatch(EnumA enum_a, EnumB enum_b) {
  return static_cast<size_t>(enum_a) == static_cast<size_t>(enum_b);
}

// Verify android_xr::schemas::GenericMaterialLightingModel and
// schemas::GenericMaterialLightingModel enums match.
static_assert(
    DoEnumsMatch(schemas::GenericMaterialLightingModel::Lit,
                 android_xr::schemas::GenericMaterialLightingModel::Lit),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(schemas::GenericMaterialLightingModel::Unlit,
                 android_xr::schemas::GenericMaterialLightingModel::Unlit),
    "Enum mismatch");
static_assert(android_xr::schemas::GenericMaterialLightingModel::MAX ==
                  android_xr::schemas::GenericMaterialLightingModel::Unlit,
              "New fields added but assert not updated");

// Verify android_xr::schemas::GenericMaterialBlendMode and
// schemas::GenericMaterialBlendMode enums match.
static_assert(
    DoEnumsMatch(schemas::GenericMaterialBlendMode::Opaque,
                 android_xr::schemas::GenericMaterialBlendMode::Opaque),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(schemas::GenericMaterialBlendMode::Masked,
                 android_xr::schemas::GenericMaterialBlendMode::Masked),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(schemas::GenericMaterialBlendMode::Transparent,
                 android_xr::schemas::GenericMaterialBlendMode::Transparent),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(schemas::GenericMaterialBlendMode::Refractive,
                 android_xr::schemas::GenericMaterialBlendMode::Refractive),
    "Enum mismatch");

static_assert(android_xr::schemas::GenericMaterialBlendMode::MAX ==
                  android_xr::schemas::GenericMaterialBlendMode::Refractive,
              "New fields added but assert not updated");

// Verify android_xr::schemas::GenericMaterialDoubleSidedMode and
// schemas::GenericMaterialDoubleSidedMode enums match.
static_assert(
    DoEnumsMatch(
        schemas::GenericMaterialDoubleSidedMode::SingleSided,
        android_xr::schemas::GenericMaterialDoubleSidedMode::SingleSided),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(
        schemas::GenericMaterialDoubleSidedMode::DoubleSided,
        android_xr::schemas::GenericMaterialDoubleSidedMode::DoubleSided),
    "Enum mismatch");
static_assert(
    android_xr::schemas::GenericMaterialDoubleSidedMode::MAX ==
        android_xr::schemas::GenericMaterialDoubleSidedMode::DoubleSided,
    "New fields added but assert not updated");

// Verify android_xr::schemas::GenericMaterialDepthClearMaterial and
// schemas::GenericMaterialDepthClearMaterial enums match.
static_assert(
    DoEnumsMatch(
        schemas::GenericMaterialDepthClearMaterial::Disabled,
        android_xr::schemas::GenericMaterialDepthClearMaterial::Disabled),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(
        schemas::GenericMaterialDepthClearMaterial::Enabled,
        android_xr::schemas::GenericMaterialDepthClearMaterial::Enabled),
    "Enum mismatch");
static_assert(
    android_xr::schemas::GenericMaterialDepthClearMaterial::MAX ==
        android_xr::schemas::GenericMaterialDepthClearMaterial::Enabled,
    "New fields added but assert not updated");

// Verify android_xr::schemas::MinFilter and filament::TextureSampler::MinFilter
// enums match.
static_assert(DoEnumsMatch(filament::TextureSampler::MinFilter::NEAREST,
                           android_xr::schemas::MinFilter::NEAREST),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::TextureSampler::MinFilter::LINEAR,
                           android_xr::schemas::MinFilter::LINEAR),
              "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::TextureSampler::MinFilter::NEAREST_MIPMAP_NEAREST,
                 android_xr::schemas::MinFilter::NEAREST_MIPMAP_NEAREST),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::TextureSampler::MinFilter::LINEAR_MIPMAP_NEAREST,
                 android_xr::schemas::MinFilter::LINEAR_MIPMAP_NEAREST),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::TextureSampler::MinFilter::NEAREST_MIPMAP_LINEAR,
                 android_xr::schemas::MinFilter::NEAREST_MIPMAP_LINEAR),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR,
                 android_xr::schemas::MinFilter::LINEAR_MIPMAP_LINEAR),
    "Enum mismatch");
static_assert(android_xr::schemas::MinFilter::MAX ==
                  android_xr::schemas::MinFilter::LINEAR_MIPMAP_LINEAR,
              "New fields added but assert not updated");

}  // namespace

flatbuffers::Offset<android_xr::schemas::GenericMaterialSpec>
CreateGenericMaterialSpec(flatbuffers::FlatBufferBuilder& fbb,
                          const GenericMaterialSpec& spec) {
  return android_xr::schemas::CreateGenericMaterialSpec(
      fbb,
      static_cast<android_xr::schemas::GenericMaterialLightingModel>(
          spec.GetLightingModel()),
      static_cast<android_xr::schemas::GenericMaterialBlendMode>(
          spec.GetBlendMode()),
      static_cast<android_xr::schemas::GenericMaterialDoubleSidedMode>(
          spec.GetDoubleSidedMode()),
      static_cast<android_xr::schemas::GenericMaterialDepthClearMaterial>(
          spec.GetDepthClearMaterial()));
}

GenericMaterialSpec FromFlatbuffer(
    const android_xr::schemas::GenericMaterialSpec& flatbuffer) {
  return GenericMaterialSpec(
      static_cast<schemas::GenericMaterialLightingModel>(
          flatbuffer.lighting_model()),
      static_cast<schemas::GenericMaterialBlendMode>(flatbuffer.blend_mode()),
      static_cast<schemas::GenericMaterialDoubleSidedMode>(
          flatbuffer.double_sided_mode()),
      static_cast<schemas::GenericMaterialDepthClearMaterial>(
          flatbuffer.depth_clear_material()));
}

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

}  // namespace imp::split_engine
