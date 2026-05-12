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

#include "core/split_engine/materials/builtin/builtin_generic_spec_helpers.h"

#include <type_traits>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

absl::StatusOr<android_xr::schemas::GenericMaterialLightingModel> Pack(
    const schemas::GenericMaterialLightingModel& model) {
  switch (model) {
    case schemas::GenericMaterialLightingModel::Lit:
      return android_xr::schemas::GenericMaterialLightingModel::Lit;
    case schemas::GenericMaterialLightingModel::Unlit:
      return android_xr::schemas::GenericMaterialLightingModel::Unlit;
      // No default case to ensure that all enum values are handled at compile
      // time.
  }
  // Handle the case where the enum value is not handled above at runtime.
  return absl::InvalidArgumentError(absl::StrFormat(
      "Unknown lighting model: %d",
      static_cast<
          std::underlying_type_t<schemas::GenericMaterialLightingModel>>(
          model)));
}

absl::StatusOr<schemas::GenericMaterialLightingModel> Unpack(
    const android_xr::schemas::GenericMaterialLightingModel& model) {
  switch (model) {
    case android_xr::schemas::GenericMaterialLightingModel::Lit:
      return schemas::GenericMaterialLightingModel::Lit;
    case android_xr::schemas::GenericMaterialLightingModel::Unlit:
      return schemas::GenericMaterialLightingModel::Unlit;
      // No default case to ensure that all enum values are handled at compile
      // time.
  }
  // Handle the case where the enum value is not handled above at runtime.
  return absl::InvalidArgumentError(absl::StrFormat(
      "Unknown lighting model: %d",
      static_cast<std::underlying_type_t<
          android_xr::schemas::GenericMaterialLightingModel>>(model)));
}

absl::StatusOr<android_xr::schemas::GenericMaterialBlendMode> Pack(
    const schemas::GenericMaterialBlendMode& blend_mode) {
  switch (blend_mode) {
    case schemas::GenericMaterialBlendMode::Opaque:
      return android_xr::schemas::GenericMaterialBlendMode::Opaque;
    case schemas::GenericMaterialBlendMode::Masked:
      return android_xr::schemas::GenericMaterialBlendMode::Masked;
    case schemas::GenericMaterialBlendMode::Transparent:
      return android_xr::schemas::GenericMaterialBlendMode::Transparent;
    case schemas::GenericMaterialBlendMode::Refractive:
      return android_xr::schemas::GenericMaterialBlendMode::Refractive;
      // No default case to ensure that all enum values are handled at compile
      // time.
  }
  // Handle the case where the enum value is not handled above at runtime.
  return absl::InvalidArgumentError(absl::StrFormat(
      "Unknown blend mode: %d",
      static_cast<std::underlying_type_t<schemas::GenericMaterialBlendMode>>(
          blend_mode)));
}

absl::StatusOr<schemas::GenericMaterialBlendMode> Unpack(
    const android_xr::schemas::GenericMaterialBlendMode& blend_mode) {
  switch (blend_mode) {
    case android_xr::schemas::GenericMaterialBlendMode::Opaque:
      return schemas::GenericMaterialBlendMode::Opaque;
    case android_xr::schemas::GenericMaterialBlendMode::Masked:
      return schemas::GenericMaterialBlendMode::Masked;
    case android_xr::schemas::GenericMaterialBlendMode::Transparent:
      return schemas::GenericMaterialBlendMode::Transparent;
    case android_xr::schemas::GenericMaterialBlendMode::Refractive:
      return schemas::GenericMaterialBlendMode::Refractive;
      // No default case to ensure that all enum values are handled at compile
      // time.
  }
  // Handle the case where the enum value is not handled above at runtime.
  return absl::InvalidArgumentError(absl::StrFormat(
      "Unknown blend mode: %d",
      static_cast<std::underlying_type_t<
          android_xr::schemas::GenericMaterialBlendMode>>(blend_mode)));
}

// Converts a schemas::GenericMaterialDoubleSidedMode to an
// android_xr::schemas::GenericMaterialDoubleSidedMode.
absl::StatusOr<android_xr::schemas::GenericMaterialDoubleSidedMode> Pack(
    const schemas::GenericMaterialDoubleSidedMode& double_sided_mode) {
  switch (double_sided_mode) {
    case schemas::GenericMaterialDoubleSidedMode::DoubleSided:
      return android_xr::schemas::GenericMaterialDoubleSidedMode::DoubleSided;
    case schemas::GenericMaterialDoubleSidedMode::SingleSided:
      return android_xr::schemas::GenericMaterialDoubleSidedMode::SingleSided;
      // No default case to ensure that all enum values are handled at compile
      // time.
  }
  // Handle the case where the enum value is not handled above at runtime.
  return absl::InvalidArgumentError(absl::StrFormat(
      "Unknown double-sided mode: %d",
      static_cast<
          std::underlying_type_t<schemas::GenericMaterialDoubleSidedMode>>(
          double_sided_mode)));
}

absl::StatusOr<schemas::GenericMaterialDoubleSidedMode> Unpack(
    const android_xr::schemas::GenericMaterialDoubleSidedMode&
        double_sided_mode) {
  switch (double_sided_mode) {
    case android_xr::schemas::GenericMaterialDoubleSidedMode::DoubleSided:
      return schemas::GenericMaterialDoubleSidedMode::DoubleSided;
    case android_xr::schemas::GenericMaterialDoubleSidedMode::SingleSided:
      return schemas::GenericMaterialDoubleSidedMode::SingleSided;
      // No default case to ensure that all enum values are handled at compile
      // time.
  }
  // Handle the case where the enum value is not handled above at runtime.
  return absl::InvalidArgumentError(
      absl::StrFormat("Unknown double-sided mode: %d",
                      static_cast<std::underlying_type_t<
                          android_xr::schemas::GenericMaterialDoubleSidedMode>>(
                          double_sided_mode)));
}

absl::StatusOr<android_xr::schemas::GenericMaterialDepthClearMaterial> Pack(
    const schemas::GenericMaterialDepthClearMaterial& depth_clear_material) {
  switch (depth_clear_material) {
    case schemas::GenericMaterialDepthClearMaterial::Disabled:
      return android_xr::schemas::GenericMaterialDepthClearMaterial::Disabled;
    case schemas::GenericMaterialDepthClearMaterial::Enabled:
      return android_xr::schemas::GenericMaterialDepthClearMaterial::Enabled;
      // No default case to ensure that all enum values are handled at compile
      // time.
  }
  // Handle the case where the enum value is not handled above at runtime.
  return absl::InvalidArgumentError(absl::StrFormat(
      "Unknown depth clear material: %d",
      static_cast<
          std::underlying_type_t<schemas::GenericMaterialDepthClearMaterial>>(
          depth_clear_material)));
}

absl::StatusOr<schemas::GenericMaterialDepthClearMaterial> Unpack(
    const android_xr::schemas::GenericMaterialDepthClearMaterial&
        depth_clear_material) {
  switch (depth_clear_material) {
    case android_xr::schemas::GenericMaterialDepthClearMaterial::Disabled:
      return schemas::GenericMaterialDepthClearMaterial::Disabled;
    case android_xr::schemas::GenericMaterialDepthClearMaterial::Enabled:
      return schemas::GenericMaterialDepthClearMaterial::Enabled;
      // No default case to ensure that all enum values are handled at compile
      // time.
  }
  // Handle the case where the enum value is not handled above at runtime.
  return absl::InvalidArgumentError(absl::StrFormat(
      "Unknown depth clear material: %d",
      static_cast<std::underlying_type_t<
          android_xr::schemas::GenericMaterialDepthClearMaterial>>(
          depth_clear_material)));
}

absl::StatusOr<flatbuffers::Offset<android_xr::schemas::GenericMaterialSpec>>
Pack(flatbuffers::FlatBufferBuilder& fbb, const GenericMaterialSpec& spec) {
  MP_ASSIGN_OR_RETURN(
      android_xr::schemas::GenericMaterialLightingModel lighting_model,
      Pack(spec.GetLightingModel()));
  MP_ASSIGN_OR_RETURN(android_xr::schemas::GenericMaterialBlendMode blend_mode,
                   Pack(spec.GetBlendMode()));
  MP_ASSIGN_OR_RETURN(
      android_xr::schemas::GenericMaterialDoubleSidedMode double_sided_mode,
      Pack(spec.GetDoubleSidedMode()));
  MP_ASSIGN_OR_RETURN(android_xr::schemas::GenericMaterialDepthClearMaterial
                       depth_clear_material,
                   Pack(spec.GetDepthClearMaterial()));
  return android_xr::schemas::CreateGenericMaterialSpec(
      fbb, lighting_model, blend_mode, double_sided_mode, depth_clear_material);
}

absl::StatusOr<GenericMaterialSpec> Unpack(
    const android_xr::schemas::GenericMaterialSpec& spec) {
  MP_ASSIGN_OR_RETURN(schemas::GenericMaterialLightingModel lighting_model,
                   Unpack(spec.lighting_model()));
  MP_ASSIGN_OR_RETURN(schemas::GenericMaterialBlendMode blend_mode,
                   Unpack(spec.blend_mode()));
  MP_ASSIGN_OR_RETURN(schemas::GenericMaterialDoubleSidedMode double_sided_mode,
                   Unpack(spec.double_sided_mode()));
  MP_ASSIGN_OR_RETURN(
      schemas::GenericMaterialDepthClearMaterial depth_clear_material,
      Unpack(spec.depth_clear_material()));
  return GenericMaterialSpec(lighting_model, blend_mode, double_sided_mode,
                             depth_clear_material);
}

}  // namespace imp::split_engine
