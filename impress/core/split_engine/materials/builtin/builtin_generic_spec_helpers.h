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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_GENERIC_SPEC_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_GENERIC_SPEC_HELPERS_H_

#include "absl/log/check.h"
#include "absl/status/statusor.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// Converts a schemas::GenericMaterialLightingModel to an
// android_xr::schemas::GenericMaterialLightingModel.
absl::StatusOr<android_xr::schemas::GenericMaterialLightingModel> Pack(
    const schemas::GenericMaterialLightingModel& model);

// Converts an android_xr::schemas::GenericMaterialLightingModel to a
// schemas::GenericMaterialLightingModel.
absl::StatusOr<schemas::GenericMaterialLightingModel> Unpack(
    const android_xr::schemas::GenericMaterialLightingModel& model);

// Converts a schemas::GenericMaterialBlendMode to an
// android_xr::schemas::GenericMaterialBlendMode.
absl::StatusOr<android_xr::schemas::GenericMaterialBlendMode> Pack(
    const schemas::GenericMaterialBlendMode& blend_mode);

// Converts an android_xr::schemas::GenericMaterialBlendMode to a
// schemas::GenericMaterialBlendMode.
absl::StatusOr<schemas::GenericMaterialBlendMode> Unpack(
    const android_xr::schemas::GenericMaterialBlendMode& blend_mode);

// Converts a schemas::GenericMaterialDoubleSidedMode to an
// android_xr::schemas::GenericMaterialDoubleSidedMode.
absl::StatusOr<android_xr::schemas::GenericMaterialDoubleSidedMode> Pack(
    const schemas::GenericMaterialDoubleSidedMode& double_sided_mode);

// Converts an android_xr::schemas::GenericMaterialDoubleSidedMode to a
// schemas::GenericMaterialDoubleSidedMode.
absl::StatusOr<schemas::GenericMaterialDoubleSidedMode> Unpack(
    const android_xr::schemas::GenericMaterialDoubleSidedMode&
        double_sided_mode);

// Converts a schemas::GenericMaterialDepthClearMaterial to an
// android_xr::schemas::GenericMaterialDepthClearMaterial.
absl::StatusOr<android_xr::schemas::GenericMaterialDepthClearMaterial> Pack(
    const schemas::GenericMaterialDepthClearMaterial& depth_clear_material);

// Converts an android_xr::schemas::GenericMaterialDepthClearMaterial to a
// schemas::GenericMaterialDepthClearMaterial.
absl::StatusOr<schemas::GenericMaterialDepthClearMaterial> Unpack(
    const android_xr::schemas::GenericMaterialDepthClearMaterial&
        depth_clear_material);

// Packs a GenericMaterialSpec into an android_xr::schemas::GenericMaterialSpec.
absl::StatusOr<flatbuffers::Offset<android_xr::schemas::GenericMaterialSpec>>
Pack(flatbuffers::FlatBufferBuilder& fbb, const GenericMaterialSpec& spec);

// Unpacks an android_xr::schemas::GenericMaterialSpec into a
// GenericMaterialSpec.
absl::StatusOr<GenericMaterialSpec> Unpack(
    const android_xr::schemas::GenericMaterialSpec& spec);

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_GENERIC_SPEC_HELPERS_H_
