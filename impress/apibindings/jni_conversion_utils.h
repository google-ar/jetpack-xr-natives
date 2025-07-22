/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_JNI_CONVERSION_UTILS_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_JNI_CONVERSION_UTILS_H_

#include "absl/status/statusor.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/common/jni_helpers.h"
#include "core/material_library/generic_material_spec.h"

// JNI conversion utility functions for the Impress API Bindings layer.
namespace imp {

// Builds a GenericMaterialSpec from individual JNI integer values.
absl::StatusOr<imp::GenericMaterialSpec> BuildGenericMaterialSpecFromValues(
    jint lighting_model_value, jint blend_mode_value,
    jint double_sided_mode_value);

// Builds a Filament TextureSampler from individual JNI integer values.
absl::StatusOr<filament::TextureSampler> BuildTextureSamplerFromValues(
    jint min_filter_value, jint mag_filter_value, jint wrap_mode_s_value,
    jint wrap_mode_t_value, jint wrap_mode_r_value, jint compare_mode_value,
    jint compare_func_value, jint anisotropy_log2_value);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_JNI_CONVERSION_UTILS_H_
