/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_PARAMETER_CONSTANTS_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_PARAMETER_CONSTANTS_H_

#include "absl/strings/string_view.h"

namespace imp {

// Public parameter names supported by GenericMaterial::SetParameter.
//
// Note: Which parameters are available is determiend by a combination of the
// GenericMaterialSpec that the GenericMaterial was loaded with & the features
// enabled in the MaterialPackage used. Typically, the MaterialPackage is
// specified via GltfAsset::LoadOptions.

// LINT.IfChange(generic_material_parameters)

// The base color factor (float4). Default is [1, 1, 1, 1].
constexpr absl::string_view kBaseColorFactor = "baseColorFactor";

// The base color texture (Texture).
constexpr absl::string_view kBaseColorTexture = "baseColorTexture";

// The metallic factor (float). Default is 1.0.
constexpr absl::string_view kMetallicFactor = "metallicFactor";

// The roughness factor (float). Default is 1.0.
constexpr absl::string_view kRoughnessFactor = "roughnessFactor";

// The metallic-roughness texture (Texture).
constexpr absl::string_view kMetallicRoughnessTexture =
    "metallicRoughnessTexture";

// The normal scale (float). Default is 1.0.
constexpr absl::string_view kNormalScale = "normalScale";

// The normal texture (Texture).
constexpr absl::string_view kNormalTexture = "normalTexture";

// The ambient occlusion strength (float). Default is 1.0.
constexpr absl::string_view kAoStrength = "aoStrength";

// The ambient occlusion texture (Texture).
constexpr absl::string_view kAoTexture = "aoTexture";

// The emissive factor (float3). Default is [0, 0, 0].
constexpr absl::string_view kEmissiveFactor = "emissiveFactor";

// The emissive texture (Texture).
constexpr absl::string_view kEmissiveTexture = "emissiveTexture";

// The clearcoat factor (float3).
// Sets clearcoat intensity (x), roughness (y), and normal scale (z).
constexpr absl::string_view kClearcoatRoughnessNormalFactors =
    "clearcoat_roughness_normal_factors";

// The clearcoat texture (Texture).
constexpr absl::string_view kClearcoatTexture = "clearcoatTexture";

// The clearcoat normal texture (Texture).
constexpr absl::string_view kClearcoatNormalTexture = "clearcoatNormalTexture";

// The clearcoat roughness texture (Texture).
constexpr absl::string_view kClearcoatRoughnessTexture =
    "clearcoatRoughnessTexture";

// The sheen color factor (float3). Default is [0, 0, 0].
constexpr absl::string_view kSheenColorFactor = "sheenColorFactor";

// The sheen color texture (Texture).
constexpr absl::string_view kSheenColorTexture = "sheenColorTexture";

// The sheen roughness factor (float). Default is 0.0.
constexpr absl::string_view kSheenRoughnessFactor = "sheenRoughnessFactor";

// The sheen roughness texture (Texture).
constexpr absl::string_view kSheenRoughnessTexture = "sheenRoughnessTexture";

// The transmission factor (float). Default is 0.0.
constexpr absl::string_view kTransmissionFactor = "transmissionFactor";

// The transmission texture (Texture).
constexpr absl::string_view kTransmissionTexture = "transmissionTexture";

// The thickness factor (float). Default is 0.0.
constexpr absl::string_view kThicknessFactor = "thicknessFactor";

// The thickness texture (Texture).
constexpr absl::string_view kThicknessTexture = "thicknessTexture";

// The attenuation distance (float). Default is max float.
constexpr absl::string_view kAttenuationDistance = "attenuationDistance";

// The attenuation color (float3). Default is [1, 1, 1].
constexpr absl::string_view kAttenuationColor = "attenuationColor";

// The index of refraction (float). Default is 1.5.
constexpr absl::string_view kIndexOfRefraction = "indexOfRefraction";

// LINT.ThenChange(
//     //depot/google3/third_party/impress/core/loader/data/generic_material_lit.mat.template.glsl:generic_material_parameters,
//     //depot/google3/third_party/impress/core/material_library/generic_material.cc:supported_parameters
// )

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_PARAMETER_CONSTANTS_H_
