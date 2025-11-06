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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_CONSTANTS_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_CONSTANTS_H_

#include <array>

#include "absl/strings/string_view.h"
#include "core/math/vec.h"

namespace imp {

// Material parameter names emitted by gltf materials.

// LINT.IfChange(generic_material_parameters)
constexpr const absl::string_view kBaseColorIndex = "baseColorIndex";
constexpr const absl::string_view kBaseColorFactor = "baseColorFactor";
constexpr const absl::string_view kMetallicRoughnessIndex =
    "metallicRoughnessIndex";
constexpr const absl::string_view kMetallicFactor = "metallicFactor";
constexpr const absl::string_view kRoughnessFactor = "roughnessFactor";
constexpr const absl::string_view kNormalIndex = "normalIndex";
constexpr const absl::string_view kNormalScale = "normalScale";
constexpr const absl::string_view kAoIndex = "aoIndex";
constexpr const absl::string_view kAoStrength = "aoStrength";
constexpr const absl::string_view kEmissiveIndex = "emissiveIndex";
constexpr const absl::string_view kEmissiveFactor = "emissiveFactor";
constexpr const absl::string_view kClearcoatRoughnessNormalFactors =
    "clearcoat_roughness_normal_factors";
constexpr const absl::string_view kClearcoatIndex = "clearcoatIndex";
constexpr const absl::string_view kClearcoatRoughnessIndex =
    "clearcoatRoughnessIndex";
constexpr const absl::string_view kClearcoatNormalIndex =
    "clearcoatNormalIndex";
constexpr const absl::string_view kSheenColorIndex = "sheenColorIndex";
constexpr const absl::string_view kSheenColorFactor = "sheenColorFactor";
constexpr const absl::string_view kSheenRoughnessIndex = "sheenRoughnessIndex";
constexpr const absl::string_view kSheenRoughnessFactor =
    "sheenRoughnessFactor";
constexpr const absl::string_view kTransmissionIndex = "transmissionIndex";
constexpr const absl::string_view kTransmissionFactor = "transmissionFactor";
constexpr const absl::string_view kIndexOfRefraction = "indexOfRefraction";
constexpr const absl::string_view kSamplersUvBitflags = "samplers_uv_bitflags";
constexpr const absl::string_view kSamplersUvMatrices = "samplers_uv_matrices";
constexpr const absl::string_view kEstimatedDepthTexture =
    "estimatedDepthTexture";
constexpr const absl::string_view kCameraTexture = "cameraTexture";
constexpr const absl::string_view kFeatureIdTexture0 = "featureIdTexture0";
constexpr const absl::string_view kFeatureIdTexture1 = "featureIdTexture1";
constexpr const absl::string_view kFeatureIdTexture2 = "featureIdTexture2";
constexpr const absl::string_view kFeatureIdTexture3 = "featureIdTexture3";
constexpr const std::array<absl::string_view, 4> kFeatureIdTextureNames = {
    kFeatureIdTexture0, kFeatureIdTexture1, kFeatureIdTexture2,
    kFeatureIdTexture3};

constexpr float4 kDefaultBaseColorFactor = kOne4;
constexpr float kDefaultMetallicFactor = 1;
constexpr float kDefaultRoughnessFactor = 1;
constexpr float kDefaultNormalFactor = 1;
constexpr float kDefaultAmbientOcclusionFactor = 1;
constexpr float3 kDefaultEmissiveFactor = kZero3;
// Default clearcoat factor has R (intensity) = 0, G (roughness) = 0, and B
// (normal) = 1. The normal value comes from the ClearcoatNormalTexture's
// NormalTextureInfo scale.
constexpr float3 kDefaultClearcoatFactor = float3(0, 0, 1);
constexpr float3 kDefaultSheenColorFactor = kZero3;
constexpr float kDefaultSheenRoughnessFactor = 0;
constexpr float kDefaultTransmissionFactor = 0;
constexpr float kDefaultIndexOfRefraction = 1.5;
constexpr float kDefaultAlphaCutoff = 0.5;

// LINT.ThenChange(//depot/google3/third_party/impress/core/loader/data/generic_material_lit.mat.template.glsl:generic_material_parameters)

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_CONSTANTS_H_
