/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_GSPLAT_GSPLAT_MATERIAL_PARAMS_H_
#define THIRD_PARTY_IMPRESS_CORE_GSPLAT_GSPLAT_MATERIAL_PARAMS_H_

#include "absl/strings/string_view.h"

namespace imp {

// LINT.IfChange
// Material Constants
inline constexpr absl::string_view kUseTrianglesForSplatsConstant =
    "useTrianglesForSplats";

// Material parameters
inline constexpr absl::string_view kBaseColorParameter = "baseColor";
inline constexpr absl::string_view kColorDataTextureParameter =
    "splatDataColor";
inline constexpr absl::string_view kCov3dDataTextureParameter =
    "splatDataCov3d";
inline constexpr absl::string_view kMagicWindowFromUserWorldMatrixParameter =
    "magicWindowFromUserWorldMatrix";
inline constexpr absl::string_view kMagicWindowOffscreenResolutionParameter =
    "magicWindowOffscreenResolution";
inline constexpr absl::string_view kMainViewResolutionParameter =
    "mainViewResolution";
inline constexpr absl::string_view kMaxScreenSizeParameter = "maxScreenSize";
inline constexpr absl::string_view kMinScreenSizeParameter = "minScreenSize";
inline constexpr absl::string_view kOpacityScaleParameter = "opacityScale";
inline constexpr absl::string_view kPositionDataTextureParameter =
    "splatDataPosition";
inline constexpr absl::string_view kSortedIndicesParameter = "sortedIndices";
inline constexpr absl::string_view kSplatDataPrecomputedParameter =
    "splatDataPrecomputed";
inline constexpr absl::string_view kSplatScaleParameter = "splatScale";
inline constexpr absl::string_view kWindowDimensionInMagicWindowParameter =
    "windowDimensionInMagicWindow";
// LINT.ThenChange(
//   //depot/google3/third_party/split_engine/schemas/split_engine_material.fbs,
//   //depot/google3/third_party/impress/core/split_engine/materials/builtin/gsplat/gsplat_material_deserializer.cc
//   //depot/google3/third_party/impress/core/split_engine/materials/builtin/gsplat/builtin_magic_window_panel.mat,
//   //depot/google3/third_party/impress/core/split_engine/materials/builtin/gsplat/builtin_gsplat.mat,
//   //depot/google3/third_party/impress/core/split_engine/materials/builtin/gsplat/builtin_gsplat_data_precompute.mat,
//   //depot/google3/third_party/impress/core/gsplat/gsplat_renderer.cc,
// )

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_GSPLAT_GSPLAT_MATERIAL_PARAMS_H_
