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

#ifndef THIRD_PARTY_IMPRESS_CORE_XR_MEDIA_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_XR_MEDIA_UTILS_H_

#include "absl/strings/string_view.h"

namespace imp {

// Main texture used for single view media types as well as the primary view
// of multiview media types.
static constexpr absl::string_view kTextureParameter = "mediaTexture";

// Auxiliary textures used for the secondary view of multiview media types and
// the depth textures.
static constexpr absl::string_view kAuxiliaryTextureParameter =
    "mediaAuxiliaryTexture";
static constexpr absl::string_view kPrimaryDepthTextureParameter =
    "mediaPrimaryDepthTexture";
static constexpr absl::string_view kSecondaryDepthTextureParameter =
    "mediaSecondaryDepthTexture";

// Parameter used to pass the stereo type information to the material.
static constexpr absl::string_view kStereoTypeParameter = "stereoType";

// Boolean parameter to indicate if the material should perform color
// conversion.
static constexpr absl::string_view kEnableColorConversionParameter =
    "enableColorConversion";

// Boolean parameter to indicate if the material is being used for video.
static constexpr absl::string_view kIsVideoParameter = "isVideo";

// Matrix parameter for sRGB color transformation.
static constexpr absl::string_view kColorTransformMatrixSRGBParameter =
    "colorTransformMatrixSRGB";

// Matrix parameter for Display P3 color transformation.
static constexpr absl::string_view kColorTransformMatrixDisplayP3Parameter =
    "colorTransformMatrixDisplayP3";

// Integer parameter for the transfer function.
static constexpr absl::string_view kTransferFunctionParameter =
    "transferFunction";

// Integer parameter for the maximum content light level (MaxCLL) in nits.
static constexpr absl::string_view kMaxContentLightLevelParameter =
    "maxContentLightLevel";

// TODO: (broken link) - Investigate if we need to rescale the color to full
// range.
// // Boolean parameter to indicate if the video uses limited range (studio
// // range).
// static constexpr absl::string_view kIsVideoRangeLimitedParameter =
//     "isVideoRangeLimited";

// // Boolean parameter to indicate if the video is 10-bit.
// static constexpr absl::string_view kIsVideo10BitParameter = "isVideo10Bit";

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_XR_MEDIA_UTILS_H_
