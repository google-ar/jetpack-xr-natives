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

#include "core/video/video_color_space.h"

#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/config.h"
#include "core/math/mat.h"

#if IMP_PLATFORM(ANDROID)
#include <android/data_space.h>
#endif  // IMP_PLATFORM(ANDROID)

namespace imp::video {

VideoColorSpace::VideoColorSpace(Standard standard, Transfer transfer,
                                 Range range)
    : standard_(standard), transfer_(transfer), range_(range) {}

#if IMP_PLATFORM(ANDROID)
VideoColorSpace::VideoColorSpace(int32_t dataspace)
    : VideoColorSpace::VideoColorSpace(static_cast<HalDataspace>(dataspace)) {}

VideoColorSpace::VideoColorSpace(ADataSpace dataspace)
    : VideoColorSpace::VideoColorSpace(static_cast<HalDataspace>(dataspace)) {}

VideoColorSpace::VideoColorSpace(VideoColorSpace::HalDataspace dataspace) {
  // Handle full color spaces
  switch (dataspace) {
    case HalDataspace::kUnknown:
    case HalDataspace::kArbitrary:
      standard_ = Standard::kUnknown;
      transfer_ = Transfer::kUnknown;
      range_ = Range::kUnknown;
      return;

    case HalDataspace::kAdobeRGB:
      standard_ = Standard::kAdobeRGB;
      transfer_ = Transfer::kGamma_2_2;
      range_ = Range::kFull;
      return;

    case HalDataspace::kBT2020:
      standard_ = Standard::kBT2020;
      transfer_ = Transfer::kSDR;
      range_ = Range::kFull;
      return;

    case HalDataspace::kBT2020_HLG:
      standard_ = Standard::kBT2020;
      transfer_ = Transfer::kHLG;
      range_ = Range::kFull;
      return;

    case HalDataspace::kBT2020_ITU_HLG:
      standard_ = Standard::kBT2020;
      transfer_ = Transfer::kHLG;
      range_ = Range::kLimited;
      return;

    case HalDataspace::kBT2020_PQ:
      standard_ = Standard::kBT2020;
      transfer_ = Transfer::kST2084;
      range_ = Range::kFull;
      return;

    case HalDataspace::kBT2020_ITU_PQ:
      standard_ = Standard::kBT2020;
      transfer_ = Transfer::kST2084;
      range_ = Range::kLimited;
      return;

    case HalDataspace::kBT601_525:
    case HalDataspace::kV0_BT601_525:
      standard_ = Standard::kBT601_525;
      transfer_ = Transfer::kSDR;
      range_ = Range::kLimited;
      return;

    case HalDataspace::kBT601_625:
    case HalDataspace::kV0_BT601_625:
      standard_ = Standard::kBT601_PAL;
      transfer_ = Transfer::kSDR;
      range_ = Range::kLimited;
      return;

    case HalDataspace::kBT709:
    case HalDataspace::kV0_BT709:
      standard_ = Standard::kBT709;
      transfer_ = Transfer::kSDR;
      range_ = Range::kLimited;
      return;

    case HalDataspace::kDCI_P3:
      standard_ = Standard::kDCI_P3;
      transfer_ = Transfer::kGamma_2_2;  // Approximation
      range_ = Range::kFull;
      return;

    case HalDataspace::kDisplayP3:
      standard_ = Standard::kDisplayP3;
      transfer_ = Transfer::kSRGB;
      range_ = Range::kFull;
      return;

    case HalDataspace::kDisplayP3_Linear:
      standard_ = Standard::kDisplayP3;
      transfer_ = Transfer::kLinear;
      range_ = Range::kFull;
      return;

    case HalDataspace::kDisplayBT2020:
      standard_ = Standard::kBT2020;
      transfer_ = Transfer::kSRGB;
      range_ = Range::kFull;
      return;

    case HalDataspace::kJFIF:
    case HalDataspace::kV0_JFIF:
      standard_ = Standard::kBT601_525;
      transfer_ = Transfer::kSDR;
      range_ = Range::kFull;
      return;

    case HalDataspace::kSRGB:
    case HalDataspace::kV0_SRGB:
      standard_ = Standard::kBT709;
      transfer_ = Transfer::kSRGB;
      range_ = Range::kFull;
      return;

    case HalDataspace::kSRGB_Linear:
    case HalDataspace::kV0_SRGB_Linear:
      standard_ = Standard::kBT709;
      transfer_ = Transfer::kLinear;
      range_ = Range::kFull;
      return;

    case HalDataspace::kV0_SCRGB:
      standard_ = Standard::kBT709;
      transfer_ = Transfer::kSRGB;
      range_ = Range::kExtended;
      return;

    case HalDataspace::kV0_SCRGB_Linear:
      standard_ = Standard::kBT709;
      transfer_ = Transfer::kLinear;
      range_ = Range::kExtended;
      return;

    case HalDataspace::kBT2020_Linear:
      standard_ = Standard::kBT2020;
      transfer_ = Transfer::kLinear;
      range_ = Range::kFull;
      return;

    case HalDataspace::kBT2020_ITU:
      standard_ = Standard::kBT2020;
      transfer_ = Transfer::kSDR;
      range_ = Range::kLimited;
      return;

    case HalDataspace::kDepth:
      standard_ = Standard::kUnknown;
      transfer_ = Transfer::kUnknown;
      range_ = Range::kUnknown;
      return;

    case HalDataspace::kSensor:
      standard_ = Standard::kUnknown;
      transfer_ = Transfer::kUnknown;
      range_ = Range::kUnknown;
      return;

    case HalDataspace::kDynamicDepth:
    case HalDataspace::kJPEG_APP_SEGMENTS:
    case HalDataspace::kHEIF:
      standard_ = Standard::kUnknown;
      transfer_ = Transfer::kUnknown;
      range_ = Range::kUnknown;
      return;

    default:
      break;
  }

  // Extract individual components
  int32_t standard = (static_cast<int32_t>(dataspace) &
                      static_cast<int32_t>(HalDataspace::kStandardMask)) >>
                     static_cast<int32_t>(HalDataspace::kStandardShift);
  int32_t transfer = (static_cast<int32_t>(dataspace) &
                      static_cast<int32_t>(HalDataspace::kTransferMask)) >>
                     static_cast<int32_t>(HalDataspace::kTransferShift);
  int32_t range = (static_cast<int32_t>(dataspace) &
                   static_cast<int32_t>(HalDataspace::kRangeMask)) >>
                  static_cast<int32_t>(HalDataspace::kRangeShift);

  switch (standard) {
    case static_cast<int32_t>(HalDataspace::kStandardBT709) >>
        static_cast<int32_t>(HalDataspace::kStandardShift):
      standard_ = Standard::kBT709;
      break;
    case static_cast<int32_t>(HalDataspace::kStandardBT601_625) >>
        static_cast<int32_t>(HalDataspace::kStandardShift):
      standard_ = Standard::kBT601_PAL;
      break;
    case static_cast<int32_t>(HalDataspace::kStandardBT601_525) >>
        static_cast<int32_t>(HalDataspace::kStandardShift):
      standard_ = Standard::kBT601_525;
      break;
    case static_cast<int32_t>(HalDataspace::kStandardBT2020) >>
        static_cast<int32_t>(HalDataspace::kStandardShift):
      standard_ = Standard::kBT2020;
      break;
    case static_cast<int32_t>(HalDataspace::kStandardDCI_P3) >>
        static_cast<int32_t>(HalDataspace::kStandardShift):
      standard_ = Standard::kDCI_P3;
      break;
    case static_cast<int32_t>(HalDataspace::kStandardADOBE_RGB) >>
        static_cast<int32_t>(HalDataspace::kStandardShift):
      standard_ = Standard::kAdobeRGB;
      break;
    default:
      standard_ = Standard::kUnknown;
      break;
  }

  switch (transfer) {
    case static_cast<int32_t>(HalDataspace::kTransferLinear) >>
        static_cast<int32_t>(HalDataspace::kTransferShift):
      transfer_ = Transfer::kLinear;
      break;
    case static_cast<int32_t>(HalDataspace::kTransferSRGB) >>
        static_cast<int32_t>(HalDataspace::kTransferShift):
      transfer_ = Transfer::kSRGB;
      break;
    case static_cast<int32_t>(HalDataspace::kTransferSMPTE_170M) >>
        static_cast<int32_t>(HalDataspace::kTransferShift):
      transfer_ = Transfer::kSDR;
      break;
    case static_cast<int32_t>(HalDataspace::kTransferGamma2_2) >>
        static_cast<int32_t>(HalDataspace::kTransferShift):
      transfer_ = Transfer::kGamma_2_2;
      break;
    case static_cast<int32_t>(HalDataspace::kTransferST2084) >>
        static_cast<int32_t>(HalDataspace::kTransferShift):
      transfer_ = Transfer::kST2084;
      break;
    case static_cast<int32_t>(HalDataspace::kTransferHLG) >>
        static_cast<int32_t>(HalDataspace::kTransferShift):
      transfer_ = Transfer::kHLG;
      break;
    case static_cast<int32_t>(HalDataspace::kTransferUnspecified) >>
        static_cast<int32_t>(HalDataspace::kTransferShift):
    default:
      transfer_ = Transfer::kUnknown;
      break;
  }

  switch (range) {
    case static_cast<int32_t>(HalDataspace::kRangeFull) >>
        static_cast<int32_t>(HalDataspace::kRangeShift):
      range_ = Range::kFull;
      break;
    case static_cast<int32_t>(HalDataspace::kRangeLimited) >>
        static_cast<int32_t>(HalDataspace::kRangeShift):
      range_ = Range::kLimited;
      break;
    case static_cast<int32_t>(HalDataspace::kRangeExtended) >>
        static_cast<int32_t>(HalDataspace::kRangeShift):
      range_ = Range::kExtended;
      break;
    default:
      range_ = Range::kUnknown;
      break;
  }
}

#endif  // IMP_PLATFORM(ANDROID)

void VideoColorSpace::SetStandard(Standard standard) { standard_ = standard; }

void VideoColorSpace::SetTransfer(Transfer transfer) { transfer_ = transfer; }

void VideoColorSpace::SetRange(Range range) { range_ = range; }

void VideoColorSpace::SetLumaBitDepth(int luma_bitdepth) {
  luma_bitdepth_ = luma_bitdepth;
}

void VideoColorSpace::SetChromaBitDepth(int chroma_bitdepth) {
  chroma_bitdepth_ = chroma_bitdepth;
}

void VideoColorSpace::SetMaxContentLightLevel(float content_light_level) {
  max_content_light_level_ = content_light_level;
}

VideoColorSpace::Standard VideoColorSpace::GetStandard() const {
  return standard_;
}

VideoColorSpace::Transfer VideoColorSpace::GetTransfer() const {
  return transfer_;
}

VideoColorSpace::Range VideoColorSpace::GetRange() const { return range_; }

int VideoColorSpace::GetLumaBitDepth() const { return luma_bitdepth_; }

int VideoColorSpace::GetChromaBitDepth() const { return chroma_bitdepth_; }

float VideoColorSpace::GetMaxContentLightLevel() const {
  return max_content_light_level_;
}

absl::StatusOr<mat3f> VideoColorSpace::GetColorTransformMatrixSRGB() {
  return GetColorTransformMatrix(VideoColorSpace::DisplayColorSpace::kSRGB);
}

absl::StatusOr<mat3f> VideoColorSpace::GetColorTransformMatrixDisplayP3() {
  return GetColorTransformMatrix(
      VideoColorSpace::DisplayColorSpace::kDisplayP3);
}

absl::StatusOr<mat3f> VideoColorSpace::GetColorTransformMatrix(
    VideoColorSpace::DisplayColorSpace display_color_space) {
  // If display color space is sRGB and the standard is BT709, then the color
  // transform matrix is the identity matrix.
  if (display_color_space == VideoColorSpace::DisplayColorSpace::kSRGB &&
      standard_ == VideoColorSpace::Standard::kBT709) {
    return imp::kIdentityMat3f;
  }

  mat3f display_to_xyz_matrix =
      (display_color_space == VideoColorSpace::DisplayColorSpace::kDisplayP3)
          ? kDisplayP3FromXYZ
          : kBT709FromXYZ;

  switch (standard_) {
    case VideoColorSpace::Standard::kBT709:
      return display_to_xyz_matrix * kXYZFromBT709;
    case VideoColorSpace::Standard::kBT601_PAL:
      return display_to_xyz_matrix * kXYZFromBT601_625;
    case VideoColorSpace::Standard::kBT2020:
      return display_to_xyz_matrix * kXYZFromBT2020;
    case VideoColorSpace::Standard::kBT601_525:
      return display_to_xyz_matrix * kXYZFromBT601_525;
    case VideoColorSpace::Standard::kDisplayP3:
      return display_to_xyz_matrix * kXYZFromDisplayP3;
    case VideoColorSpace::Standard::kDCI_P3:
      return display_to_xyz_matrix * kXYZFromDCI_P3;
    case VideoColorSpace::Standard::kAdobeRGB:
      return display_to_xyz_matrix * kXYZFromAdobeRGB;
    default:
      return absl::InvalidArgumentError("Unsupported standard.");
  }
}

std::string VideoColorSpace::ToString() const {
  return absl::StrFormat(
      "VideoColorSpace {standard: %v, transfer: %v, range: %v, luma_bitdepth: "
      "%d, chroma_bitdepth: %d, max_content_light_level: %f}",
      standard_, transfer_, range_, luma_bitdepth_, chroma_bitdepth_,
      max_content_light_level_);
}

bool VideoColorSpace::operator==(const VideoColorSpace& other) const {
  return standard_ == other.standard_ && transfer_ == other.transfer_ &&
         range_ == other.range_ && luma_bitdepth_ == other.luma_bitdepth_ &&
         chroma_bitdepth_ == other.chroma_bitdepth_ &&
         max_content_light_level_ == other.max_content_light_level_;
}

}  // namespace imp::video
