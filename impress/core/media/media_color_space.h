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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_COLOR_SPACE_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_COLOR_SPACE_H_

#include <cstdint>
#include <string>

#include "absl/status/statusor.h"
#include "core/config.h"
#include "core/math/mat.h"

#if IMP_PLATFORM(ANDROID)
#include <android/data_space.h>
#endif  // IMP_PLATFORM(ANDROID)

namespace imp {

class MediaColorSpace {
 public:
  // Color space of a media asset. The enum values are defined to match
  // the values returned by media3. Please see here for more information:
  // third_party/java_src/android_libs/media/libraries/common/src/main/java/androidx/media3/common/C.java
  // For the enum values, please see here:
  // (broken link)
  // TODO: zakerinasab - Apparently Exoplayer doesn't support BT601-525 and
  // Display-P3. Figure out if this is important.
  enum class Standard : int {
    kUnknown = 0,
    kBT709 = 1,
    kBT601_PAL = 2,
    kBT2020 = 6,
    // Additional standard values not supported by Exoplayer.
    kBT601_525 = 0xf0,
    kDisplayP3 = 0xf1,
    kDCI_P3 = 0xf2,
    kAdobeRGB = 0xf3,
  };

  // Color transfer of a media asset. The enum values are defined to match
  // the values returned by media3. Please see here for more information:
  // third_party/java_src/android_libs/media/libraries/common/src/main/java/androidx/media3/common/C.java
  // For the enum values (except sRGB and Gamma 2.2), please see here:
  // (broken link)
  enum class Transfer : int {
    kUnknown = 0,
    kLinear = 1,
    kSRGB = 2,
    kSDR = 3,  // SMPTE170M
    kGamma_2_2 = 10,
    kST2084 = 6,
    kHLG = 7,
  };

  // Color range of a media asset. The enum values are defined to match
  // the values returned by media3. Please see here for more information:
  // third_party/java_src/android_libs/media/libraries/common/src/main/java/androidx/media3/common/C.java
  // For the enum values, please see here:
  // (broken link)
  enum class Range : int {
    kUnknown = 0,
    kFull = 1,
    kLimited = 2,
    kExtended = 3,
  };

  static constexpr uint16_t kMaxContentLightLevelUnknown = 0;

  template <typename Sink>
  void AbslStringify(Sink& sink, Standard e) {
    switch (e) {
      case Standard::kUnknown:
        sink.Append("kUnknown");
        break;
      case Standard::kBT709:
        sink.Append("kBT709");
        break;
      case Standard::kBT601_PAL:
        sink.Append("kBT601_PAL");
        break;
      case Standard::kBT2020:
        sink.Append("kBT2020");
        break;
      case Standard::kBT601_525:
        sink.Append("kBT601_525");
        break;
      case Standard::kDisplayP3:
        sink.Append("kDisplayP3");
        break;
      case Standard::kDCI_P3:
        sink.Append("kDCI_P3");
        break;
      case Standard::kAdobeRGB:
        sink.Append("kAdobeRGB");
        break;
    }
  }

  template <typename Sink>
  void AbslStringify(Sink& sink, Transfer e) {
    switch (e) {
      case Transfer::kUnknown:
        sink.Append("kUnknown");
        break;
      case Transfer::kLinear:
        sink.Append("kLinear");
        break;
      case Transfer::kSRGB:
        sink.Append("kSRGB");
        break;
      case Transfer::kSDR:
        sink.Append("kSDR");
        break;
      case Transfer::kGamma_2_2:
        sink.Append("kGamma_2_2");
        break;
      case Transfer::kST2084:
        sink.Append("kST2084");
        break;
      case Transfer::kHLG:
        sink.Append("kHLG");
        break;
    }
  }

  template <typename Sink>
  void AbslStringify(Sink& sink, Range e) {
    switch (e) {
      case Range::kUnknown:
        sink.Append("kUnknown");
        break;
      case Range::kFull:
        sink.Append("kFull");
        break;
      case Range::kLimited:
        sink.Append("kLimited");
        break;
      case Range::kExtended:
        sink.Append("kExtended");
        break;
    }
  }

  static absl::StatusOr<Standard> ToColorStandard(int color_standard);
  static absl::StatusOr<Transfer> ToColorTransfer(int color_transfer);
  static absl::StatusOr<Range> ToColorRange(int color_range);
  static absl::StatusOr<uint16_t> ToMaxContentLightLevel(
      int max_content_light_level);

  // Matrices for converting between various colorspaces. Determined by
  // applying the steps from SMPTE RP 177 to the chromaticity coordinates
  // of the primaries and white point of the supported colorspaces.
  // Source:
  // (broken link)
  // clang-format off
  constexpr static mat3f kXYZFromBT601_525{
           0.3935209037, 0.2123763607, 0.0187390907,
           0.3652580767, 0.7010598569, 0.1119339267,
           0.1916769467, 0.0865637824, 0.9583847334};

  constexpr static mat3f kXYZFromBT601_625{
           0.4305538133, 0.22200431,   0.02018221,
           0.3415498035, 0.7066547659, 0.1295533738,
           0.1783523102, 0.0713409241, 0.939322167};

  constexpr static mat3f kXYZFromBT709{
           0.4123907993, 0.2126390059, 0.0193308187,
           0.3575843394, 0.7151686788, 0.1191947798,
           0.1804807884, 0.0721923154, 0.9505321522};

  constexpr static mat3f kXYZFromDisplayP3{
           0.4865709486, 0.2289745641, 0.0,
           0.2656676932, 0.6917385218, 0.0451133819,
           0.1982172852, 0.0792869141, 1.0439443689};

  constexpr static mat3f kXYZFromDCI_P3{
           0.4865709486, 0.2289745641, 0.0,
           0.2656676932, 0.6917385218, 0.0451133819,
           0.1982172852, 0.0792869141, 1.0439443689};

  constexpr static mat3f kXYZFromAdobeRGB{
           0.57667, 0.18556, 0.18823,
           0.29734, 0.62736, 0.07529,
           0.02703, 0.07069, 0.99134};

  constexpr static mat3f kXYZFromBT2020{
           0.6369580483, 0.262700212,  0.0,
           0.1446169036, 0.6779980715, 0.028072693,
           0.1688809752, 0.0593017165, 1.0609850577};

  constexpr static mat3f kBT709FromXYZ{
           3.2409699419,  -0.9692436363, 0.0556300797,
           -1.5373831776, 1.8759675015,  -0.2039769589,
           -0.4986107603, 0.0415550574,  1.0569715142};

  constexpr static mat3f kDisplayP3FromXYZ{
           2.4934969119,  -0.8294889696, 0.0358458302,
           -0.9313836179, 1.7626640603,  -0.0761723893,
           -0.4027107845, 0.0236246858,  0.9568845240};
  // clang-format on

  MediaColorSpace() = default;
  MediaColorSpace(
      Standard standard, Transfer transfer, Range range,
      uint16_t max_content_light_level = kMaxContentLightLevelUnknown);

#if IMP_PLATFORM(ANDROID)
  // Android Hardware Abstraction Layer (HAL) dataspace values.
  // These values originate from the Android system's graphics framework,
  // specifically:
  // - platform/system/core/libsystem/include/system/graphics-base-v1.0.h
  // - platform/system/core/libsystem/include/system/graphics-base-v1.1.h
  // - platform/system/core/libsystem/include/system/graphics-base-v1.2.h
  // While ADataSpace constants are the preferred representation,
  // SurfaceTexture::getDataSpace() can return HAL dataspace values that are not
  // fully covered by ADataSpace. This enum provides a comprehensive mapping.
  enum class HalDataspace : int32_t {
    kUnknown = 0,
    kArbitrary = 1,
    kStandardShift = 16,
    kStandardMask = 4128768,                     // (63 << STANDARD_SHIFT)
    kStandardUnspecified = 0,                    // (0 << STANDARD_SHIFT)
    kStandardBT709 = 65536,                      // (1 << STANDARD_SHIFT)
    kStandardBT601_625 = 131072,                 // (2 << STANDARD_SHIFT)
    kStandardBT601_625_Unadjusted = 196608,      // (3 << STANDARD_SHIFT)
    kStandardBT601_525 = 262144,                 // (4 << STANDARD_SHIFT)
    kStandardBT601_525_Unadjusted = 327680,      // (5 << STANDARD_SHIFT)
    kStandardBT2020 = 393216,                    // (6 << STANDARD_SHIFT)
    kStandardBT2020_ConstantLuminance = 458752,  // (7 << STANDARD_SHIFT)
    kStandardBT470M = 524288,                    // (8 << STANDARD_SHIFT)
    kStandardFilm = 589824,                      // (9 << STANDARD_SHIFT)
    kStandardDCI_P3 = 655360,                    // (10 << STANDARD_SHIFT)
    kStandardADOBE_RGB = 720896,                 // (11 << STANDARD_SHIFT)
    kTransferShift = 22,
    kTransferMask = 130023424,       // (31 << TRANSFER_SHIFT)
    kTransferUnspecified = 0,        // (0 << TRANSFER_SHIFT)
    kTransferLinear = 4194304,       // (1 << TRANSFER_SHIFT)
    kTransferSRGB = 8388608,         // (2 << TRANSFER_SHIFT)
    kTransferSMPTE_170M = 12582912,  // (3 << TRANSFER_SHIFT)
    kTransferGamma2_2 = 16777216,    // (4 << TRANSFER_SHIFT)
    kTransferGamma2_6 = 20971520,    // (5 << TRANSFER_SHIFT)
    kTransferGamma2_8 = 25165824,    // (6 << TRANSFER_SHIFT)
    kTransferST2084 = 29360128,      // (7 << TRANSFER_SHIFT)
    kTransferHLG = 33554432,         // (8 << TRANSFER_SHIFT)
    kRangeShift = 27,
    kRangeMask = 939524096,      // (7 << RANGE_SHIFT)
    kRangeUnspecified = 0,       // (0 << RANGE_SHIFT)
    kRangeFull = 134217728,      // (1 << RANGE_SHIFT)
    kRangeLimited = 268435456,   // (2 << RANGE_SHIFT)
    kRangeExtended = 402653184,  // (3 << RANGE_SHIFT)
    kSRGB_Linear = 512,
    kV0_SRGB_Linear =
        138477568,  // ((STANDARD_BT709 | TRANSFER_LINEAR) | RANGE_FULL)
    kV0_SCRGB_Linear =
        406913024,  // ((STANDARD_BT709 | TRANSFER_LINEAR) | RANGE_EXTENDED)
    kSRGB = 513,
    kV0_SRGB = 142671872,  // ((STANDARD_BT709 | TRANSFER_SRGB) | RANGE_FULL)
    kV0_SCRGB =
        411107328,  // ((STANDARD_BT709 | TRANSFER_SRGB) | RANGE_EXTENDED)
    kJFIF = 257,
    kV0_JFIF =
        146931712,  // ((STANDARD_BT601_625 | TRANSFER_SMPTE_170M) | RANGE_FULL)
    kBT601_625 = 258,
    kV0_BT601_625 = 281149440,  // ((STANDARD_BT601_625 | TRANSFER_SMPTE_170M) |
                                // RANGE_LIMITED)
    kBT601_525 = 259,
    kV0_BT601_525 = 281280512,  // ((STANDARD_BT601_525 | TRANSFER_SMPTE_170M) |
                                // RANGE_LIMITED)
    kBT709 = 260,
    kV0_BT709 =
        281083904,  // ((STANDARD_BT709 | TRANSFER_SMPTE_170M) | RANGE_LIMITED)
    kDCI_P3_Linear =
        139067392,  // ((STANDARD_DCI_P3 | TRANSFER_LINEAR) | RANGE_FULL)
    kDCI_P3 =
        155844608,  // ((STANDARD_DCI_P3 | TRANSFER_GAMMA2_6) | RANGE_FULL)
    kDisplayP3_Linear =
        139067392,  // ((STANDARD_DCI_P3 | TRANSFER_LINEAR) | RANGE_FULL)
    kDisplayP3 = 143261696,  // ((STANDARD_DCI_P3 | TRANSFER_SRGB) | RANGE_FULL)
    kAdobeRGB =
        151715840,  // ((STANDARD_ADOBE_RGB | TRANSFER_GAMMA2_2) | RANGE_FULL)
    kBT2020_Linear =
        138805248,  // ((STANDARD_BT2020 | TRANSFER_LINEAR) | RANGE_FULL)
    kBT2020 =
        147193856,  // ((STANDARD_BT2020 | TRANSFER_SMPTE_170M) | RANGE_FULL)
    kBT2020_PQ =
        163971072,  // ((STANDARD_BT2020 | TRANSFER_ST2084) | RANGE_FULL)
    kDepth = 4096,
    kSensor = 4097,
    kBT2020_ITU =
        281411584,  // ((STANDARD_BT2020 | TRANSFER_SMPTE_170M) | RANGE_LIMITED)
    kBT2020_ITU_PQ =
        298188800,  // ((STANDARD_BT2020 | TRANSFER_ST2084) | RANGE_LIMITED)
    kBT2020_ITU_HLG =
        302383104,  // ((STANDARD_BT2020 | TRANSFER_HLG) | RANGE_LIMITED)
    kBT2020_HLG = 168165376,  // ((STANDARD_BT2020 | TRANSFER_HLG) | RANGE_FULL)
    kDisplayBT2020 =
        142999552 /* ((STANDARD_BT2020 | TRANSFER_SRGB) | RANGE_FULL) */,
    kDynamicDepth = 4098 /* 0x1002 */,
    kJPEG_APP_SEGMENTS = 4099 /* 0x1003 */,
    kHEIF = 4100 /* 0x1004 */
  };

  MediaColorSpace(int32_t dataspace);
  MediaColorSpace(ADataSpace dataspace);
  MediaColorSpace(HalDataspace dataspace);
#endif  // IMP_PLATFORM(ANDROID)

  void SetStandard(Standard standard);
  void SetTransfer(Transfer transfer);
  void SetRange(Range range);
  void SetLumaBitDepth(int luma_bitdepth);
  void SetChromaBitDepth(int chroma_bitdepth);
  void SetMaxContentLightLevel(uint16_t max_content_light_level);

  Standard GetStandard() const;
  Transfer GetTransfer() const;
  Range GetRange() const;
  int GetLumaBitDepth() const;
  int GetChromaBitDepth() const;
  uint16_t GetMaxContentLightLevel() const;

  absl::StatusOr<mat3f> GetColorTransformMatrixSRGB();
  absl::StatusOr<mat3f> GetColorTransformMatrixDisplayP3();

  // Returns true if all the color space parameters (color standard, transfer
  // and range) are set to a known value.
  bool IsKnown() const;

  // Returns true if all the color space parameters (color standard, transfer
  // and range) are set to an unknown value.
  bool IsUnknown() const;

  bool operator==(const MediaColorSpace& other) const;

  std::string ToString() const;

 private:
  Standard standard_ = Standard::kUnknown;
  Transfer transfer_ = Transfer::kUnknown;
  Range range_ = Range::kUnknown;
  // Default to 8 bits for unset luma bitdepth (SDR/BT.709).
  int32_t luma_bitdepth_ = 8;
  // Default to 8 bits for unset chroma bitdepth (SDR/BT.709).
  int32_t chroma_bitdepth_ = 8;
  // Default to 0 for unset max content light level.
  uint16_t max_content_light_level_ = kMaxContentLightLevelUnknown;

  // Supported display color spaces for color conversion.
  enum class DisplayColorSpace : int {
    kSRGB = 1,
    kDisplayP3 = 2,
  };

  absl::StatusOr<mat3f> GetColorTransformMatrix(
      DisplayColorSpace display_color_space);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_COLOR_SPACE_H_
