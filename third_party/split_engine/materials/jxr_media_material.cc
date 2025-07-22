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

#include "split_engine/materials/jxr_media_material.h"

#include <sys/stat.h>

#include <cstddef>
#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/texture.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace android_xr {

namespace {

template <typename EnumA, typename EnumB>
constexpr bool DoEnumsMatch(EnumA enum_a, EnumB enum_b) {
  return static_cast<size_t>(enum_a) == static_cast<size_t>(enum_b);
}

// Verify imp::MediaStereoMode and android_xr::schemas::Texture3dStereoType
// enums match.
static_assert(
    DoEnumsMatch(
        imp::MediaStereoMode::kMonoscopic,
        android_xr::schemas::BuiltInMaterial1b616c8aStereoType::MONOSCOPIC),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(
        imp::MediaStereoMode::kTopBottom,
        android_xr::schemas::BuiltInMaterial1b616c8aStereoType::TOP_BOTTOM),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(
        imp::MediaStereoMode::kLeftRight,
        android_xr::schemas::BuiltInMaterial1b616c8aStereoType::LEFT_RIGHT),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(
        imp::MediaStereoMode::kStereoMesh,
        android_xr::schemas::BuiltInMaterial1b616c8aStereoType::STEREO_MESH),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(imp::MediaStereoMode::kInterleavedLeftPrimary,
                 android_xr::schemas::BuiltInMaterial1b616c8aStereoType::
                     INTERLEAVED_LEFT_PRIMARY),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(imp::MediaStereoMode::kInterleavedRightPrimary,
                 android_xr::schemas::BuiltInMaterial1b616c8aStereoType::
                     INTERLEAVED_RIGHT_PRIMARY),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(imp::MediaStereoMode::kInterleavedLeftPrimaryWithDepth,
                 android_xr::schemas::BuiltInMaterial1b616c8aStereoType::
                     INTERLEAVED_LEFT_PRIMARY_WITH_DEPTH),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(imp::MediaStereoMode::kInterleavedRightPrimaryWithDepth,
                 android_xr::schemas::BuiltInMaterial1b616c8aStereoType::
                     INTERLEAVED_RIGHT_PRIMARY_WITH_DEPTH),
    "Enum mismatch");

static_assert(android_xr::schemas::BuiltInMaterial1b616c8aStereoType::MAX ==
                  android_xr::schemas::BuiltInMaterial1b616c8aStereoType::
                      INTERLEAVED_RIGHT_PRIMARY_WITH_DEPTH,
              "New fields added but assert not updated");

// Verify imp::MediaColorSpace::Standard and
// android_xr::schemas::ColorStandard enums match.
static_assert(DoEnumsMatch(imp::MediaColorSpace::Standard::kUnknown,
                           android_xr::schemas::ColorStandard::UNSPECIFIED),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Standard::kBT709,
                           android_xr::schemas::ColorStandard::BT709),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Standard::kBT601_PAL,
                           android_xr::schemas::ColorStandard::BT601_PAL),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Standard::kBT2020,
                           android_xr::schemas::ColorStandard::BT2020),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Standard::kBT601_525,
                           android_xr::schemas::ColorStandard::BT601_525),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Standard::kDisplayP3,
                           android_xr::schemas::ColorStandard::DISPLAY_P3),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Standard::kDCI_P3,
                           android_xr::schemas::ColorStandard::DCI_P3),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Standard::kAdobeRGB,
                           android_xr::schemas::ColorStandard::ADOBE_RGB),
              "Enum mismatch");

static_assert(android_xr::schemas::ColorStandard::MAX ==
                  android_xr::schemas::ColorStandard::ADOBE_RGB,
              "New ColorStandard fields added but MAX assert not updated");

// Verify imp::MediaColorSpace::Transfer and
// android_xr::schemas::ColorTransfer enums match.
static_assert(DoEnumsMatch(imp::MediaColorSpace::Transfer::kUnknown,
                           android_xr::schemas::ColorTransfer::UNSPECIFIED),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Transfer::kLinear,
                           android_xr::schemas::ColorTransfer::LINEAR),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Transfer::kSRGB,
                           android_xr::schemas::ColorTransfer::SRGB),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Transfer::kSDR,
                           android_xr::schemas::ColorTransfer::SDR),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Transfer::kGamma_2_2,
                           android_xr::schemas::ColorTransfer::GAMMA_2_2),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Transfer::kST2084,
                           android_xr::schemas::ColorTransfer::ST2084),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Transfer::kHLG,
                           android_xr::schemas::ColorTransfer::HLG),
              "Enum mismatch");

static_assert(android_xr::schemas::ColorTransfer::MAX ==
                  android_xr::schemas::ColorTransfer::GAMMA_2_2,
              "New ColorTransfer fields added but MAX assert not updated");

// Verify imp::MediaColorSpace::Range and
// android_xr::schemas::ColorRange enums match.
static_assert(DoEnumsMatch(imp::MediaColorSpace::Range::kUnknown,
                           android_xr::schemas::ColorRange::UNSPECIFIED),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Range::kFull,
                           android_xr::schemas::ColorRange::FULL),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Range::kLimited,
                           android_xr::schemas::ColorRange::LIMITED),
              "Enum mismatch");
static_assert(DoEnumsMatch(imp::MediaColorSpace::Range::kExtended,
                           android_xr::schemas::ColorRange::EXTENDED),
              "Enum mismatch");

static_assert(android_xr::schemas::ColorRange::MAX ==
                  android_xr::schemas::ColorRange::EXTENDED,
              "New ColorRange fields added but MAX assert not updated");

// Verify imp::MediaShapeType and
// android_xr::schemas::BuiltInMaterial1b616c8aShapeType enums match.
static_assert(
    DoEnumsMatch(
        imp::MediaShapeType::kDefaultFlat,
        android_xr::schemas::BuiltInMaterial1b616c8aShapeType::DEFAULT_FLAT),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(imp::MediaShapeType::kVR180,
                 android_xr::schemas::BuiltInMaterial1b616c8aShapeType::VR180),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(
        imp::MediaShapeType::kFull360,
        android_xr::schemas::BuiltInMaterial1b616c8aShapeType::FULL360),
    "Enum mismatch");

static_assert(
    android_xr::schemas::BuiltInMaterial1b616c8aShapeType::MAX ==
        android_xr::schemas::BuiltInMaterial1b616c8aShapeType::FULL360,
    "New BuiltInMaterial1b616c8aShapeType fields added but MAX assert "
    "not updated");

}  // namespace

imp::Future<std::unique_ptr<JxrMediaMaterial>> JxrMediaMaterial::Create(
    imp::BaseView& view, imp::MediaShapeType shape_type,
    bool use_super_sampling) {
  // Verify the shape is supported.
  switch (shape_type) {
    case imp::MediaShapeType::kDefaultFlat:
    case imp::MediaShapeType::kVR180:
    case imp::MediaShapeType::kFull360:
      break;
    default:
      return imp::Future<
          std::unique_ptr<JxrMediaMaterial>>(absl::FailedPreconditionError(
          "JxrMediaViewer GetMaterialAssetUrl did not find a material for the "
          "state's given shape type."));
  }

  android_xr::schemas::Bool use_super_sampling_packed =
      imp::split_engine::Pack(use_super_sampling);
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterial1b616c8a>
      spec_offset = android_xr::schemas::CreateBuiltInMaterial1b616c8a(
          *fbb,
          static_cast<android_xr::schemas::BuiltInMaterial1b616c8aShapeType>(
              shape_type),
          &use_super_sampling_packed);
  return RequestBuiltInMaterial(
             view, std::move(fbb),
             android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial1b616c8a,
             spec_offset.Union())
      .Then(
          [&view](imp::split_engine::PlaceholderOrBuiltInMaterialPtr material) {
            return absl::WrapUnique(
                new JxrMediaMaterial(view, std::move(material)));
          });
}

JxrMediaMaterial::JxrMediaMaterial(
    imp::BaseView& view,
    imp::split_engine::PlaceholderOrBuiltInMaterialPtr material)
    : SplitEngineMaterial(view,
                          android_xr::schemas::BuiltInMaterialParameters::
                              BuiltInMaterial1b616c8aParameters,
                          std::move(material)) {}

JxrMediaMaterial::~JxrMediaMaterial() { Cleanup(); }

flatbuffers::Offset<void> JxrMediaMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      primary_texture;
  if (primary_texture_) {
    primary_texture =
        texture_parameter_creator.Create(fbb, primary_texture_.Borrow());
  }
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      auxiliary_texture;
  if (auxiliary_texture_) {
    auxiliary_texture =
        texture_parameter_creator.Create(fbb, auxiliary_texture_.Borrow());
  }

  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      primary_alpha_mask;
  if (primary_alpha_mask_) {
    primary_alpha_mask =
        texture_parameter_creator.Create(fbb, primary_alpha_mask_.Borrow());
  }
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      auxiliary_alpha_mask;
  if (auxiliary_alpha_mask_) {
    auxiliary_alpha_mask =
        texture_parameter_creator.Create(fbb, auxiliary_alpha_mask_.Borrow());
  }

  flatbuffers::Offset<
      android_xr::schemas::BuiltInMaterial1b616c8aMediaColorSpaceParameter>
      media_color_space_parameters = android_xr::schemas::
          CreateBuiltInMaterial1b616c8aMediaColorSpaceParameter(
              fbb,
              static_cast<android_xr::schemas::ColorStandard>(
                  color_space_.GetStandard()),
              static_cast<android_xr::schemas::ColorTransfer>(
                  color_space_.GetTransfer()),
              static_cast<android_xr::schemas::ColorRange>(
                  color_space_.GetRange()),
              color_space_.GetMaxContentLightLevel());

  return android_xr::schemas::CreateBuiltInMaterial1b616c8aParameters(
             fbb, primary_texture, auxiliary_texture,
             static_cast<
                 android_xr::schemas::BuiltInMaterial1b616c8aStereoType>(
                 stereo_type_),
             primary_alpha_mask, auxiliary_alpha_mask,
             imp::split_engine::PointerFromOptional(feather_radius_),
             media_color_space_parameters)
      .Union();
}

void JxrMediaMaterial::SetPrimaryTexture(
    imp::OwnedOrBorrowedTexturePtr texture) {
  primary_texture_ = std::move(texture);
  MarkParametersDirty();
}

void JxrMediaMaterial::SetAuxiliaryTexture(
    imp::OwnedOrBorrowedTexturePtr texture) {
  auxiliary_texture_ = std::move(texture);
  MarkParametersDirty();
}

void JxrMediaMaterial::SetPrimaryDepthTexture(
    imp::OwnedOrBorrowedTexturePtr texture) {
  LOG(FATAL) << "Depth texture is unsupported on built-in material";
}
void JxrMediaMaterial::SetAuxiliaryDepthTexture(
    imp::OwnedOrBorrowedTexturePtr texture) {
  LOG(FATAL) << "Depth texture is unsupported on built-in material";
}

void JxrMediaMaterial::SetStereoType(imp::MediaStereoMode stereo_type) {
  stereo_type_ = stereo_type;
  MarkParametersDirty();
}
void JxrMediaMaterial::SetPrimaryAlphaMask(
    imp::OwnedOrBorrowedTexturePtr alpha_mask) {
  primary_alpha_mask_ = std::move(alpha_mask);
  MarkParametersDirty();
}

void JxrMediaMaterial::SetAuxiliaryAlphaMask(
    imp::OwnedOrBorrowedTexturePtr auxiliary_alpha_mask) {
  auxiliary_alpha_mask_ = std::move(auxiliary_alpha_mask);
  MarkParametersDirty();
}

void JxrMediaMaterial::SetFeatherRadius(imp::float2 feather_radius) {
  feather_radius_ = imp::split_engine::Pack(feather_radius);
  MarkParametersDirty();
}

void JxrMediaMaterial::SetContentColorMetadata(
    imp::MediaColorSpace color_space) {
  color_space_ = color_space;
  MarkParametersDirty();
}

}  // namespace android_xr
