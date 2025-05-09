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
#include "core/media/media_type.h"
#include "core/render/texture.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

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

}  // namespace

imp::Future<std::unique_ptr<JxrMediaMaterial>> JxrMediaMaterial::Create(
    imp::BaseView& view,
    android_xr::schemas::BuiltInMaterial1b616c8aShapeType shape_type) {
  // Verify the shape is supported.
  switch (shape_type) {
    case android_xr::schemas::BuiltInMaterial1b616c8aShapeType::FULL360:
    case android_xr::schemas::BuiltInMaterial1b616c8aShapeType::VR180:
    case android_xr::schemas::BuiltInMaterial1b616c8aShapeType::DEFAULT_FLAT:
      break;
    default:
      return imp::Future<
          std::unique_ptr<JxrMediaMaterial>>(absl::FailedPreconditionError(
          "JxrMediaViewer GetMaterialAssetUrl did not find a material for the "
          "state's given shape type."));
  }

  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterial1b616c8a>
      spec_offset = android_xr::schemas::CreateBuiltInMaterial1b616c8a(
          *fbb,
          static_cast<android_xr::schemas::BuiltInMaterial1b616c8aShapeType>(
              shape_type));
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

  return android_xr::schemas::CreateBuiltInMaterial1b616c8aParameters(
             fbb, primary_texture, auxiliary_texture,
             static_cast<
                 android_xr::schemas::BuiltInMaterial1b616c8aStereoType>(
                 stereo_type_),
             primary_alpha_mask, auxiliary_alpha_mask,
             imp::split_engine::PointerFromOptional(feather_radius_))
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

}  // namespace android_xr
