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

#include "core/split_engine/materials/builtin/builtin_custom_material.h"

#include <sys/types.h>

#include <functional>
#include <optional>
#include <utility>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/common/registry.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/media/media_color_space.h"
#include "core/render/display_color_space.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/split_engine_external_texture_color_space_store.h"
#include "core/view/base_view.h"

namespace imp::split_engine {
namespace {
constexpr absl::string_view kColorConversionMatrixParameter =
    "colorConversionMatrix";
constexpr absl::string_view kTransferFunctionParameter = "transferFunction";
constexpr absl::string_view kMaxContentLightLevelParameter =
    "maxContentLightLevel";
}  // namespace

BuiltInCustomMaterial::BuiltInCustomMaterial(BridgeId bridge_id,
                                             OwnedMaterialPtr material)
    : bridge_id_(bridge_id), material_(std::move(material)) {}

BridgeId BuiltInCustomMaterial::GetBridgeId() const { return bridge_id_; }

BorrowedMaterialPtr BuiltInCustomMaterial::GetMaterialInternal(
    SmallSourceLocation loc) const {
  return material_.Borrow(loc);
}

void BuiltInCustomMaterial::UpdateColorSpaceParameters(
    BaseView& view, std::optional<TextureId> texture_id) {
  // Start with the default color space parameters.
  imp::mat3f color_transform_matrix =
      default_color_space_.GetColorTransformMatrixDisplayP3().value_or(
          imp::kIdentityMat3f);
  int transfer_function = static_cast<int>(default_color_space_.GetTransfer());
  int max_content_light_level = default_color_space_.GetMaxContentLightLevel();

  // SplitEngineExternalTextureColorSpaceStore has the information about the
  // color space of the surface texture.
  absl::StatusOr<
      std::reference_wrapper<SplitEngineExternalTextureColorSpaceStore>>
      color_space_manager =
          view.GetRegistry().Get<SplitEngineExternalTextureColorSpaceStore>();
  if (!color_space_manager.ok()) {
    // Fall back to the sRGB color transform matrix if
    // `SplitEngineExternalTextureColorSpaceStore` is not registered by the
    // renderer (e.g., in local mode).
    color_transform_matrix =
        default_color_space_.GetColorTransformMatrixSRGB().value_or(
            imp::kIdentityMat3f);
  }

  if (color_space_manager.ok() && texture_id.has_value()) {
    absl::StatusOr<MediaColorSpace> source_texture_color_space =
        color_space_manager->get().GetTextureColorSpace(GetBridgeId(),
                                                        *texture_id);
    if (source_texture_color_space.ok() &&
        source_texture_color_space->GetStandard() !=
            MediaColorSpace::Standard::kUnknown) {
      color_transform_matrix = source_texture_color_space.value()
                                   .GetColorTransformMatrixDisplayP3()
                                   .value_or(imp::kIdentityMat3f);
      transfer_function =
          static_cast<int>(source_texture_color_space.value().GetTransfer());
      max_content_light_level =
          source_texture_color_space.value().GetMaxContentLightLevel();
    }
  }

#if IMP_PLATFORM(ANDROID)
  if (transfer_function == static_cast<int>(MediaColorSpace::Transfer::kSDR)) {
    // The default behavior on the vast majority of Android devices is to use
    // SRGB instead of SMPTE170M for SurfaceView composition.
    // See:
    // (broken link)/
    transfer_function = static_cast<int>(MediaColorSpace::Transfer::kSRGB);
  }
#endif

  GetMaterial()->SetParameter(kColorConversionMatrixParameter,
                              color_transform_matrix);
  GetMaterial()->SetParameter(kTransferFunctionParameter, transfer_function);
  GetMaterial()->SetParameter(kMaxContentLightLevelParameter,
                              max_content_light_level);
}

void BuiltInCustomMaterial::OverrideColorSpaceParameters(
    BaseView& view, MediaColorSpace color_space) {
  imp::mat3f color_transform_matrix;

  switch (GetRequiredDisplayColorSpace()) {
    case DisplayColorSpace::kBT709:
      color_transform_matrix =
          color_space.GetColorTransformMatrixSRGB().value_or(
              imp::kIdentityMat3f);
      break;
    case DisplayColorSpace::kP3:
      color_transform_matrix =
          color_space.GetColorTransformMatrixDisplayP3().value_or(
              imp::kIdentityMat3f);
      break;
    default:
      IMP_LOG(imp::ERROR) << "Unsupported display color space. Falling back to SRGB.";
      color_transform_matrix =
          color_space.GetColorTransformMatrixSRGB().value_or(
              imp::kIdentityMat3f);
      break;
  }
  int transfer_function = static_cast<int>(color_space.GetTransfer());
  int max_content_light_level = color_space.GetMaxContentLightLevel();

  GetMaterial()->SetParameter(kColorConversionMatrixParameter,
                              color_transform_matrix);
  GetMaterial()->SetParameter(kTransferFunctionParameter, transfer_function);
  GetMaterial()->SetParameter(kMaxContentLightLevelParameter,
                              max_content_light_level);
}

}  // namespace imp::split_engine
