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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_CUSTOM_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_CUSTOM_MATERIAL_H_

#include <optional>

#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/media/media_color_space.h"
#include "core/render/display_color_space.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/view/base_view.h"

namespace imp::split_engine {

// Color correction mode for a material. This is used to determine how the
// material should be rendered when the content color space is provided.
enum class ColorCorrectionMode {
  // Color correction and tonemapping are disabled.
  kDisabled,
  // Attempt automatic color correction and tonemapping using content metadata
  // if available. Defaults to tonemapping to the display color space, assuming
  // the content is in the sRGB color space.
  kSystemBestEffort,
  // User-provided color correction and tonemapping parameters will be used,
  // overriding defaults.
  kUserOverride,
};

// The base class for all built-in materials that are not generic materials.
// This class holds the OwnedMaterialPtr and implements GetMaterialInternal.
class BuiltInCustomMaterial : public BuiltInMaterial {
 public:
  BuiltInCustomMaterial(BridgeId bridge_id, OwnedMaterialPtr material);

  DisplayColorSpace GetRequiredDisplayColorSpace() const override {
    return DisplayColorSpace::kBT709;
  }

 protected:
  BridgeId GetBridgeId() const;

  BorrowedMaterialPtr GetMaterialInternal(
      SmallSourceLocation loc) const override;

  // Updates the material's color space parameters. Attempts to retrieve
  // color information from the BufferItem associated with the given
  // Filament::Texture (texture_id). If unavailable, default color space
  // parameters are applied.
  void UpdateColorSpaceParameters(BaseView& view,
                                  std::optional<TextureId> texture_id);

  // Overrides the material's color space parameters using the color space
  // explicitly provided by the user.
  void OverrideColorSpaceParameters(BaseView& view,
                                    MediaColorSpace color_space);

 private:
  // The bridge ID of the app using the material. This is required to get the
  // color space of the material from the SplitEngineRenderer since the texture
  // in question is associated with the app's context.
  BridgeId bridge_id_;
  OwnedMaterialPtr material_;
  // Initialize the default color space to BT709/sRGB.
  // When the color space is not available, we assume the color space is
  // BT709/sRGB.
  MediaColorSpace default_color_space_{MediaColorSpace::Standard::kBT709,
                                       MediaColorSpace::Transfer::kSRGB,
                                       MediaColorSpace::Range::kFull};
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_CUSTOM_MATERIAL_H_
