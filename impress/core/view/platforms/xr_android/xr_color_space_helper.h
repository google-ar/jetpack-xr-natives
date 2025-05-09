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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_COLOR_SPACE_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_COLOR_SPACE_HELPER_H_

#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/common/rememberer.h"
#include "core/math/mat.h"
#include "core/render/display_color_space.h"
#include "core/view/base_view.h"

namespace imp {

// Helper class to manage color space conversion for Android XR applications.
// The conversion is done using material global parameters in Filament, which
// act as a 3x3 color conversion matrix.
class XrColorSpaceHelper : public Rememberer {
 public:
  // Returns a precompile constant that enables color conversion in materials.
  static MaterialPreCompileConstant GetColorSpacePrecompileConstant();

  explicit XrColorSpaceHelper(BaseView& view);

  // Sets the color space to use for color conversion. Updates the material
  // global matrix to reflect the new color space.
  void SetColorSpace(DisplayColorSpace color_space);

  // Returns the current color space.
  DisplayColorSpace GetColorSpace() const;

 private:
  const mat3f& GetColorConversionMatrix() const;
  void SetMaterialGlobalMatrix(const mat3f& color_conversion_matrix) const;

  BaseView& view_;
  DisplayColorSpace color_space_ = DisplayColorSpace::kBT709;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_COLOR_SPACE_HELPER_H_
