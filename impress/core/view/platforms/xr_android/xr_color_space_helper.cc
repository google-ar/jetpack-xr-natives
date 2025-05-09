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

#include "core/view/platforms/xr_android/xr_color_space_helper.h"

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/View.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_manager.h"
#include "core/render/display_color_space.h"
#include "core/view/base_view.h"
#include "core/view/view_events.h"

namespace imp {

namespace {

// BT.709/sRGB to Display P3 conversion matrix
constexpr mat3f kBT709ToP3Mat3f{0.822462f, 0.033194f, 0.017083f,
                                0.177538f, 0.966806f, 0.072397f,
                                0.000000f, 0.000000f, 0.910520f};

constexpr absl::string_view kEnableColorConversionPrecompileConstantName =
    "enableColorConversion";

void SetViewColorConversionMatrixMaterialGlobal(filament::View* view,
                                                const mat3f& matrix) {
  view->setMaterialGlobal(0, float4((matrix[0]), 0.0f));
  view->setMaterialGlobal(1, float4((matrix[1]), 0.0f));
  view->setMaterialGlobal(2, float4((matrix[2]), 0.0f));
}

}  // namespace

MaterialPreCompileConstant
XrColorSpaceHelper::GetColorSpacePrecompileConstant() {
  MaterialPreCompileConstant color_space_constant;
  color_space_constant.name = kEnableColorConversionPrecompileConstantName;
  color_space_constant.value = true;
  return color_space_constant;
}

XrColorSpaceHelper::XrColorSpaceHelper(BaseView& view) : view_(view) {
  SetMaterialGlobalMatrix(GetColorConversionMatrix());
  view_.GetDispatcher().Connect(
      [this](const FilamentViewCreatedEvent& event) {
        SetViewColorConversionMatrixMaterialGlobal(event.view,
                                                   GetColorConversionMatrix());
      },
      this);
}

DisplayColorSpace XrColorSpaceHelper::GetColorSpace() const {
  return color_space_;
}

void XrColorSpaceHelper::SetColorSpace(DisplayColorSpace color_space) {
  if (color_space_ != color_space) {
    color_space_ = color_space;
    SetMaterialGlobalMatrix(GetColorConversionMatrix());
  }
}

const mat3f& XrColorSpaceHelper::GetColorConversionMatrix() const {
  switch (color_space_) {
    case DisplayColorSpace::kBT709:
      return kIdentityMat3f;
    case DisplayColorSpace::kP3:
      return kBT709ToP3Mat3f;
    default:
      IMP_LOG(imp::ERROR) << "Unknown color space enum value: "
                 << static_cast<int>(color_space_);
      // Fall back to identity matrix
      return kIdentityMat3f;
  }
}

void XrColorSpaceHelper::SetMaterialGlobalMatrix(
    const mat3f& color_conversion_matrix) const {
  filament::View* view = view_.GetHost()->GetView();
  SetViewColorConversionMatrixMaterialGlobal(view, color_conversion_matrix);

  for (filament::View* view : view_.GetFilamentViews()) {
    SetViewColorConversionMatrixMaterialGlobal(view, color_conversion_matrix);
  }
}

}  // namespace imp
