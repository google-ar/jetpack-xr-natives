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

#include "core/editor/ui/euler_angle_field.h"

#include <string>

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/editor/layout/helpers.h"
#include "core/math/almost_equal.h"
#include "core/math/quat.h"

namespace imp::editor {

bool EulerAngleField::DrawFields(absl::string_view label, quatf rotation) {
  if (!RoughlyEqual(rotation, QuatFromEuler(cached_euler_angles_))) {
    cached_euler_angles_ = EulerFromQuatClamped(rotation);
  }

  float3 old_euler_angles = cached_euler_angles_;
  bool fields_modified =
      ImGui::InputFloat3(
          editor::GenerateUniqueImGuiLabel(label, &cached_euler_angles_,
                                           EditorControlFlags::kDefault)
              .c_str(),
          cached_euler_angles_.v, "%.3f",
          ImGuiInputTextFlags_CharsScientific) &&
      !AlmostEqual(cached_euler_angles_, old_euler_angles);
  if (fields_modified) {
    cached_euler_angles_ =
        FloatModulo(cached_euler_angles_, 360.0f, Clamp::kNonNegative);
  }

  return fields_modified;
}

float3 EulerAngleField::GetCurrentEulerAngles() const {
  return cached_euler_angles_;
}

quatf EulerAngleField::GetCurrentRotation() const {
  return QuatFromEuler(cached_euler_angles_);
}

}  // namespace imp::editor
