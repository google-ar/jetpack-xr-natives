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

#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/editor/editor_field_control.h"
#include "core/editor/layout/editor_control_flags.h"
#include "core/editor/layout/helpers.h"
#include "core/math/almost_equal.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"

namespace imp::editor {

bool EulerAngleField::DrawFields(const absl::string_view label,
                                 const quatf rotation,
                                 const bool all_same_rotation[3]) {
  UpdateCurrentRotation(rotation);

  const bool default_all_same[3] = {true, true, true};
  const bool* all_same =
      all_same_rotation ? all_same_rotation : default_all_same;

  const float3 old_euler_angles = cached_euler_angles_;
  const bool fields_modified =
      EditorFieldControl::DrawMultiInput3(
          editor::GenerateUniqueImGuiLabel(label, &cached_euler_angles_,
                                           EditorControlFlags::kDefault)
              .c_str(),
          cached_euler_angles_.v, all_same, "%.3f",
          ImGuiInputTextFlags_CharsScientific) &&
      !AlmostEqual(cached_euler_angles_, old_euler_angles);

  if (!fields_modified) return false;

  cached_euler_angles_ =
      FloatModulo(cached_euler_angles_, 360.0f, Clamp::kNonNegative);

  return true;
}

void EulerAngleField::UpdateCurrentRotation(quatf rotation) {
  if (!RoughlyEqual(rotation, QuatFromEuler(cached_euler_angles_))) {
    cached_euler_angles_ = EulerFromQuatClamped(rotation);
  }
}

float3 EulerAngleField::GetCurrentEulerAngles() const {
  return cached_euler_angles_;
}

quatf EulerAngleField::GetCurrentRotation() const {
  return QuatFromEuler(cached_euler_angles_);
}

}  // namespace imp::editor
