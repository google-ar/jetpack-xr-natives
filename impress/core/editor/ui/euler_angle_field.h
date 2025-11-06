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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_EULER_ANGLE_FIELD_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_EULER_ANGLE_FIELD_H_

#include "absl/strings/string_view.h"
#include "core/math/quat.h"
#include "core/math/vec.h"

namespace imp::editor {

// Helper class that handles the conversion between quaternion and euler angles
// and draws the euler angles to screen as ImGui::InputFloat3.
class EulerAngleField {
 public:
  // Draws the euler angle fields as ImGui::InputFloat3.
  // Returns true if the fields are modified. Returns false otherwise.
  bool DrawFields(absl::string_view label, quatf rotation);

  // Updates the cached euler angles to match the given rotation if they are not
  // already roughly equal rotations.
  void UpdateCurrentRotation(quatf rotation);

  // Returns the current euler angles.
  float3 GetCurrentEulerAngles() const;

  // Returns the current rotation as quaternion from the euler angle fields.
  quatf GetCurrentRotation() const;

 private:
  float3 cached_euler_angles_ = {0.0f, 0.0f, 0.0f};
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_UI_EULER_ANGLE_FIELD_H_
