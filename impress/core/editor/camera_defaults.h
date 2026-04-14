// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_CAMERA_DEFAULTS_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_CAMERA_DEFAULTS_H_

#include "core/math/math.h"
#include "core/math/vec.h"

namespace imp::editor {

// Default values for the Editor viewport camera.
struct CameraDefaults {
  // The default camera angle constants.
  static constexpr float kPitch = -45;
  static constexpr float kYaw = 45;

  static constexpr float kNearClipPlane = 0.01f;
  static constexpr float kFarClipPlane = 250.0f;

  // Distance from a non-Renderer Node to place the camera when focusing on it.
  static constexpr float kDefaultFocusDistance = 7.0f;

  // The default local position of the camera relative to the pivot.
  static constexpr float3 kStartingPosition = float3(3.5, 5, 3.5);
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_CAMERA_DEFAULTS_H_
