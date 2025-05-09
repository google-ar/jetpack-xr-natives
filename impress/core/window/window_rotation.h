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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_WINDOW_ROTATION_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_WINDOW_ROTATION_H_

#include "core/common/log.h"
#include "core/common/platform_helpers.h"
namespace imp::window {

// Display rotation specified by @c android.view.Surface constants: @c
// ROTATION_0, @c ROTATION_90, @c ROTATION_180 and @c ROTATION_270
enum class WindowRotation {
  kRotation0 = 0,
  kRotation90 = 1,
  kRotation180 = 2,
  kRotation270 = 3
};

// Maps integer values specified by @c android.view.Surface constants: @c
// ROTATION_0, @c ROTATION_90, @c ROTATION_180 and @c ROTATION_270 to the native
// type, WindowRotation.
inline constexpr WindowRotation ToWindowRotation(int value) {
  switch (value) {
    case 0:
      return WindowRotation::kRotation0;
      break;
    case 1:
      return WindowRotation::kRotation90;
      break;
    case 2:
      return WindowRotation::kRotation180;
      break;
    case 3:
      return WindowRotation::kRotation270;
      break;
  }
  IMP_LOG(imp::FATAL) << "Invalid parameter to SetDisplayRotation";
  return WindowRotation::kRotation0;
}

}  // namespace imp::window

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_WINDOW_ROTATION_H_
