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

#ifndef THIRD_PARTY_IMPRESS_CORE_ACTIONS_ACTION_CONFIG_H_
#define THIRD_PARTY_IMPRESS_CORE_ACTIONS_ACTION_CONFIG_H_
#include "absl/strings/string_view.h"
namespace imp {
/*
 * Commonly used input action names, primarily based on the OpenXR spec.
 *
 * In Impress XR apps, by default, the XrActionController will register a
 * action set consisting of the input action names in this file and based on
 * OpenXR's Khronos Simple Controller interaction profile. See
 * third_party/impress/core/view/platforms/xr_android/xr_action_config.h for
 * these defaults.
 */

/*
 * Matches OpenXR standard pose identifiers.
 * https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#semantic-path-standard-pose-identifiers
 */
inline constexpr absl::string_view kDefaultAimActionName = "aim";
inline constexpr absl::string_view kDefaultGripActionName = "grip";

/*
 * Matches OpenXR standard identifiers.
 * https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#semantic-path-standard-identifiers
 */
inline constexpr absl::string_view kDefaultBackActionName = "back";
inline constexpr absl::string_view kDefaultMenuActionName = "menu";
inline constexpr absl::string_view kDefaultSystemActionName = "system";
inline constexpr absl::string_view kDefaultTriggerActionName = "trigger";
inline constexpr absl::string_view kDefaultSelectActionName = "select";

/*
 * Matches OpenXR reserved user/ paths.
 * https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#semantic-path-user
 * Eye subaction path:
 * https://registry.khronos.org/OpenXR/specs/1.1/html/xrspec.html#XR_EXT_eye_gaze_interaction
 */
inline constexpr absl::string_view kDefaultLeftHandSubactionPath =
    "/user/hand/left";
inline constexpr absl::string_view kDefaultRightHandSubactionPath =
    "/user/hand/right";
inline constexpr absl::string_view kDefaultEyeSubactionPath = "/user/eyes_ext";

/*
 * Matches XR_EXT_hand_interaction action names.
 * https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#XR_EXT_hand_interaction
 *
 * Note: XR_EXT_hand_interaction is not yet supported in standalone Impress
 * OpenXR apps.
 * TODO Support hand tracking in standalone Impress XR apps.
 */
inline constexpr absl::string_view kDefaultPinchPoseActionName = "pinch_pose";
inline constexpr absl::string_view kDefaultPinchGestureActionName =
    "pinch_gesture";

/*
 * Other commonly used input action names.
 */
inline constexpr absl::string_view kDefaultScrollActionName = "scroll";
inline constexpr absl::string_view kDefaultZoomActionName = "zoom";
inline constexpr absl::string_view kDefaultLongPressActionName = "long_press";

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_ACTIONS_ACTION_CONFIG_H_
