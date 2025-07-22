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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_ACTION_CONFIG_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_XR_ANDROID_XR_ACTION_CONFIG_H_

#include <vector>

#include "core/view/platforms/xr_android/xr_action_params.proto.imp.h"
namespace imp {

class XrActionConfig {
 public:
  // Creates parameters for a default "app" action set.
  static const std::vector<XrActionSetParams>& GetDefaultXrActionSetParams();

  // Creates parameters for the default set of XrActions. This includes the
  // "aim" pose, the "grip" pose, the "menu" action, and the "select" action.
  // These actions correspond with the Khronos simple controller and the OpenXR
  // standard pose identifiers.
  //
  // NOTE: The "/user/hand/left" subaction_path, the "/user/hand/right"
  // subaction_path, and the "aim" action are required by default for
  // ControllerInputHandler to recognize controllers and send
  // ControllerHitEvents.
  //
  // OpenXR context:
  // https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#semantic-path-standard-pose-identifiers
  //
  // TODO Support output actions (XR_ACTION_TYPE_VIBRATION_OUTPUT).
  static const std::vector<XrActionParams>& GetDefaultXrActionParams();

  // Creates default parameters for the suggested action bindings. These default
  // bindings are for the OpenXR Khronos Simple Controller, which represents the
  // minimal controller functionality that OpenXR runtimes should support.
  //
  // From OpenXR Spec:
  //
  // "This interaction profile provides basic pose, button, and haptic support
  // for applications with simple input needs. There is no hardware associated
  // with the profile, and runtimes which support this profile should map the
  // input paths provided to whatever the appropriate paths are on the actual
  // hardware."
  //
  // https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#_khronos_simple_controller_profile
  static const std::vector<XrActionSetBindingParams>&
  GetDefaultActionSetBindingParams();
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_ACTIONS_XR_XR_ACTION_CONFIG_H_
