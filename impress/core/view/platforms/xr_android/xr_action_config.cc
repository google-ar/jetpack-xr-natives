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

#include "core/view/platforms/xr_android/xr_action_config.h"

#include <string>
#include <vector>

#include "core/actions/action_config.h"
#include "core/view/platforms/xr_android/xr_action_params.proto.imp.h"

namespace imp {

const std::vector<XrActionSetParams>&
XrActionConfig::GetDefaultXrActionSetParams() {
  static std::vector<XrActionSetParams>* xr_action_set_params =
      new std::vector<XrActionSetParams>{
          {.action_set_name = std::string(kDefaultActionSetName),
           .localized_action_set_name = "App",
           .priority = 0}};
  return *xr_action_set_params;
}

const std::vector<XrActionParams>& XrActionConfig::GetDefaultXrActionParams() {
  std::vector<std::string> default_subaction_paths{
      std::string(imp::kDefaultLeftHandSubactionPath),
      std::string(imp::kDefaultRightHandSubactionPath)};
  std::vector<std::string> mouse_subaction_paths{
      std::string(kDefaultMouseSubactionPath)};

  std::string default_action_set_name = std::string(kDefaultActionSetName);

  static std::vector<imp::XrActionParams>* xr_action_params =
      new std::vector<XrActionParams>{
          // Add mouse and eye gaze interaction action params.
          {.action_name = std::string(imp::kDefaultAimActionName),
           .localized_action_name = "Aim",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::POSE_INPUT,
           .subaction_paths = {std::string(imp::kDefaultLeftHandSubactionPath),
                               std::string(imp::kDefaultRightHandSubactionPath),
                               std::string(kDefaultMouseSubactionPath),
                               std::string(kDefaultEyeSubactionPath)}},
          {.action_name = std::string(imp::kDefaultSelectActionName),
           .localized_action_name = "Select",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::BOOLEAN_INPUT,
           .subaction_paths = {std::string(imp::kDefaultLeftHandSubactionPath),
                               std::string(imp::kDefaultRightHandSubactionPath),
                               std::string(kDefaultMouseSubactionPath)}},
          {.action_name = std::string(kSecondaryActionName),
           .localized_action_name = "Secondary",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::BOOLEAN_INPUT,
           .subaction_paths = mouse_subaction_paths},
          {.action_name = std::string(kTertiaryActionName),
           .localized_action_name = "Tertiary",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::BOOLEAN_INPUT,
           .subaction_paths = mouse_subaction_paths},
          {.action_name = std::string(imp::kDefaultScrollActionName),
           .localized_action_name = "Scroll",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::VECTOR2F_INPUT,
           .subaction_paths = {std::string(imp::kDefaultLeftHandSubactionPath),
                               std::string(imp::kDefaultRightHandSubactionPath),
                               std::string(kDefaultMouseSubactionPath)}},

          // Add hand interaction action params. "Select" and "Aim" already
          // exist.
          {.action_name = std::string(kDefaultMenuActionName),
           .localized_action_name = "Menu",
           .action_set_name = default_action_set_name,
           .action_type = XrActionParams::ActionType::BOOLEAN_INPUT,
           .subaction_paths = default_subaction_paths},
          {.action_name = std::string(kDefaultGripActionName),
           .localized_action_name = "Grip",
           .action_set_name = default_action_set_name,
           .action_type = XrActionParams::ActionType::POSE_INPUT,
           .subaction_paths = default_subaction_paths},
          {.action_name = std::string(kDefaultHapticActionName),
           .localized_action_name = "Haptic",
           .action_set_name = default_action_set_name,
           .action_type = XrActionParams::ActionType::HAPTIC_OUTPUT,
           .subaction_paths = default_subaction_paths},

          // Add controller action params. "Select", "Aim", "Menu", "Grip",
          // "Scroll" and
          // "Haptic" already exist.
          {.action_name = std::string(imp::kDefaultTriggerActionName),
           .localized_action_name = "Trigger",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::FLOAT_INPUT,
           .subaction_paths = default_subaction_paths},
          {.action_name = std::string(imp::kDefaultBackActionName),
           .localized_action_name = "Back",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::BOOLEAN_INPUT,
           .subaction_paths = default_subaction_paths},
          {.action_name = std::string(imp::kDefaultSystemActionName),
           .localized_action_name = "System",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::BOOLEAN_INPUT,
           .subaction_paths = default_subaction_paths},
          {.action_name = std::string(kThumbStickClickActionName),
           .localized_action_name = "Thumb Stick Click",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::BOOLEAN_INPUT,
           .subaction_paths = default_subaction_paths},
          {.action_name = std::string(kSqueezeActionName),
           .localized_action_name = "Squeeze",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::FLOAT_INPUT,
           .subaction_paths = default_subaction_paths},
          {.action_name = std::string(kThumbStickXActionName),
           .localized_action_name = "Thumb Stick X",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::FLOAT_INPUT,
           .subaction_paths = default_subaction_paths},
          {.action_name = std::string(kThumbStickYActionName),
           .localized_action_name = "Thumb Stick Y",
           .action_set_name = default_action_set_name,
           .action_type = imp::XrActionParams::ActionType::FLOAT_INPUT,
           .subaction_paths = default_subaction_paths}};

  return *xr_action_params;
}

const std::vector<XrActionSetBindingParams>&
XrActionConfig::GetDefaultActionSetBindingParams() {
  // Add controller action set binding params.
  static std::vector<imp::XrActionSetBindingParams>*
      xr_action_set_binding_params = new std::vector<XrActionSetBindingParams>{
          {.action_set_name = "app",
           .interaction_profile_path =
               "/interaction_profiles/khr/simple_controller",
           .action_binding_params = {
               {.action_name = std::string(kDefaultSelectActionName),
                .binding_path = "/user/hand/left/input/select/click"},
               {.action_name = std::string(kDefaultSelectActionName),
                .binding_path = "/user/hand/right/input/select/click"},
               {.action_name = std::string(kDefaultMenuActionName),
                .binding_path = "/user/hand/left/input/menu/click"},
               {.action_name = std::string(kDefaultMenuActionName),
                .binding_path = "/user/hand/right/input/menu/click"},
               {.action_name = std::string(kDefaultGripActionName),
                .binding_path = "/user/hand/left/input/grip/pose"},
               {.action_name = std::string(kDefaultGripActionName),
                .binding_path = "/user/hand/right/input/grip/pose"},
               {.action_name = std::string(kDefaultAimActionName),
                .binding_path = "/user/hand/left/input/aim/pose"},
               {.action_name = std::string(kDefaultAimActionName),
                .binding_path = "/user/hand/right/input/aim/pose"},
               {.action_name = std::string(kDefaultHapticActionName),
                .binding_path = "/user/hand/left/output/haptic"},
               {.action_name = std::string(kDefaultHapticActionName),
                .binding_path = "/user/hand/right/output/haptic"},
           }}};

  // Add controller action set binding params.
  xr_action_set_binding_params->push_back(
      {.action_set_name = std::string(kDefaultActionSetName),
       .interaction_profile_path =
           "/interaction_profiles/oculus/touch_controller",
       .action_binding_params = {
           {.action_name = std::string(kSqueezeActionName),
            .binding_path = "/user/hand/left/input/squeeze/value"},
           {.action_name = std::string(kSqueezeActionName),
            .binding_path = "/user/hand/right/input/squeeze/value"},
           {.action_name = std::string(kThumbStickClickActionName),
            .binding_path = "/user/hand/left/input/thumbstick/click"},
           {.action_name = std::string(kThumbStickClickActionName),
            .binding_path = "/user/hand/right/input/thumbstick/click"},
           {.action_name = std::string(kThumbStickXActionName),
            .binding_path = "/user/hand/left/input/thumbstick/x"},
           {.action_name = std::string(kThumbStickXActionName),
            .binding_path = "/user/hand/right/input/thumbstick/x"},
           {.action_name = std::string(kThumbStickYActionName),
            .binding_path = "/user/hand/left/input/thumbstick/y"},
           {.action_name = std::string(kThumbStickYActionName),
            .binding_path = "/user/hand/right/input/thumbstick/y"},
           {.action_name = std::string(imp::kDefaultScrollActionName),
            .binding_path = "/user/hand/left/input/thumbstick"},
           {.action_name = std::string(imp::kDefaultScrollActionName),
            .binding_path = "/user/hand/right/input/thumbstick"},
           {.action_name = std::string(imp::kDefaultSelectActionName),
            .binding_path = "/user/hand/left/input/x/click"},
           {.action_name = std::string(imp::kDefaultSelectActionName),
            .binding_path = "/user/hand/right/input/a/click"},
           {.action_name = std::string(imp::kDefaultBackActionName),
            .binding_path = "/user/hand/left/input/y/click"},
           {.action_name = std::string(imp::kDefaultBackActionName),
            .binding_path = "/user/hand/right/input/b/click"},
           {.action_name = std::string(imp::kDefaultTriggerActionName),
            .binding_path = "/user/hand/left/input/trigger/value"},
           {.action_name = std::string(imp::kDefaultTriggerActionName),
            .binding_path = "/user/hand/right/input/trigger/value"},
           {.action_name = std::string(imp::kDefaultMenuActionName),
            .binding_path = "/user/hand/left/input/menu/click"},
           {.action_name = std::string(imp::kDefaultSystemActionName),
            .binding_path = "/user/hand/right/input/system/click"},
           {.action_name = std::string(imp::kDefaultGripActionName),
            .binding_path = "/user/hand/left/input/grip/pose"},
           {.action_name = std::string(imp::kDefaultGripActionName),
            .binding_path = "/user/hand/right/input/grip/pose"},
           {.action_name = std::string(imp::kDefaultAimActionName),
            .binding_path = "/user/hand/left/input/aim/pose"},
           {.action_name = std::string(imp::kDefaultAimActionName),
            .binding_path = "/user/hand/right/input/aim/pose"},
           {.action_name = std::string(imp::kDefaultHapticActionName),
            .binding_path = "/user/hand/left/output/haptic"},
           {.action_name = std::string(imp::kDefaultHapticActionName),
            .binding_path = "/user/hand/right/output/haptic"},
       }});

  // Add eye gaze interaction action set binding params.
  xr_action_set_binding_params->push_back(
      {.action_set_name = std::string(kDefaultActionSetName),
       .interaction_profile_path =
           "/interaction_profiles/ext/eye_gaze_interaction",
       .action_binding_params = {
           {.action_name = std::string(imp::kDefaultAimActionName),
            .binding_path = "/user/eyes_ext/input/gaze_ext/pose"},
       }});

  // Add mouse interaction action set binding params.
  xr_action_set_binding_params->push_back(
      {.action_set_name = std::string(kDefaultActionSetName),
       .interaction_profile_path =
           "/interaction_profiles/android/mouse_interaction_android",
       .action_binding_params = {
           {.action_name = std::string(imp::kDefaultAimActionName),
            .binding_path = "/user/mouse/input/aim/pose"},
           {.action_name = std::string(imp::kDefaultSelectActionName),
            .binding_path = "/user/mouse/input/select/click"},
           {.action_name = std::string(kSecondaryActionName),
            .binding_path = "/user/mouse/input/secondary_android/click"},
           {.action_name = std::string(kTertiaryActionName),
            .binding_path = "/user/mouse/input/tertiary_android/click"},
           {.action_name = std::string(imp::kDefaultScrollActionName),
            .binding_path = "/user/mouse/input/scroll_android/value"},
       }});
  return *xr_action_set_binding_params;
}

}  // namespace imp
