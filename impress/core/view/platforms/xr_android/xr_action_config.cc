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

namespace imp {

const std::vector<XrActionSetParams>&
XrActionConfig::GetDefaultXrActionSetParams() {
  static const std::vector<XrActionSetParams>* xr_action_set_params =
      new std::vector<XrActionSetParams>{
          {.action_set_name = std::string(kDefaultActionSetName),
           .localized_action_set_name = "App",
           .priority = 0}};
  return *xr_action_set_params;
}

const std::vector<XrActionParams>& XrActionConfig::GetDefaultXrActionParams() {
  std::vector<std::string> default_subaction_paths{
      std::string(kDefaultLeftHandSubactionPath),
      std::string(kDefaultRightHandSubactionPath)};
  std::string default_action_set_name(kDefaultActionSetName);
  static const std::vector<XrActionParams>* xr_action_params =
      new std::vector<XrActionParams>{
          {.action_name = std::string(kDefaultSelectActionName),
           .localized_action_name = "Select",
           .action_set_name = default_action_set_name,
           .action_type = XrActionParams::ActionType::BOOLEAN_INPUT,
           .subaction_paths = default_subaction_paths},
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
          {.action_name = std::string(kDefaultAimActionName),
           .localized_action_name = "Aim",
           .action_set_name = default_action_set_name,
           .action_type = XrActionParams::ActionType::POSE_INPUT,
           .subaction_paths = default_subaction_paths},
          {.action_name = std::string(kDefaultHapticActionName),
           .localized_action_name = "Haptic",
           .action_set_name = default_action_set_name,
           .action_type = XrActionParams::ActionType::HAPTIC_OUTPUT,
           .subaction_paths = default_subaction_paths},
      };
  return *xr_action_params;
}

const std::vector<XrActionSetBindingParams>&
XrActionConfig::GetDefaultActionSetBindingParams() {
  static const std::vector<XrActionSetBindingParams>*
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
  return *xr_action_set_binding_params;
}

}  // namespace imp
