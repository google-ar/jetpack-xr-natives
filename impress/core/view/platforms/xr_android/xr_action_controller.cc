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

#include "core/view/platforms/xr_android/xr_action_controller.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/actions/action_config.h"
#include "core/actions/controller_input_handler.h"
#include "core/actions/input_action_event.h"
#include "core/common/robin_set.h"
#include "core/common/trace.h"
#include "core/view/platforms/xr_android/openxr_includes.h"
#include "core/view/platforms/xr_android/xr_action_params.proto.imp.h"
#include "core/view/platforms/xr_android/xr_helpers.h"
#include "core/view/platforms/xr_android/xr_session_host.h"
#include "core/view/utils/string_hasher.h"
#include "core/view/utils/string_map.h"
#include "core/xr/openxr_events.h"
#include "robin_map/include/tsl/robin_map.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {
namespace {

// Returns absl::OkStatus() the provided XrActionSetsParams have unique,
// non-empty names.
absl::Status ValidateXrActionSetParams(
    const std::vector<XrActionSetParams>& all_xr_action_set_params) {
  // Store all action_set_name and action_name pairs, effectively verifying that
  // each action set has a unique name.
  RobinSet<std::string> xr_action_set_names;
  for (const XrActionSetParams& xr_action_set_params :
       all_xr_action_set_params) {
    if (xr_action_set_params.action_set_name.empty()) {
      return absl::InvalidArgumentError("XrActionSetParams had empty name.");
    }
    if (xr_action_set_params.localized_action_set_name.empty()) {
      return absl::InvalidArgumentError(
          "XrActionSetParams had empty localized name.");
    }
    xr_action_set_names.insert(xr_action_set_params.action_set_name);
  }
  if (xr_action_set_names.size() != all_xr_action_set_params.size()) {
    return absl::InvalidArgumentError(
        "action_set_name was not unique for each XrActionSetParams.");
  }
  return absl::OkStatus();
}

// Returns absl::OkStatus() the provided XrActionParams has non-empty fields and
// valid references to XrActionSetParams.
absl::Status ValidateXrActionParams(
    const std::vector<XrActionParams>& all_xr_action_params,
    const std::vector<XrActionSetParams>& all_xr_action_set_params) {
  // Create a set of all action set names.
  RobinSet<std::string> xr_action_set_names;
  for (const XrActionSetParams& xr_action_set_params :
       all_xr_action_set_params) {
    xr_action_set_names.insert(xr_action_set_params.action_set_name);
  }
  // Store all action_set_name and action_name pairs, effectively verifying
  // that within each action set, each action has a unique name.
  RobinSet<std::pair<std::string, std::string>>
      xr_action_set_xr_action_name_pairs;
  for (const XrActionParams& xr_action_params : all_xr_action_params) {
    if (xr_action_params.action_name.empty()) {
      return absl::InvalidArgumentError(
          "XrActionParams had empty action_name.");
    }
    if (xr_action_params.localized_action_name.empty()) {
      return absl::InvalidArgumentError(
          "XrActionParams had empty localized_action_name.");
    }
    if (xr_action_params.action_set_name.empty()) {
      return absl::InvalidArgumentError(
          "XrActionParams had empty action_set_name.");
    }
    if (xr_action_params.action_type == XrActionParams::NOT_SET) {
      return absl::InvalidArgumentError(
          "XrActionParams had empty action_type.");
    }
    if (!xr_action_set_names.contains(xr_action_params.action_set_name)) {
      return absl::InvalidArgumentError(
          "action_set_name did not refer to a provided XrActionSetParams.");
    }
    xr_action_set_xr_action_name_pairs.insert(
        {xr_action_params.action_set_name, xr_action_params.action_name});
  }
  if (xr_action_set_xr_action_name_pairs.size() !=
      all_xr_action_params.size()) {
    return absl::InvalidArgumentError(
        "Multiple XrActionParams have both the same action_name and the same"
        "action_set_name. Action names must be unique in each action set.");
  }
  return absl::OkStatus();
}

// Returns absl::OkStatus() the provided XrActionSetBindingParams has non-empty
// fields and valid references to XrActionSetParams and XrActionParams.
absl::Status ValidateXrActionSetBindingParams(
    const std::vector<XrActionParams>& all_xr_action_params,
    const std::vector<XrActionSetParams>& all_xr_action_set_params,
    const std::vector<XrActionSetBindingParams>&
        all_xr_action_set_binding_params) {
  // Create a set of all action set names.
  RobinSet<std::string> xr_action_set_names;
  for (const XrActionSetParams& xr_action_set_params :
       all_xr_action_set_params) {
    xr_action_set_names.insert(xr_action_set_params.action_set_name);
  }
  // Create a set of all action names.
  RobinSet<std::string> xr_action_names;
  for (const XrActionParams& xr_action_params : all_xr_action_params) {
    xr_action_names.insert(xr_action_params.action_name);
  }
  // Validate the XrActionSetBindingParams.
  for (const XrActionSetBindingParams& xr_action_set_binding_params :
       all_xr_action_set_binding_params) {
    if (xr_action_set_binding_params.action_set_name.empty()) {
      return absl::InvalidArgumentError(
          "XrActionSetBindingParams had empty action_set_name.");
    }
    if (!xr_action_set_names.contains(
            xr_action_set_binding_params.action_set_name)) {
      return absl::InvalidArgumentError(
          "XrActionSetBindingParams references an invalid action_set_name.");
    }
    if (xr_action_set_binding_params.interaction_profile_path.empty()) {
      return absl::InvalidArgumentError(
          "XrActionSetBindingParams had empty interaction_profile_path.");
    }
    for (const XrActionBindingParams& xr_action_binding_params :
         xr_action_set_binding_params.action_binding_params) {
      if (xr_action_binding_params.action_name.empty()) {
        return absl::InvalidArgumentError(
            "XrActionBindingParams had emptyaction_name.");
      }
      if (!xr_action_names.contains(xr_action_binding_params.action_name)) {
        return absl::InvalidArgumentError(
            "XrActionBindingParams referenced an action_name that was not "
            "present in the list of XrActionParams.");
      }
      if (xr_action_binding_params.action_name.empty()) {
        return absl::InvalidArgumentError(
            "XrActionBindingParams had no binding_path.");
      }
    }
  }
  return absl::OkStatus();
}

absl::StatusOr<XrPath> StringToXrPath(const XrSessionHost& xr_session_host,
                                      absl::string_view path_string) {
  XrPath xr_path;
  MP_RETURN_IF_ERROR(xr_session_host.ToStatus(xrStringToPath(
      xr_session_host.GetXrInstance(), path_string.data(), &xr_path)));
  return xr_path;
}

}  // namespace

XrActionController::XrActionController(XrSessionHost& xr_session_host)
    : view_(xr_session_host.GetView()), xr_session_host_(xr_session_host) {
  view_->GetDispatcher().Connect(
      [this](const OpenXrFocusedWaitFrameEvent& ev) {
        if (!have_action_sets_been_attached_) {
          return;
        }
        absl::Status status = OnFocusedWaitFrame();
        if (!status.ok()) {
          IMP_LOG(imp::ERROR) << "OnFocusedWaitFrame error: " << status;
        }
      },
      view_);
  view_->GetDispatcher().Connect(
      [this](const imp::OpenXrSessionBeginEvent& event) {
        switch (xr_session_action_config_) {
          case XrSessionActionConfig::kUseXrActionDefaults:
            AttachDefaultXrActions();
            return;
          case XrSessionActionConfig::kOmitXrActionDefaults:
            return;
        }
      },
      view_);
}

absl::Status XrActionController::AttachXrActions(
    std::vector<XrActionSetParams> xr_action_set_params,
    std::vector<XrActionParams> xr_action_params,
    std::vector<XrActionSetBindingParams> xr_action_set_binding_params) {
  if (xr_session_host_.GetXrInstance() == XR_NULL_HANDLE) {
    return absl::InternalError("XrInstance is NULL");
  }
  if (have_action_sets_been_attached_) {
    return absl::InternalError("XrActionSets have already been attached.");
  }

  if (input_handler_config_ == XrActionController::InputHandlerConfig::
                                   kCreateDefaultControllerInputHandler) {
    // Push a ControllerInputHandler to process data generated by
    // XrActionController.
    view_->GetInputManager().PushInputHandler(
        std::make_unique<ControllerInputHandler>(
            view_, kDefaultAimActionName, kDefaultLeftHandSubactionPath,
            kDefaultRightHandSubactionPath));
  }

  MP_RETURN_IF_ERROR(ValidateXrActionSetParams(xr_action_set_params));
  MP_RETURN_IF_ERROR(
      ValidateXrActionParams(xr_action_params, xr_action_set_params));
  MP_RETURN_IF_ERROR(ValidateXrActionSetBindingParams(
      xr_action_params, xr_action_set_params, xr_action_set_binding_params));

  // Store a list of XrActionSet handles.
  std::vector<XrActionSet> xr_action_sets;

  // Set up the XrActionSets and XrActions.
  for (const XrActionSetParams& xr_action_set_params : xr_action_set_params) {
    MP_ASSIGN_OR_RETURN(XrActionSet xr_action_set_handle,
                     SetupXrActionSet(xr_action_set_params));
    xr_action_sets.push_back(xr_action_set_handle);
  }

  // Create the actions.
  for (const XrActionParams& xr_action_params : xr_action_params) {
    MP_RETURN_IF_ERROR(SetupXrAction(xr_action_params));
  }

  // Suggest bindings for the XrActions.
  for (const XrActionSetBindingParams& xr_action_set_binding_params :
       xr_action_set_binding_params) {
    MP_RETURN_IF_ERROR(SetupXrActionSetBindings(xr_action_set_binding_params));
  }

  // Attach the XrActionSets. This makes them immutable:
  // https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#xrAttachSessionActionSets
  XrSessionActionSetsAttachInfo attach_info{
      .type = XR_TYPE_SESSION_ACTION_SETS_ATTACH_INFO,
      .countActionSets = static_cast<uint32_t>(xr_action_sets.size()),
      .actionSets = xr_action_sets.data(),
  };
  MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xrAttachSessionActionSets(
      xr_session_host_.GetXrSession(), &attach_info)));

  have_action_sets_been_attached_ = true;
  return absl::OkStatus();
}

void XrActionController::AttachDefaultXrActions() {
  if (have_action_sets_been_attached_) {
    return;
  }
  absl::Status status = AttachXrActions();
  if (!status.ok()) {
    IMP_LOG(imp::FATAL) << "AttachXrActions error: " << status;
  }
}

template <>
absl::StatusOr<InputActionState<bool>>
XrActionController::FetchInputActionState<bool>(
    const XrActionReference& xr_action_reference, XrPath xr_subaction_path) {
  MP_ASSIGN_OR_RETURN(
      XrActionStateGetInfo xr_action_state_get_info,
      BuildXrActionStateGetInfo(xr_action_reference, xr_subaction_path));

  XrInputTracingDataANDROIDSYS input_tracing_data = {
      .type = XR_TYPE_INPUT_TRACING_DATA_ANDROIDSYS,
  };

  XrActionStateBoolean state = {
      .type = XR_TYPE_ACTION_STATE_BOOLEAN,
      .next = &input_tracing_data,
  };

  XrResult xr_get_action_state_boolean_result = xrGetActionStateBoolean(
      xr_session_host_.GetXrSession(), &xr_action_state_get_info, &state);
  MP_RETURN_IF_ERROR(
      xr_session_host_.ToStatus(xr_get_action_state_boolean_result));

  return absl::StatusOr<InputActionState<bool>>({
      .name = xr_action_reference.xr_action_params.action_name,
      .has_changed_since_last_sync =
          static_cast<bool>(state.changedSinceLastSync),
      .last_change_time = state.lastChangeTime,
      .is_active = static_cast<bool>(state.isActive),
      .current_state = static_cast<bool>(state.currentState),
      .event_tracing_id = input_tracing_data.eventTracingId,
  });
}

template <>
absl::StatusOr<InputActionState<float>>
XrActionController::FetchInputActionState<float>(
    const XrActionReference& xr_action_reference, XrPath xr_subaction_path) {
  MP_ASSIGN_OR_RETURN(
      XrActionStateGetInfo xr_action_state_get_info,
      BuildXrActionStateGetInfo(xr_action_reference, xr_subaction_path));

  XrInputTracingDataANDROIDSYS input_tracing_data = {
      .type = XR_TYPE_INPUT_TRACING_DATA_ANDROIDSYS,
  };

  XrActionStateFloat state = {
      .type = XR_TYPE_ACTION_STATE_FLOAT,
      .next = &input_tracing_data,
  };

  XrResult xr_get_action_state_float_result = xrGetActionStateFloat(
      xr_session_host_.GetXrSession(), &xr_action_state_get_info, &state);
  MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xr_get_action_state_float_result));

  return absl::StatusOr<InputActionState<float>>({
      .name = xr_action_reference.xr_action_params.action_name,
      .has_changed_since_last_sync =
          static_cast<bool>(state.changedSinceLastSync),
      .last_change_time = state.lastChangeTime,
      .is_active = static_cast<bool>(state.isActive),
      .current_state = state.currentState,
      .event_tracing_id = input_tracing_data.eventTracingId,
  });
}

template <>
absl::StatusOr<InputActionState<float2>>
XrActionController::FetchInputActionState<float2>(
    const XrActionReference& xr_action_reference, XrPath xr_subaction_path) {
  MP_ASSIGN_OR_RETURN(
      XrActionStateGetInfo xr_action_state_get_info,
      BuildXrActionStateGetInfo(xr_action_reference, xr_subaction_path));
  XrActionStateVector2f state = {
      .type = XR_TYPE_ACTION_STATE_VECTOR2F,
  };
  XrResult xr_get_action_state_vector2f_result = xrGetActionStateVector2f(
      xr_session_host_.GetXrSession(), &xr_action_state_get_info, &state);
  MP_RETURN_IF_ERROR(
      xr_session_host_.ToStatus(xr_get_action_state_vector2f_result));

  return absl::StatusOr<InputActionState<float2>>({
      .name = xr_action_reference.xr_action_params.action_name,
      .has_changed_since_last_sync =
          static_cast<bool>(state.changedSinceLastSync),
      .last_change_time = state.lastChangeTime,
      .is_active = static_cast<bool>(state.isActive),
      .current_state = {state.currentState.x, state.currentState.y},
  });
}

absl::StatusOr<std::optional<InputActionState<Transform<float>>>>
XrActionController::FetchTransformInputActionState(
    const XrActionReference& xr_action_reference, XrPath xr_subaction_path) {
  MP_ASSIGN_OR_RETURN(
      XrActionStateGetInfo xr_action_state_get_info,
      BuildXrActionStateGetInfo(xr_action_reference, xr_subaction_path));
  XrActionStatePose state = {
      .type = XR_TYPE_ACTION_STATE_POSE,
  };
  XrResult xr_get_action_state_pose_result = xrGetActionStatePose(
      xr_session_host_.GetXrSession(), &xr_action_state_get_info, &state);
  MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xr_get_action_state_pose_result));

  if (!state.isActive) {
    return absl::StatusOr<std::optional<InputActionState<Transform<float>>>>(
        std::nullopt);
  }

  // Locates the pose action.
  XrSpaceLocation location = {
      .type = XR_TYPE_SPACE_LOCATION,
  };
  XrResult xr_locate_space_result =
      xrLocateSpace(xr_action_reference.xr_subaction_path_to_xr_action_space.at(
                        xr_subaction_path),
                    xr_session_host_.GetXrSpace(),
                    xr_session_host_.GetPredictedDisplayTime(), &location);
  MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xr_locate_space_result));

  // If both the position and orientation are reported as invalid, do not read.
  if ((location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT) == 0 &&
      (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) == 0) {
    return absl::StatusOr<std::optional<InputActionState<Transform<float>>>>(
        std::nullopt);
  }

  return absl::StatusOr<std::optional<InputActionState<Transform<float>>>>(
      InputActionState<Transform<float>>{
          .name = xr_action_reference.xr_action_params.action_name,
          // This is always marked true for Transform actions.
          .has_changed_since_last_sync = true,
          .is_active = true,
          .current_state = ToTransform(location.pose),
          .is_position_valid = (location.locationFlags &
                                XR_SPACE_LOCATION_POSITION_VALID_BIT) != 0,
          .is_rotation_valid = (location.locationFlags &
                                XR_SPACE_LOCATION_ORIENTATION_VALID_BIT) != 0,
          .is_position_tracked = (location.locationFlags &
                                  XR_SPACE_LOCATION_POSITION_TRACKED_BIT) != 0,
          .is_rotation_tracked =
              (location.locationFlags &
               XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT) != 0});
}

absl::StatusOr<XrActionStateGetInfo>
XrActionController::BuildXrActionStateGetInfo(
    const XrActionReference& xr_action_reference, XrPath xr_subaction_path) {
  XrAction xr_action_handle = xr_action_reference.xr_action_handle;

  XrActionStateGetInfo xr_action_state_get_info = {
      .type = XR_TYPE_ACTION_STATE_GET_INFO,
      .action = xr_action_handle,
  };
  xr_action_state_get_info.subactionPath = xr_subaction_path;
  return xr_action_state_get_info;
}

absl::StatusOr<XrActionSet> XrActionController::SetupXrActionSet(
    XrActionSetParams xr_action_set_params) {
  // Build the XrActionSet.
  XrActionSetCreateInfo xr_action_set_create_info = {
      .type = XR_TYPE_ACTION_SET_CREATE_INFO,
      .priority = xr_action_set_params.priority};
  strncpy(xr_action_set_create_info.actionSetName,
          xr_action_set_params.action_set_name.c_str(),
          XR_MAX_ACTION_SET_NAME_SIZE);
  strncpy(xr_action_set_create_info.localizedActionSetName,
          xr_action_set_params.localized_action_set_name.c_str(),
          XR_MAX_LOCALIZED_ACTION_SET_NAME_SIZE);
  XrActionSetReference xr_action_set_reference{.xr_action_set_params =
                                                   xr_action_set_params};

  // Create the XrActionSet.
  XrResult xr_create_action_set_result = xrCreateActionSet(
      xr_session_host_.GetXrInstance(), &xr_action_set_create_info,
      &xr_action_set_reference.xr_action_set_handle);
  MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xr_create_action_set_result));

  // Save a reference to the new XrActionSet.
  action_set_name_to_xr_action_set_reference_[xr_action_set_params
                                                  .action_set_name] =
      xr_action_set_reference;

  return xr_action_set_reference.xr_action_set_handle;
}

absl::Status XrActionController::SetupXrAction(
    XrActionParams xr_action_params) {
  // Transform strings into XrPaths.
  std::vector<XrPath> subaction_paths;
  tsl::robin_map<XrPath, std::string> xr_subaction_path_to_xr_subaction_path;
  for (const std::string& subaction_path_string :
       xr_action_params.subaction_paths) {
    MP_ASSIGN_OR_RETURN(XrPath xr_subaction_path,
                     StringToXrPath(xr_session_host_, subaction_path_string));
    subaction_paths.push_back(xr_subaction_path);
    xr_subaction_path_to_xr_subaction_path[xr_subaction_path] =
        subaction_path_string;
  }

  XrActionType xr_action_type;
  switch (xr_action_params.action_type) {
    case XrActionParams::ActionType::BOOLEAN_INPUT:
      xr_action_type = XR_ACTION_TYPE_BOOLEAN_INPUT;
      break;
    case XrActionParams::ActionType::FLOAT_INPUT:
      xr_action_type = XR_ACTION_TYPE_FLOAT_INPUT;
      break;
    case XrActionParams::ActionType::VECTOR2F_INPUT:
      xr_action_type = XR_ACTION_TYPE_VECTOR2F_INPUT;
      break;
    case XrActionParams::ActionType::POSE_INPUT:  // Transform<float>
      xr_action_type = XR_ACTION_TYPE_POSE_INPUT;
      break;
    case XrActionParams::ActionType::HAPTIC_OUTPUT:
      xr_action_type = XR_ACTION_TYPE_VIBRATION_OUTPUT;
      break;
    case XrActionParams::ActionType::NOT_SET:
      return absl::InvalidArgumentError("XrActionParams was NOT_SET.");
  }

  // Create the XrAction.
  XrActionCreateInfo xr_action_create_info = {
      .type = XR_TYPE_ACTION_CREATE_INFO,
      .actionType = xr_action_type,
      .countSubactionPaths = static_cast<uint32_t>(subaction_paths.size()),
      .subactionPaths = subaction_paths.data(),
  };
  strncpy(xr_action_create_info.actionName,
          xr_action_params.action_name.c_str(), XR_MAX_ACTION_NAME_SIZE);
  strncpy(xr_action_create_info.localizedActionName,
          xr_action_params.localized_action_name.c_str(),
          XR_MAX_LOCALIZED_ACTION_NAME_SIZE);

  // Resolve the XrActionSet handle that was created earlier.
  XrActionSetReference& xr_action_set_reference =
      action_set_name_to_xr_action_set_reference_.at(
          xr_action_params.action_set_name);
  XrActionReference xr_action_reference = {
      .xr_action_params = xr_action_params,
      .xr_subaction_path_to_subaction_path =
          std::move(xr_subaction_path_to_xr_subaction_path),
      .action_name_hash = StringHasher()(xr_action_params.action_name)};

  XrResult xr_create_action_result = xrCreateAction(
      xr_action_set_reference.xr_action_set_handle, &xr_action_create_info,
      &xr_action_reference.xr_action_handle);
  MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xr_create_action_result));

  // For XR_ACTION_TYPE_POSE_INPUT XrActions, we need an XrActionSpace.
  if (xr_action_params.action_type == XrActionParams::ActionType::POSE_INPUT) {
    // Create one XrSpace per subaction path.
    for (int i = 0; i < subaction_paths.size(); ++i) {
      std::string subaction_path_string = xr_action_params.subaction_paths[i];
      XrPath xr_subaction_path = subaction_paths[i];
      XrSpace xr_action_space = XR_NULL_HANDLE;
      XrActionSpaceCreateInfo action_space_info{
          .type = XR_TYPE_ACTION_SPACE_CREATE_INFO,
          .action = xr_action_reference.xr_action_handle,
          .subactionPath = xr_subaction_path,
      };
      action_space_info.poseInActionSpace.orientation.w = 1.f;
      XrResult xr_create_action_space_result =
          xrCreateActionSpace(xr_session_host_.GetXrSession(),
                              &action_space_info, &xr_action_space);
      MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xr_create_action_space_result));
      xr_action_reference
          .xr_subaction_path_to_xr_action_space[xr_subaction_path] =
          xr_action_space;
    }
  }
  // Save the XrAction.
  xr_action_set_reference
      .action_name_to_xr_action[xr_action_params.action_name] =
      xr_action_reference.xr_action_handle;
  xr_action_set_reference
      .xr_action_to_xr_action_reference[xr_action_reference.xr_action_handle] =
      xr_action_reference;
  return absl::OkStatus();
}

absl::Status XrActionController::SetupXrActionSetBindings(
    XrActionSetBindingParams xr_action_set_binding_params) {
  // Resolve the XrActionSet.
  XrActionSetReference& xr_action_set_reference =
      action_set_name_to_xr_action_set_reference_.at(
          xr_action_set_binding_params.action_set_name);
  // Create the vector of XrActionSuggestBindings.
  std::vector<XrActionSuggestedBinding> bindings;
  for (const XrActionBindingParams& xr_action_binding_params :
       xr_action_set_binding_params.action_binding_params) {
    XrPath binding_path;
    XrResult xr_string_to_path_result = xrStringToPath(
        xr_session_host_.GetXrInstance(),
        xr_action_binding_params.binding_path.c_str(), &binding_path);
    MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xr_string_to_path_result));
    XrAction xr_action =
        xr_action_set_reference
            .action_name_to_xr_action[xr_action_binding_params.action_name];
    bindings.push_back(XrActionSuggestedBinding{
        .action = xr_action_set_reference.xr_action_to_xr_action_reference
                      .at(xr_action)
                      .xr_action_handle,
        .binding = binding_path});
  }

  // Suggest these bindings to the default interaction profile.
  XrInteractionProfileSuggestedBinding suggested_bindings{
      .type = XR_TYPE_INTERACTION_PROFILE_SUGGESTED_BINDING,
      .countSuggestedBindings = static_cast<uint32_t>(bindings.size()),
      .suggestedBindings = bindings.data(),
  };
  MP_ASSIGN_OR_RETURN(
      suggested_bindings.interactionProfile,
      StringToXrPath(xr_session_host_,
                     xr_action_set_binding_params.interaction_profile_path));
  XrResult xr_suggest_interaction_profile_bindings_result =
      xrSuggestInteractionProfileBindings(xr_session_host_.GetXrInstance(),
                                          &suggested_bindings);
  if (xr_suggest_interaction_profile_bindings_result ==
      XR_ERROR_PATH_UNSUPPORTED) {
    // If the interaction profile is unsupported, we can safely ignore it.
    return absl::OkStatus();
  }
  MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(
      xr_suggest_interaction_profile_bindings_result));
  return absl::OkStatus();
}

// Go through each action and update it and send to inputmanager.
absl::Status XrActionController::OnFocusedWaitFrame() {
  IMP_TRACE();
  if (!have_action_sets_been_attached_) {
    return absl::InternalError("XrActionSets have not yet been attached.");
  }
  std::optional<XrTime> predicted_time =
      xr_session_host_.GetPredictedDisplayTime();
  if (!predicted_time.has_value()) {
    return absl::InternalError("No predicted time.");
  }
  if (xr_session_host_.GetXrSpace() == XR_NULL_HANDLE) {
    return absl::InternalError("Reference space is null.");
  }

  // Sync the XrActions in each XrActionSet.
  for (auto it = action_set_name_to_xr_action_set_reference_.begin();
       it != action_set_name_to_xr_action_set_reference_.end(); ++it) {
    auto& xr_action_set_reference = it.value();
    XrActiveActionSet xr_active_action_set = {
        .actionSet = xr_action_set_reference.xr_action_set_handle,
        .subactionPath = XR_NULL_PATH,
    };
    XrActionsSyncInfo sync_info = {
        .type = XR_TYPE_ACTIONS_SYNC_INFO,
        .countActiveActionSets = 1,
        .activeActionSets = &xr_active_action_set,
    };

    // Syncs the XrActions.
    XrResult xr_sync_actions_result =
        xrSyncActions(xr_session_host_.GetXrSession(), &sync_info);
    MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xr_sync_actions_result));

    // Builds InputActionEvents and sends them to the InputManager.
    MP_RETURN_IF_ERROR(SyncXrActionSetState(xr_action_set_reference));
  }
  return absl::OkStatus();
}

absl::Status XrActionController::SyncXrActionSetState(
    XrActionSetReference& xr_action_set_reference) {
  IMP_TRACE();
  // We bundle together the states of XrActions with the same XrActionSet and
  // the same subaction path.
  // The result is an InputActionEvent that, generally, represents all
  // input from a single device or controller.

  tsl::robin_map<XrPath, InputActionEvent>&
      subaction_path_to_input_action_event =
          xr_action_set_reference.xr_subaction_path_to_input_action_event;
  for (const auto& [xr_action, xr_action_reference] :
       xr_action_set_reference.xr_action_to_xr_action_reference) {
    for (const auto& [xr_subaction_path, subaction_path] :
         xr_action_reference.xr_subaction_path_to_subaction_path) {
      absl::optional<InputActionStateVariant> input_action_state =
          absl::nullopt;
      switch (xr_action_reference.xr_action_params.action_type) {
        case XrActionParams::ActionType::BOOLEAN_INPUT: {
          MP_ASSIGN_OR_RETURN(input_action_state,
                           FetchInputActionState<bool>(xr_action_reference,
                                                       xr_subaction_path));
          break;
        }
        case XrActionParams::ActionType::FLOAT_INPUT: {
          MP_ASSIGN_OR_RETURN(input_action_state,
                           FetchInputActionState<float>(xr_action_reference,
                                                        xr_subaction_path));
          break;
        }
        case XrActionParams::ActionType::VECTOR2F_INPUT: {
          MP_ASSIGN_OR_RETURN(input_action_state,
                           FetchInputActionState<float2>(xr_action_reference,
                                                         xr_subaction_path));
          break;
        }
        case XrActionParams::ActionType::POSE_INPUT: {
          MP_ASSIGN_OR_RETURN(input_action_state,
                           FetchTransformInputActionState(xr_action_reference,
                                                          xr_subaction_path));
          break;
        }
        case XrActionParams::ActionType::HAPTIC_OUTPUT: {
          // Haptic is an output action that is triggered immediately and does
          // not depend on syncing.
          break;
        }
        default: {
          return absl::InternalError("Unsupported ActionType");
        }
      }
      if (input_action_state.has_value()) {
        // Group by subaction path.
        InputActionEvent& input_action_event =
            subaction_path_to_input_action_event[xr_subaction_path];
        input_action_event.action_set_name =
            xr_action_set_reference.xr_action_set_params.action_set_name;
        input_action_event.subaction_path = subaction_path;

        // Use the pre-computed hash since this is a hot-spot for execution.
        auto hint = input_action_event.action_name_to_input_action_state.find(
            xr_action_reference.xr_action_params.action_name,
            xr_action_reference.action_name_hash);
        input_action_event.action_name_to_input_action_state.insert_or_assign(
            hint, xr_action_reference.xr_action_params.action_name,
            *input_action_state);
      }
    }
  }
  // Store the InputActionEvent in InputManager.
  for (auto it = subaction_path_to_input_action_event.begin();
       it != subaction_path_to_input_action_event.end(); ++it) {
    auto& input_action_event = subaction_path_to_input_action_event[it->first];
    view_->GetInputManager().PushInputActionEvent(input_action_event);
    input_action_event.action_name_to_input_action_state.clear();
  }
  return absl::OkStatus();
}

absl::Status XrActionController::ApplyHapticFeedback(
    ApplyHapticFeedbackOptions options, absl::string_view subaction_path,
    std::optional<absl::string_view> action_set_name,
    absl::string_view action_name) {
  MP_ASSIGN_OR_RETURN(
      XrHapticActionInfo haptic_action_info,
      BuildXrHapticActionInfo(subaction_path, action_set_name, action_name));
  // Apply the haptic output action through OpenXR.
  XrHapticVibration vibration{
      .type = XR_TYPE_HAPTIC_VIBRATION,
      .duration = options.duration,
      .frequency = options.frequency,
      .amplitude = options.amplitude,
  };
  XrResult xr_apply_haptic_feedback_result = xrApplyHapticFeedback(
      xr_session_host_.GetXrSession(), &haptic_action_info,
      (const XrHapticBaseHeader*)&vibration);
  MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xr_apply_haptic_feedback_result));
  return absl::OkStatus();
}

absl::Status XrActionController::StopHapticFeedback(
    absl::string_view subaction_path,
    std::optional<absl::string_view> action_set_name,
    absl::string_view action_name) {
  MP_ASSIGN_OR_RETURN(
      XrHapticActionInfo haptic_action_info,
      BuildXrHapticActionInfo(subaction_path, action_set_name, action_name));
  XrResult xr_stop_haptic_feedback_result = xrStopHapticFeedback(
      xr_session_host_.GetXrSession(), &haptic_action_info);
  MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xr_stop_haptic_feedback_result));
  return absl::OkStatus();
}

absl::StatusOr<XrHapticActionInfo> XrActionController::BuildXrHapticActionInfo(
    absl::string_view subaction_path,
    std::optional<absl::string_view> action_set_name,
    absl::string_view action_name) {
  absl::string_view resolved_action_set_name;
  // Use the only action set registered, if there is only one.
  // This is the default case.
  if (!action_set_name.has_value()) {
    if (action_set_name_to_xr_action_set_reference_.size() != 1) {
      return absl::InvalidArgumentError(
          "There is not exactly one action set registered, action_set_name "
          "must be specified.");
    } else {
      resolved_action_set_name =
          action_set_name_to_xr_action_set_reference_.begin()->first;
    }
  } else {
    resolved_action_set_name = *action_set_name;
  }
  if (!action_set_name_to_xr_action_set_reference_.contains(
          resolved_action_set_name)) {
    return absl::InvalidArgumentError(
        "The provided action set name did not match that of any registered "
        "action set.");
  }
  // Validate both action_set_name and action_name.
  XrActionSetReference xr_action_set_reference =
      action_set_name_to_xr_action_set_reference_.at(resolved_action_set_name);

  XrAction xr_action = xr_action_set_reference
                           .action_name_to_xr_action[std::string(action_name)];
  if (auto it = xr_action_set_reference.xr_action_to_xr_action_reference.find(
          xr_action);
      it != xr_action_set_reference.xr_action_to_xr_action_reference.end()) {
    // XrActionReference was found
    XrActionReference xr_action_reference = it->second;
    if (xr_action_reference.xr_action_params.action_type !=
        XrActionParams::ActionType::HAPTIC_OUTPUT) {
      return absl::InvalidArgumentError(
          "The XrAction referenced to by action_name is not a HAPTIC_OUTPUT "
          "action.");
    }

    XrPath xr_path;
    MP_RETURN_IF_ERROR(xr_session_host_.ToStatus(xrStringToPath(
        xr_session_host_.GetXrInstance(), subaction_path.data(), &xr_path)));

    return XrHapticActionInfo{.type = XR_TYPE_HAPTIC_ACTION_INFO,
                              .action = xr_action_reference.xr_action_handle,
                              .subactionPath = xr_path};
  }
  return absl::InvalidArgumentError(
      "The provided action name did not match that of any registered "
      "action.");
}

void XrActionController::SetXrSessionActionConfig(
    XrActionController::XrSessionActionConfig xr_session_action_config) {
  xr_session_action_config_ = xr_session_action_config;
}

void XrActionController::SetInputHandlerConfig(
    XrActionController::InputHandlerConfig input_handler_config) {
  input_handler_config_ = input_handler_config;
}

}  // namespace imp
