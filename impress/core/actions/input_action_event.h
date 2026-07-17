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

#ifndef THIRD_PARTY_IMPRESS_CORE_INPUT_XR_XR_INPUT_ACTION_EVENT_H_
#define THIRD_PARTY_IMPRESS_CORE_INPUT_XR_XR_INPUT_ACTION_EVENT_H_

#include <cstdint>
#include <string>
#include <variant>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "core/common/platform_helpers.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/view/utils/string_map.h"

namespace imp {

template <typename T>
struct InputActionState {
  std::string name;
  bool has_changed_since_last_sync;
  int64_t last_change_time;
  bool is_active;
  T current_state;
  int64_t event_tracing_id;
};

template <>
struct InputActionState<Transform<float>> {
  std::string name;
  bool has_changed_since_last_sync;
  int64_t last_change_time;
  bool is_active;
  Transform<float> current_state;
  // If the position/pose of the Transform<float> action is valid.
  bool is_position_valid;
  // If the rotation/orientation of the Transform<float> action is valid.
  bool is_rotation_valid;
  // If the position/pose is actively being tracked. If false, the
  // rotation may represent a last known or inferred value.
  bool is_position_tracked;
  // If the rotation/orientation is actively being tracked. If false, the
  // rotation may represent a last known or inferred value.
  bool is_rotation_tracked;
};

using InputActionStateVariant =
    std::variant<InputActionState<bool>, InputActionState<float>,
                 InputActionState<int>, InputActionState<float2>,
                 InputActionState<Transform<float>>>;

// An InputActionEvent holds a batch of InputActionStates. All InputActionStates
// in the same batch share the same action set and subaction path. Generally,
// this means they come from the same controller.
struct InputActionEvent {
  std::string action_set_name;
  std::string subaction_path;
  StringMap<InputActionStateVariant> action_name_to_input_action_state;

  bool HasInputActionState(absl::string_view name) const {
    return action_name_to_input_action_state.contains(name);
  }

  // Returns the InputActionStateVariant for the given action name. Fatals if
  // the InputActionState cannot be resolved.
  const InputActionStateVariant& GetInputActionStateVariant(
      absl::string_view name) const {
    if (auto input_action_state = action_name_to_input_action_state.find(name);
        input_action_state != action_name_to_input_action_state.end()) {
      return input_action_state->second;
    } else {
      IMP_LOG(imp::FATAL) << "action name \"" << name
                 << "\" not found in InputActionEvent with action_set_name \""
                 << action_set_name << "\" and subaction_path \""
                 << subaction_path << "\"";
    }
  }

  // Returns the InputActionState for the given action name. Fatals if the
  // InputActionState cannot be resolved or exists with a type different than
  // the one specified.
  template <typename T>
  const InputActionState<T>& GetInputActionState(absl::string_view name) const {
    const InputActionStateVariant& input_action_state_variant =
        GetInputActionStateVariant(name);
    if (!std::holds_alternative<InputActionState<T>>(
            input_action_state_variant)) {
      IMP_LOG(imp::FATAL) << "Type mismatch for action name \"" << name << "\"";
    }
    return absl::get<InputActionState<T>>(input_action_state_variant);
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_INPUT_XR_XR_INPUT_ACTION_EVENT_H_
