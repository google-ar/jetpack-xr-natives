/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_INTERACTION_STATES_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_INTERACTION_STATES_H_

#include "absl/time/time.h"
#include "extensions/sceneviewerxr/ux/ramp.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"
#include "core/collision/ray.h"
#include "core/common/smooth.h"
#include "core/math/math.h"

namespace svxr {
namespace interaction_states {

// State for when the machine is initialized.
struct Initialized {};

// State for a model that is not currently being interacted with.
struct Idle {
  Ramp<float> inactive_time_counter;
  bool gaze_input_since_last_update = false;
};

// State for when the asset is being translated.
struct Translation {
  imp::Ray initial_world_space_ray;
  imp::Ray current_world_space_ray;

  absl::Duration active_duration = absl::ZeroDuration();
  imp::float3 initial_world_space_rig_to_hit;
  imp::float3 initial_world_space_hit_position;
  imp::Smooth<imp::float3> rig_local_position;
  imp::Smooth<imp::float3> footprint_local_position;
  bool is_right;
  bool is_active;
  float cumulative_change_delta;
  float initial_distance_ratio;
  bool has_translated = false;
};

// State for when the asset is being rotated about the Y axis.
struct Rotation {
  absl::Duration active_duration = absl::ZeroDuration();
  imp::Ray initial_world_space_ray;
  imp::Ray current_world_space_ray;
  imp::quatf initial_rig_rotation;
  bool is_right;
  bool is_rotating_after_two_handed_scale;
  float cumulative_change_delta;
  bool has_rotated = false;
};

// State initiated with a pinch after a short tap, performs scaling
struct OneHandedScale {
  imp::Ray initial_world_space_ray;
  imp::Ray current_world_space_ray;

  float initial_model_log_scale;
  bool is_right;
  bool has_scaled = false;
};

// State initiated by both cursors interacting with the object.
struct TwoHandedScale {
  imp::Ray initial_world_space_ray_right;
  imp::Ray initial_world_space_ray_left;
  imp::Ray current_world_space_ray_right;
  imp::Ray current_world_space_ray_left;

  float initial_model_log_scale;
  bool was_right_translation;
  bool was_left_translation;
  bool has_scaled = false;
};

struct ScaleReset {
  float initial_model_log_scale;
  float final_model_log_scale;
  Ramp<float> minimum_display_duration;
};

struct AccessibilityScale {
  Ramp<float> timeout;
};

// State machine for interactions.
using Machine =
    StateMachine<Initialized, Idle, Translation, Rotation, OneHandedScale,
                 TwoHandedScale, ScaleReset, AccessibilityScale>;

}  // namespace interaction_states
}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_INTERACTION_STATES_H_
