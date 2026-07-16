/*
 * Copyright 2026 Google LLC
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

#include "extensions/sceneviewerxr/ux/interaction_states/shutdown.h"

#include <cmath>

#include "core/common/smooth.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"

namespace svxr {
namespace interaction_states {

Machine::OptionalState Update(const imp::FrameTime& delta_time, Shutdown& state,
                              InteractionOwner& owner) {
  float current_log_scale = owner.GetModelLogScale().Get();
  float distance = std::abs(current_log_scale - state.target_model_log_scale);
  bool model_scaled_down = distance < kScaleArrivalThreshold;

  bool stopping = model_scaled_down && state.minimum_duration.IsAtTarget();

  if (stopping && !state.shutdown_called) {
    state.shutdown_called = true;
    if (owner.GetUiEventListener()) {
      owner.GetUiEventListener()->OnShutdown();
    }
    owner.TriggerShutdownCallback();
  }

  state.minimum_duration.Step(delta_time.GetDeltaTime());

  return {};
}

}  // namespace interaction_states
}  // namespace svxr
