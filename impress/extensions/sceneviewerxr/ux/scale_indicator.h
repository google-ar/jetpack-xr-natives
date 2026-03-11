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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_SCALE_INDICATOR_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_SCALE_INDICATOR_H_

#include <optional>

#include "absl/time/time.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/ramp.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"
#include "core/math/math.h"
#include "core/ncsb/component.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/subspace_root.h"
#include "vr/android_xr/sceneviewerxr/api/sceneviewerxr_session_listener.h"

namespace svxr {

struct ScaleIndicatorStates {
  // Initial state.
  struct Initialized {};

  // State for a completely invisible scale indicator.
  struct Hidden {};

  // State for a visible footprint.
  struct Active {
    absl::Duration last_interaction_time;
    Ramp<float> hide_delay;
    Ramp<float> alpha;
    Ramp<float> percentage;
    Ramp<float> y_offset;
  };

  // State machine type for scale indicator.
  using Machine = StateMachine<Initialized, Hidden, Active>;
};

// Represents a white rounded rectangle that appears at the base of a model.
class ScaleIndicator : public imp::Component,
                       public ScaleIndicatorStates::Machine::Observer {
  using float2 = imp::float2;
  using Machine = ScaleIndicatorStates::Machine;

 public:
  ScaleIndicator();
  ~ScaleIndicator() override;

  // Initializes a scale indicator component.
  void Setup(imp::NodeHandle model_node,
             SceneViewerXrSessionListener* session_listener,
             android_xr::SubspaceRoot* subspace_root);
  // Cleans up a scale indicator component.
  void Cleanup();

  // Explicitly updated by the client, don't use Component::Update mechanism.
  void OnUpdate(const imp::FrameTime& delta_time,
                const InteractionMode& interaction_data);

  bool IsVisible();
  float GetScalePercentage();
  float GetAlpha();
  float GetYOffset();

 private:
  // Observer method.
  void OnStateChange(const Machine& machine,
                     const Machine::State& current_state,
                     const Machine::State& next_state) override;
  // Non-trivial update methods.
  Machine::OptionalState UpdateActive(ScaleIndicatorStates::Active& state,
                                      const InteractionMode& interaction_data,
                                      const imp::FrameTime& delta_time);
  Machine::OptionalState UpdateHidden(ScaleIndicatorStates::Hidden& state,
                                      const InteractionMode& interaction_data);
  // Helper methods.
  float GetModelScalePercentage();
  void UpdateTransformAndView();

  // State machine for the footprint.
  ScaleIndicatorStates::Machine machine_;
  // The node containing the model the footprint grounds.
  imp::NodeHandle model_node_;
  // The session listener to notify of changes in the menu UI.
  SceneViewerXrSessionListener* session_listener_;
  // The subspaceroot instance to be able to update the panel view.
  android_xr::SubspaceRoot* subspace_root_;
};

}  // namespace svxr
#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_SCALE_INDICATOR_H_
