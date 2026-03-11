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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_LOAD_INDICATOR_COMPONENT_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_LOAD_INDICATOR_COMPONENT_H_

#include "extensions/sceneviewerxr/ux/ramp.h"
#include "extensions/sceneviewerxr/ux/scene_viewer_component.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/subspace_root.h"
#include "vr/android_xr/sceneviewerxr/api/sceneviewerxr_session_listener.h"

namespace svxr {

// State machine for the load indicator.
struct LoadIndicatorStates {
  struct Initialize {};

  // Waiting for model loading to begin.
  struct Connecting {
    float initial_download_size;
  };

  // Actively loading model from url.
  struct Loading {
    float initial_download_size;
  };

  // Model has finished loading.
  struct Loaded {
    Ramp<float> visibility_factor;
    Ramp<float> hide_delay;
  };

  // Indicator has closed.
  struct Closed {};

  // State machine type for load indicator.
  using Machine = StateMachine<Initialize, Connecting, Loading, Loaded, Closed>;
};

// Watches the progress of model loading and controls display accordingly.
class LoadIndicatorComponent : public imp::Component,
                               public LoadIndicatorStates::Machine::Observer {
  using Machine = LoadIndicatorStates::Machine;

 public:
  LoadIndicatorComponent();
  ~LoadIndicatorComponent() override;

  void Setup(SceneViewerXrSessionListener* session_listener,
             android_xr::SubspaceRoot& subspace_root);
  void Update(const imp::FrameTime& delta_time);
  void Close();

  void Reset();

 protected:
  SceneViewerXrSessionListener* session_listener_;
  android_xr::SubspaceRoot* subspace_root_;

  // State machine for the load indicator.
  LoadIndicatorStates::Machine machine_;

  void OnStateChange(const Machine& machine,
                     const Machine::State& current_state,
                     const Machine::State& next_state) override {}

  // State machine update loops.
  Machine::OptionalState UpdateConnecting(
      LoadIndicatorStates::Connecting& state, const imp::FrameTime& delta_time);
  Machine::OptionalState UpdateLoading(LoadIndicatorStates::Loading& state,
                                       const imp::FrameTime& delta_time);
  Machine::OptionalState UpdateLoaded(LoadIndicatorStates::Loaded& state,
                                      const imp::FrameTime& delta_time);

  void UpdatePanel(const imp::FrameTime& delta_time, float download_progress,
                   float alpha = 1.0f);

  // Variables related to node position are always relevant.
  imp::Transform<float> node_transform_;
  Ramp<imp::float3> lerped_node_position_;

  imp::NodeHandle rig_node_;

  void CalculateNodeTransform();
};

}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_LOAD_INDICATOR_COMPONENT_H_
