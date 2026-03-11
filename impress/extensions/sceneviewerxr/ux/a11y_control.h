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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_A11Y_CONTROL_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_A11Y_CONTROL_H_

#include "core/ncsb/component.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/subspace_root.h"
#include "vr/android_xr/sceneviewerxr/api/sceneviewerxr_session_listener.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/ramp.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"

namespace svxr {

namespace A11yControlStates {

// Initial state for state machine.
struct Initialized {};

// State for a model that is not currently being interacted with.
struct Hidden {
  Ramp<float> alpha;
};

// State for when the asset is being interacted with.
struct Active {
  Ramp<float> alpha;
};

// State machine for interactions.
using Machine = StateMachine<Initialized, Hidden, Active>;
};  // namespace A11yControlStates

class A11yControl : public imp::Component,
                    public A11yControlStates::Machine::Observer {
 public:
  enum class A11yControlType {
    kRotateLeft,
    kRotateRight,
    kScale,
  };

  A11yControl() = default;
  ~A11yControl() override;

  void Setup(SceneViewerXrSessionListener* session_listener,
             android_xr::SubspaceRoot* subspace_root);
  void Cleanup();

  void OnUpdate(const imp::FrameTime& delta_time,
                const InteractionMode& interaction_data);

  // Observer methods.
  void OnStateChange(
      const A11yControlStates::Machine& machine,
      const A11yControlStates::Machine::State& current_state,
      const A11yControlStates::Machine::State& next_state) override;

  bool IsVisible();
  float GetAlpha();

 protected:
  A11yControlType type_;

 private:
  A11yControlStates::Machine::OptionalState UpdateActive(
      A11yControlStates::Active& state, const InteractionMode& interaction_data,
      const imp::FrameTime& delta_time);
  A11yControlStates::Machine::OptionalState UpdateHidden(
      A11yControlStates::Hidden& state,
      const InteractionMode& interaction_data);

  void UpdateTransformAndView();

  A11yControlStates::Machine machine_{A11yControlStates::Initialized{}, this};
  SceneViewerXrSessionListener* session_listener_;
  android_xr::SubspaceRoot* subspace_root_;
  imp::ComponentHandle<Footprint> footprint_;
};

class A11yRotateLeftControl : public A11yControl {
 public:
  A11yRotateLeftControl() { type_ = A11yControlType::kRotateLeft; }
};

class A11yRotateRightControl : public A11yControl {
 public:
  A11yRotateRightControl() { type_ = A11yControlType::kRotateRight; }
};

class A11yScaleControl : public A11yControl {
 public:
  A11yScaleControl() { type_ = A11yControlType::kScale; }
};

}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_A11Y_CONTROL_H_
