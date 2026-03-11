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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_MENU_PANEL_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_MENU_PANEL_H_

#include <optional>

#include "absl/time/time.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
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

struct MenuPanelStates {
  // Initial state.
  struct Initialized {};

  // State for a completely invisible menu panel.
  struct Hidden {};

  // State for a visible menu panel.
  struct Active {
    absl::Duration last_interaction_time;
    Ramp<float> alpha;
    Ramp<float> percentage;
  };

  // State machine type for menu panel.
  using Machine = StateMachine<Initialized, Hidden, Active>;
};

// Represents the main menu panel.
class MenuPanel : public imp::Component,
                  public MenuPanelStates::Machine::Observer {
  using float2 = imp::float2;
  using Machine = MenuPanelStates::Machine;

 public:
  MenuPanel();
  ~MenuPanel() override;

  // Initializes a menu panel component.
  void Setup(SceneViewerXrSessionListener* session_listener,
             android_xr::SubspaceRoot* subspace_root);

  // Cleans up a menu panel component.
  void Cleanup();

  // Explicitly updated by the client, don't use Component::Update mechanism.
  void OnUpdate(const imp::FrameTime& delta_time,
                const InteractionMode& interaction_data);

  bool IsVisible();
  float GetAlpha();

 private:
  // Observer method.
  void OnStateChange(const Machine& machine,
                     const Machine::State& current_state,
                     const Machine::State& next_state) override;
  // Non-trivial update methods.
  Machine::OptionalState UpdateActive(MenuPanelStates::Active& state,
                                      const InteractionMode& interaction_data,
                                      const imp::FrameTime& delta_time);
  Machine::OptionalState UpdateHidden(MenuPanelStates::Hidden& state,
                                      const InteractionMode& interaction_data);

  // Updates the position and orientation of the panel based on the model's
  // and camera's position and applies all the changes to the Android view.
  void UpdateTransformAndView();

  // State machine for the menu panel.
  MenuPanelStates::Machine machine_;
  // The node containing the model the menu panel grounds.
  imp::NodeHandle model_node_;
  // The session listener to notify of changes in the menu UI.
  SceneViewerXrSessionListener* session_listener_;
  // The subspaceroot instance to be able to update the panel view.
  android_xr::SubspaceRoot* subspace_root_;
  // This component depends on the footprint;
  ::imp::ComponentHandle<Footprint> footprint_;
};

}  // namespace svxr
#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_MENU_PANEL_H_
