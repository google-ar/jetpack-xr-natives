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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_SCENE_VIEWER_COMPONENT_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_SCENE_VIEWER_COMPONENT_H_

#include <cstdint>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/time/time.h"
#include "extensions/sceneviewerxr/ux/a11y_control.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"
#include "extensions/sceneviewerxr/ux/menu_panel.h"
#include "extensions/sceneviewerxr/ux/plane.h"
#include "extensions/sceneviewerxr/ux/scale_indicator.h"
#include "extensions/sceneviewerxr/ux/state_machine.h"
#include "core/audio/audio_player.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/common/smooth.h"
#include "core/math/quat.h"
#include "core/ncsb/component.h"
#include "imp.h"
#include "split_engine/input/split_engine_input_event.h"
#include "split_engine/subspace_root.h"
#include "vr/android_xr/sceneviewerxr/api/sceneviewerxr_session_listener.h"

namespace svxr {

// The SceneViewerComponent builds a rig with a 3D model and UX elements.  This
// lives in space flinger rather than aGSA in order to satisfy MH6 deliverables.
// TODO Move this code out of space flinger into some more durable
// place.
class SceneViewerComponent : public imp::Component,
                             public interaction_states::Machine::Observer,
                             public interaction_states::InteractionOwner {
  using InteractionMachine = interaction_states::Machine;
  using OptionalInteractionState = InteractionMachine::OptionalState;

 public:
  SceneViewerComponent();

  absl::Status Setup(imp::NodeHandle model_node,
                     SceneViewerXrSessionListener* session_listener,
                     android_xr::SubspaceRoot& subspace_root);

  void SetModelSound(absl::string_view sound_url);
  void SetModelTitle(absl::string_view title);
  void SetupAndPlayAnimation();

  void Cleanup();
  void Update(const imp::FrameTime& delta_time);

  void ResetSize();

  void OnEnvironmentVisibilityChanged(bool visibility);
  void SetIdleTimeoutEnabled(bool enabled);
  void SetVisible(bool visible);

  void ClearPlanes();
  void AddPlane(imp::ComponentHandle<Plane> plane);
  // Accessibility methods
  void RequestAccessibilityRotateLeft();
  void RequestAccessibilityRotateRight();
  void RequestAccessibilityScale(float scale);

  // InteractionOwner methods
  // LINT.IfChange
  InteractionMode& GetInteractionData() override;
  imp::ComponentHandle<Footprint> GetFootprint() override;
  imp::NodeHandle GetFootprintNode() override;
  imp::NodeHandle GetModelNode() override;
  imp::NodeHandle GetRigNode() override;
  imp::float3 GetHeadPosition() override;
  imp::Smooth<float>& GetModelLogScale() override;
  float GetResetLogScale() override;
  void ResetRigPosition() override;
  void ToggleResetScaleType() override;
  bool IsTalkbackEnabled() override;
  bool IsIdleTimeoutEnabled() override;
  // LINT.ThenChange(//depot/google3/vr/android_xr/sceneviewerxr/ux/interaction_states_tests/interaction_states_test_fixture.h)

 private:
// TODO: Custom cursors are not yet supported yet.
#ifdef CUSTOM_CURSOR_SPRITES
  // Helper structure for representing a billboarded, textured quad.
  struct TexturedQuad {
    // The node holding the quad geometry.
    imp::NodeHandle quad_node;
    // Handle to the RenderComponent.
    imp::ComponentHandle<imp::RenderComponent> render_component;
  };

  // Visual state for floating cursors on the model.
  struct CursorVisualState {
    imp::float3 target_pos;
  };
#endif

  InteractionMode interaction_data_;

  enum class InputEventSource : uint32_t {
    kModel,
    kFootprint,
    kSubspaceRoot,
  };
  enum class ResetScaleType : uint32_t {
    kInitialScale,
    kOneToOne,
  };
  enum class EnvironmentType : uint32_t {
    kUnknown,
    kPassthrough,
    kHomeEnvironment,
  };

  struct AxisBounds {
    float min;
    float max;
  };
  static constexpr struct AxisBounds kEnvironmentYBounds =
      AxisBounds(-2.0f, 5.0f);
  static constexpr struct AxisBounds kPassthroughYBounds =
      AxisBounds(-10.0f, 6.0f);
  static constexpr struct AxisBounds kEnvironmentXZBounds =
      AxisBounds(.001f, 2.75f);
  static constexpr struct AxisBounds kPassthroughXZBounds =
      AxisBounds(.001f, 10.0f);

  // Processes SplitEngineInputEventProto for input.
  void HandleInputEvent(const android_xr::SplitEngineInputEvent& event,
                        InputEventSource source);

  // Handles the digested contents of a motion event.
  void HandleInputInternal(const imp::Ray& ray, imp::NodeHandle receiver,
                           const imp::float3& hit_position,
                           imp::Flags<InputFlag> input_flags);

  // Non-trivial update methods.
  OptionalInteractionState UpdateTranslation(
      interaction_states::Translation& state, const imp::FrameTime& delta_time);
  OptionalInteractionState UpdateRotation(interaction_states::Rotation& state,
                                          const imp::FrameTime& delta_time);
  OptionalInteractionState UpdateOneHandedScale(
      interaction_states::OneHandedScale& state,
      const imp::FrameTime& delta_time);
  OptionalInteractionState UpdateTwoHandedScale(
      interaction_states::TwoHandedScale& state,
      const imp::FrameTime& delta_time);
  OptionalInteractionState UpdateScaleReset(
      interaction_states::ScaleReset& state, const imp::FrameTime& delta_time);
  OptionalInteractionState UpdateAccessibilityScale(
      interaction_states::AccessibilityScale& state,
      const imp::FrameTime& delta_time);

  // Input handling methods
  OptionalInteractionState HandleInitializedInput(
      interaction_states::Initialized& state);
  OptionalInteractionState HandleTranslationInput(
      interaction_states::Translation& state, const imp::Ray& ray,
      imp::NodeHandle receiver, imp::Flags<InputFlag> input_flags);
  OptionalInteractionState HandleRotationInput(
      interaction_states::Rotation& state, const imp::Ray& ray,
      imp::NodeHandle receiver, imp::Flags<InputFlag> input_flags);
  OptionalInteractionState HandleOneHandedScaleInput(
      interaction_states::OneHandedScale& state, const imp::Ray& ray,
      imp::NodeHandle receiver, imp::Flags<InputFlag> input_flags);
  OptionalInteractionState HandleTwoHandedScaleInput(
      interaction_states::TwoHandedScale& state, const imp::Ray& ray,
      imp::NodeHandle receiver, const imp::float3& hit_position,
      imp::Flags<InputFlag> input_flags);
  // Observer methods.
  void OnStateChange(const InteractionMachine& machine,
                     const InteractionMachine::State& current_state,
                     const InteractionMachine::State& next_state) override;

  // Setup and return a new idle state.
  interaction_states::Idle SetupIdleState();

  // Helper methods
  void SetModelScale(float model_scale);
  void PlaySound();

  // State machine for interaction.
  InteractionMachine interaction_machine_;
  // The rig is a logical node that contains the footprint and the model.
  imp::NodeHandle rig_node_ = imp::NodeHandle();
  // The model that is being displayed by Scene Viewer.
  imp::NodeHandle model_node_ = imp::NodeHandle();
  // The unit offset from the authored origin to the rig-relative origin.
  imp::float3 model_offset_ = imp::float3(0.0f);
  // The lower/upper bounds for the model scale.
  AxisBounds model_log_scale_limits_ = AxisBounds(0.0f, 0.0f);
  // A controller for the model scale.
  imp::Smooth<float> model_log_scale_;
  // A controller for the rig position.
  imp::Smooth<imp::float3> rig_position_;
  // Reference to the animator.
  imp::ComponentHandle<imp::GltfAnimator> animator_;
  // Reference to the audio player.
  imp::ComponentHandle<imp::AudioPlayer> audio_player_;
  // The model title.
  absl::string_view model_title_;

  // The initial rotation of the rig node when the model last started rotating.
  imp::quatf initial_rig_rotation_;
  // A controller for the model rotation amount around some axis.
  imp::Smooth<imp::quatf> rig_rotation_;
  // Saved handle to the camera component.
  imp::ComponentHandle<imp::CameraComponent> camera_ =
      imp::ComponentHandle<imp::CameraComponent>();
  // Handle to the footprint component placed on the rig node.
  imp::ComponentHandle<Footprint> footprint_ =
      imp::ComponentHandle<Footprint>();
  // Handle to the scale indicator component placed on the rig node.
  imp::ComponentHandle<ScaleIndicator> scale_indicator_ =
      imp::ComponentHandle<ScaleIndicator>();
  // Handle to the menu panel component placed on the rig node.
  imp::ComponentHandle<MenuPanel> menu_panel_;
  // Handle to the a11y rotate left control component placed on the rig node.
  imp::ComponentHandle<A11yRotateLeftControl> a11y_rotate_left_control_;
  // Handle to the a11y rotate right control component placed on the rig node.
  imp::ComponentHandle<A11yRotateRightControl> a11y_rotate_right_control_;
  // Handle to the a11y scale control component placed on the rig node.
  imp::ComponentHandle<A11yScaleControl> a11y_scale_control_;
#if defined(CUSTOM_CURSOR_SPRITES)
  // Visual state for the cursors.
  std::optional<CursorVisualState> right_visual_state_ = std::nullopt;
  std::optional<CursorVisualState> left_visual_state_ = std::nullopt;
#endif

  imp::Dispatcher::Connection footprint_event_connection_;
  imp::Dispatcher::Connection model_event_connection_;
  android_xr::SplitEngineInputEvent prev_right_input_;
  android_xr::SplitEngineInputEvent prev_left_input_;

  SceneViewerXrSessionListener* session_listener_;
  android_xr::SubspaceRoot* subspace_root_;

  imp::ComponentHandle<imp::AudioPlayer> drop_audio_player_;
  imp::ComponentHandle<imp::AudioPlayer> lift_audio_player_;
  bool dropped_ = false;  // Indicates if the model was dropped onto a plane.

  bool DoesRayIntersectModel(imp::Ray ray);
  bool IsInputEventHovering(const android_xr::SplitEngineInputEvent& event);
  bool is_previous_left_ray_hovering_ = false;
  bool is_previous_right_ray_hovering_ = false;

  // Stores the initial model scale to be able to reset to it.
  float initial_model_scale_ = 1.0f;
  // Stores the initial distance between camera-model to be able to reset to it.
  float initial_model_distance_to_camera_ = 0.0f;
  // Stores the type of reset scaling to be performed.
  ResetScaleType reset_scale_type_ = ResetScaleType::kOneToOne;

  // Stores the type of environment the user is in.
  EnvironmentType environment_type_ = EnvironmentType::kUnknown;
  // Stores a flag to control if the idle timeout should be enabled or not.
  bool idle_timeout_enabled_ = true;

  std::vector<imp::ComponentHandle<Plane>> planes_;

  // Projects the target position on to each plane, checks for overlap with the
  // plane geometry, and returns the target position for the footprint.
  imp::float3 ComputeFootprintPositionFromPlanes(imp::float3 target_position,
                                                 imp::float3 rig_to_target);

  bool FootprintReceivesInput();
  imp::float3 GetRigToCameraXz();
  float ConstrainElastically(float value, AxisBounds range, float scale);
  void RequestUpdateRigPositionFromCamera(
      const imp::SmoothParameters& parameters);
  void ConstrainRigPosition();
  void CalculateModelScaleLimits();
  void PlayDropSound();
  void PauseAnimationAndSound();
  void ResumeAnimationAndSound();
};

}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_SCENE_VIEWER_COMPONENT_H_
