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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_MODEL_INTERACTION_UX_SCENE_VIEWER_COMPONENT_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_MODEL_INTERACTION_UX_SCENE_VIEWER_COMPONENT_H_

#include <cstdint>

#include "absl/status/status.h"
#include "core/audio/audio_player.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/common/smooth.h"
#include "core/math/quat.h"
#include "core/ncsb/component.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/idle.h"
#include "extensions/sceneviewerxr/ux/interaction_states/initial.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"
#include "extensions/sceneviewerxr/ux/interaction_states/rotation.h"
#include "extensions/sceneviewerxr/ux/ui_event_listener.h"
#include "imp.h"
#include "split_engine/input/split_engine_input_event.h"

namespace imp {

// This component is a fork of the scene_viewer_component in the
// vr/android_xr/sceneviewerxr/ux directory. It has been stripped down
// and modified to only include what is necessary to support JXR.
// TODO: Unify this component with the original
// SceneViewerComponent.
class SceneViewerComponent : public imp::Component,
                             public svxr::interaction_states::Machine::Observer,
                             public svxr::interaction_states::InteractionOwner {
  using InteractionMachine = svxr::interaction_states::Machine;
  using OptionalInteractionState = InteractionMachine::OptionalState;

 public:
  SceneViewerComponent();

  absl::Status Setup(imp::NodeHandle target_node, bool system_movable);

  void Cleanup();
  void Update(const imp::FrameTime& delta_time);

  void SetVisible(bool visible);

  // InteractionOwner methods
  // LINT.IfChange
  svxr::InteractionMode& GetInteractionData() override;
  imp::ComponentHandle<svxr::Footprint> GetFootprint() override;
  imp::NodeHandle GetFootprintNode() override;
  imp::NodeHandle GetModelNode() override;
  imp::NodeHandle GetRigNode() override;
  imp::ComponentHandle<imp::CameraComponent> GetCamera() override;
  imp::float3 GetHeadPosition() override;
  imp::Smooth<float>& GetModelLogScale() override;
  float GetResetLogScale() override;
  void ResetRigPosition() override;
  void ToggleResetScaleType() override;
  bool IsTalkbackEnabled() override;
  bool IsIdleTimeoutEnabled() override;
  void SetModelLogScale(imp::SmoothParameters parameters,
                        float model_log_scale) override;
  imp::Smooth<imp::float3>& GetRigPosition() override;
  float GetInitialModelScale() override;
  void SetInitialModelScale(float initial_model_scale) override;
  float GetInitialModelDistanceToCamera() override;
  void SetInitialModelDistanceToCamera(
      float initial_model_distance_to_camera) override;
  svxr::ResetScaleType GetResetScaleType() override;
  void SetResetScaleType(svxr::ResetScaleType reset_scale_type) override;
  void SetRigPosition(imp::SmoothParameters parameters,
                      imp::float3 rig_position) override;
  void SetRigRotation(imp::SmoothParameters parameters,
                      imp::quatf rig_rotation) override;
  void SetRigRotationTarget(imp::quatf rig_rotation) override;
  void CalculateModelScaleLimits() override;
  void SetModelScale(float model_scale) override;
  void RequestUpdateRigPositionFromCamera(
      const imp::SmoothParameters& parameters) override;
  svxr::AxisBounds GetModelLogScaleLimits() override;
  float ConstrainElastically(float value, svxr::AxisBounds range,
                             float scale) override;
  svxr::UiEventListener* GetUiEventListener() override;
  void TriggerShutdownCallback() override;

  // Translation State Dependencies
  std::optional<imp::float3> GetAnchorSnapPosition(
      imp::float3 footprint_position_local) override;
  imp::float3 ComputeFootprintPositionFromPlanes(
      imp::float3 target_position, imp::float3 rig_to_target) override;
  void PlayDropSound() override;
  void PlayLiftSound() override;
  void PlayGrabSound() override;
  void PlayReleaseSound() override;
  bool IsPassthrough() override;

  // LINT.ThenChange(//depot/google3/third_party/impress/extensions/sceneviewerxr/ux/interaction_states_tests/interaction_states_test_fixture.h)

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

  svxr::InteractionMode interaction_data_;

  enum class InputEventSource : uint32_t {
    kModel,
    kFootprint,
    kSubspaceRoot,
  };
  enum class EnvironmentType : uint32_t {
    kUnknown,
    kPassthrough,
    kHomeEnvironment,
  };

  static constexpr struct svxr::AxisBounds kEnvironmentYBounds =
      svxr::AxisBounds(-2.0f, 5.0f);
  static constexpr struct svxr::AxisBounds kPassthroughYBounds =
      svxr::AxisBounds(-10.0f, 6.0f);
  static constexpr struct svxr::AxisBounds kEnvironmentXZBounds =
      svxr::AxisBounds(.001f, 2.75f);
  static constexpr struct svxr::AxisBounds kPassthroughXZBounds =
      svxr::AxisBounds(.001f, 10.0f);

  // Processes SplitEngineInputEventProto for input.
  void HandleInputEvent(const android_xr::SplitEngineInputEvent& event,
                        InputEventSource source);

  // Handles the digested contents of a motion event.
  void HandleInputInternal(const imp::Ray& ray, imp::NodeHandle receiver,
                           const imp::float3& hit_position,
                           imp::Flags<svxr::InputFlag> input_flags);

  // Non-trivial update methods.
  OptionalInteractionState UpdateTwoHandedScale(
      svxr::interaction_states::TwoHandedScale& state,
      const imp::FrameTime& delta_time);
  OptionalInteractionState UpdateScaleReset(
      const imp::FrameTime& delta_time,
      svxr::interaction_states::ScaleReset& state);
  OptionalInteractionState UpdateAccessibilityScale(
      svxr::interaction_states::AccessibilityScale& state,
      const imp::FrameTime& delta_time);

  // Input handling methods
  OptionalInteractionState HandleInitializedInput(
      svxr::interaction_states::Initialized& state);
  OptionalInteractionState HandleTwoHandedScaleInput(
      svxr::interaction_states::TwoHandedScale& state, const imp::Ray& ray,
      imp::NodeHandle receiver, const imp::float3& hit_position,
      imp::Flags<svxr::InputFlag> input_flags);
  OptionalInteractionState HandleTranslationInput(
      svxr::interaction_states::Translation& state, const imp::Ray& ray,
      imp::NodeHandle receiver, imp::Flags<svxr::InputFlag> input_flags);
  // Observer methods.
  void OnStateChange(const InteractionMachine& machine,
                     const InteractionMachine::State& current_state,
                     const InteractionMachine::State& next_state) override;

  // Setup and return a new idle state.
  svxr::interaction_states::Idle SetupIdleState();

  // Helper methods
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
  svxr::AxisBounds model_log_scale_limits_ = svxr::AxisBounds(0.0f, 0.0f);
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
  imp::ComponentHandle<svxr::Footprint> footprint_ =
      imp::ComponentHandle<svxr::Footprint>();
#if defined(CUSTOM_CURSOR_SPRITES)
  // Visual state for the cursors.
  std::optional<CursorVisualState> right_visual_state_ = std::nullopt;
  std::optional<CursorVisualState> left_visual_state_ = std::nullopt;
#endif

  imp::Dispatcher::Connection footprint_event_connection_;
  imp::Dispatcher::Connection model_event_connection_;
  android_xr::SplitEngineInputEvent prev_right_input_;
  android_xr::SplitEngineInputEvent prev_left_input_;

  imp::NodeHandle subspace_root_;

  imp::ComponentHandle<imp::AudioPlayer> drop_audio_player_;
  imp::ComponentHandle<imp::AudioPlayer> lift_audio_player_;
  imp::ComponentHandle<imp::AudioPlayer> grab_audio_player_;
  imp::ComponentHandle<imp::AudioPlayer> release_audio_player_;
  bool dropped_ = false;  // Indicates if the model was dropped onto a plane.

  imp::Box GetModelBounds() const;
  bool DoesRayIntersectModel(imp::Ray ray);
  bool IsInputEventHovering(const android_xr::SplitEngineInputEvent& event);
  bool is_previous_left_ray_hovering_ = false;
  bool is_previous_right_ray_hovering_ = false;
  bool is_footprint_initialized_ = false;

  // Stores the initial model scale to be able to reset to it.
  float initial_model_scale_ = 1.0f;
  // Stores the initial distance between camera-model to be able to reset to it.
  float initial_model_distance_to_camera_ = 0.0f;
  // Stores the type of reset scaling to be performed.
  svxr::ResetScaleType reset_scale_type_ = svxr::ResetScaleType::kOneToOne;

  // Stores the type of environment the user is in.
  EnvironmentType environment_type_ = EnvironmentType::kUnknown;
  // Stores a flag to control if the idle timeout should be enabled or not.
  bool idle_timeout_enabled_ = true;
  // Stores a flag if the component should move the model or not.
  bool system_movable_ = false;
  // Stores the distance between the rig and the ray origin when the user
  // starts to pinch the rig.
  float origin_to_hit_position_distance_ = 0.0f;
  // Stores the offset between the rig and the hit position when the user
  // starts to pinch the rig.
  imp::float3 rig_to_hit_position_offset_ = imp::kZero3;
  // Stores the hit position of the ray obtained from the split engine input
  // event.
  imp::float3 event_hit_position_ = imp::kZero3;
  // Stores the transform of the hit node when obtained from the split engine
  // input event.
  imp::mat4f event_hit_node_transform_;

  bool FootprintReceivesInput();
  void CreateFootprint(const imp::FrameTime& delta_time);
  imp::float3 GetRigToCameraXz();

  void ConstrainRigPosition();
  void PauseAnimationAndSound();
  void ResumeAnimationAndSound();
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_MODEL_INTERACTION_UX_SCENE_VIEWER_COMPONENT_H_
