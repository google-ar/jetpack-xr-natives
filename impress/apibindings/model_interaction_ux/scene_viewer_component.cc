// Copyright 2025 Google LLC
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

#include "apibindings/model_interaction_ux/scene_viewer_component.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/camera/camera_component.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/common/smooth.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/utils/frame_time.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/idle.h"
#include "extensions/sceneviewerxr/ux/interaction_states/initial.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"
#include "extensions/sceneviewerxr/ux/interaction_states/rotation.h"
#include "extensions/sceneviewerxr/ux/interaction_states/scale_reset.h"
#include "extensions/sceneviewerxr/ux/interaction_states/translation.h"
#include "extensions/sceneviewerxr/ux/ui_event_listener.h"
#include "split_engine/input/split_engine_input_event.h"


namespace imp {

namespace {

using ::android_xr::SplitEngineInputEvent;
using float2 = ::imp::float2;
using float3 = ::imp::float3;
using float4 = ::imp::float4;
using int2 = ::imp::int2;
using mat4f = ::imp::mat4f;
using mat4 = ::imp::mat4;
using quatf = ::imp::quatf;
using Box = ::imp::Box;
template <typename T>
using Transform = ::imp::Transform<T>;
using RayHit = ::imp::RayHit;
using NodeHandle = ::imp::NodeHandle;
using InteractionMachine = svxr::interaction_states::Machine;
using OptionalInteractionState = InteractionMachine::OptionalState;
using InteractionMode = svxr::InteractionMode;
using InputFlag = svxr::InputFlag;

constexpr auto kPlaneEpsilon = 1e-3f;

constexpr auto kOneHandedScaleUnit = float3(100.f, 0.17f, 0.13f);
constexpr auto kOneHandedScaleDeadzone = float3(0.05f);
constexpr auto kOneHandedScaleThrow = 2.0f;
constexpr auto kOneHandedScaleMultiplier = 1.0f;

constexpr auto kMinimumRotationDelta = .5f;

// If the model is within this distance of the camera it won't be scaled up.
constexpr auto kMinAllowedModelToCameraDistance = 0.4f;
constexpr auto kBeginDistanceConstraint =
    kMinAllowedModelToCameraDistance + 1.5f;

constexpr float kElasticScale = 10.f;

// The rate used when correcting the rig position based on translation bounds.
constexpr auto kTranslationRigCorrectionRate = 5.0f;
constexpr auto kIdleRigCorrectionRate = 0.15f;

// The delay before the footprint is initialized. This is to avoid the
// footprint being initialized when the model is not fully loaded.
constexpr auto kFootprintInitializationDelay = 2.5f;

// Ray origin/forward are in world space.
mat4f GetRayFromWorldSpace(const imp::Ray& ray) {
  auto world_from_ray =
      mat4f::lookAt(ray.origin, ray.origin + ray.direction, imp::kUp);

  return inverse(world_from_ray);
}

}  // namespace

// TODO: Add unit tests for this component.
SceneViewerComponent::SceneViewerComponent()
    : interaction_machine_(svxr::interaction_states::Initialized{}, this),
      model_node_(),
      footprint_() {}
absl::Status SceneViewerComponent::Setup(imp::NodeHandle target_node,
                                         bool system_movable) {
  auto& view = GetView();
  camera_ = view.GetCameraManager().GetCamera();

  subspace_root_ = target_node->GetParent();

  rig_node_ = view.CreateNode();

  model_node_ = target_node;
  rig_node_->SetParent(subspace_root_);
  model_node_->SetParent(rig_node_);

  model_event_connection_ = model_node_->Connect(
      [this](const SplitEngineInputEvent& event) mutable {
        HandleInputEvent(event, InputEventSource::kModel);
      },
      this);

  // Setup initial flags
  interaction_data_ = InteractionMode();
  system_movable_ = system_movable;

  return absl::OkStatus();
}

void SceneViewerComponent::CreateFootprint(const imp::FrameTime& delta_time) {
  // TODO: Remove initialization delay for footprint component.
  // This is a temporary solution to avoid an edge case where the footprint
  // doesn't get get mirrored correctly if the component is created immediately
  // after the subspace is has been created.
  if (delta_time.GetElapsedSeconds() < kFootprintInitializationDelay) {
    return;
  }
  is_footprint_initialized_ = true;
  rig_node_->AddComponent<svxr::Footprint>(model_node_)
      .Then([this](imp::ComponentHandle<svxr::Footprint> footprint) {
        footprint_ = footprint;
        // Listen to input events on the footprint
        auto footprint_node = footprint_->FootprintNode();
        footprint_node->SetEnabled(false);
        footprint_->OnInteractionMachineInitialized();
        footprint_event_connection_ = footprint_node->Connect(
            [this](const SplitEngineInputEvent& event) {
              event_hit_node_transform_ = event.hit_node->transform;
              event_hit_position_ = event.hit_node->hit_position;
              HandleInputEvent(event, InputEventSource::kFootprint);
            },
            this);
      })
      .KeptBy(this);
}

void SceneViewerComponent::Cleanup() {
  model_node_->SetParent(rig_node_->GetParent());
  rig_node_->SetParent(NodeHandle());
}

void SceneViewerComponent::SetModelScale(float model_scale) {
  auto model_transform = imp::Transform<float>(model_node_->GetLocalTrs());
  model_transform.scale = float3(model_scale);
  model_transform.translation = model_offset_ * model_scale;
  model_node_->SetLocalTrs(model_transform.AsMat4());

  footprint_->OnModelSizeChanged();
}

bool SceneViewerComponent::DoesRayIntersectModel(imp::Ray ray) {
  imp::Box bounds =
      model_node_->GetComponent<imp::GltfRenderer>()->GetLocalBounds();
  auto intersection = imp::collision::AABBIntersectsRay(
      bounds, ray.GetTransformed(inverse(model_node_->GetWorldTrs())));

  return intersection.has_value();
}

bool SceneViewerComponent::IsInputEventHovering(
    const SplitEngineInputEvent& event) {
  auto subspace_from_world = subspace_root_->GetWorldTrs();
  auto world_from_subspace = inverse(subspace_from_world);
  imp::Ray ray((world_from_subspace * event.origin).xyz,
               (world_from_subspace * float4(event.direction, 0)).xyz);

  if (event.dispatch_flag ==
      SplitEngineInputEvent::DispatchFlag::CAPTURED_POINTER) {
    return DoesRayIntersectModel(ray);
  }
  return true;
}

void SceneViewerComponent::HandleInputEvent(const SplitEngineInputEvent& event,
                                            InputEventSource source) {
  if (event.origin == imp::kZero3) {
    return;
  }

  bool is_mouse = event.device_type == SplitEngineInputEvent::DeviceType::MOUSE;
  bool is_right =
      event.pointer_type == SplitEngineInputEvent::PointerType::RIGHT;
  bool is_left = event.pointer_type == SplitEngineInputEvent::PointerType::LEFT;
  if (!is_mouse && !is_right && !is_left) {
    return;
  }

  auto subspace_from_world = subspace_root_->GetWorldTrs();
  auto world_from_subspace = inverse(subspace_from_world);
  // The transform for the panel (in task space).

  imp::Ray ray((world_from_subspace * event.origin).xyz,
               (world_from_subspace * float4(event.direction, 0)).xyz);

  imp::Flags<svxr::InputFlag> input_flags;
  if (is_right || is_mouse) {
    input_flags = svxr::GenerateInputFlags(event, prev_right_input_,
                                           IsInputEventHovering(event),
                                           is_previous_right_ray_hovering_);
    prev_right_input_ = event;
  } else {
    input_flags = svxr::GenerateInputFlags(event, prev_left_input_,
                                           IsInputEventHovering(event),
                                           is_previous_left_ray_hovering_);
    prev_left_input_ = event;
  }

  auto hit_position = imp::kZero3;
  bool hit_node_is_valid = event.hit_node && event.hit_node->target;
  imp::NodeHandle receiver_node = event.GetTargetNode();
  if (hit_node_is_valid) {
    // TODO: hit_position is routed into the shader as a worldspace
    // position, but it doesn't match the hit point.  Why are these two
    // formulations of how to compute the hit position both incorrect (and
    // different)? Why are neither of these correct? Or at least identically
    // incorrect? hit_position = ray.origin + ray.direction;
    hit_position = (world_from_subspace * event.hit_node->hit_position).xyz;
  } else if (event.dispatch_flag ==
                 SplitEngineInputEvent::DispatchFlag::CAPTURED_POINTER &&
             input_flags.Test(svxr::InputFlag::kIsHover)) {
    receiver_node = model_node_;
  }
  // TODO: Determine if this should be called even if there is
  // not a primary hit. Also, what about the secondary hit?
  HandleInputInternal(ray, receiver_node, hit_position, input_flags);

  if (footprint_ && (source != InputEventSource::kSubspaceRoot) &&
      FootprintReceivesInput()) {
    footprint_->HandleInputEvent(hit_position, is_right,
                                 ReceiverIsFootprint(receiver_node));
  }
}

bool SceneViewerComponent::FootprintReceivesInput() {
  return interaction_machine_.ApplyWithAlternatives(
      // During translation, the footprint can move around and does not get
      // input.
      [](svxr::interaction_states::Translation& state) -> bool {
        return false;
      },
      // For all other states, the footprint receives input.
      [](auto& state) -> bool { return true; });
}

void SceneViewerComponent::HandleInputInternal(
    const imp::Ray& ray, imp::NodeHandle receiver, const float3& hit_position,
    imp::Flags<svxr::InputFlag> input_flags) {
  // Update the state machine in response to handling input.
  interaction_machine_.UpdateWithAlternatives(
      [&](svxr::interaction_states::Initialized& state)
          -> OptionalInteractionState { return HandleInput(state, *this); },
      [&](svxr::interaction_states::Translation& state)
          -> OptionalInteractionState {
        return HandleTranslationInput(state, ray, receiver, input_flags);
      },
      [&](svxr::interaction_states::Idle& state) -> OptionalInteractionState {
        if (footprint_ && (footprint_->IsScaleHandle(receiver) ||
                           footprint_->IsCloseToScaleHandle(hit_position))) {
          if (input_flags.Test(InputFlag::kIsDown)) {
            interaction_data_.SetTransform(
                InteractionMode::TransformMode::kScale);
            return svxr::interaction_states::OneHandedScale{
                .initial_world_space_ray = ray,
                .current_world_space_ray = ray,
                .initial_model_log_scale = model_log_scale_.Get(),
                .is_right = input_flags.Test(InputFlag::kIsRight),
                .has_scaled = false};
          }
        }
        return HandleInput(state, ray, receiver, hit_position, input_flags,
                           *this);
      },
      [&](svxr::interaction_states::Rotation& state)
          -> OptionalInteractionState {
        return svxr::interaction_states::HandleInput(state, ray, receiver,
                                                     input_flags, *this);
      },
      [](auto& state) -> OptionalInteractionState { return {}; });
}

imp::float3 SceneViewerComponent::GetRigToCameraXz() {
  auto world_from_camera = camera_->GetCamera()->getModelMatrix();
  float3 camera_world_position = (world_from_camera * imp::kZero3).xyz;
  float3 camera_world_position_xz =
      (camera_world_position)*float3(1.f, 0.f, 1.f);
  float3 rig_world_position = (rig_node_->GetWorldTrs() * imp::kZero3).xyz;
  float3 rig_world_position_xz = (rig_world_position)*float3(1.f, 0.f, 1.f);
  return camera_world_position_xz - rig_world_position_xz;
}

float SceneViewerComponent::ConstrainElastically(float value, AxisBounds range,
                                                 float scale) {
  if (value < range.min) {
    float excess = range.min - value;
    float adjusted_excess = std::log(1.f + excess * scale) / scale;
    return range.min - adjusted_excess;
  } else if (value > range.max) {
    float excess = value - range.max;
    float adjusted_excess = std::log(1.f + excess * scale) / scale;
    return range.max + adjusted_excess;
  }

  return value;
}

void SceneViewerComponent::RequestUpdateRigPositionFromCamera(
    const imp::SmoothParameters& parameters) {
  // Radius of a circle that encloses the scaled footprint.
  imp::float3 rig_to_camera_xz = GetRigToCameraXz();
  float current_distance_to_camera = length(rig_to_camera_xz);

  float footprint_radius = length(footprint_->GetFootprintSize()) * 0.5f;
  float minimum_distance_to_camera = footprint_radius +
                                     kMinAllowedModelToCameraDistance +
                                     svxr::kFootprintSlop;

  // If camera is closer than minimum distance, we smoothly translate the
  // rig.
  if (current_distance_to_camera < minimum_distance_to_camera) {
    imp::float3 delta_world_space =
        -(normalize(rig_to_camera_xz) *
          (minimum_distance_to_camera - current_distance_to_camera));
    if (rig_node_->GetWorldPosition() != rig_position_.Get()) {
      rig_position_.Setup(parameters, rig_node_->GetWorldPosition());
    }
    rig_position_.SetTarget(rig_position_.Get() + delta_world_space);
  }
}

void SceneViewerComponent::ConstrainRigPosition() {
  bool is_translating = interaction_data_.TestTransform(
      InteractionMode::TransformMode::kTranslate);
  bool no_transform =
      interaction_data_.TestTransform(InteractionMode::TransformMode::kNothing);

  if (no_transform) {
    RequestUpdateRigPositionFromCamera(
        svxr::kSmoothFastResolvingPositionParameters);
  }

  // If we are translating, we want to correct the target y position more
  // aggressively than when the rig is catching up to a given target position.
  float correction_rate =
      is_translating ? kTranslationRigCorrectionRate : kIdleRigCorrectionRate;

  imp::float3 target_position = rig_position_.GetTarget();
  AxisBounds world_y_bounds =
      (environment_type_ == EnvironmentType::kHomeEnvironment
           ? kEnvironmentYBounds
           : kPassthroughYBounds);
  AxisBounds world_xz_bounds =
      (environment_type_ == EnvironmentType::kHomeEnvironment
           ? kEnvironmentXZBounds
           : kPassthroughXZBounds);

  // Convert the bounds to rig space.
  AxisBounds rig_y_bounds;
  rig_y_bounds.min = (inverse(rig_node_->GetWorldTrs()) *
                      imp::float3(0, world_y_bounds.min, 0))
                         .y;
  rig_y_bounds.max = (inverse(rig_node_->GetWorldTrs()) *
                      imp::float3(0, world_y_bounds.max, 0))
                         .y;

  // Track whether the model is near the camera.
  auto world_from_camera = camera_->GetCamera()->getModelMatrix();
  float3 camera_world_position = (world_from_camera * imp::kZero3).xyz;
  float3 rig_to_camera_xz =
      (camera_world_position - rig_position_.Get()) * float3(1.f, 0.f, 1.f);
  bool model_near_camera = length(rig_to_camera_xz) <= kBeginDistanceConstraint;

  // Keeps the rig from overshooting dramatically past the current bounds
  // if the rig position after an environment switch is far away.
  if (no_transform) {
    // If the rig is idle and far from the camera, reset its target to its
    // current position. This prevents drift from previous interactions but
    // allows the push-back from camera to complete.
    if (!model_near_camera && rig_position_.IsAtTarget()) {
      target_position = rig_position_.Get();
    }
  }

  // Constrain the current y target within the y bounds.
  target_position.y =
      ConstrainElastically(target_position.y, rig_y_bounds, correction_rate);

  // Constrain current x and z target only against furthest allowed distance.
  // We want to preserve modifications from RequestUpdateRigPositionFromCamera.
  if (!model_near_camera) {
    auto length_xz =
        length(imp::float3(target_position.x, 0.f, target_position.z));
    auto constrained_length_xz =
        ConstrainElastically(length_xz, world_xz_bounds, correction_rate);
    target_position.x = target_position.x * constrained_length_xz / length_xz;
    target_position.z = target_position.z * constrained_length_xz / length_xz;
  }

  // Set the target position.
  rig_position_.SetTarget(target_position);
}

void SceneViewerComponent::CalculateModelScaleLimits() {
  imp::Box local_bounds =
      model_node_->GetComponent<imp::GltfRenderer>()->GetLocalBounds();

  // TODO: Consider making the scale limits dynamic based on
  // the distance to the camera.
  // Determine the scale limits.
  float largest_dimension =
      std::max(local_bounds.halfExtent.x,
               std::max(local_bounds.halfExtent.y, local_bounds.halfExtent.z)) *
      2.f;
  float smallest_dimension =
      std::min(local_bounds.halfExtent.x,
               std::min(local_bounds.halfExtent.y, local_bounds.halfExtent.z)) *
      2.f;

  float scale_min = svxr::kSmallestModelSize /
                    std::max(largest_dimension, svxr::kModelSizeEpsilon);
  float scale_max = svxr::kLargestModelSize /
                    std::max(smallest_dimension, svxr::kModelSizeEpsilon);

  // Account for the framed model size.
  // TODO: Be sure to revisit this after deeper discussion
  // on how to handle scaling for edge cases has been held.
  scale_min = std::min(std::min(scale_min, scale_max), initial_model_scale_);
  scale_max = std::max(std::max(scale_min, scale_max), initial_model_scale_);

  model_log_scale_limits_ =
      AxisBounds{std::log(scale_min), std::log(scale_max)};

  // Account for the environment type.
  AxisBounds world_y_bounds =
      (environment_type_ == EnvironmentType::kHomeEnvironment
           ? kEnvironmentYBounds
           : kPassthroughYBounds);
  AxisBounds world_xz_bounds =
      (environment_type_ == EnvironmentType::kHomeEnvironment
           ? kEnvironmentXZBounds
           : kPassthroughXZBounds);

  constexpr float kPadding = 0.25f;
  float radius = length(
      imp::float3(local_bounds.halfExtent.x, 0.f, local_bounds.halfExtent.z));
  float world_y_range =
      std::log((world_y_bounds.max - world_y_bounds.min - kPadding) /
               std::max(radius, svxr::kModelSizeEpsilon));
  float world_xz_range = std::log(
      (world_xz_bounds.max - kMinAllowedModelToCameraDistance - kPadding) /
      std::max(radius, svxr::kModelSizeEpsilon));
  model_log_scale_limits_.max = std::min(
      model_log_scale_limits_.max, std::min(world_y_range, world_xz_range));
}

void SceneViewerComponent::OnStateChange(
    const InteractionMachine& machine,
    const InteractionMachine::State& current_state,
    const InteractionMachine::State& next_state) {}

void SceneViewerComponent::Update(const imp::FrameTime& delta_time) {
  interaction_data_.SetActive(InteractionMode::ActiveMode::kNothing);

  interaction_machine_.UpdateWithAlternatives(
      [&](svxr::interaction_states::Translation& state)
          -> OptionalInteractionState {
        interaction_data_.SetActive(InteractionMode::ActiveMode::kInteracting);
        return svxr::interaction_states::Update(delta_time, state, *this);
      },
      [&](svxr::interaction_states::Rotation& state)
          -> OptionalInteractionState {
        interaction_data_.SetActive(InteractionMode::ActiveMode::kInteracting);
        return svxr::interaction_states::Update(state, delta_time, *this);
      },
      [](auto& state) -> OptionalInteractionState { return {}; });
  if (footprint_) {
    footprint_->OnUpdate(delta_time, interaction_data_);
  } else if (!is_footprint_initialized_) {
    CreateFootprint(delta_time);
  }
}

svxr::interaction_states::Idle SceneViewerComponent::SetupIdleState() {
  interaction_data_.SetTransform(InteractionMode::TransformMode::kNothing);
  return svxr::interaction_states::SetupIdleState();
}

void SceneViewerComponent::SetVisible(bool visible) {
  if (visible) {
    rig_node_->SetEnabled(true);
    // TODO: this is not used, should be removed once
    // the behavior matches SVXR.
    ResumeAnimationAndSound();
  } else {
    rig_node_->SetEnabled(false);
    // Set the model to selected state to ensure that the panel is shown and not
    // to assume that the timeout happened while the model was hidden.
    interaction_data_.SetSelected(InteractionMode::SelectedMode::kSelected);
    PauseAnimationAndSound();
  }
}

InteractionMode& SceneViewerComponent::GetInteractionData() {
  return interaction_data_;
}
imp::ComponentHandle<svxr::Footprint> SceneViewerComponent::GetFootprint() {
  return footprint_;
}
imp::NodeHandle SceneViewerComponent::GetFootprintNode() {
  return footprint_ ? footprint_->FootprintNode() : imp::NodeHandle();
}
imp::NodeHandle SceneViewerComponent::GetModelNode() { return model_node_; }
imp::NodeHandle SceneViewerComponent::GetRigNode() { return rig_node_; }
imp::ComponentHandle<imp::CameraComponent> SceneViewerComponent::GetCamera() {
  return camera_;
}
bool SceneViewerComponent::IsTalkbackEnabled() { return false; }
bool SceneViewerComponent::IsIdleTimeoutEnabled() {
  return idle_timeout_enabled_;
}

float3 SceneViewerComponent::GetHeadPosition() { return imp::kZero3; }

imp::Smooth<float>& SceneViewerComponent::GetModelLogScale() {
  return model_log_scale_;
}

float SceneViewerComponent::GetResetLogScale() { return 0.0f; }

std::optional<imp::float3> SceneViewerComponent::GetAnchorSnapPosition(
    imp::float3 footprint_position_local) {
  // TODO: enable snappable planes affordance for JXR.
  return std::nullopt;
}

void SceneViewerComponent::PlayDropSound() {
  if (drop_audio_player_) {
    if (!drop_audio_player_->Play().ok()) { IMP_LOG(imp::ERROR) << "Failed to play drop audio"; }
  }
}

void SceneViewerComponent::PlayLiftSound() {
  if (lift_audio_player_) {
    if (!lift_audio_player_->Play().ok()) { IMP_LOG(imp::ERROR) << "Failed to play lift audio"; }
  }
}

bool SceneViewerComponent::IsPassthrough() {
  return environment_type_ == EnvironmentType::kPassthrough;
}

imp::float3 SceneViewerComponent::ComputeFootprintPositionFromPlanes(
    float3 target_position, float3 rig_to_target) {
  // TODO: enable snappable planes affordance for JXR.
  return imp::kZero3;
}

OptionalInteractionState SceneViewerComponent::UpdateOneHandedScale(
    svxr::interaction_states::OneHandedScale& state,
    const imp::FrameTime& delta_time) {
  float3 delta_translation = state.current_world_space_ray.origin -
                             state.initial_world_space_ray.origin;
  float3 delta_translation_with_deadzone =
      greaterThan(delta_translation, kOneHandedScaleDeadzone) *
          (delta_translation - kOneHandedScaleDeadzone) +
      lessThan(delta_translation, -kOneHandedScaleDeadzone) *
          (delta_translation + kOneHandedScaleDeadzone);
  float3 unit_delta_translation =
      delta_translation_with_deadzone / kOneHandedScaleUnit;
  float scalar_delta_translation = dot(unit_delta_translation, float3(1.f));
  float scale_offset =
      pow(fabs(scalar_delta_translation), kOneHandedScaleThrow) *
      ((scalar_delta_translation < 0.f) ? -1.f : 1.f) *
      kOneHandedScaleMultiplier;

  if (std::abs(scale_offset) > 1e-5f) {
    state.has_scaled = true;
  }

  auto model_log_scale = state.initial_model_log_scale + scale_offset;
  auto constrained_model_log_scale = ConstrainElastically(
      model_log_scale, model_log_scale_limits_, kElasticScale);
  model_log_scale_.SetTarget(constrained_model_log_scale);
  return {};
}

OptionalInteractionState SceneViewerComponent::UpdateTwoHandedScale(
    svxr::interaction_states::TwoHandedScale& state,
    const imp::FrameTime& delta_time) {
  float initial_pinch_distance =
      length(state.initial_world_space_ray_left.origin -
             state.initial_world_space_ray_right.origin);
  float current_pinch_distance =
      length(state.current_world_space_ray_left.origin -
             state.current_world_space_ray_right.origin);

  float current_scale = current_pinch_distance / initial_pinch_distance;
  if (std::abs(current_scale - 1.0f) > 1e-5f) {
    state.has_scaled = true;
  }

  float model_log_scale =
      std::log(std::exp(state.initial_model_log_scale) * current_scale);
  auto constrained_model_log_scale = ConstrainElastically(
      model_log_scale, model_log_scale_limits_, kElasticScale);

  model_log_scale_.SetTarget(constrained_model_log_scale);
  return {};
}

OptionalInteractionState SceneViewerComponent::UpdateScaleReset(
    const imp::FrameTime& delta_time,
    svxr::interaction_states::ScaleReset& state) {
  return svxr::interaction_states::Update(delta_time, state, *this);
}

OptionalInteractionState SceneViewerComponent::HandleOneHandedScaleInput(
    svxr::interaction_states::OneHandedScale& state, const imp::Ray& ray,
    imp::NodeHandle receiver, imp::Flags<InputFlag> input_flags) {
  // Ignore events for pointers which did not initiate one handed scale.
  if (state.is_right != input_flags.Test(InputFlag::kIsRight)) {
    return {};
  }

  if (!input_flags.Test(InputFlag::kIsDown)) {
    return OptionalInteractionState{SetupIdleState()};
  } else {
    // If we are still down, we are potentially scaling.
    interaction_data_.SetTransform(InteractionMode::TransformMode::kScale);
    state.current_world_space_ray = ray;
  }
  return {};
}

OptionalInteractionState SceneViewerComponent::HandleTwoHandedScaleInput(
    svxr::interaction_states::TwoHandedScale& state, const imp::Ray& ray,
    imp::NodeHandle receiver, const float3& hit_position,
    imp::Flags<InputFlag> input_flags) {
  if (input_flags.Test(InputFlag::kIsDownStopping)) {
    // Transition to single-hand gesture
    bool use_translation = false;
    if (use_translation) {
      // do we need to save hit positions in two-handed scale to safely
      // degrade to translation?
    } else {
      auto rig_rotation = rig_node_->GetLocalRotation();
      auto& other_ray = input_flags.Test(InputFlag::kIsRight)
                            ? state.current_world_space_ray_left
                            : state.current_world_space_ray_right;

      return OptionalInteractionState{svxr::interaction_states::Rotation{
          .initial_world_space_ray = other_ray,
          .current_world_space_ray = other_ray,
          .initial_rig_rotation = rig_rotation,
          .is_right = !input_flags.Test(InputFlag::kIsRight),
          .is_rotating_after_two_handed_scale = true,
          .cumulative_change_delta = 0}};
    }
  }
  interaction_data_.SetTransform(InteractionMode::TransformMode::kScale);
  (input_flags.Test(InputFlag::kIsRight) ? state.current_world_space_ray_right
                                         : state.current_world_space_ray_left) =
      ray;
  return {};
}

OptionalInteractionState SceneViewerComponent::HandleTranslationInput(
    svxr::interaction_states::Translation& state, const imp::Ray& ray,
    imp::NodeHandle receiver, imp::Flags<InputFlag> input_flags) {
  auto returned_state = HandleInput(state, ray, receiver, input_flags, *this);
  // Check if the system is movable. When it is not, the client
  // (SceneCore/Compose) is responsible for handling the translation.
  if (system_movable_ && !returned_state.has_value()) {
    auto rig_position = rig_node_->GetLocalPosition();
    // Hit positions are sent from SpF relative to the task space. SplitEngine
    // converts the hit position to be relative to the subspace they will be
    // handled by (i.e. the hit node). We convert origin & direction to also be
    // relative to the subspace before operating on them.
    auto origin = (event_hit_node_transform_ * float4(ray.origin, 1.0f)).xyz;
    auto direction = normalize(
        (event_hit_node_transform_ * float4(ray.direction, 0.0f)).xyz);

    if (input_flags.Test(InputFlag::kIsDownStarting)) {
      // Calculate the distance between the hit position and the origin.
      origin_to_hit_position_distance_ = length(event_hit_position_ - origin);
      // Calculate the pinch point along the ray scaled by the distance to the
      // hit position.
      auto pinchPoint = origin + (direction * origin_to_hit_position_distance_);

      // Calculate the offset between the rig and the pinch point when pinch
      // starts.
      rig_to_hit_position_offset_ = rig_position - pinchPoint;
    } else if (input_flags.Test(InputFlag::kIsDown)) {
      // Calculate the pinch point along the ray scaled by the distance to the
      // hit position.
      auto pinchPoint = origin + (direction * origin_to_hit_position_distance_);
      // Use the pinchPoint and offset to cached when the pinch first started
      // to determine the new rig position.
      rig_position = pinchPoint + rig_to_hit_position_offset_;

      rig_position_.SetTarget(rig_position);
      rig_node_->SetLocalPosition(rig_position);
    }
  }
  return returned_state;
}

void SceneViewerComponent::PauseAnimationAndSound() {
  if (animator_ && animator_->IsEnabled()) {
    animator_->SetEnabled(false);
    if (audio_player_) {
      auto status = audio_player_->Pause();
      if (!status.ok()) {
        IMP_LOG(imp::ERROR) << "Failed to pause audio: " << status;
      }
    }
  }
}

// TODO: this is not used, should be removed once
// the behavior matches SVXR.
void SceneViewerComponent::ResumeAnimationAndSound() {
  if (animator_ && !animator_->IsEnabled()) {
    animator_->SetEnabled(true);
    if (audio_player_) {
      auto status = audio_player_->Play();
      if (!status.ok()) {
        IMP_LOG(imp::ERROR) << "Failed to play audio: " << status;
      }
    }
  }
}

void SceneViewerComponent::ResetRigPosition() {}

void SceneViewerComponent::ToggleResetScaleType() {}

void SceneViewerComponent::SetModelLogScale(imp::SmoothParameters parameters,
                                            float model_log_scale) {
  model_log_scale_.Setup(parameters, model_log_scale);
}

imp::Smooth<imp::float3>& SceneViewerComponent::GetRigPosition() {
  return rig_position_;
}

float SceneViewerComponent::GetInitialModelScale() {
  return initial_model_scale_;
}
void SceneViewerComponent::SetInitialModelScale(float initial_model_scale) {
  initial_model_scale_ = initial_model_scale;
}

float SceneViewerComponent::GetInitialModelDistanceToCamera() {
  return initial_model_distance_to_camera_;
}
void SceneViewerComponent::SetInitialModelDistanceToCamera(
    float initial_model_distance_to_camera) {
  initial_model_distance_to_camera_ = initial_model_distance_to_camera;
}

svxr::ResetScaleType SceneViewerComponent::GetResetScaleType() {
  return reset_scale_type_;
}
void SceneViewerComponent::SetResetScaleType(
    svxr::ResetScaleType reset_scale_type) {
  reset_scale_type_ = reset_scale_type;
}

void SceneViewerComponent::SetRigPosition(imp::SmoothParameters parameters,
                                          imp::float3 rig_position) {
  rig_position_.Setup(parameters, rig_position);
}
void SceneViewerComponent::SetRigRotationTarget(imp::quatf rig_rotation) {
  rig_rotation_.SetTarget(rig_rotation);
}
void SceneViewerComponent::SetRigRotation(imp::SmoothParameters parameters,
                                          imp::quatf rig_rotation) {
  rig_rotation_.Setup(parameters, rig_rotation);
}

svxr::UiEventListener* SceneViewerComponent::GetUiEventListener() {
  return nullptr;
}

}  // namespace imp
