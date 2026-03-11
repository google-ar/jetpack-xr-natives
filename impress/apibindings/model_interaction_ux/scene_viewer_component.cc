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
#include "absl/time/time.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/input_flag.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/idle.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_states.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/ray.h"
#include "core/common/enum_flags.h"
#include "core/common/smooth.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/input/pointer_input_handler.h"
#include "core/view/utils/frame_time.h"
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

constexpr auto kMinimumTranslationDelta = .002f;
constexpr auto kMinimumRotationDelta = .5f;
constexpr auto kDistanceRatioDeltaScale = 1.0f;

// If the model is within this distance of the camera it won't be scaled up.
constexpr auto kMinAllowedModelToCameraDistance = 0.4f;
constexpr auto kMinAllowedModelToCameraDistanceDuringTranslation = 0.15f;
constexpr auto kBeginDistanceConstraint =
    kMinAllowedModelToCameraDistance + 1.5f;

constexpr float kElasticScale = 10.f;

// The rate used when correcting the rig position based on translation bounds.
constexpr auto kTranslationRigCorrectionRate = 5.0f;
constexpr auto kIdleRigCorrectionRate = 0.15f;

// The delay before the footprint is initialized. This is to avoid the
// footprint being initialized when the model is not fully loaded.
constexpr auto kFootprintInitializationDelay = 1.5f;

// Ray origin/forward are in world space.
mat4f GetRayFromWorldSpace(const imp::Ray& ray) {
  auto world_from_ray =
      mat4f::lookAt(ray.origin, ray.origin + ray.direction, imp::kUp);

  return inverse(world_from_ray);
}

}  // namespace

SceneViewerComponent::SceneViewerComponent()
    : interaction_machine_(svxr::interaction_states::Initialized{}, this),
      model_node_(),
      footprint_() {}
absl::Status SceneViewerComponent::Setup(imp::NodeHandle target_node) {
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
        footprint_event_connection_ = footprint_node->Connect(
            [this](const SplitEngineInputEvent& event) {
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

// TODO: Enable input handling for translation.
void SceneViewerComponent::HandleInputInternal(
    const imp::Ray& ray, imp::NodeHandle receiver, const float3& hit_position,
    imp::Flags<svxr::InputFlag> input_flags) {
  // Update the state machine in response to handling input.
  interaction_machine_.UpdateWithAlternatives(
      [this](svxr::interaction_states::Initialized& state)
          -> OptionalInteractionState { return HandleInitializedInput(state); },
      [this, &hit_position, &ray, &input_flags, &receiver](
          svxr::interaction_states::Idle& state) -> OptionalInteractionState {
        return HandleInput(state, ray, receiver, hit_position, input_flags,
                           *this);
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
      AxisBounds(std::log(scale_min), std::log(scale_max));

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

// TODO: Enable input handling for translation.
void SceneViewerComponent::OnStateChange(
    const InteractionMachine& machine,
    const InteractionMachine::State& current_state,
    const InteractionMachine::State& next_state) {}

void SceneViewerComponent::Update(const imp::FrameTime& delta_time) {
  interaction_data_.SetActive(InteractionMode::ActiveMode::kNothing);

  // TODO: Enable input handling for translation.
  interaction_machine_.UpdateWithAlternatives(
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
bool SceneViewerComponent::IsTalkbackEnabled() { return false; }
bool SceneViewerComponent::IsIdleTimeoutEnabled() {
  return idle_timeout_enabled_;
}

float3 SceneViewerComponent::GetHeadPosition() { return imp::kZero3; }

imp::Smooth<float>& SceneViewerComponent::GetModelLogScale() {
  return model_log_scale_;
}

float SceneViewerComponent::GetResetLogScale() { return 0.0f; }

imp::float3 SceneViewerComponent::ComputeFootprintPositionFromPlanes(
    float3 target_position, float3 rig_to_target) {
  // TODO: enable snappable planes affordance for JXR.
  return imp::kZero3;
}

// TODO: Enable input handling for translation.
OptionalInteractionState SceneViewerComponent::UpdateTranslation(
    svxr::interaction_states::Translation& state,
    const imp::FrameTime& delta_time) {
  auto scene_viewer_node = GetNode();
  state.active_duration += delta_time.GetDeltaTime();

  if (state.is_active) {
    mat4f initial_ray_from_world =
        GetRayFromWorldSpace(state.initial_world_space_ray);
    mat4f current_ray_from_world =
        GetRayFromWorldSpace(state.current_world_space_ray);
    float3 initial_ray_relative_hit_position =
        (initial_ray_from_world * state.initial_world_space_hit_position).xyz;
    float3 current_world_space_hit_position =
        (inverse(current_ray_from_world) * initial_ray_relative_hit_position)
            .xyz;

    float3 origin_delta = state.current_world_space_ray.origin -
                          state.initial_world_space_ray.origin;
    float3 target_delta = origin_delta * (state.initial_distance_ratio - 1.f);

    current_world_space_hit_position += target_delta * kDistanceRatioDeltaScale;

    float3 current_world_space_rig_position =
        current_world_space_hit_position - state.initial_world_space_rig_to_hit;
    float3 target_position = scene_viewer_node->LocalFromWorldPoint(
        current_world_space_rig_position);

    float3 footprint_target_position = imp::kZero3;
    if (environment_type_ == EnvironmentType::kPassthrough) {
      footprint_target_position = ComputeFootprintPositionFromPlanes(
          current_world_space_hit_position,
          state.initial_world_space_rig_to_hit);
    }

    state.footprint_local_position.SetTarget(footprint_target_position);
    state.rig_local_position.SetTarget(target_position);
  }

  // Compare the distance between the camera and the rig.
  auto world_from_camera = camera_->GetCamera()->getModelMatrix();
  float3 camera_world_position = (world_from_camera * imp::kZero3).xyz;
  float3 camera_world_position_xz =
      (camera_world_position)*float3(1.f, 0.f, 1.f);
  float3 rig_world_position = (rig_node_->GetWorldTrs() * imp::kZero3).xyz;
  float3 rig_world_position_xz = (rig_world_position)*float3(1.f, 0.f, 1.f);
  float3 rig_to_camera_xz = camera_world_position_xz - rig_world_position_xz;

  float current_distance_to_camera = length(rig_to_camera_xz);

  state.footprint_local_position.Step(delta_time.GetDeltaSeconds());
  state.rig_local_position.Step(delta_time.GetDeltaSeconds());

  float next_distance_to_camera =
      length(camera_world_position_xz -
             state.rig_local_position.Get() * imp::float3(1.f, 0.f, 1.f));

  float footprint_radius = length(footprint_->GetFootprintSize()) * 0.5f;
  float minimum_distance_to_camera =
      footprint_radius + kMinAllowedModelToCameraDistanceDuringTranslation +
      svxr::kFootprintSlop;

  // If camera would be closer than before and closer than minimum distance, we
  // stop translating.
  if (next_distance_to_camera < minimum_distance_to_camera &&
      next_distance_to_camera < current_distance_to_camera) {
    state.footprint_local_position.Setup(
        svxr::kSmoothSlowResolvingPositionParameters,
        footprint_->FootprintNode()->GetLocalPosition());
    state.rig_local_position.Setup(svxr::kSmoothSlowResolvingPositionParameters,
                                   rig_node_->GetLocalPosition());
    rig_node_->SetLocalPosition(state.rig_local_position.Get());
  } else {
    state.footprint_local_position.SetParameters(
        svxr::kSmoothFastResolvingPositionParameters);
    state.rig_local_position.SetParameters(
        svxr::kSmoothFastResolvingPositionParameters);
  }

  float delta =
      distance(rig_node_->GetLocalPosition(), state.rig_local_position.Get());
  state.cumulative_change_delta += delta;
  rig_position_.SetTarget(state.rig_local_position.Get());
  footprint_->FootprintNode()->SetLocalPosition(
      state.footprint_local_position.Get());

  // If inactive, check if the targets are reached before returning to idle.
  if (!state.is_active && state.footprint_local_position.IsAtTarget() &&
      state.rig_local_position.IsAtTarget()) {
    // PlayDropSound();
    return OptionalInteractionState{SetupIdleState()};
  }

  return {};
}

OptionalInteractionState SceneViewerComponent::UpdateRotation(
    svxr::interaction_states::Rotation& state,
    const imp::FrameTime& delta_time) {
  state.active_duration += delta_time.GetDeltaTime();
  imp::mat4 world_from_camera = camera_->GetCamera()->getModelMatrix();
  imp::mat4 camera_from_world = inverse(world_from_camera);

  float rotation_turntable_angle = 0.f;
  float rotation_scaling_delta = 0.f;
  {
    // Project the initial and current ray direction into the XZ and YZ planes.
    float3 initial_direction_world = state.initial_world_space_ray.direction;
    float3 current_direction_world = state.current_world_space_ray.direction;

    float3 initial_direction =
        (camera_from_world * float4(initial_direction_world, 0.f)).xyz;
    float3 current_direction =
        (camera_from_world * float4(current_direction_world, 0.f)).xyz;

    float3 initial_ray_xz =
        initial_direction - dot(initial_direction, imp::kUp);
    float3 current_ray_xz =
        current_direction - dot(current_direction, imp::kUp);
    float3 initial_ray_yz =
        initial_direction - dot(initial_direction, imp::kRight);
    float3 current_ray_yz =
        current_direction - dot(current_direction, imp::kRight);

    float dot_xz = dot(initial_ray_xz, current_ray_xz);
    float dot_yz = dot(initial_ray_yz, current_ray_yz);
    float determinant_xz = current_ray_xz.x * initial_ray_xz.z -
                           current_ray_xz.z * initial_ray_xz.x;
    float determinant_yz = initial_ray_yz.y * current_ray_yz.z -
                           initial_ray_yz.z * current_ray_yz.y;
    float angle_xz = atan2(determinant_xz, dot_xz);
    float angle_yz = atan2(determinant_yz, dot_yz);

    constexpr auto kRotationTurntableAmplificationScale = -1.0f;
    rotation_turntable_angle = kRotationTurntableAmplificationScale * angle_xz;

    constexpr auto kRotationScaleAmplificationScale = 0.07f;
    constexpr auto kRotationScalingDeadzoneSize = 0.125f;
    float adjusted_angle_yz =
        angle_yz > 0 ? std::max(0.f, angle_yz - kRotationScalingDeadzoneSize)
                     : std::min(0.f, angle_yz + kRotationScalingDeadzoneSize);
    rotation_scaling_delta =
        kRotationScaleAmplificationScale * adjusted_angle_yz;
  }

  float translation_turntable_angle = 0.f;
  float translation_scaling_delta = 0.f;
  {
    // Project the delta translation into the XY plane.
    float3 delta_translation_world = state.current_world_space_ray.origin -
                                     state.initial_world_space_ray.origin;
    float3 delta_translation =
        (camera_from_world * float4(delta_translation_world, 0.f)).xyz;
    float3 delta_translation_xy =
        delta_translation - dot(delta_translation, imp::kForward);

    constexpr auto kTranslationTurntableAmplificationScale = 4.0f;
    translation_turntable_angle =
        kTranslationTurntableAmplificationScale * delta_translation_xy.x;

    constexpr auto kTranslationScaleAmplificationScale = 2.0f;
    constexpr auto kTranslationScaleDeadzoneSize = 0.1f;
    float adjusted_delta_translation =
        delta_translation_xy.y > 0
            ? std::max(0.f,
                       delta_translation_xy.y - kTranslationScaleDeadzoneSize)
            : std::min(0.f,
                       delta_translation_xy.y + kTranslationScaleDeadzoneSize);
    translation_scaling_delta =
        kTranslationScaleAmplificationScale * adjusted_delta_translation;
  }

  float turntable_angle =
      rotation_turntable_angle + translation_turntable_angle;
  state.cumulative_change_delta += turntable_angle;

  if (state.cumulative_change_delta > kMinimumRotationDelta ||
      state.cumulative_change_delta < -kMinimumRotationDelta) {
    state.has_rotated = true;
  }

  auto turntable_rotation = quatf::fromAxisAngle(imp::kUp, turntable_angle);
  rig_rotation_.SetTarget(state.initial_rig_rotation * turntable_rotation);

  return {};
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

// TODO: this is not used, should be removed once
// the behavior matches SVXR.
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

// TODO: this is not used, should be removed once
// the behavior matches SVXR.
OptionalInteractionState SceneViewerComponent::UpdateScaleReset(
    svxr::interaction_states::ScaleReset& state,
    const imp::FrameTime& delta_time) {
  RequestUpdateRigPositionFromCamera(
      svxr::kSmoothFastResolvingPositionParameters);

  // Note that since scale is updated after state machines, this state is exited
  // the frame after reaching unit scale.
  bool stopping = model_log_scale_.IsAtTarget() &&
                  model_log_scale_.Get() == state.final_model_log_scale &&
                  state.minimum_display_duration.IsAtTarget();
  if (stopping) {
    interaction_data_.SetTransform(InteractionMode::TransformMode::kNothing);
    return OptionalInteractionState{SetupIdleState()};
  }
  state.minimum_display_duration.Step(delta_time.GetDeltaTime());

  return {};
}

OptionalInteractionState SceneViewerComponent::HandleInitializedInput(
    svxr::interaction_states::Initialized& state) {
  if (!footprint_) {
    // Defer entering Idle until our UX is ready.
    return {};
  }
  return OptionalInteractionState{SetupIdleState()};
}

// TODO: Enable input handling for translation.
OptionalInteractionState SceneViewerComponent::HandleTranslationInput(
    svxr::interaction_states::Translation& state, const imp::Ray& ray,
    imp::NodeHandle receiver, imp::Flags<InputFlag> input_flags) {
  if (!state.is_active) {
    // Ignore all events if the state is not active.
    return {};
  }

  // Handle events for pointers which did not initiate translation.
  if (state.is_right != input_flags.Test(InputFlag::kIsRight)) {
    if (input_flags.Test(InputFlag::kIsDownStarting)) {
      // Start two-handed scale.
      auto model_scale = model_node_->GetLocalScale().x;
      constexpr auto kEpsilon = 1e-5f;
      auto model_log_scale = std::log(std::max(kEpsilon, model_scale));

      auto& ray_right = state.is_right ? state.current_world_space_ray : ray;
      auto& ray_left = state.is_right ? ray : state.current_world_space_ray;
      return OptionalInteractionState{svxr::interaction_states::TwoHandedScale{
          .initial_world_space_ray_right = ray_right,
          .initial_world_space_ray_left = ray_left,
          .current_world_space_ray_right = ray_right,
          .current_world_space_ray_left = ray_left,
          .initial_model_log_scale = model_log_scale,
          .was_right_translation =
              state.is_right ? true : ReceiverInitiatesTranslation(receiver),
          .was_left_translation =
              state.is_right ? ReceiverInitiatesTranslation(receiver) : true,
      }};
    }
    // Ignore all other events from other pointers.
    return {};
  }

  if (input_flags.Test(InputFlag::kIsDownStopping)) {
    interaction_data_.SetPointer(InteractionMode::PointerMode::kNothing);
    if (interaction_data_.TestTransform(
            InteractionMode::TransformMode::kTranslate)) {
      interaction_data_.SetTransform(InteractionMode::TransformMode::kNothing);
    } else if (ReceiverIsModel(receiver)) {
      bool is_footprint_enabled = interaction_data_.ToggleSelect();
      footprint_->SetColliderEnabled(is_footprint_enabled);
    }
  }

  if (!input_flags.Test(InputFlag::kIsDown)) {
    interaction_data_.SetTransform(InteractionMode::TransformMode::kNothing);

    // Check to see if our footprint is offset (indicating the model should
    // now interpolate to match that offset).
    if (length(state.footprint_local_position.GetTarget()) > kPlaneEpsilon) {
      auto footprint_target = rig_node_->WorldFromLocalPoint(
          state.footprint_local_position.GetTarget());
      auto footprint_delta = footprint_target - rig_node_->GetWorldPosition();
      // Force to reset the positions to the current ones via snapping before
      // setting the new targets.
      state.rig_local_position.Setup(
          svxr::kSmoothSlowResolvingPositionParameters,
          rig_node_->GetLocalPosition());
      state.rig_local_position.SetTarget(rig_node_->GetLocalPosition() +
                                         footprint_delta);
      state.footprint_local_position.Setup(
          svxr::kSmoothSlowResolvingPositionParameters,
          footprint_->FootprintNode()->GetLocalPosition());
      state.footprint_local_position.SetTarget(imp::kZero3);
    }

    if (state.footprint_local_position.IsAtTarget() &&
        state.rig_local_position.IsAtTarget()) {
      return OptionalInteractionState{SetupIdleState()};
    } else {
      // Disable further input, don't exit state until we're at target.
      state.is_active = false;
    }
  }
  state.current_world_space_ray = ray;

  if (state.cumulative_change_delta > kMinimumTranslationDelta) {
    interaction_data_.SetTransform(InteractionMode::TransformMode::kTranslate);
    state.has_translated = true;
  }

  return {};
}

// TODO: this is not used, should be removed once
// the behavior matches SVXR.
OptionalInteractionState SceneViewerComponent::HandleRotationInput(
    svxr::interaction_states::Rotation& state, const imp::Ray& ray,
    imp::NodeHandle receiver, imp::Flags<InputFlag> input_flags) {
  if (state.is_right != input_flags.Test(InputFlag::kIsRight)) {
    // Handle events for pointers which did not initiate rotation.
    if (input_flags.Test(InputFlag::kIsDownStarting)) {
      // Start two-handed scale.
      auto model_scale = model_node_->GetLocalScale().x;
      constexpr auto kEpsilon = 1e-5f;
      auto model_log_scale = std::log(std::max(kEpsilon, model_scale));

      auto& ray_right = state.is_right ? state.current_world_space_ray : ray;
      auto& ray_left = state.is_right ? ray : state.current_world_space_ray;
      return OptionalInteractionState{svxr::interaction_states::TwoHandedScale{
          .initial_world_space_ray_right = ray_right,
          .initial_world_space_ray_left = ray_left,
          .current_world_space_ray_right = ray_right,
          .current_world_space_ray_left = ray_left,
          .initial_model_log_scale = model_log_scale,
          .was_right_translation =
              state.is_right ? false : ReceiverInitiatesTranslation(receiver),
          .was_left_translation =
              state.is_right ? ReceiverInitiatesTranslation(receiver) : false,
      }};
    }
    // Ignore all other events for the other pointer.
    return {};
  }

  if (input_flags.Test(InputFlag::kIsDownStopping)) {
    if (state.cumulative_change_delta > kMinimumRotationDelta ||
        state.cumulative_change_delta < -kMinimumRotationDelta) {
      interaction_data_.SetTransform(InteractionMode::TransformMode::kNothing);
    } else if (!state.is_rotating_after_two_handed_scale &&
               ReceiverIsModel(receiver)) {
      // Toggle select if we are not performing two-handed scale.
      bool is_footprint_enabled = interaction_data_.ToggleSelect();
      footprint_->SetColliderEnabled(is_footprint_enabled);
    }
  }

  if (!input_flags.Test(InputFlag::kIsDown)) {
    interaction_data_.SetTransform(InteractionMode::TransformMode::kNothing);
    return OptionalInteractionState{SetupIdleState()};
  } else {
    state.current_world_space_ray = ray;
  }

  if (state.cumulative_change_delta > kMinimumRotationDelta ||
      state.cumulative_change_delta < -kMinimumRotationDelta) {
    interaction_data_.SetTransform(InteractionMode::TransformMode::kRotate);
  }
  return {};
}

// TODO: this is not used, should be removed once
// the behavior matches SVXR.
OptionalInteractionState SceneViewerComponent::HandleOneHandedScaleInput(
    svxr::interaction_states::OneHandedScale& state, const imp::Ray& ray,
    imp::NodeHandle receiver, imp::Flags<InputFlag> input_flags) {
  // Ignore events for pointers which did not initiate one handed scale.
  if (state.is_right != input_flags.Test(InputFlag::kIsRight)) {
    return {};
  }

  if (!input_flags.Test(InputFlag::kIsDown)) {
    return OptionalInteractionState{SetupIdleState()};
  } else if (ReceiverIsModel(receiver)) {
    interaction_data_.SetTransform(InteractionMode::TransformMode::kScale);
    state.current_world_space_ray = ray;
  }
  return {};
}

// TODO: this is not used, should be removed once
// the behavior matches SVXR.
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

}  // namespace imp
