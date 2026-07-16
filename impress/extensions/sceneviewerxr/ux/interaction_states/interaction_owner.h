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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_INTERACTION_OWNER_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_INTERACTION_OWNER_H_

#include <optional>

#include "core/camera/camera_component.h"
#include "core/common/smooth.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/ui_event_listener.h"

namespace svxr {
namespace interaction_states {

class InteractionOwner {
 public:
  virtual ~InteractionOwner() = default;

  virtual InteractionMode& GetInteractionData() = 0;

  virtual imp::ComponentHandle<Footprint> GetFootprint() = 0;
  virtual imp::NodeHandle GetFootprintNode() = 0;
  virtual imp::NodeHandle GetModelNode() = 0;
  virtual imp::NodeHandle GetRigNode() = 0;
  virtual imp::ComponentHandle<imp::CameraComponent> GetCamera() = 0;

  virtual imp::float3 GetHeadPosition() = 0;

  virtual imp::Smooth<float>& GetModelLogScale() = 0;
  virtual void SetModelLogScale(imp::SmoothParameters parameters,
                                float model_log_scale) = 0;
  virtual float GetResetLogScale() = 0;
  virtual void ResetRigPosition() = 0;
  virtual void ToggleResetScaleType() = 0;

  virtual bool IsTalkbackEnabled() = 0;
  virtual bool IsIdleTimeoutEnabled() = 0;

  virtual imp::Smooth<imp::float3>& GetRigPosition() = 0;

  virtual float GetInitialModelScale() = 0;
  virtual void SetInitialModelScale(float initial_model_scale) = 0;

  virtual float GetInitialModelDistanceToCamera() = 0;
  virtual void SetInitialModelDistanceToCamera(
      float initial_model_distance_to_camera) = 0;

  virtual ResetScaleType GetResetScaleType() = 0;
  virtual void SetResetScaleType(ResetScaleType reset_scale_type) = 0;

  virtual void SetRigPosition(imp::SmoothParameters parameters,
                              imp::float3 rig_position) = 0;
  virtual void SetRigRotation(imp::SmoothParameters parameters,
                              imp::quatf rig_rotation) = 0;
  virtual void SetRigRotationTarget(imp::quatf rig_rotation) = 0;

  virtual void CalculateModelScaleLimits() = 0;
  virtual svxr::AxisBounds GetModelLogScaleLimits() = 0;
  virtual void SetModelScale(float model_scale) = 0;

  virtual void RequestUpdateRigPositionFromCamera(
      const imp::SmoothParameters& parameters) = 0;

  virtual float ConstrainElastically(float value, svxr::AxisBounds range,
                                     float scale) = 0;

  virtual UiEventListener* GetUiEventListener() = 0;

  virtual void TriggerShutdownCallback() = 0;

  // Translation State Dependencies
  virtual std::optional<imp::float3> GetAnchorSnapPosition(
      imp::float3 footprint_position_local) = 0;
  virtual imp::float3 ComputeFootprintPositionFromPlanes(
      imp::float3 target_position, imp::float3 rig_to_target) = 0;
  virtual void PlayDropSound() = 0;
  virtual void PlayLiftSound() = 0;
  virtual void PlayGrabSound() = 0;
  virtual void PlayReleaseSound() = 0;
  // TODO: Abstract EnvironmentType or remove this dependency if
  // possible.
  virtual bool IsPassthrough() = 0;

  bool ReceiverIsModel(imp::NodeHandle receiver);
  bool ReceiverIsFootprint(imp::NodeHandle receiver);
  bool ReceiverInitiatesTranslation(imp::NodeHandle receiver);
};

}  // namespace interaction_states
}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_INTERACTION_OWNER_H_
