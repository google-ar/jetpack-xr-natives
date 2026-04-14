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

#ifndef THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_TESTS_INTERACTION_STATES_TEST_FIXTURE_H_
#define THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_TESTS_INTERACTION_STATES_TEST_FIXTURE_H_

#include <optional>

#include "gmock/gmock.h"
#include "core/camera/camera_component.h"
#include "core/common/smooth.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "extensions/sceneviewerxr/ux/constants.h"
#include "extensions/sceneviewerxr/ux/footprint.h"
#include "extensions/sceneviewerxr/ux/interaction_mode.h"
#include "extensions/sceneviewerxr/ux/interaction_states/interaction_owner.h"
#include "extensions/sceneviewerxr/ux/ui_event_listener.h"
#include "testing/view_fixture.h"

namespace svxr {
namespace interaction_states {

class MockInteractionOwner : public InteractionOwner {
 public:
  ~MockInteractionOwner() override = default;

  MOCK_METHOD(InteractionMode&, GetInteractionData, (), (override));
  MOCK_METHOD(imp::ComponentHandle<Footprint>, GetFootprint, (), (override));
  MOCK_METHOD(imp::NodeHandle, GetFootprintNode, (), (override));
  MOCK_METHOD(imp::NodeHandle, GetModelNode, (), (override));
  MOCK_METHOD(imp::NodeHandle, GetRigNode, (), (override));
  MOCK_METHOD(imp::ComponentHandle<imp::CameraComponent>, GetCamera, (),
              (override));
  MOCK_METHOD(imp::float3, GetHeadPosition, (), (override));
  MOCK_METHOD(imp::Smooth<float>&, GetModelLogScale, (), (override));
  MOCK_METHOD(float, GetResetLogScale, (), (override));
  MOCK_METHOD(void, ResetRigPosition, (), (override));
  MOCK_METHOD(void, ToggleResetScaleType, (), (override));
  MOCK_METHOD(bool, IsTalkbackEnabled, (), (override));
  MOCK_METHOD(bool, IsIdleTimeoutEnabled, (), (override));
  MOCK_METHOD(void, SetModelLogScale,
              (imp::SmoothParameters parameters, float model_log_scale),
              (override));
  MOCK_METHOD(imp::Smooth<imp::float3>&, GetRigPosition, (), (override));
  MOCK_METHOD(float, GetInitialModelScale, (), (override));
  MOCK_METHOD(void, SetInitialModelScale, (float initial_model_scale),
              (override));
  MOCK_METHOD(float, GetInitialModelDistanceToCamera, (), (override));
  MOCK_METHOD(void, SetInitialModelDistanceToCamera,
              (float initial_model_distance_to_camera), (override));
  MOCK_METHOD(ResetScaleType, GetResetScaleType, (), (override));
  MOCK_METHOD(void, SetResetScaleType, (ResetScaleType reset_scale_type),
              (override));
  MOCK_METHOD(void, SetRigPosition,
              (imp::SmoothParameters parameters, imp::float3 rig_position),
              (override));
  MOCK_METHOD(void, SetRigRotation,
              (imp::SmoothParameters parameters, imp::quatf rig_rotation),
              (override));
  MOCK_METHOD(void, SetRigRotationTarget, (imp::quatf rig_rotation),
              (override));
  MOCK_METHOD(void, CalculateModelScaleLimits, (), (override));
  MOCK_METHOD(void, SetModelScale, (float model_scale), (override));
  MOCK_METHOD(void, RequestUpdateRigPositionFromCamera,
              (const imp::SmoothParameters& parameters), (override));
  MOCK_METHOD(std::optional<imp::float3>, GetAnchorSnapPosition,
              (imp::float3 footprint_position_local), (override));
  MOCK_METHOD(imp::float3, ComputeFootprintPositionFromPlanes,
              (imp::float3 target_position, imp::float3 rig_to_target),
              (override));
  MOCK_METHOD(void, PlayDropSound, (), (override));
  MOCK_METHOD(void, PlayLiftSound, (), (override));
  MOCK_METHOD(void, PlayGrabSound, (), (override));
  MOCK_METHOD(void, PlayReleaseSound, (), (override));
  MOCK_METHOD(bool, IsPassthrough, (), (override));
  MOCK_METHOD(UiEventListener*, GetUiEventListener, (), (override));
  MOCK_METHOD(svxr::AxisBounds, GetModelLogScaleLimits, (), (override));
  MOCK_METHOD(float, ConstrainElastically,
              (float value, svxr::AxisBounds range, float scale), (override));
};

class InteractionStatesTestFixture : public imp::testing::ViewFixture {
 public:
  MockInteractionOwner owner_;
  InteractionMode interaction_data_;
};

}  // namespace interaction_states
}  // namespace svxr

#endif  // THIRD_PARTY_IMPRESS_EXTENSIONS_SCENEVIEWERXR_UX_INTERACTION_STATES_TESTS_INTERACTION_STATES_TEST_FIXTURE_H_
