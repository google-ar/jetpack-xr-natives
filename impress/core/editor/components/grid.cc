// Copyright 2024 Google LLC
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

#include "core/editor/components/grid.h"

#include <string>

#include "absl/status/status.h"
#include "core/collision/collision_helpers.h"
#include "core/collision/plane.h"
#include "core/collision/ray.h"
#include "core/common/registry.h"
#include "core/editor/components/grid_asset.h"
#include "core/editor/editor.h"
#include "core/editor/events.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/render/primitive_shape_renderer.h"
#include "core/view/framework/render/primitive_shape_renderer_state.proto.imp.h"

namespace imp::editor {
// This value is subtracted from the local y position of the grid to avoid z
// buffer rendering issues with the model.
constexpr float kGridPositionOffset = .0001f;
// Scales the grid to help ensure it's always visible.
constexpr float kGridScale = 4.0f;
// Scales the grid to help ensure it's always visible.
constexpr float kGridScaleMin = 0.1f;

Future<absl::Status> Grid::Setup() {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();

  GetNode()->SetWorldRotation(QuatFromEuler({-90.0f, 0.0f, 0.0f}));

  PrimitiveShapeRendererState primitive_shape_state;
  primitive_shape_state.primitive = {
      .material =
          MaterialDefinition{.asset = std::string(
                                 grid_data::kGridMaterialCmat.GetIdentifier())},
      .mesh = PrimitiveShapeRendererState::QuadMesh{}};
  primitive_shape_state.frustrum_culling_mode =
      PrimitiveShapeRendererState::FrustrumCullingMode::DISABLED;

  return GetNode()
      ->AddComponentWithState<PrimitiveShapeRenderer>(primitive_shape_state)
      .Then([this, &editor](
                ComponentHandle<PrimitiveShapeRenderer> primitive_renderer) {
        grid_renderer_ = GetNode()->GetComponent<PrimitiveShapeRenderer>();
        editor.GetDispatcher().Connect(
            [this](const EditorSettingChangedEvent& event) mutable {
              if (event.grid_enabled.has_value()) {
                grid_renderer_->SetEnabled(*event.grid_enabled);
              }
            },
            this);
        return absl::OkStatus();
      });
}

void Grid::Update(const FrameTime& frame_time) {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  ComponentHandle<CameraComponent> editor_camera = editor.GetCamera();

  // Position the grid based on the position of the camera by projecting a ray
  // onto a plane pointing upwards from the origin from the center
  // of the camera. This makes it so the grid appears infinite.
  Plane plane({0, 1, 0}, 0);
  Ray ray = editor_camera->WorldRayFromUVPoint({0.5f, 0.5f});
  float3 point;
  collision::Result result = collision::PlaneIntersectsRay(plane, ray, &point);
  if (result == collision::Result::kDoesNotIntersect) {
    return;
  }

  point.y = -kGridPositionOffset;
  GetNode()->SetWorldPosition(point);

  float3 dist_vec = editor_camera->WorldFromClipPoint({0.5, 0.5, 0}) -
                    GetNode()->GetWorldPosition();
  if (AlmostEqual(dist_vec, kZero3)) {
    return;
  }

  // Scale based on distance from the camera.
  float dist = norm(dist_vec);
  if (dist < kGridScaleMin) {
    return;
  }
  GetNode()->SetLocalScale(float3(dist * kGridScale));
}

}  // namespace imp::editor
