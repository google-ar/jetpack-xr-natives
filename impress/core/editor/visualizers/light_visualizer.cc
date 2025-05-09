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

#include "core/editor/visualizers/light_visualizer.h"

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/visualizers/light_assets/light_visualizer_assets.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/lighting/light_component.h"
#include "core/view/framework/lighting/light_state.proto.imp.h"
#include "core/view/framework/scene/scene_system.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

namespace {
const float kModelScale = 0.8f;

void RemoveShadowForAllMeshes(NodeHandle node) {
  auto mesh = node->GetComponent<GltfMesh>();
  if (mesh) {
    mesh->SetShadowCastingMode(GltfMesh::ShadowMode::kNone);
    mesh->SetShadowReceivingMode(GltfMesh::ShadowMode::kNone);
  }
  for (auto child : node->GetChildren()) {
    RemoveShadowForAllMeshes(child);
  }
}
}  // namespace

void LightVisualizer::Setup(NodeHandle light) {
  target_ = light;

  GetView()
      .GetSceneSystem()
      .LoadScene(light_visualizer_assets::kLightVisualizerIsf, GetNode())
      .Then([this](const absl::StatusOr<NodeHandle>& model) {
        if (!model.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to load light visualizer: " << model.status();
          return;
        }
        light_visualizer_ = *model;
        RemoveShadowForAllMeshes(light_visualizer_);
      })
      .KeptBy(this);
}

void LightVisualizer::Update(const FrameTime& frame_time) {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  if (!editor.GetEditorRoot()->IsActive()) return;
  if (!target_) return;
  if (!target_->IsActive()) {
    GetNode()->SetEnabled(false);
    return;
  }
  GetNode()->SetEnabled(true);

  ComponentHandle<CameraComponent> editor_camera = editor.GetCamera();
  float distance;

  if (GetView().IsPreciseTranslationEnabled()) {
    GetNode()->SetWorldPositionPrecise(target_->GetWorldPositionPrecise());
    distance = (float)norm(target_->GetWorldPositionPrecise() -
                           editor_camera->GetNode()->GetWorldPositionPrecise());
  } else {
    GetNode()->SetWorldPosition(target_->GetWorldPosition());
    distance = norm(target_->GetWorldPosition() -
                    editor_camera->GetNode()->GetWorldPosition());
  }

  GetNode()->SetWorldScale(distance);

  LightState::Type light_type =
      target_->GetComponent<LightComponent>()->GetType();
  if (light_type == LightState::Type::SUN ||
      light_type == LightState::Type::DIRECTIONAL) {
    GetNode()->SetWorldRotation(target_->GetWorldRotation());
  } else if (light_type == LightState::Type::POINT) {
    GetNode()->SetWorldRotation(editor_camera->GetNode()->GetWorldRotation());
  } else if (light_type == LightState::Type::FOCUSED_SPOT ||
             light_type == LightState::Type::SPOT) {
    // Semi-circle visualizer facing the camera.
    GetNode()->SetWorldRotation(editor_camera->GetNode()->GetWorldRotation());
    // Arrow pointing to light direction.
    if (light_visualizer_) {
      // Find spot_light model.
      for (auto child : light_visualizer_->GetChildren()) {
        if (light_model_map_[child->GetName()] ==
            light_state_map_[light_type]) {
          // Find the arrow in spot_light model.
          for (auto grandchild : child->GetChildren()) {
            if (light_model_map_[grandchild->GetName()] ==
                light_state_map_[LightState::Type::DIRECTIONAL]) {
              grandchild->SetWorldRotation(target_->GetWorldRotation());
              grandchild->SetWorldScale(distance * kModelScale);
              break;
            }
          }
          break;
        }
      }
    }
  }
  UpdateVisualizer(light_type);
}

void LightVisualizer::UpdateVisualizer(LightState::Type visualizer) {
  // In case that the isf is not loaded yet, do nothing.
  if (!light_visualizer_) return;
  for (auto child : light_visualizer_->GetChildren()) {
    if (light_model_map_[child->GetName()] == light_state_map_[visualizer]) {
      child->SetEnabled(true);
    } else {
      child->SetEnabled(false);
    }
  }
}

}  // namespace imp::editor
