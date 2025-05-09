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

#include "core/editor/visualizers/camera_visualizer.h"

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/visualizers/camera_visualizer_assets.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_scene.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/scene/scene_system.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

constexpr float kCameraMeshScaleFactor = 0.02f;

void CameraVisualizer::Setup(NodeHandle camera) {
  target_ = camera;

  GetView()
      .GetSceneSystem()
      .LoadScene(camera_visualizer_assets::kCameraVisualizerIsf, GetNode())
      .Then([this](const absl::StatusOr<NodeHandle>& model) {
        if (!model.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to load camera visualizer: " << model.status();
          return;
        }
        if (auto scene = (*model)->GetComponent<GltfScene>()) {
          scene->ForAllNodes([](NodeHandle node) {
            auto mesh = node->GetComponent<GltfMesh>();
            if (mesh) {
              mesh->SetShadowCastingMode(GltfMesh::ShadowMode::kNone);
              mesh->SetShadowReceivingMode(GltfMesh::ShadowMode::kNone);
            }
          });
        }
      })
      .KeptBy(this);
}

void CameraVisualizer::Update(const FrameTime& frame_time) {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  if (!editor.GetEditorRoot()->IsActive()) return;
  if (!target_) return;

  // Disable the visualizer if it would visualize the camera from which we
  // are currently looking, otherwise we will be inside the camera model.
  ComponentHandle<CameraComponent> editor_camera = editor.GetCamera();
  if (editor_camera->IsActive()) {
    GetNode()->SetEnabled(target_ != editor_camera->GetNode());
  } else {
    GetNode()->SetEnabled(target_ !=
                          GetView().GetCameraManager().GetCamera()->GetNode());
  }
  float distance;
  if (GetView().IsPreciseTranslationEnabled()) {
    GetNode()->SetWorldPositionPrecise(target_->GetWorldPositionPrecise());
    distance = norm(target_->GetWorldPositionPrecise() -
                    editor_camera->GetNode()->GetWorldPositionPrecise());
  } else {
    GetNode()->SetWorldPosition(target_->GetWorldPosition());
    distance = norm(target_->GetWorldPosition() -
                    editor_camera->GetNode()->GetWorldPosition());
  }
  GetNode()->SetWorldRotation(target_->GetWorldRotation());
  GetNode()->SetWorldScale(kCameraMeshScaleFactor * distance);
}

}  // namespace imp::editor
