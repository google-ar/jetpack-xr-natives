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


#include "core/editor/editor_input_handler.h"

#include <vector>

#include "absl/algorithm/container.h"
#include "filament/filament/include/filament/Scene.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/editor_constants.h"
#include "core/input/pointer_event.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/collision/collision_manager.h"
#include "core/view/framework/collision/ray_hit.h"
#include "core/view/framework/display_layer/display_layer_manager.h"
#include "core/view/framework/input/pointer_input_handler.h"

namespace imp::editor {
namespace {
template <typename T>
// Sort first by presence in scene, then by hit distance.
bool compareRayHits(filament::Scene* scene, const T& hit1, const T& hit2) {
  // If only one of the nodes is in the scene, sort it to the top.
  bool firstNodeInScene = scene->hasEntity(hit1.node->GetEntity());
  bool secondNodeInScene = scene->hasEntity(hit2.node->GetEntity());
  if (firstNodeInScene != secondNodeInScene) return firstNodeInScene;
  // Otherwise, sort by distance.
  return hit1.distance < hit2.distance;
}
}  // namespace
EditorInputHandler::EditorInputHandler(BaseView* view, Dispatcher& dispatcher)
    : PointerInputHandler(view, &dispatcher) {}

std::vector<RayHit> EditorInputHandler::IntersectPointer(const Pointer& p) {
  Editor& editor = view_->GetRegistry().Get<Editor>()->get();
  ComponentHandle<CameraComponent> editor_camera = editor.GetCamera();
  CollisionManager& collision_manager = view_->GetCollisionManager();
  std::vector<RayHit> ray_hits = collision_manager.IntersectAll(
      editor_camera->WorldRayFromPixelPoint(p.point));
  auto* scene = GetEditorOverlayScene();
  if (!scene) {
    return ray_hits;
  }
  absl::c_sort(ray_hits, [&scene](const auto& hit1, const auto& hit2) {
    return compareRayHits(scene, hit1, hit2);
  });
  return ray_hits;
}

std::vector<DoubleRayHit> EditorInputHandler::IntersectPointerPrecise(
    const Pointer& p) {
  Editor& editor = view_->GetRegistry().Get<Editor>()->get();
  ComponentHandle<CameraComponent> camera = editor.GetActiveCamera();
  CollisionManager& collision_manager = view_->GetCollisionManager();
  std::vector<DoubleRayHit> double_ray_hits =
      collision_manager.IntersectAllPrecise(
          camera->WorldRayFromPixelPointPrecise(p.point));
  auto* scene = GetEditorOverlayScene();
  if (!scene) {
    return double_ray_hits;
  }
  absl::c_sort(double_ray_hits, [&scene](const auto& hit1, const auto& hit2) {
    return compareRayHits(scene, hit1, hit2);
  });
  return double_ray_hits;
}

filament::Scene* EditorInputHandler::GetEditorOverlayScene() const {
  if (!view_->GetDisplayLayerManager().GetLayerEnabled(kOverlayGroup)) {
    return nullptr;
  }
  return view_->GetGroupsManager().GetScene(kOverlayGroup);
}

}  // namespace imp::editor
