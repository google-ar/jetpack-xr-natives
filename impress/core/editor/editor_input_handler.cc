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

#include <optional>
#include <vector>

#include "absl/algorithm/container.h"
#include "filament/filament/include/filament/Scene.h"
#include "core/camera/camera_component.h"
#include "core/collision/ray.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/editor_constants.h"
#include "core/geometry/shapes/rect.h"
#include "core/input/input_manager.h"
#include "core/input/pointer_event.h"
#include "core/input/pointer_event_processor.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
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
// Returns the UV coordinates of the pointer in the viewport.
// If the viewport rect does not exist, returns nullopt.
std::optional<float2> GetPointerUVInViewport(const Pointer& p,
                                             const Editor& editor) {
  const std::optional<Rect> viewport_rect = editor.GetViewportRect();

  if (!viewport_rect) return std::nullopt;

  const float2 size = viewport_rect->half_extent * 2.0f;
  const float2 uv = {p.point.x / size.x, p.point.y / size.y};
  return uv;
}
}  // namespace

EditorInputHandler::EditorInputHandler(BaseView* view, Dispatcher& dispatcher)
    : PointerInputHandler(view, &dispatcher), dispatcher_(dispatcher) {}

void EditorInputHandler::Update(InputManager* input_manager) {
  PointerInputHandler::Update(input_manager);
}

std::vector<RayHit> EditorInputHandler::IntersectPointer(const Pointer& p) {
  Editor& editor = view_->GetRegistry().Get<Editor>()->get();
  ComponentHandle<CameraComponent> camera = editor.GetActiveCamera();
  CollisionManager& collision_manager = view_->GetCollisionManager();

  Ray ray;
  const std::optional<float2> uv = GetPointerUVInViewport(p, editor);
  if (uv) {  // Custom viewport rect has been set.
    ray = camera->WorldRayFromUVPoint(*uv);
  } else {  // Default viewport rect.
    ray = camera->WorldRayFromPixelPoint(p.point);
  }

  std::vector<RayHit> ray_hits = collision_manager.IntersectAll(ray);
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

  DoubleRay ray;
  const std::optional<float2> uv = GetPointerUVInViewport(p, editor);
  if (uv) {
    ray = camera->WorldRayFromUVPointPrecise(*uv);
  } else {
    ray = camera->WorldRayFromPixelPointPrecise(p.point);
  }

  std::vector<DoubleRayHit> double_ray_hits =
      collision_manager.IntersectAllPrecise(ray);
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
