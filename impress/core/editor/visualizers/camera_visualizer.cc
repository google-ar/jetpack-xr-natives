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

#include <utility>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/camera/camera_manager.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/visualizers/camera_visualizer_assets.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/scene/scene_system.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

namespace {

// Scale factor for the camera visualizer mesh.
constexpr float kCameraMeshScaleFactor = 0.02f;

// Color of the camera visualizer mesh.
constexpr float3 kCameraMeshColor = float3(0.4f, 0.4f, 0.4f);

// Alpha value for the overlay material.
constexpr float kOverlayAlpha = 0.5f;

// Color of the transparent overlay mesh.
constexpr float4 kCameraOverlayColor = float4(kCameraMeshColor, kOverlayAlpha);

// Base color parameter name for the camera visualizer mesh.
constexpr absl::string_view kCameraMeshColorParam = "baseColor";

// Scale factor parameter name for the camera visualizer mesh.
constexpr absl::string_view kCameraMeshScaleFactorParam = "scaleFactor";

// Group name for the overlay material.
constexpr absl::string_view kOverlayName = "overlay";

void RecursiveApply(NodeHandle node, BorrowedMaterialPtr material) {
  auto mesh = node->GetComponent<GltfMesh>();
  if (mesh) {
    mesh->SetShadowCastingMode(GltfMesh::ShadowMode::kNone);
    mesh->SetShadowReceivingMode(GltfMesh::ShadowMode::kNone);
    mesh->SetMaterialOverride(material);
  }
  for (const NodeHandle& child : node->GetChildren()) {
    RecursiveApply(child, material);
  }
}

void ApplyMaterialOverrides(NodeHandle node, BorrowedMaterialPtr opaque,
                            BorrowedMaterialPtr overlay) {
  for (const NodeHandle& child : node->GetChildren()) {
    if (child->GetName() == kOverlayName) {
      RecursiveApply(child, overlay);
    } else {
      RecursiveApply(child, opaque);
    }
  }
}

}  // namespace

void CameraVisualizer::Cleanup() {
  if (visualizer_model_) {
    RecursiveApply(visualizer_model_, nullptr);
  }
}

void CameraVisualizer::Setup(NodeHandle camera) {
  target_ = camera;

  LoadCameraIsf()
      .Then([this](NodeHandle node) {
        visualizer_model_ = node;
        return LoadMaterials(node);
      })
      .Then([this](NodeHandle node) {
        ApplyMaterialOverrides(node, opaque_material_.Borrow(),
                               overlay_material_.Borrow());
      })
      .KeptBy(this);
}

imp::Future<NodeHandle> CameraVisualizer::LoadCameraIsf() {
  return GetView()
      .GetSceneSystem()
      .LoadScene(camera_visualizer_assets::kCameraVisualizerIsf,
                 {.parent = GetNode()})
      .Then([](absl::StatusOr<NodeHandle> node) -> absl::StatusOr<NodeHandle> {
        if (!node.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to load camera visualizer: " << node.status();
        }
        return node;
      });
}

imp::Future<NodeHandle> CameraVisualizer::LoadMaterials(NodeHandle model) {
  auto& factory = GetView().GetMaterialFactory();

  auto load_opaque = factory.LoadMaterial(
      camera_visualizer_assets::kGizmoVisualizerOpaqueMaterialCmat);
  auto load_overlay = factory.LoadMaterial(
      camera_visualizer_assets::kGizmoVisualizerOverlayMaterialCmat);

  return load_opaque.Combine(load_overlay)
      .Then([this, model, load_opaque, load_overlay](
                absl::Status status) mutable -> absl::StatusOr<NodeHandle> {
        if (!status.ok()) return status;

        opaque_material_ = std::move(load_opaque.Move().value());
        opaque_material_->SetParameter(kCameraMeshColorParam, kCameraMeshColor);
        opaque_material_->SetParameter(kCameraMeshScaleFactorParam,
                                       kCameraMeshScaleFactor);

        overlay_material_ = std::move(load_overlay.Move().value());
        overlay_material_->SetParameter(kCameraMeshColorParam,
                                        kCameraOverlayColor);
        overlay_material_->SetParameter(kCameraMeshScaleFactorParam,
                                        kCameraMeshScaleFactor);
        // Only draw the overlay material when we're obstructed.
        overlay_material_->GetFilamentMaterialInstance()->setDepthFunc(
            filament::MaterialInstance::DepthFunc::NE);
        return model;
      });
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

  if (GetView().IsPreciseTranslationEnabled()) {
    GetNode()->SetWorldPositionPrecise(target_->GetWorldPositionPrecise());
  } else {
    GetNode()->SetWorldPosition(target_->GetWorldPosition());
  }
  GetNode()->SetWorldRotation(target_->GetWorldRotation());
}

}  // namespace imp::editor
