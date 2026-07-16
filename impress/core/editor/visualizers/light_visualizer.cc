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

#include <utility>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/visualizers/light_assets/light_visualizer_assets.h"
#include "core/lighting/light_component.h"
#include "core/lighting/light_state.proto.imp.h"
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

// Scale factor parameter name for the light visualizer mesh.
constexpr absl::string_view kScaleFactorParam = "scaleFactor";

// Color parameter name for the light visualizer mesh.
constexpr absl::string_view kColorParam = "baseColor";

// Scale factor for the light visualizer mesh.
constexpr float kScaleFactor = 1.0f;

// Alpha value for the overlay material.
constexpr float kOverlayAlpha = 0.4f;

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
    for (const NodeHandle& grandchild : child->GetChildren()) {
      if (grandchild->GetName() == kOverlayName) {
        RecursiveApply(grandchild, overlay);
      } else {
        RecursiveApply(grandchild, opaque);
      }
    }
  }
}

}  // namespace

void LightVisualizer::Cleanup() {
  if (light_visualizer_) {
    RecursiveApply(light_visualizer_, nullptr);
  }
}

void LightVisualizer::Setup(NodeHandle light) {
  target_ = light;

  LoadLightIsf()
      .Then([this](NodeHandle node) {
        light_visualizer_ = node;
        return LoadMaterials(node);
      })
      .Then([this](NodeHandle node) {
        ApplyMaterialOverrides(node, opaque_material_.Borrow(),
                               overlay_material_.Borrow());
      })
      .KeptBy(this);
}

imp::Future<NodeHandle> LightVisualizer::LoadLightIsf() {
  return GetView()
      .GetSceneSystem()
      .LoadScene(light_visualizer_assets::kLightVisualizerIsf,
                 {.parent = GetNode()})
      .Then([](absl::StatusOr<NodeHandle> node) -> absl::StatusOr<NodeHandle> {
        if (!node.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to load light visualizer: " << node.status();
        }
        return node;
      });
}

imp::Future<NodeHandle> LightVisualizer::LoadMaterials(NodeHandle model) {
  auto& factory = GetView().GetMaterialFactory();

  auto load_opaque = factory.LoadMaterial(
      light_visualizer_assets::kGizmoVisualizerOpaqueMaterialCmat);
  auto load_overlay = factory.LoadMaterial(
      light_visualizer_assets::kGizmoVisualizerOverlayMaterialCmat);

  return load_opaque.Combine(load_overlay)
      .Then([this, model, load_opaque, load_overlay](
                absl::Status status) mutable -> absl::StatusOr<NodeHandle> {
        if (!status.ok()) {
          return status;
        }
        opaque_material_ = std::move(load_opaque.Move().value());
        opaque_material_->SetParameter(kScaleFactorParam, kScaleFactor);

        overlay_material_ = std::move(load_overlay.Move().value());
        overlay_material_->SetParameter(kScaleFactorParam, kScaleFactor);
        // Only draw the overlay material when we're obstructed.
        overlay_material_->GetFilamentMaterialInstance()->setDepthFunc(
            filament::MaterialInstance::DepthFunc::NE);
        return model;
      });
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

  if (GetView().IsPreciseTranslationEnabled()) {
    GetNode()->SetWorldPositionPrecise(target_->GetWorldPositionPrecise());
  } else {
    GetNode()->SetWorldPosition(target_->GetWorldPosition());
  }

  float3 color = target_->GetComponent<LightComponent>()->GetColor();
  if (opaque_material_) {
    opaque_material_->SetParameter(kColorParam, float4(color, 1.0f));
  }
  if (overlay_material_) {
    overlay_material_->SetParameter(kColorParam, float4(color, kOverlayAlpha));
  }

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
