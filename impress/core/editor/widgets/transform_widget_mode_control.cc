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

#include "core/editor/widgets/transform_widget_mode_control.h"

#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/async/future.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/visualizers/transform_gizmo_assets.h"
#include "core/geometry/shapes/box.h"
#include "core/input/key_codes.h"
#include "core/input/keyboard_event.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/view/framework/assets/gltf_collider.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/gestures/hover_gesture.h"
#include "core/view/framework/gestures/tap_gesture.h"

namespace imp::editor {

namespace {
// Gizmo material parameter names.
constexpr absl::string_view kBaseColorParam = "baseColor";
constexpr absl::string_view kScaleFactorParam = "scaleFactor";

// The scale factors for the transform widget vertices.
constexpr float kDefaultScaleFactor = 0.0f;
constexpr float kHoverScaleFactor = 0.0025f;

// Size of the collider in local space for the control cube.
constexpr float kColliderSize = 2.0f;

// The colors for each aspect of the transform widget.
constexpr float3 kControlColor = {0.675f, 0.929f, 1.0f};  // GM3 Cyan 90
constexpr float3 kHoverColor = {0.989f, 0.741f, 0.0f};    // GM3 Yellow 98
}  // namespace

imp::Future<absl::Status> TransformWidgetModeControl::Setup() {
  Editor& editor = GetView().GetRegistry().Get<Editor>()->get();
  editor.GetDispatcher().Connect(
      GetNode(),
      [this](const imp::TapGesture::TapEvent& event) mutable {
        CycleMode();
        return imp::Dispatcher::kAccept;
      },
      this);

  // Update material parameters when hovered.
  editor.GetDispatcher().Connect(
      GetNode(),
      [this](const imp::HoverGesture::HoverEvent& event) mutable {
        if (event.state == HoverGesture::ENTER && !hovered_) {
          hovered_ = true;
          UpdateMaterial();
        } else if (event.state == HoverGesture::EXIT && hovered_) {
          hovered_ = false;
          UpdateMaterial();
        }
        return imp::Dispatcher::kAccept;
      },
      this);

  auto handle_keyboard = [this](const imp::KeyboardEvent& event) {
    // Don't handle key presses if ImGui is using the keyboard.
    if (ImGui::GetIO().WantTextInput) return;

    if (event.type != KeyboardEventType::kOnDown) return;

    switch (event.key.code) {
      case VirtualKeyCode::VK_w:
        SetMode(Mode::kTranslate);
        break;
      case VirtualKeyCode::VK_e:
        SetMode(Mode::kRotate);
        break;
      case VirtualKeyCode::VK_r:
        SetMode(Mode::kScale);
        break;
      default:
        break;
    }
  };

  editor.GetDispatcher().Connect(handle_keyboard, this);

  return GetView()
      .GetMaterialFactory()
      .LoadMaterial(transform_gizmo_assets::kTransformGizmoMaterialCmat)
      .Then([this](absl::StatusOr<OwnedMaterialPtr> material) -> absl::Status {
        if (!material.ok()) {
          IMP_LOG(imp::ERROR) << "Failed to load transform gizmo material: "
                     << material.status();
          return material.status();
        }

        material_ = std::move(*material);
        UpdateMaterial();

        ComponentHandle<GltfMesh> gltf_mesh =
            GetNode()->GetComponent<GltfMesh>();

        if (!gltf_mesh) {
          return absl::FailedPreconditionError(
              "GltfMesh component not found on node.");
        }

        gltf_mesh->SetMaterialOverride(material_.Borrow());

        Box bounds;
        const float half_size = kColliderSize / 2.0f;
        const float3 max_bounds = {half_size, half_size, half_size};
        const float3 min_bounds = -max_bounds;
        bounds.set(min_bounds, max_bounds);

        GetNode()->RemoveComponent<GltfCollider>();  // Don't need this anymore.
        GetNode()->AddComponent<BoxCollider>(bounds);

        return absl::OkStatus();
      });
}

void TransformWidgetModeControl::CycleMode() {
  if (!IsActive()) return;

  switch (mode_) {
    case Mode::kTranslate:
      SetMode(Mode::kRotate);
      break;
    case Mode::kRotate:
      SetMode(Mode::kScale);
      break;
    case Mode::kScale:
      SetMode(Mode::kTranslate);
      break;
  }
}

void TransformWidgetModeControl::SetMode(Mode mode) {
  if (!IsActive()) return;

  mode_ = mode;
  state_.translate->SetEnabled(mode_ == Mode::kTranslate);
  state_.rotate->SetEnabled(mode_ == Mode::kRotate);
  state_.scale->SetEnabled(mode_ == Mode::kScale);
}

void TransformWidgetModeControl::UpdateMaterial() {
  if (!material_) return;

  float3 color = kControlColor;
  float scale_factor = kDefaultScaleFactor;

  if (hovered_) {
    color = kHoverColor;
    scale_factor = kHoverScaleFactor;
  }

  material_->SetParameter(kBaseColorParam, float4(color, 1.0f));
  material_->SetParameter(kScaleFactorParam, scale_factor);
}

}  // namespace imp::editor
