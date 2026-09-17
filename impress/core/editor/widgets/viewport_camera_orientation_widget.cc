// Copyright 2026 Google LLC
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

#include "core/editor/widgets/viewport_camera_orientation_widget.h"

#include <array>
#include <cmath>
#include <cstddef>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/camera/camera_state.proto.imp.h"
#include "core/collision/ray.h"
#include "core/common/registry.h"
#include "core/editor/components/camera_rotate.h"
#include "core/editor/editor.h"
#include "core/editor/widgets/viewport_camera_orientation_widget_assets.h"
#include "core/editor/widgets/viewport_camera_orientation_widget_state.proto.imp.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/gestures/hover_gesture.h"
#include "core/view/framework/gestures/tap_gesture.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

namespace {
// Number of axis indicators (+/- X, Y, Z).
constexpr int kNumAxisHandles = 6;

constexpr float3 kXColor = {0.961f, 0.369f, 0.341f};  // GM3 Red 60
constexpr float3 kYColor = {0.502f, 0.855f, 0.533f};  // GM3 Green 80
constexpr float3 kZColor = {0.196f, 0.443f, 0.918f};  // GM3 Blue 50
constexpr float3 kNegativeAxisColor = {0.78f, 0.78f, 0.78f};
constexpr float3 kCenterColor = {0.56f, 0.56f, 0.56f};
constexpr float3 kHoverColor = {0.989f, 0.741f, 0.0f};  // GM3 Yellow 98

constexpr float kDefaultScaleFactor = 0.0f;
constexpr float kHoverScaleFactor = 0.05f;

// When to begin fading out an axis indicator.
// This is the normalized dot product of the editor camera's view direction
// and the axis indicator's normal vector.
// The closer the value is to 1, the more the indicator is facing the camera.
constexpr float kFadeStart = 0.85f;
// When to fully fade out an axis indicator.
constexpr float kFadeEnd = 0.95f;

// The maximum aspect ratio of the viewport before we move the widget inwards
// to avoid skewing that results from perspective projection.
constexpr float kMaxAspectRatio = 24.0f / 9.0f;

// Animation len for switching between Perspective and Orthographic projection.
constexpr float kProjectionTransitionDurationSeconds = 0.3f;

// Material parameter names.
constexpr absl::string_view kAxisDirectionParam = "axisDirection";
constexpr absl::string_view kBaseColorParam = "baseColor";
constexpr absl::string_view kScaleFactorParam = "scaleFactor";

// Axis directions in world space.
constexpr std::array<float3, kNumAxisHandles> kAxisDirections = {
    kRight, kLeft, kUp, kDown, kBack, kForward};

// Colors for each axes' material.
constexpr std::array<float3, kNumAxisHandles> kAxisColors = {
    kXColor, kNegativeAxisColor, kYColor, kNegativeAxisColor,
    kZColor, kNegativeAxisColor};

// Pitch and yaw angles to snap the camera to.
struct SnapTarget {
  float pitch;
  float yaw;
};

// Snap targets for each axis.
constexpr std::array<SnapTarget, kNumAxisHandles> kSnapTargets = {
    SnapTarget{0.0f, 90.0f},   // +X
    SnapTarget{0.0f, -90.0f},  // -X
    SnapTarget{-90.0f, 0.0f},  // +Y
    SnapTarget{90.0f, 0.0f},   // -Y
    SnapTarget{0.0f, 0.0f},    // +Z
    SnapTarget{0.0f, 180.0f},  // -Z
};

// Linearly interpolates between two matrices.
mat4 LerpMatrices(const mat4& a, const mat4& b, double t) {
  mat4 result;
  for (int col = 0; col < 4; ++col) {
    result[col] = a[col] * (1.0 - t) + b[col] * t;
  }
  return result;
}

// Smoothstep function for easing the projection animation.
double SmoothStep(double x) { return x * x * (3.0 - 2.0 * x); }

}  // namespace

Future<absl::Status> ViewportCameraOrientationWidget::Setup() {
  editor_ = &GetView().GetRegistry().Get<Editor>()->get();

  return GetView()
      .GetAssetManager()
      .LoadMaterial(viewport_camera_orientation_widget_assets::
                        kViewportCameraOrientationAxisMaterialCmat)
      .Then([this](const absl::StatusOr<AssetPtr<MaterialAsset>> material_asset)
                -> absl::Status {
        if (!material_asset.ok()) return material_asset.status();

        MaterialFactory& material_factory = GetView().GetMaterialFactory();

        center_material_ = material_factory.CreateMaterial(*material_asset);

        for (int i = 0; i < kNumAxisHandles; ++i) {
          axis_materials_[i] = material_factory.CreateMaterial(*material_asset);
        }

        SetupMaterials();
        return absl::OkStatus();
      });
}

void ViewportCameraOrientationWidget::SetupMaterials() {
  const std::array<NodeHandle, kNumAxisHandles> axis_nodes = GetAxisNodes();
  for (int i = 0; i < kNumAxisHandles; ++i) {
    SetupAxisHandle(axis_nodes[i], i);
  }

  SetupCenter();
  UpdateColors();
}

void ViewportCameraOrientationWidget::SetupCenter() {
  SetMaterialOverrideOnNode(state_.center, center_material_.Borrow());
  SetupCollider(state_.center);

  editor_->GetDispatcher().Connect(
      state_.center,
      [this](const TapGesture::TapEvent& event) {
        ToggleProjection();
        return Dispatcher::kAccept;
      },
      this);

  editor_->GetDispatcher().Connect(
      state_.center,
      [this](const HoverGesture::HoverEvent& event) {
        if (event.state == HoverGesture::ENTER) {
          center_hovered_ = true;
        } else if (event.state == HoverGesture::EXIT) {
          center_hovered_ = false;
        }
        UpdateColors();
        return Dispatcher::kAccept;
      },
      this);
}

void ViewportCameraOrientationWidget::SetupAxisHandle(const NodeHandle node,
                                                      const int axis_index) {
  axis_materials_[axis_index]->SetParameter(kAxisDirectionParam,
                                            kAxisDirections[axis_index]);

  SetMaterialOverrideOnNode(node, axis_materials_[axis_index].Borrow());

  SetupCollider(node);

  editor_->GetDispatcher().Connect(
      node,
      [this, axis_index](const TapGesture::TapEvent& event) {
        SnapToAxis(axis_index);
        return Dispatcher::kAccept;
      },
      this);

  editor_->GetDispatcher().Connect(
      node,
      [this, axis_index](const HoverGesture::HoverEvent& event) {
        if (event.state == HoverGesture::ENTER) {
          hovered_axis_ = axis_index;
        } else if (event.state == HoverGesture::EXIT &&
                   hovered_axis_ == axis_index) {
          hovered_axis_ = -1;
        }
        UpdateColors();
        return Dispatcher::kAccept;
      },
      this);
}

float2 ViewportCameraOrientationWidget::GetWidgetScreenPosition(
    const CameraComponent& camera) const {
  const float2 viewport_size = camera.GetViewportSize();
  const float w = viewport_size.x;
  const float h = viewport_size.y;
  const float aspect_ratio = w / h;

  const float padding = static_cast<float>(state_.padding.Value());
  const float radius = static_cast<float>(state_.radius.Value());

  float px = 0.0f;
  float py = 0.0f;

  float extra_w = 0.0f;
  // If we exceed the max aspect ration, we move the widget inwards to avoid
  // skewing that results from it existing within a perspective projection.
  if (aspect_ratio > kMaxAspectRatio) {
    extra_w = (w - h * kMaxAspectRatio) * 0.5f;
  }

  switch (state_.corner.Value()) {
    case ViewportCameraOrientationWidgetState::CORNER_UNSPECIFIED:
    case ViewportCameraOrientationWidgetState::TOP_LEFT:
      px = extra_w + padding + radius;
      py = padding + radius;
      break;
    case ViewportCameraOrientationWidgetState::TOP_RIGHT:
      px = w - extra_w - padding - radius;
      py = padding + radius;
      break;
    case ViewportCameraOrientationWidgetState::BOTTOM_LEFT:
      px = extra_w + padding + radius;
      py = h - padding - radius;
      break;
    case ViewportCameraOrientationWidgetState::BOTTOM_RIGHT:
      px = w - extra_w - padding - radius;
      py = h - padding - radius;
      break;
  }
  return float2(px, py);
}

void ViewportCameraOrientationWidget::UpdateColors() {
  const ComponentHandle<CameraComponent> camera = editor_->GetCamera();
  if (!camera) return;

  const float2 viewport_size = camera->GetViewportSize();
  if (viewport_size.x <= 0.0f || viewport_size.y <= 0.0f) return;

  const float2 px_py = GetWidgetScreenPosition(*camera);
  const Ray ray = camera->WorldRayFromPixelPoint(px_py);

  const float3 cam_forward = camera->GetNode()->GetWorldRotation() * kForward;
  const quatf r_skew = quatf::fromDirectedRotation(cam_forward, ray.direction);

  UpdateColors(ray, r_skew);
}

void ViewportCameraOrientationWidget::UpdateColors(const Ray& ray,
                                                   const quatf& r_skew) {
  center_material_->SetParameter(
      kBaseColorParam,
      float4(center_hovered_ ? kHoverColor : kCenterColor, 1.0f));

  const std::array<NodeHandle, kNumAxisHandles> axis_nodes = GetAxisNodes();

  for (int i = 0; i < kNumAxisHandles; ++i) {
    const float3 rotated_axis = r_skew * kAxisDirections[i];
    const float abs_dot = std::abs(dot(rotated_axis, ray.direction));

    // Disable the handle if it's parallel to the view direction.
    // Reduces visual clutter and makes it easier to select the center.
    if (abs_dot >= kFadeEnd) {
      axis_nodes[i]->SetEnabled(false);
      continue;
    }

    axis_nodes[i]->SetEnabled(true);
    float alpha = 1.0f;

    // Fade out the handle if it's getting close to the view direction.
    if (abs_dot > kFadeStart) {
      alpha = (kFadeEnd - abs_dot) / (kFadeEnd - kFadeStart);
    }

    // Apply hover color if this is the hovered axis.
    const float3 color = (hovered_axis_ == i) ? kHoverColor : kAxisColors[i];
    axis_materials_[i]->SetParameter(kBaseColorParam, float4(color, alpha));
  }
}

void ViewportCameraOrientationWidget::SetupCollider(NodeHandle node) {
  const ComponentHandle<GltfMesh> gltf_mesh = node->GetComponent<GltfMesh>();
  node->AddComponent<BoxCollider>(gltf_mesh->GetLocalBounds());
}

void ViewportCameraOrientationWidget::SnapToAxis(int axis_index) {
  const ComponentHandle<CameraComponent> camera = editor_->GetCamera();

  if (!camera) return;

  ComponentHandle<CameraRotate> camera_rotate =
      camera->GetNode()->GetComponent<CameraRotate>();

  if (!camera_rotate.IsValid()) return;

  if (axis_index >= 0 && axis_index < kNumAxisHandles) {
    camera_rotate->InterpolateToPitchAndYaw(kSnapTargets[axis_index].pitch,
                                            kSnapTargets[axis_index].yaw);
  }
}

void ViewportCameraOrientationWidget::ToggleProjection() {
  ComponentHandle<CameraComponent> camera = editor_->GetCamera();
  if (!camera) return;

  if (is_proj_animating_) {
    is_proj_animating_ = false;
  }

  proj_start_matrix_ = camera->GetProjectionMatrixPrecise();

  const bool current_is_perspective =
      (camera->GetProjectionType() == CameraState::ProjectionType::PERSPECTIVE);
  if (is_proj_animating_) {
    target_is_ortho_ = !target_is_ortho_;
  } else {
    target_is_ortho_ = current_is_perspective;
  }

  if (target_is_ortho_) {
    camera->SetOrthographicProjection(orthographic_scale_);
    proj_target_matrix_ = camera->GetProjectionMatrixPrecise();
  } else {
    camera->SetPerspectiveProjection(perspective_fov_, perspective_near_,
                                     perspective_far_);
    proj_target_matrix_ = camera->GetProjectionMatrixPrecise();
  }

  camera->SetCustomProjection(proj_start_matrix_);
  is_proj_animating_ = true;
  proj_anim_time_ = 0.0f;
}

void ViewportCameraOrientationWidget::HandleProjectionAnimation(
    const FrameTime& frame_time, CameraComponent& camera) {
  if (!is_proj_animating_) {
    StoreProjectionSettings(camera);
    return;
  }

  proj_anim_time_ += frame_time.GetDeltaSeconds();
  const float a =
      clamp(proj_anim_time_ / kProjectionTransitionDurationSeconds, 0.0f, 1.0f);
  const double t = SmoothStep(static_cast<double>(a));
  const mat4 p_curr = LerpMatrices(proj_start_matrix_, proj_target_matrix_, t);
  camera.SetCustomProjection(p_curr);

  if (a < 1.0f) return;  // Not done animating yet.

  is_proj_animating_ = false;
  if (target_is_ortho_) {
    camera.SetOrthographicProjection(orthographic_scale_);
  } else {
    camera.SetPerspectiveProjection(perspective_fov_, perspective_near_,
                                    perspective_far_);
  }
}

void ViewportCameraOrientationWidget::StoreProjectionSettings(
    const imp::CameraComponent& camera) {
  // Cache the active configuration if not animating
  if (camera.GetProjectionType() == CameraState::ProjectionType::PERSPECTIVE) {
    perspective_fov_ = camera.GetVerticalFovInDegrees();
    perspective_near_ = camera.GetNearClip();
    perspective_far_ = camera.GetFarClip();
  } else if (camera.GetProjectionType() ==
             CameraState::ProjectionType::ORTHOGRAPHIC) {
    orthographic_scale_ = camera.GetOrthographicScale();
  }
}

void ViewportCameraOrientationWidget::Update(const FrameTime& frame_time) {
  ComponentHandle<CameraComponent> camera = editor_->GetCamera();
  if (!camera) return;

  HandleProjectionAnimation(frame_time, *camera);

  const float2 viewport_size = camera->GetViewportSize();
  if (viewport_size.x <= 0.0f || viewport_size.y <= 0.0f) return;

  // Calculate the world pos such that it's at a fixed screen space size/pos.
  const float h = viewport_size.y;

  const float radius = static_cast<float>(state_.radius.Value());
  const float depth = static_cast<float>(state_.depth.Value());

  const float2 px_py = GetWidgetScreenPosition(*camera);
  const Ray ray = camera->WorldRayFromPixelPoint(px_py);

  GetNode()->SetWorldPosition(ray.origin + ray.direction * depth);

  // Orient the widget to match the camera's view direction.
  // This is skewed to account for the camera's projection matrix.
  // Without the skew, the widget would appear rotated in perspective mode even
  // when the camera is looking straight in a cardinal direction.
  const float3 cam_forward = camera->GetNode()->GetWorldRotation() * kForward;
  const float3 ray_dir = ray.direction;
  const quatf r_skew = quatf::fromDirectedRotation(cam_forward, ray_dir);
  GetNode()->SetWorldRotation(r_skew);

  // Update the scale of the widget to account for the camera's projection.
  // This keeps the scale of the widget fixed in screen space.
  const mat4 p_curr = camera->GetProjectionMatrixPrecise();
  const double proj_y_scale = p_curr[1][1];
  float scale = 1.0f;

  // If the projection matrix is valid, then we can calculate the scale.
  // Otherwise, we'll just use the default scale.
  if (std::abs(proj_y_scale) > 1e-6) {
    const double proj_w_from_z = p_curr[2][3];
    const double proj_w_constant = p_curr[3][3];
    const double clip_w =
        proj_w_from_z * -static_cast<double>(depth) + proj_w_constant;
    scale = static_cast<float>((2.0 * radius / h) * (clip_w / proj_y_scale));
  }

  GetNode()->SetLocalScale(float3(scale));

  // When hovering we push the vertices outwards in local space.
  // The material has this in its vertex shader.
  const float push = center_hovered_ ? kHoverScaleFactor : kDefaultScaleFactor;
  center_material_->SetParameter(kScaleFactorParam, push);

  for (int i = 0; i < kNumAxisHandles; ++i) {
    const float push =
        (hovered_axis_ == i) ? kHoverScaleFactor : kDefaultScaleFactor;
    axis_materials_[i]->SetParameter(kScaleFactorParam, push);
  }
  UpdateColors(ray, r_skew);
}

std::array<NodeHandle, kNumAxisHandles>
ViewportCameraOrientationWidget::GetAxisNodes() const {
  return {state_.x_positive, state_.x_negative, state_.y_positive,
          state_.y_negative, state_.z_positive, state_.z_negative};
}

void ViewportCameraOrientationWidget::SetMaterialOverrideOnNode(
    const NodeHandle node, const BorrowedMaterialPtr material) {
  ComponentHandle<GltfMesh> mesh = node->GetComponent<GltfMesh>();
  const int mesh_count = mesh->GetPrimitiveCount();
  for (size_t i = 0; i < mesh_count; ++i) {
    mesh->SetMaterialOverride(material, i);
  }
}

}  // namespace imp::editor
