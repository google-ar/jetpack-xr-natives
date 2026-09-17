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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VIEWPORT_CAMERA_ORIENTATION_WIDGET_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VIEWPORT_CAMERA_ORIENTATION_WIDGET_H_

#include <array>

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/camera/camera_component.h"
#include "core/collision/ray.h"
#include "core/editor/editor.h"
#include "core/editor/widgets/viewport_camera_orientation_widget_state.proto.imp.h"
#include "core/materials/material.h"
#include "core/math/math.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {

// A viewport corner widget displaying current editor camera orientation
// and allowing snapping the camera to different worldspace axes.
class ViewportCameraOrientationWidget : public Component {
 public:
  static constexpr bool kExcludeFromEditor = true;
  static constexpr UpdateMode kUpdateMode = UpdateMode::kAlwaysUpdate;
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEnd;

  Future<absl::Status> Setup();
  void Update(const FrameTime& frame_time);

 private:
  void SetupMaterials();
  void SetupCenter();
  void SetupAxisHandle(NodeHandle node, int axis_index);
  void SetupCollider(NodeHandle node);
  void SnapToAxis(int axis_index);
  void ToggleProjection();
  void HandleProjectionAnimation(const FrameTime& frame_time,
                                 CameraComponent& camera);
  void StoreProjectionSettings(const CameraComponent& camera);
  void UpdateColors();
  void UpdateColors(const Ray& ray, const quatf& r_skew);
  float2 GetWidgetScreenPosition(const CameraComponent& camera) const;
  std::array<NodeHandle, 6> GetAxisNodes() const;
  void SetMaterialOverrideOnNode(NodeHandle node, BorrowedMaterialPtr material);

  ViewportCameraOrientationWidgetState state_;
  OwnedMaterialPtr center_material_;
  std::array<OwnedMaterialPtr, 6> axis_materials_;
  Editor* editor_ = nullptr;
  int hovered_axis_ = -1;
  bool center_hovered_ = false;
  bool is_proj_animating_ = false;
  float proj_anim_time_ = 0.0f;
  mat4 proj_start_matrix_;
  mat4 proj_target_matrix_;
  bool target_is_ortho_ = false;
  float perspective_fov_ = 45.0f;
  float perspective_near_ = 0.1f;
  float perspective_far_ = 100.0f;
  float orthographic_scale_ = 5.0f;

 public:
  using IsfInfo = IsfInfo<&ViewportCameraOrientationWidget::state_>;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_VIEWPORT_CAMERA_ORIENTATION_WIDGET_H_
