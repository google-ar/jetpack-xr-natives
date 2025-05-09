/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_ASPECT_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_ASPECT_H_

#include <optional>

#include "absl/status/status.h"
#include "core/editor/command_manager.h"
#include "core/editor/editor.h"
#include "core/editor/widgets/transform_widget_aspect_state.proto.imp.h"
#include "core/ncsb/component.h"
#include "core/ncsb/isf_info.h"
#include "core/view/framework/camera/camera_component.h"

namespace imp::editor {

// A component for each arm of the 3D transform widget. Attach this component
// to one of the arm sub-nodes and mark it in the ISF with the appropriate axis
// and TransformAspect (translate, rotate, or scale).
class TransformWidgetAspect : public imp::Component {
 public:
  static constexpr bool kExcludeFromEditor = true;

  absl::Status Setup();
  void Update(const FrameTime& frame_time);

 private:
  // Helper to get the camera to use (editor or normal).
  ComponentHandle<CameraComponent> GetCamera() const;

  float3 GetAxis() const;

  // Compute the aspect change based on user input. Add undo/redo if commit.
  void UpdateAspect(bool commit = false);

  // Computes the closest point between the ray projected by the cursor and the
  // axis this widget controls.
  std::optional<float3> ComputeClosestPoint() const;

  TransformWidgetAspectState state_;
  NodeHandle active_node_;
  CommandManager* command_manager_;
  float3 position_start_;
  quatf rotation_start_;
  float3 scale_start_;
  float2 pointer_start_;
  float2 pointer_start_delta_from_model_center_;
  float3 pointer_start_closest_point_to_axis_;
  float2 pointer_current_;
  bool dragging_;
  Editor* editor_;

 public:
  using IsfInfo = IsfInfo<&TransformWidgetAspect::state_>;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_TRANSFORM_WIDGET_ASPECT_H_
