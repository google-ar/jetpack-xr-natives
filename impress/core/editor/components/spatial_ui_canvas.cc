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

#include "core/editor/components/spatial_ui_canvas.h"

#include <string>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "dear_imgui/imgui.h"
#include "core/async/future.h"
#include "core/editor/components/world_space_editor_ui_assets.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node.h"
#include "core/ncsb/node_handle.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/collision/box_collider.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/render/primitive_shape_renderer.h"
#include "core/view/framework/render/primitive_shape_renderer_state.proto.imp.h"
#include "core/view/utils/frame_time.h"

namespace imp::editor {
namespace {
constexpr float kCanvasScale = 4.0f;
constexpr float kHandleOffset = 0.1f;

// LINT.IfChange(material_params)
constexpr absl::string_view kCanvasTextureName = "globalBaseColor";
constexpr absl::string_view kCornerUv = "cornerUv";
constexpr absl::string_view kSizeUv = "sizeUv";
// LINT.ThenChange(//depot/google3/third_party/impress/core/editor/components/spatial_ui_canvas_texture.mat)
}  // namespace

Future<absl::Status> SpatialUiCanvas::Setup(absl::string_view name,
                                            float2 content_position,
                                            float2 content_size,
                                            BorrowedTexturePtr texture) {
  name_ = name;
  content_position_ = content_position;
  content_size_ = content_size;
  texture_ = std::move(texture);

  PrimitiveShapeRendererState primitive_shape_state;
  primitive_shape_state.primitive = {
      .material =
          MaterialDefinition{
              .asset = std::string(
                  world_space_editor_ui_assets::kSpatialUiCanvasTextureCmat
                      .GetIdentifier())},
      .mesh = PrimitiveShapeRendererState::QuadMesh{.size = float2{1.0f},
                                                    .flip_uv = true}};

  return GetNode()
      ->AddComponentWithState<PrimitiveShapeRenderer>(primitive_shape_state)
      .Then([this](ComponentHandle<PrimitiveShapeRenderer> primitive_renderer) {
        primitive_renderer_ = primitive_renderer;
        primitive_renderer_->GetMaterial()->SetParameter(kCanvasTextureName,
                                                         texture_);
        UpdateCanvas({.name = name_,
                      .content_position = content_position_,
                      .content_size = content_size_});

        // The canvas is an 1x1 quad locally and its front face is on the z=0
        // plane. Add a small box collider to make it hittable.
        GetNode()->AddComponent<BoxCollider>(
            Box{{0.0f, 0.0f, -0.001}, {0.5f, 0.5f, 0.0005f}});
      });
}

void SpatialUiCanvas::Update(FrameTime& frame_time) {
  // Face the canvas towards the camera.
  float3 canvas_to_camera = normalize(
      GetNode()->GetWorldPosition() -
      GetView().GetCameraManager().GetCamera()->GetNode()->GetWorldPosition());
  GetNode()->SetWorldForward(canvas_to_camera, kUp);
}

void SpatialUiCanvas::UpdateCanvas(SpatialUiCanvasSettings settings) {
  if (settings.name != name_) {
    IMP_LOG(imp::ERROR) << "SpatialUiCanvas::UpdateCanvas: this canvas is not for "
               << settings.name;
    return;
  }
  content_position_ = settings.content_position;
  content_size_ = settings.content_size;
  int2 global_content_size = texture_->GetSize();
  if (global_content_size.x <= 0 || global_content_size.y <= 0) {
    return;
  }
  float2 canvas_size = float2{content_size_.x / global_content_size.x,
                              content_size_.y / global_content_size.x} *
                       kCanvasScale;
  // Set the top-left corner of the canvas to be aligned with the grab handle.
  GetNode()->SetLocalPosition(float3{canvas_size.x / 2.0f + kHandleOffset,
                                     -canvas_size.y / 2.0f - kHandleOffset,
                                     0.0f});
  // Scale the canvas to fit the texture.
  GetNode()->SetLocalScale(float3{canvas_size.x, canvas_size.y, 1.0f});

  float2 size_uv = {content_size_.x / global_content_size.x,
                    content_size_.y / global_content_size.y};
  float2 corner_uv = {content_position_.x / global_content_size.x,
                      content_position_.y / global_content_size.y};
  primitive_renderer_->GetMaterial()->SetParameter(kCornerUv, corner_uv);
  primitive_renderer_->GetMaterial()->SetParameter(kSizeUv, size_uv);
}

SpatialUiCanvas::SpatialUiCanvasSettings SpatialUiCanvas::GetSettings() {
  return {.content_position = content_position_, .content_size = content_size_};
}

ImVec2 SpatialUiCanvas::CalculateImGuiPointFromWorldPoint(float3 world_point) {
  float3 local_hit = GetNode()->LocalFromWorldPoint(world_point);
  // Convert from quad-space to [0,1] space and flip y.
  float2 normalized_hit = {local_hit.x + 0.5f, -local_hit.y + 0.5f};

  // Convert from [0,1] space to ImGui pixel space.
  return {normalized_hit.x * content_size_.x + content_position_.x,
          normalized_hit.y * content_size_.y + content_position_.y};
}

}  // namespace imp::editor
