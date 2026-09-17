// Copyright 2025 Google LLC
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

#include "core/gsplat/magic_window/magic_window_renderer.h"

#include <optional>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "core/assets/asset_ptr.h"
#include "core/async/future.h"
#include "core/common/owned_ptr.h"
#include "core/gsplat/gsplat_asset.h"
#include "core/gsplat/magic_window/magic_window_renderer_state.proto.imp.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
#include "core/render/texture.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/view/utils/frame_time.h"
#include "split_engine/materials/gsplat_material_serializer.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp {
namespace {

std::optional<TexturePipelineRendererProjectionQuad> ToProjectionQuad(
    float2 quad_size, const std::optional<MagicWindowQuadState>& quad_state) {
  if (quad_state.has_value()) {
    return TexturePipelineRendererProjectionQuad{
        .size = quad_size,
        .center = quad_state->center,
        .rotation = quad_state->rotation,
    };
  } else {
    return std::nullopt;
  }
}

}  // namespace

Future<absl::Status> MagicWindowRenderer::Setup() {
  NodeHandle magic_window_node = GetNode();
  if (!state_.render_group.has_value() || state_.render_group->empty()) {
    return absl::InternalError(
        "Render group is required to select the rendered content.");
  }
  return android_xr::GsplatMaterialSerializer::Create(
             magic_window_node, AssetPtr<GSplatAsset>(),
             android_xr::schemas::GsplatMode::MAGIC_WINDOW,
             /*use_triangles_for_splats=*/false, imp::BorrowedTexturePtr(),
             *state_.render_group, state_.offscreen_texture_resolution)
      .Then([this](imp::OwnedPtr<android_xr::GsplatMaterialSerializer>
                       material_serializer) {
        material_serializer_ = std::move(material_serializer);
        UpdateMaterialParameters();
      });
}

void MagicWindowRenderer::Update(const FrameTime& frame_time) {
  UpdateMaterialParameters();
}

void MagicWindowRenderer::UpdateMaterialParameters() {
  material_serializer_->SetMagicWindowProjectionQuad(ToProjectionQuad(
      float2(*state_.magic_window_width, *state_.magic_window_height),
      state_.magic_window_quad_state));
}

void MagicWindowRenderer::SetProjectionQuadState(
    const std::optional<MagicWindowQuadState>& quad_state) {
  state_.magic_window_quad_state = quad_state;
  UpdateMaterialParameters();
}

}  // namespace imp
