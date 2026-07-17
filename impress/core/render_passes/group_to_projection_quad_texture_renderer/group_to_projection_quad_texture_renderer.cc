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

#include "core/render_passes/group_to_projection_quad_texture_renderer/group_to_projection_quad_texture_renderer.h"

#include <optional>
#include <string>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/optional_with_default.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/render_passes/group_to_projection_quad_texture_renderer/group_to_projection_quad_texture_renderer_state.proto.imp.h"
#include "core/render_passes/texture_pipeline_renderer.h"
#include "core/render_passes/texture_pipeline_renderer_helper.h"
#include "core/render_passes/texture_pipeline_renderer_projection_quad.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/view/utils/frame_time.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"

namespace imp {
namespace {

// The suffix of the depth texture used in each rendering pass.
constexpr char kDepthSuffix[] = "_depth";

}  // namespace

Future<absl::Status> GroupToProjectionQuadTextureRenderer::Setup() {
  return SetupWithState();
}

Future<absl::Status> GroupToProjectionQuadTextureRenderer::Setup(
    absl::string_view texture_name, absl::string_view render_pass_group,
    float2 texture_size, const ProjectionQuadState& projection_quad_state) {
  state_.texture_name = std::string(texture_name);
  state_.render_group_name = std::string(render_pass_group);
  state_.texture_size = texture_size;
  state_.projection_quad_state = projection_quad_state;

  return SetupWithState();
}

Future<absl::Status> GroupToProjectionQuadTextureRenderer::SetupWithState() {
  return CreateTexturePipelineRenderer();
}

void GroupToProjectionQuadTextureRenderer::Update(
    const imp::FrameTime& frame_time) {
  // TODO: Delay initial use of TPR to workaround black screen.
  static int kFrameDelayForTPR = GetFrameDelayForTPR(GetNode()->GetView());
  static int frame_delay_counter = 0;
  if (texture_pipeline_renderer_.IsValid() &&
      frame_delay_counter++ > kFrameDelayForTPR) {
    texture_pipeline_renderer_->GetNode()->SetEnabled(true);
  }
}

Future<absl::Status>
GroupToProjectionQuadTextureRenderer::CreateTexturePipelineRenderer() {
  if (!state_.texture_name.HasValue() || state_.texture_name->empty()) {
    return absl::InternalError(
        "GroupToProjectionQuadTextureRenderer::CreateTexturePipeline failed: "
        "texture_name "
        "is invalid.");
  }

  if (!state_.render_group_name.HasValue() ||
      state_.render_group_name->empty()) {
    return absl::InternalError(
        "GroupToProjectionQuadTextureRenderer::CreateTexturePipeline failed: "
        "render_group_name is invalid.");
  }

  if (!state_.texture_size.HasValue() || state_.texture_size->x <= 0 ||
      state_.texture_size->y <= 0) {
    return absl::InternalError(
        "GroupToProjectionQuadTextureRenderer::CreateTexturePipeline failed: "
        "texture_size is invalid.");
  }

  imp::TexturePipelineRendererState renderer_state;
  renderer_state.passes.emplace_back(ConfigurePass());

  imp::NodeHandle texture_pipeline_node = GetNode()->CreateChildNode();
  // TODO: Delay initial use of TPR to workaround black screen.
  texture_pipeline_node->SetEnabled(false);
  texture_pipeline_node->SetName("TexturePipeline");
  return texture_pipeline_node
      ->AddComponentWithState<imp::TexturePipelineRenderer>(renderer_state)
      .Then(
          [this](
              absl::StatusOr<imp::ComponentHandle<imp::TexturePipelineRenderer>>
                  status) mutable -> absl::Status {
            if (!status.ok()) {
              return absl::InternalError(absl::StrFormat(
                  "GroupToProjectionQuadTextureRenderer::CreateTexturePipeline "
                  "failed: failed to add texture pipeline renderer with the "
                  "following error message \"%s\"",
                  status.status().ToString()));
            }

            texture_pipeline_renderer_ = *status;
            return UpdateTexturePipelineRendererProjectionQuad();
          });
}

imp::TexturePipelineRendererState::Pass
GroupToProjectionQuadTextureRenderer::ConfigurePass() {
  return {
      .group = std::string(*state_.render_group_name),
      .color_texture_config =
          imp::TexturePipelineRendererState::Texture{
              .name = std::string(*state_.texture_name),
              .format = imp::TexturePipelineRendererState::Texture::RGBA8,
          },
      .depth_texture =
          imp::TexturePipelineRendererState::Texture{
              .name = absl::StrCat(*state_.texture_name, kDepthSuffix),
              // TODO: Investigate why DEPTH24 here would cause TPR
              // failure
              .format = imp::TexturePipelineRendererState::Texture::DEPTH32F,
          },
      .texture_size = *state_.texture_size,
      .render_settings =
          imp::render_settings::ViewRenderSettings{.post_processing_enabled =
                                                       false},
      .use_main_view_settings = true,
      // for debugging, Useful to prove this is the TPR.
      // .clear_color = imp::float4(0.0, 1.0, 0.0, 1.0),
      .use_main_view_camera_projection_matrix = false,
  };
}

absl::Status GroupToProjectionQuadTextureRenderer::
    UpdateTexturePipelineRendererProjectionQuad() {
  if (state_.projection_quad_state.HasValue()) {
    if (!state_.projection_quad_state->size.HasValue() ||
        state_.projection_quad_state->size->x < 0 ||
        state_.projection_quad_state->size->y < 0) {
      return absl::InvalidArgumentError(
          "GroupToProjectionQuadTextureRenderer::"
          "UpdateTexturePipelineRendererProjectionQuad: invalid size value");
    }

    if (!state_.projection_quad_state->center.HasValue()) {
      return absl::InvalidArgumentError(
          "GroupToProjectionQuadTextureRenderer::"
          "UpdateTexturePipelineRendererProjectionQuad: invalid center value");
    }

    if (!state_.projection_quad_state->rotation.HasValue()) {
      return absl::InvalidArgumentError(
          "GroupToProjectionQuadTextureRenderer::"
          "UpdateTexturePipelineRendererProjectionQuad: invalid rotation "
          "value");
    }

    texture_pipeline_renderer_->SetProjectionQuad(
        TexturePipelineRendererProjectionQuad{
            .size = *state_.projection_quad_state->size,
            .center = *state_.projection_quad_state->center,
            .rotation = *state_.projection_quad_state->rotation});
  } else {
    texture_pipeline_renderer_->SetProjectionQuad(std::nullopt);
  }

  return absl::OkStatus();
}

absl::Status GroupToProjectionQuadTextureRenderer::SetProjectionQuadStateInfo(
    const std::optional<ProjectionQuadState>& projection_quad_state) {
  state_.projection_quad_state = projection_quad_state;
  return UpdateTexturePipelineRendererProjectionQuad();
}

}  // namespace imp
