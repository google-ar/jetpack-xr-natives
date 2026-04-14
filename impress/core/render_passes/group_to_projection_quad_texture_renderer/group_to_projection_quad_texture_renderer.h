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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_GROUP_TO_PROJECTION_QUAD_TEXTURE_RENDERER_GROUP_TO_PROJECTION_QUAD_TEXTURE_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_GROUP_TO_PROJECTION_QUAD_TEXTURE_RENDERER_GROUP_TO_PROJECTION_QUAD_TEXTURE_RENDERER_H_

#include <optional>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/isf_info.h"
#include "core/render_passes/group_to_projection_quad_texture_renderer/group_to_projection_quad_texture_renderer_state.proto.imp.h"
#include "core/render_passes/texture_pipeline_renderer.h"
#include "core/render_passes/texture_pipeline_renderer_state.proto.imp.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Renderer Component that sets up a TexturePipelineRenderer to render
// monoscopic / stereoscopic camera view onto an offscreen texture being
// displayed on a given quad in the scene.
class GroupToProjectionQuadTextureRenderer : public imp::Component {
 public:
  Future<absl::Status> Setup();

  Future<absl::Status> Setup(absl::string_view texture_name,
                             absl::string_view render_pass_group,
                             float2 texture_size,
                             const ProjectionQuadState& projection_quad_state);

  Future<absl::Status> SetupWithState();

  void Update(const imp::FrameTime& frame_time);

  // Set the state information for the Projection Quad used by the
  // TexturePipelineRenderer.
  absl::Status SetProjectionQuadStateInfo(
      const std::optional<ProjectionQuadState>& projection_quad_state);

 private:
  // Creates the Pipeline Renderer to facilitate rendering the offscreen
  // texture.
  Future<absl::Status> CreateTexturePipelineRenderer();

  // Creates a single rendering pass for drawing to the offscreen texture.
  TexturePipelineRendererState::Pass ConfigurePass();

  // Updates the TexturePipelineRenderer's Projection Quad using the current
  // ProjectionQuadState info from this GroupToProjectionQuadTextureRenderer.
  absl::Status UpdateTexturePipelineRendererProjectionQuad();

  // The TexturePipelineRenderer responsible for facilitating the rendering
  // process to the texture.
  ComponentHandle<imp::TexturePipelineRenderer> texture_pipeline_renderer_;

  GroupToProjectionQuadTextureRendererState state_;

 public:
  using IsfInfo = IsfInfo<&GroupToProjectionQuadTextureRenderer::state_>;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_GROUP_TO_PROJECTION_QUAD_TEXTURE_RENDERER_GROUP_TO_PROJECTION_QUAD_TEXTURE_RENDERER_H_
