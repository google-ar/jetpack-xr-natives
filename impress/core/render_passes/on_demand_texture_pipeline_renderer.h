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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_ON_DEMAND_TEXTURE_PIPELINE_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_ON_DEMAND_TEXTURE_PIPELINE_RENDERER_H_

#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/View.h"
#include "filament/filament/include/filament/Viewport.h"
#include "core/render/texture.h"
#include "core/render/texture_registry.h"
#include "core/render_passes/on_demand_texture_pipeline_render_params.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"

namespace imp {

// A standalone renderer designed for on-demand offscreen rendering.
//
// This renders into the provided color_texture, with a support of both creating
// and retrieving pre-registered Textures via TextureRegistry. It returns either
// a Texture registration or a Texture pointer respectively. Note that using
// pre-registered Textures is not supported for depth textures (you can only
// create a new depth texture).
//
// OnDemandTexturePipelineRenderer does not support automatic resizing, as the
// caller should know the exact Texture size needed at the time of the render
// request.
//
// This is useful for scenarios like icon rendering, where you only need to
// render once and then reuse the same texture.
//
// Lastly, this is a synchronous call that must be made from the main thread and
// outside of the frame loop. (i.e. Should not be used from within
// ViewSecondaryRenderEvent or ViewPreRenderEvent.)
//
// You can use it by registering it:
// view
//     .GetRegistry()
//     .GetOrCreate<OnDemandTexturePipelineRenderer>(view)->Render(params);
//
// Use TexturePipelineRenderer to continuously render textures off-screen on
// each frame.
class OnDemandTexturePipelineRenderer {
 public:
  explicit OnDemandTexturePipelineRenderer(BaseView* view);
  ~OnDemandTexturePipelineRenderer();

  // Returns Texture* if the client is using the texture that is pre-registered
  // in TextureRegistry; ScopedTextureRegistration if the texture is created by
  // OnDemandTexturePipelineRenderer.
  using ColorTexture =
      std::variant<Texture*, TextureRegistry::ScopedTextureRegistration>;

  // Contains the result textures.
  struct TextureResult {
    ColorTexture color_texture;
    // Depth texture is optional.
    std::optional<TextureRegistry::ScopedTextureRegistration> depth_texture;
  };

  struct RenderResult {
    std::vector<TextureResult> textures_per_pass;
  };

  // Returns the list of the textures for each pass, status if failed.
  absl::StatusOr<RenderResult> Render(
      const OnDemandTexturePipelineRenderParams& params);

 private:
  BaseView* view_;
  // Filament View that's shared across all passes.
  filament::View* filament_view_;

  // Stores information about each render pass that is used at runtime in
  // conjunction with the data stored in OnDemandTexturePipelineRenderParams.
  struct RuntimePass {
    // This is used to assign the output textures to the pass in filament.
    filament::RenderTarget* render_target = nullptr;
    // Camera setting that this render pass uses.
    filament::Camera* camera = nullptr;
    // View port that this render pass uses.
    filament::Viewport view_port;
    // Indication of whether to render to the subregion of the output texture.
    bool rendering_to_subregion = false;
    // Indication of whether to use the main view's render settings.
    bool use_main_view_settings = false;
    // Optional render settings to use for the pass.
    std::optional<render_settings::ViewRenderSettings> render_settings;
    // Color texture that the pass writes into.
    ColorTexture color_texture;
    // Registration for the depth texture that the pass outputs, if there is
    // one.
    absl::optional<TextureRegistry::ScopedTextureRegistration>
        depth_texture_registration;
    // Group name that the pass renders.
    std::string group;
  };

  absl::StatusOr<filament::Texture*> SetupColorTexture(
      const OnDemandTexturePipelineRenderParams::Pass& pass,
      RuntimePass& runtime_pass);
  absl::StatusOr<RuntimePass> CreateRuntimePass(
      const OnDemandTexturePipelineRenderParams::Pass& pass);
  void RenderPass(RuntimePass& runtime_pass);

  // Remove default constructor, copy and move semantics.
  OnDemandTexturePipelineRenderer() = delete;
  OnDemandTexturePipelineRenderer(const OnDemandTexturePipelineRenderer&) =
      delete;
  OnDemandTexturePipelineRenderer& operator=(
      const OnDemandTexturePipelineRenderer&) = delete;
};

};  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_PASSES_ON_DEMAND_TEXTURE_PIPELINE_RENDERER_H_
