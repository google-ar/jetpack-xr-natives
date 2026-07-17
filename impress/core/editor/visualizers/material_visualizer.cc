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

#include "core/editor/visualizers/material_visualizer.h"

#include <functional>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/proto_asset.h"
#include "core/async/future.h"
#include "core/common/hash.h"
#include "core/common/registry.h"
#include "core/editor/editor.h"
#include "core/editor/visualizers/material_visualizer_assets.h"
#include "core/math/vec.h"
#include "core/ncsb/component.h"
#include "core/ncsb/path_manager.h"
#include "core/proto/proto_writer.h"
#include "core/render/primitive_shape_renderer.h"
#include "core/render/primitive_shape_renderer_state.proto.imp.h"
#include "core/render/texture.h"
#include "core/render_passes/on_demand_texture_pipeline_render_params.proto.imp.h"
#include "core/render_passes/on_demand_texture_pipeline_renderer.h"
#include "core/render_passes/texture_config.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/camera/camera_component.h"
#include "core/view/framework/lighting/light_manager.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/framework/scene/load_scene_visitor.h"
#include "core/view/framework/scene/scene_system.h"
#include "core/view/utils/proto/render_settings.proto.imp.h"

namespace imp::editor {

// This tag is used as the group name and texture name for each material render.
constexpr absl::string_view kMaterialRendererTag = "AssetLibraryMaterial_%s";
constexpr uint2 kPreviewSize(256, 256);

MaterialVisualizer::MaterialVisualizer(BaseView& view) : view_(view) {
  view_.GetSceneSystem().RegisterComponentIsfInfo<PrimitiveShapeRenderer>();
}

Future<Texture*> MaterialVisualizer::GetMaterialPreview(
    AssetPtr<ProtoAsset<MaterialDefinition>> material) {
  std::string serialized;
  if (!proto::SerializeTo(&material->GetProto(), &serialized)) {
    return Future<Texture*>(
        absl::InternalError("Failed to get hash of the material definition."));
  }

  HashValue material_definition_hash = Hash(serialized);
  std::string tag = absl::StrFormat(kMaterialRendererTag,
                                    absl::StrCat(material_definition_hash));

  // We already have a texture for this material.
  auto itr = material_texture_registrations_.find(tag);
  if (itr != material_texture_registrations_.end()) {
    return Future<Texture*>(itr.value().GetTexture());
  }

  LoadSceneVisitor visitor;
  // Set the material on the sphere to be the given material.
  visitor.OnVisit(
      [tag, material](NodeHandle node, PrimitiveShapeRendererState& state) {
        state.primitive.material = material->GetProto();
        node->SetGroups({tag});
      });
  NodeHandle parent;
  absl::StatusOr<std::reference_wrapper<Editor>> editor =
      view_.GetRegistry().Get<Editor>();
  if (editor.ok()) {
    parent = editor->get().GetEditorRoot();
  }

  // Load the scene and render the material.
  return view_.GetSceneSystem()
      .LoadScene(
          kMaterialVisualizerIsf,
          SceneSystem::LoadSceneOptions{
              .parent = parent, .load_scene_visitor = std::move(visitor)})
      .Then([this, tag](NodeHandle scene) -> absl::StatusOr<Texture*> {
        OnDemandTexturePipelineRenderParams params;
        OnDemandTexturePipelineRenderParams::Pass pass;
        pass.color_texture_config =
            TextureConfig{.name = tag, .format = TextureConfig::RGBA8};
        pass.group = tag;
        pass.render_region_size = kPreviewSize;
        pass.render_settings = imp::render_settings::ViewRenderSettings{
            .post_processing_enabled = false};
        pass.camera =
            view_.GetPathManager()
                .GetComponentsInDescendantsOrSelf<CameraComponent>(scene)[0];
        params.passes.push_back(pass);

        view_.GetLightManager().ApplyMainGroupLighting(tag);

        absl::StatusOr<OnDemandTexturePipelineRenderer::RenderResult>
            render_result =
                view_.GetRegistry()
                    .GetOrCreate<OnDemandTexturePipelineRenderer>(&view_)
                    .Render(params);
        view_.DestroyNode(scene);

        if (!render_result.ok()) {
          return render_result.status();
        }

        // Must be a single pass.
        

        OnDemandTexturePipelineRenderer::ColorTexture& color_texture_variant =
            render_result->textures_per_pass[0].color_texture;
        TextureRegistry::ScopedTextureRegistration& texture_registration =
            std::get<TextureRegistry::ScopedTextureRegistration>(
                color_texture_variant);
        Texture* texture = texture_registration.GetTexture();
        if (!texture) {
          return absl::InternalError(
              "OnDemandTexturePipelineRenderer succeeded but no texture was "
              "returned");
        }
        // Store the registration in the map.
        material_texture_registrations_.insert_or_assign(
            tag, std::move(texture_registration));

        return texture;
      });
}

}  // namespace imp::editor
