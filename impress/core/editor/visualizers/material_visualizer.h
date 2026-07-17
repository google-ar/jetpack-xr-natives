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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_MATERIAL_VISUALIZER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_MATERIAL_VISUALIZER_H_

#include "core/assets/asset_ptr.h"
#include "core/assets/proto_asset.h"
#include "core/async/future.h"
#include "core/render/texture.h"
#include "core/render/texture_registry.h"
#include "core/view/base_view.h"
#include "core/view/framework/render/material_definition.proto.imp.h"
#include "core/view/utils/string_map.h"

namespace imp::editor {

// Renders MaterialDefinitions to "preview sphere" textures.
class MaterialVisualizer {
 public:
  MaterialVisualizer(BaseView& view);

  // Creates a preview sphere, uses OnDemandTexturePipelineRenderer to render,
  // then returns the Texture for the preview.
  Future<Texture*> GetMaterialPreview(
      AssetPtr<ProtoAsset<MaterialDefinition>> material);

 private:
  BaseView& view_;
  StringMap<TextureRegistry::ScopedTextureRegistration>
      material_texture_registrations_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_VISUALIZERS_MATERIAL_VISUALIZER_H_
