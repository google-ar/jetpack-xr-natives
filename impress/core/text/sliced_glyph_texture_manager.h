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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_SLICED_GLYPH_TEXTURE_MANAGER_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_SLICED_GLYPH_TEXTURE_MANAGER_H_

#include <filament/Engine.h>

#include <functional>
#include <memory>

#include "filament/libs/utils/include/utils/Entity.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/common/bit_vector.h"
#include "core/common/paired_vector.h"
#include "core/common/rememberer.h"
#include "core/materials/material.h"
#include "core/math/math.h"
#include "core/render/texture.h"
#include "core/text/glyph_atlas_slice.h"
#include "core/view/base_view.h"

namespace imp {
namespace sliced_glyph_atlas {

// Utility class for the sliced glyph atlas that manages the composite texture
// and the blitting of individual canvas source textures into it.
class SlicedGlyphTextureManager {
 public:
  virtual ~SlicedGlyphTextureManager();

  // Kick off the async creation of the texture manager.
  static Future<std::unique_ptr<SlicedGlyphTextureManager>> CreateAsync(
      BaseView& view, uint2 atlas_size, uint2 grid_size,
      Texture* composite_texture);

  // The size of an individual slice in the composite texture.
  uint2 GetAtlasSize() const { return atlas_size_; }
  // The size of the grid of slices that make up the composite texture.
  uint2 GetGridSize() const { return grid_size_; }

  // Assigns the source texture to the blitting material.
  void PrepareBlit(SliceId slice, BorrowedTexturePtr texture);

  // Renders a single slice.
  void RenderSlice(filament::Renderer& renderer, SliceId slice);

  static std::unique_ptr<Texture> CreateCompositeTexture(BaseView& view,
                                                         uint2 atlas_size,
                                                         uint2 grid_size);

 private:
  SlicedGlyphTextureManager(BaseView& view,
                            AssetPtr<MaterialAsset> blit_material,
                            uint2 atlas_size, uint2 grid_size,
                            Texture* composite_texture);

  uint2 atlas_size_;
  uint2 grid_size_;

  filament::Engine& engine_;

  // An un-owned pointer to the composite texture. The owner of the texture
  // manager is responsible for the lifetime of the composite texture.
  Texture* composite_texture_;
  AssetPtr<MaterialAsset> blit_material_;

  filament::Camera* blit_camera_;
  filament::IndexBuffer* blit_index_buffer_;
  filament::VertexBuffer* blit_vertex_buffer_;
  filament::ColorGrading* linear_color_grading_;

  filament::RenderTarget* blit_render_target_;

  // Each slice gets a View, Scene, Material, and Entity for blitting.
  PairedVector<filament::View*, Slice> blit_views_;
  PairedVector<MaterialPtr, Slice> blit_materials_;
  PairedVector<filament::Scene*, Slice> blit_scenes_;
  PairedVector<::utils::Entity, Slice> blit_entities_;
};

}  // namespace sliced_glyph_atlas
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_SLICED_GLYPH_TEXTURE_MANAGER_H_
