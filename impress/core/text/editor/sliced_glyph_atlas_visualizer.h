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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_EDITOR_SLICED_GLYPH_ATLAS_VISUALIZER_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_EDITOR_SLICED_GLYPH_ATLAS_VISUALIZER_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "core/common/invocable.h"
#include "core/common/rememberer.h"
#include "core/editor/layout/layout_composer.h"
#include "core/editor/widget.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"

namespace imp::editor {

// Window at the bottom of the screen showing information related to all text
// rendering in Impress.
class SlicedGlyphAtlasVisualizer : public Widget, public imp::Rememberer {
 public:
  struct AtlasInfo {
    imp::Texture* texture;
    uint2 texture_size;
    uint2 grid_size;
    uint2 slice_texture_size;
    std::vector<imp::Texture*> slice_textures;
  };
  struct GlyphInfo {
    std::string glyph;
    bool is_stroke;
    float2 origin;
    Rect uv;
  };

  struct AtlasDataProvider {
    Invocable<AtlasInfo()> get_atlas_info_func;
    Invocable<std::optional<GlyphInfo>(const float2&)> get_glyph_info_func;
    Invocable<float()> get_utilization_func;
    Invocable<std::vector<GlyphInfo>()> get_all_glyph_info_func;
  };

  explicit SlicedGlyphAtlasVisualizer(BaseView& view,
                                      AtlasDataProvider provider);
  ~SlicedGlyphAtlasVisualizer() override;

  absl::string_view GetName() const override { return "Text"; }
  void DrawImGui() override;
  bool HasContent() const override;

 private:
  void Draw(const AtlasInfo& atlas_info);

  void DrawGlyphBounds(const Rect& bounds) const;
  void DrawGlyphOutline(const Rect& bounds, int color_idx = 0) const;
  void DrawGlyphOrigin(const Rect& bounds, const float2& origin) const;
  void DrawToolTip(const GlyphInfo& info, const AtlasInfo& atlas_info) const;

  BaseView& view_;
  AtlasDataProvider provider_;
  bool show_all_glyph_bounds_;
  bool show_all_glyph_origins_;
  bool enable_hover_highlight_;
  std::unique_ptr<uint8_t[]> rgba_buffer_;
  size_t selected_buffer_index_;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_EDITOR_SLICED_GLYPH_ATLAS_VISUALIZER_H_
