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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/base/nullability.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "core/async/future.h"
#include "core/atlas/atlas_packer.h"
#include "core/atlas/shelf_atlas_packer.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/ref_counter.h"
#include "core/common/rememberer.h"
#include "core/config.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/text/glyph_emulator.h"
#include "core/view/base_view.h"

#if IMP_RUNTIME(DEV)
#include "core/text/editor/glyph_atlas_visualizer.h"
#endif

namespace imp {

// Represents an atlas of glyphs from fonts.
class GlyphAtlas {
 public:
  // The size of the texture to use for the glyph atlas.
  enum class TextureSize {
    // Uses a 2048x2048 texture for the glyph atlas.
    k2048,
    // Uses a 2048x4096 texture for the glyph atlas.
    k2048_4096,
    // Uses a 4096x4096 texture for the glyph atlas.
    k4096,
  };

  // Configuration options for the glyph atlas.
  struct Config {
    // The size of the texture to use for the glyph atlas. The default is a
    // 2048x2048 texture.
    TextureSize texture_size = TextureSize::k2048;
  };

  constexpr static Config kDefaultConfig = {.texture_size = TextureSize::k2048};

  // Information about a glyph needed to render it and lay it out relative to
  // other glyphs in a string.
  struct Glyph {
    // The origin of the font as relative to the actual, visible size. Used to
    // calculate bounds of the TextRenderer.
    float2 actual_origin;
    // The actual, visible size of the glyph. Used to calculate bounds of the
    // TextRenderer.
    float2 actual_size;
    // The size in pixels of the glyph in the atlas.
    // Used to correctly scale the mesh used to render the glyph.
    float2 atlas_size;
    // The width of the glyph within the layout of a string.
    // This is not the same thing as size.x, since the advance_width is
    // dependent on adjacent glyphs in the overall string being rendered.
    float advance_width;
    // The origin point of the glyph in the atlas. The origin normally moves
    // along the baseline to the next advance width so we need this to offset
    // the draw position.
    float2 atlas_origin;
    // The top left of the glyph entry on the texture in normalized uv
    // coordinates.
    float2 uv_top_left;
    // The size of the glyph entry on the texture in normalized uv coordinates.
    // uv_bottom_right would be uv_top_left + uv_size.
    float2 uv_size;
    // If the glyph is an emoji then we should render its color from the atlas.
    bool is_emoji;
    // Used to track how many references to this glyph there are.
    RefCounter::Ref glyph_ref;
  };

  virtual ~GlyphAtlas() {}

  using TextOptions ABSL_DEPRECATED("GlyphEmulator::TextOptions") =
      GlyphEmulator::TextOptions;

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::AddFont()")
  virtual void AddFont(absl::string_view font_name,
                       std::unique_ptr<FontHolder> font_holder) = 0;

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::PrepareFont()")
  virtual Future<absl::Status> PrepareFont(absl::string_view text,
                                           const TextOptions& options) = 0;

  ABSL_DEPRECATED(
      "GetGlyphEmulator() GlyphEmulator::GetCombinedCharacterGroups()")
  virtual Future<std::vector<ScopedCanvas::GlyphGroup>>
  GetCombinedCharacterGroups(absl::string_view text,
                             const TextOptions& options) = 0;

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::GetTextMetrics()")
  virtual Future<ScopedCanvas::TextMetrics> GetTextMetrics(
      absl::string_view text, const TextOptions& options) = 0;

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::GetFontInfo()")
  virtual Future<ScopedCanvas::FontInfo> GetFontInfo(
      const TextOptions& options) = 0;

  // For the given text, returns a list of glyphs used to render the text.
  //
  // The text must be in UTF8 encoding.
  //
  // Returns an error status if unable to get the glyphs. This could happen
  // if the options specify an invalid font name, or if GlyphAtlas is being
  // destructed.
  //
  // Calling this method will add glyphs to the atlas on-demand.
  // If the atlas runs out of space to add a new glyph, it will attempt to
  // clear unused glyphs (tracked using reference counting) to make room.
  //
  // This function is thread-safe.
  virtual Future<std::vector<Glyph>> GetGlyphs(
      absl::string_view text, const GlyphEmulator::TextOptions& options) = 0;

  // Returns a raw pointer to the Texture that will be used to draw the glyphs.
  // Warning: the Texture* may become invalid if GlyphAtlas recreates the
  // canvas.
  //
  // Note: This function is not thread-safe. Because Texture interacts with
  // Filament, this function may only be called from the foreground thread.
  //
  // TODO: Add support for automatically expanding to multiple
  // textures if we run out of space.
  virtual Texture* GetTexture() = 0;

  // Returns the number of glyphs currently in the atlas.
  //
  // This function is thread-safe.
  virtual size_t GetNumCachedGlyphs() const = 0;

  // Get the super sample info based on the View, when it's ready.
  //
  // See also GlyphEmulator::GetSuperSampleInfo().
  virtual Future<GlyphEmulator::SuperSampleInfo> GetSuperSampleInfo(
      bool force_off) const = 0;

  // Provides the percentage of the atlas that is currently occupied
  //
  // This function is thread-safe.
  virtual float GetAtlasUtilization() const = 0;

#if IMP_RUNTIME(DEV)
  // Provides information of the glyph stored at the uv coordinate specified.
  //
  // This function is thread-safe.
  virtual std::optional<editor::GlyphAtlasVisualizer::GlyphAtlasInfo>
  GetGlyphAtlasInfoAt(const float2& uv) const = 0;
  // Provides information for all glyphs
  //
  // This function is thread-safe.
  virtual std::vector<editor::GlyphAtlasVisualizer::GlyphAtlasInfo>
  GetAllGlyphInfo() const = 0;
#endif
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_H_
