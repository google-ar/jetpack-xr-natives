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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_EMULATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_EMULATOR_H_

#include <memory>
#include <optional>
#include <string>
#include <variant>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/variant.h"
#include "core/async/future.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/canvas/fonts/font_params_helper.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/context.h"
#include "core/common/hash.h"
#include "core/math/vec.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/utils/string_map.h"

namespace imp {

// Emulate glyph support on platforms which don't support per-glyph rendering.
//
// If the platform-specific CanvasSource implementation supports per-glyph
// rendering, GlyphEmulator simply wraps this implementation. If not,
// GlyphEmulator splits text into what it considers "separable" and
// "inseparable" strings, and treats these strings as glyphs.
//
// Separable chunks are generally Latin letters or emoji. Inseparable chunks are
// scripts like Arabic and Burmese, whose individual codepoints aren't enough to
// determine how they should be drawn (since each glyph is a product of its
// codepoint and neighbors' codepoints).
//
// This class is thread-safe.
//
// TODO: Create/migrate unit tests from GlyphAtlas to this class
class GlyphEmulator {
 public:
  // Represents a font. Every font corresponds to a unique FontId and is used
  // for differentiating between them.
  using FontId = HashValue;
  // A globally unique identifier for a glyph, composed by a GlyphId and FontId.
  struct GlyphKey {
    ScopedCanvas::GlyphId glyph_id;
    FontId font_id;

    bool operator==(const GlyphKey& other) const {
      return glyph_id == other.glyph_id && font_id == other.font_id;
    }

    template <typename H>
    friend H AbslHashValue(H hash, const GlyphKey& glyph_key) {
      return H::combine(std::move(hash), glyph_key.glyph_id, glyph_key.font_id);
    }
  };
  // Depending on if CanvasSource supports the kGlyph feature, the glyph is
  // either represented by an Id or a string representing the character.
  using GlyphKeyOrGlyphString = absl::variant<GlyphKey, std::string>;

  // Information about a glyph needed to render it and lay it out relative to
  // other glyphs in a string.
  struct Glyph {
    // The glyph or string to draw.
    GlyphKeyOrGlyphString glyph;
    // Metrics of the glyph as returned by CanvasSource::GetGlyphMetrics or
    // CanvasSource::GetTextMetrics.
    TextMetrics metrics;
    // The width of the glyph within the layout of a string.
    // This is not the same thing as size.x, since the advance_width is
    // dependent on adjacent glyphs in the overall string being rendered.
    float advance_width;
    // If the glyph is an emoji then we should render its color from the atlas.
    bool is_emoji;
    // If this glyph was not available in the font provided to GetTextGlyphs,
    // then this is the fallback font that must be used to draw the glyph.
    // This typically occurs for text that mixes CJK languages with latin
    // languages.
    std::unique_ptr<FontHolder> fallback_font;
    // If this glyph contains non-separable script. This is to distinguish when
    // coloring is required as a comparison for the canvas key.
    bool contains_non_separable_script;
  };

  // Options that configure font information about how the glyphs from the text
  // are drawn in the atlas. The same text with different options will produce
  // different glyphs.
  // TODO (broken link) Unify default values of this struct, or at least make it
  // less confusing/error prone.
  struct TextOptions {
    // Variant which is either the name of a font registered by calling
    // GlyphEmulator::AddFont, or a SystemFontParams object which describes the
    // weight, style, and font family of a system font.
    std::variant<absl::monostate, std::string, SystemFontParams> font_params;
    // The size of the font in pixels to use for drawing the glyphs.
    std::optional<float> font_size_pixels;
    // Specifies the width of the stroke in pixels.
    // If zero (or unset) then there is no stroke.
    float stroke_width_pixels = 0.0f;
    // The color and stroke color of the text. These colors are applied in the
    // glyph atlas iff the glyphs contain emojis and non-separable scripts for
    // Desktop/wasm.
    float4 color = kZero4;
    float4 stroke_color = kZero4;
    // The amount of additional space as a percentage of the font size that
    // should be added to each glyph after layout. Default value of 0 adds no
    // additional tracking. Value of 0.5 adds half the font size of space
    // between each glyph, and -0.5 would subtract half the font size of space
    // between each glyph, leading to the glyphs overlapping each other.
    float text_tracking = 0;
    // On wasm, enable or disable the extra work needing to be done to measure
    // the typographical width of text. By default, on wasm measuring the
    // typographical width is disabled.
    bool should_measure_typographical_width = false;
    // If possible (i.e. if not on a path), disable splitting this text into
    // individual glyphs when rendering. This may improve legibility at the cost
    // of increased glyph atlas usage.
    bool force_non_separable = false;
    // If set, when rendering the text the precomputed text metrics will be used
    // instead of having the GlyphEmulator do the computation.
    std::optional<TextAndFontMetrics> precomputed_metrics = std::nullopt;
  };

  explicit GlyphEmulator(Context context);

  // For the given text, returns a list of glyphs used to render the text.
  //
  // The text must be in UTF8 encoding.
  //
  // Returns an error status if unable to get the glyphs. This could happen
  // if the options specify an invalid font name, or if GlyphEmulator is being
  // destructed.
  //
  // Calling this method will add glyphs to the atlas on-demand.
  // If the atlas runs out of space to add a new glyph, it will attempt to clear
  // unused glyphs (tracked using reference counting) to make room.
  //
  // This function is thread-safe.
  Future<std::unique_ptr<std::vector<Glyph>>> GetGlyphs(
      absl::string_view text, const ScopedCanvas::TextOptions& options,
      AsyncCanvasSource& canvas_source);

  // Registers a font with the GlyphEmulator so that it can be used for glyphs.
  //
  // If no font is registered, then the default system font can still be used.
  //
  // This function is thread-safe.
  void AddFont(absl::string_view font_name,
               std::unique_ptr<FontHolder> font_holder)
      ABSL_LOCKS_EXCLUDED(fonts_mutex_);

  // For the given text options, prepare the font for the respective system.
  //
  // In some cases such as on wasm we may need to download a font asynchronously
  // before it may be used.
  Future<absl::Status> PrepareFont(absl::string_view text,
                                   const ScopedCanvas::TextOptions& options,
                                   AsyncCanvasSource& canvas_source);

  // Returns a vector of the group numbers for each glyph based on their
  // combining character information, referenced by the index in the vector.
  // Every set of combined characters will have the same group number. Each
  // group number is unique and the numbers in the vector may not be sequential.
  //
  // Note that every entry in the vector refers to the glyph of the same index
  // in the text string. The phrase "combining character" is used to refer to
  // characters that are intended to modify other characters, which in this
  // implementation are glyphs that are meant to be rendered together with their
  // associated (grouped) glyphs. See combining character information at:
  // https://en.wikipedia.org/wiki/Combining_character
  //
  // The vector is guaranteed to be the same length as the text.
  //
  // For example, if a string contains four glyphs, and the first two glyphs are
  // to be combined and hence the same group while the latter two are separate,
  // then the first two entries in the vector will be the same group while the
  // latter two have their own groups. A valid vector returned for that could be
  // {0, 0, 2, 3};
  //
  // If the text is not separable, then all entries in the vector will be of the
  // same value.
  // If there's no combining character information, then an empty vector is
  // returned.
  Future<std::vector<ScopedCanvas::GlyphGroup>> GetCombinedCharacterGroups(
      absl::string_view text, const ScopedCanvas::TextOptions& options,
      AsyncCanvasSource& canvas_source);

  // Returns the metrics in pixels that a given text string will take up based
  // on the options passed in, and text tracking, which adds additional space
  // between each glyph.
  //
  // The final drawn size will also be impacted by the transform of the
  // TextRenderer used to draw the glyphs.
  //
  // An error is returned if the options specify an invalid font name, or if
  // GlyphEmulator is being destructed.
  //
  // This function is thread-safe.
  Future<TextMetrics> GetTextMetrics(absl::string_view text,
                                     const ScopedCanvas::TextOptions& options,
                                     AsyncCanvasSource& canvas_source);

  // Returns information about the font based on the given text options.
  //
  // If no font is specified, then the default font is used.
  // If the specified font wasn't added via AddFont, or if GlyphEmulator is
  // being destructed, then an error is returned.
  //
  // The resulting info respects the size of the font.
  //
  // This function is thread-safe.
  Future<FontInfo> GetFontInfo(const ScopedCanvas::TextOptions& options,
                               AsyncCanvasSource& canvas_source);

  // Performs a batch operation to measure the size of the given texts and
  // the font metrics for the text's text_options. This is effectively a
  // the same as calling GetTextMetrics and GetFontInfo for each text, but is
  // more efficient when measuring multiple texts in a WASM context where
  // each measurement has an overhead cost to it.
  //
  // TODO: (broken link) - Consolidate GetTextMetrics and GetFontInfo and this
  // function to simplify the API.
  //
  // This function is thread-safe.
  Future<std::vector<TextAndFontMetrics>> GetFontAndTextMetrics(
      std::vector<ScopedCanvas::TextToMeasure> texts,
      AsyncCanvasSource& canvas_source);

  // Convert GlyphEmulator::TextOptions to ScopedCanvas::TextOptions.
  //
  // In particular, this translates fonts registered with AddFont and scales the
  // text drawing options to account for HI-DPI screens or supersampling.
  //
  // To determine the value of subpixel_render_ratio, see GetSuperSampleInfo().
  absl::StatusOr<ScopedCanvas::TextOptions>
  CanvasOptionsFromGlyphEmulatorOptions(
      const TextOptions& options,
      std::optional<float2> subpixel_render_ratio = std::nullopt)
      ABSL_LOCKS_EXCLUDED(fonts_mutex_, system_fonts_mutex_);

  // On WASM we want to render text at 2x the size and super sample it if the
  // window.devicePixelRatio property is less than 2.0 and if force_off is
  // false. Note that we only use the subpixel render ratio for supersampling in
  // the x direction.
  struct SuperSampleInfo {
    bool should_super_sample;
    float2 subpixel_render_ratio;
  };
  static SuperSampleInfo GetSuperSampleInfo(float2 physical_pixel_ratio,
                                            bool force_off);

  // LINT.IfChange(font_size_pixels)
  static constexpr const int kDefaultFontSizePixels = 150.0f;
  // LINT.ThenChange(//depot/google3/third_party/impress/core/text/text_renderer_state.proto:font_size_pixels)

  // Draw a glyph in the given scoped canvas.
  static void DrawGlyph(ScopedCanvas& canvas,
                        const GlyphKeyOrGlyphString& glyph, float2 position,
                        const ScopedCanvas::TextOptions& canvas_options);

  static bool CanForceNonSeparable(bool uses_text_layout_provider,
                                   float text_tracking) {
    // If we render an unseparated string with non-zero text tracking, scuba
    // test output becomes nondeterministic.
    return !uses_text_layout_provider && text_tracking == 0.0f;
  }

 private:
  // This breaks up a string into a series of glyphs and their advance widths.
  //
  // note that this returns a std::unique_ptr<std::vector> instead of an
  // std::vector because Future.Then copies the vector otherwise. A copy of the
  // vector is illegal since Glyph contains a std::unique_ptr.
  Future<std::unique_ptr<std::vector<GlyphEmulator::Glyph>>> BreakIntoGlyphs(
      absl::string_view text, const ScopedCanvas::TextOptions& canvas_options,
      AsyncCanvasSource& canvas_source);

  // Measures the given glyphs, or extracts cached measurements for glyphs
  // that are already stored in this atlas.
  Future<std::unique_ptr<std::vector<Glyph>>> MeasureGlyphs(
      std::unique_ptr<std::vector<Glyph>> glyphs,
      ScopedCanvas::TextOptions canvas_options,
      AsyncCanvasSource& canvas_source);

  Context context_;
  // AsyncCanvasSource& canvas_source_;

  mutable absl::Mutex fonts_mutex_;
  StringMap<std::unique_ptr<FontHolder>> fonts_ ABSL_GUARDED_BY(fonts_mutex_);

  mutable absl::Mutex system_fonts_mutex_;
  absl::flat_hash_map<SystemFontParams, std::unique_ptr<FontHolder>,
                      SystemFontParamsKeyHash, SystemFontParamsKeyEquals>
      system_fonts_ ABSL_GUARDED_BY(system_fonts_mutex_);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_EMULATOR_H_
