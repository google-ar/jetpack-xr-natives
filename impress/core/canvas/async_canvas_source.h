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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_CANVAS_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_CANVAS_SOURCE_H_

#include <memory>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/math/vec.h"
#include "core/text/text_helpers.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/base_view.h"

namespace imp {

// An asynchronous version of the CanvasSource interface. Note this is
// not a 1:1 mapping of functions from CanvasSource because some functions are
// better off as bulk operations when running asynchronously.
class AsyncCanvasSource {
 public:
  virtual ~AsyncCanvasSource() = default;

  // Returns true if the feature passed in is supported by this CanvasSource.
  //
  // Feature support is determined by platform and OS version.
  //
  // Calling into a method to use an unsupported feature will cause a fatal
  // error.
  virtual bool IsFeatureSupported(ScopedCanvas::Feature feature) = 0;

  // Does an asynchronous operation to prepare the font specified in the
  // TextOptions.
  //
  // On WASM we may need to check if the provided font family is available, and
  // if not we would need to download it. As of now this is no-op on other
  // platforms, but may be extended to download fonts on those platforms in the
  // future.
  virtual Future<absl::Status> PrepareFont(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) = 0;

  struct GlyphToMeasure {
    std::variant<absl::string_view, ScopedCanvas::GlyphId> glyph;

    // The font to use to measure this glyph. If not provided, the default font
    // provided in text_options will be used.
    FontHolder* font_override;
  };

  // Performs a batch operation to measure the size of the given texts and
  // the font metrics for the text's text_options.
  virtual Future<std::vector<TextAndFontMetrics>> GetFontAndTextMetrics(
      std::vector<ScopedCanvas::TextToMeasure> texts) = 0;

  // Returns the size in pixels that the given glyph will take up when
  // drawn based on the options passed in. Prefer MeasureGlyphs when there
  // are multiple glyphs to be measured at the same time.
  // TODO: (broken link) - Consider deprecating this function in favor of
  // GetFontAndTextMetrics
  virtual Future<TextMetrics> MeasureGlyph(
      GlyphToMeasure glyph_to_measure,
      ScopedCanvas::TextOptions text_options) = 0;

  // Returns the size in pixels that the given glyphs will take up when
  // drawn based on the options passed in.
  // TODO: (broken link) - Consider deprecating this function in favor of
  // GetFontAndTextMetrics
  virtual Future<std::vector<TextMetrics>> MeasureGlyphs(
      std::vector<GlyphToMeasure> glyphs_to_measure,
      ScopedCanvas::TextOptions text_options) = 0;

  // Returns a vector of the groups each glyph belongs to based on their
  // combining character information, referenced by the index in the vector.
  //
  // The vector will be the same length as the text.
  //
  // For example, if a string contains four glyphs, and the first two glyphs are
  // to be combined and hence the same group while the latter two are separate,
  // then the first two entries in the vector will be the same value, and a
  // valid vector returned for that could be {0, 0, 2, 3};
  //
  // If the text is not separable, then all entries in the vector will be of the
  // same value.
  virtual Future<std::vector<ScopedCanvas::GlyphGroup>>
  GetCombinedCharacterGroups(absl::string_view text,
                             const ScopedCanvas::TextOptions& text_options) = 0;

  // Returns the advance widths for the characters contained in each chunk.
  //
  // Used to layout each character individually for rendering. This is different
  // than measuring the individual character independently, since the advance
  // widths are impacted by adjacent characters.
  //
  // There isn't always a 1:1 mapping between a character in a string and a
  // glyph in a font for rendering, so this won't work correctly for handling
  // ligatures, contextual alternatives, RTL, and BiDi text.
  //
  // This returns a vector of advance widths for the characters in each chunk.
  // The advance widths vector for non-separable chunks will be a single width
  // of the entire chunk.
  virtual Future<std::vector<std::vector<float>>> GetTextWidths(
      const std::vector<Chunk>& chunk,
      const ScopedCanvas::TextOptions& text_options) = 0;

  // Returns the glyphs for the string passed into this method.
  //
  // Used to layout individual glyphs within the string for rendering.
  //
  // Unlike GetTextWidths, GetTextGlyphs will correct handle ligatures,
  // contextual alternatives, RTL, and BiDi text.
  //
  // This returns a std::unique_ptr<std:vector> instead of an std::vector
  // because Future.Then copies the vector when it is not a unique_ptr. Copying
  // the vector is not allowed because GlyphAdvance contains a unique_ptr
  // itself.
  virtual Future<std::unique_ptr<std::vector<ScopedCanvas::GlyphAdvance>>>
  GetTextGlyphs(absl::string_view text,
                const ScopedCanvas::TextOptions& text_options) = 0;

  // Returns information about the font used for drawing text with the given
  // text options. Useful for laying out text.
  // TODO: (broken link) - Consider deprecating this function in favor of
  // GetFontAndTextMetrics
  virtual Future<FontInfo> GetFontInfo(
      const ScopedCanvas::TextOptions& text_options) = 0;

  // Provides a Canvas that is used to draw to the texture and access the
  // texture.
  //
  // The texture persists until either the CanvasSource is destroyed or
  // CanvasSource::StartDrawing returns a ScopedCanvas where
  // ScopedCanvas::DidTextureChange is true.
  //
  // By default, each time this is called, the texture is cleared.
  // If DrawMode::kKeepContents is passed in, then the contents will be kept
  // from the previous draw as long as the size has not changed & the
  // CanvasSource supports the feature. Currently, the feature is only supported
  // on iOS.
  //
  // NOTE: The Draw functions called on the returned ScopedCanvas don't actually
  // apply to the texture until the ScopedCanvas is destroyed.
  virtual std::unique_ptr<AsyncScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::DrawMode draw_mode = ScopedCanvas::DrawMode::kClear) = 0;

  // *EXPERIMENTAL*
  //
  // Provides a Canvas that is used to draw to the texture.
  //
  // on_texture_changed_fn is called when the texture is created or changed.
  // This is guaranteed to be called the first time StartDrawing is called. On
  // subsequent calls, it will happen when the texture is re-created. In
  // practice, the only happens on Desktop and Web when the size of the texture
  // changes.
  //
  // *IMPORTANT* All BorrowedTexturePtr objects referencing an old texture from
  // a previous call to StartDrawing must be gone by the time
  // on_texture_changed_fn returns, because at that point the old texture (if
  // there is one) will be destroyed.
  //
  // The texture persists until either the CanvasSource is destroyed or
  // on_texture_changed_fn is called.
  //
  // By default, each time this is called, the texture contents are cleared.
  // If DrawMode::kKeepContents is passed in, then the contents will be kept
  // from the previous draw as long as the size has not changed & the
  // CanvasSource supports the feature. Currently, the feature is only supported
  // on iOS.
  //
  // NOTE: The Draw functions called on the returned ScopedCanvas don't actually
  // apply to the texture until the ScopedCanvas is destroyed.
  virtual std::unique_ptr<AsyncScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
      ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) = 0;

  virtual void ForceReset() {}
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_CANVAS_SOURCE_H_
