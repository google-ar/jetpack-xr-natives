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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_CANVAS_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_CANVAS_SOURCE_H_

#include <memory>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "core/async/future.h"
#include "core/canvas/platform_canvas_source.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/context.h"
#include "core/common/small_source_location.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"

namespace imp {

// Provides an Impress Texture the can be drawn to
// dynamically. The canvas can be drawn to multiple times each time with a
// different size.
//
// As an example, this can be used to dynamically draw text to a texture and
// render it with Impress.
//
// This is implemented by wrapping platform specific functionality for drawing
// an image.
//
// Currently, this API is implemented on Android, iOS, and Desktop.
//
// This class is thread-safe ((broken link)).
class CanvasSource {
 public:
  static std::unique_ptr<CanvasSource> Create(Context context);

  // Note: This is exposed for testing. Real clients should use
  // CanvasSource::Create to create a CanvasSource.
  explicit CanvasSource(std::unique_ptr<PlatformCanvasSource> platform_source);

  // Returns true if the feature passed in is supported by this CanvasSource.
  //
  // Feature support is determined by platform and OS version.
  //
  // Calling into a method to use an unsupported feature will cause a fatal
  // error.
  bool IsFeatureSupported(ScopedCanvas::Feature feature)
      ABSL_LOCKS_EXCLUDED(platform_source_mutex_);

  // Does an asynchronous operation to prepare the font specified in the
  // TextOptions.
  //
  // On WASM we may need to check if the provided font family is available, and
  // if not we would need to download it. As of now this is no-op on other
  // platforms, but may be extended to download fonts on those platforms in the
  // future.
  Future<absl::Status> PrepareFont(
      absl::string_view text, const ScopedCanvas::TextOptions& text_options);

  // Returns the metrics that a given text string will take up when drawn given
  // the corresponding text options.
  //
  // Note that this function ignores the horizontal_alignment and
  // vertical_alignment options. Metrics are always given from the
  // "bottom-left" of the string.
  ScopedCanvas::TextMetrics GetTextMetrics(
      absl::string_view text, const ScopedCanvas::TextOptions& text_options)
      ABSL_LOCKS_EXCLUDED(platform_source_mutex_);

  // Returns the metrics that a given glyph will take up when drawn based on the
  // options passed in.
  ScopedCanvas::TextMetrics GetGlyphMetrics(
      ScopedCanvas::GlyphId glyph,
      const ScopedCanvas::TextOptions& text_options)
      ABSL_LOCKS_EXCLUDED(platform_source_mutex_);

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
  std::vector<ScopedCanvas::GlyphGroup> GetCombinedCharacterGroups(
      absl::string_view text, const ScopedCanvas::TextOptions& text_options);

  // Returns the advance widths for the characters in the string.
  //
  // Used to layout each character individually for rendering. This is different
  // than measuring the individual character independently, since the advance
  // widths are impacted by adjacent characters.
  //
  // There isn't always a 1:1 mapping between a character in a string and a
  // glyph in a font for rendering, so this won't work correctly for handling
  // ligatures, contextual alternatives, RTL, and BiDi text.
  std::vector<float> GetTextWidths(
      absl::string_view text, const ScopedCanvas::TextOptions& text_options)
      ABSL_LOCKS_EXCLUDED(platform_source_mutex_);

  // Returns the glyphs for the string passed into this method.
  //
  // Used to layout individual glyphs within the string for rendering.
  //
  // Unlike GetTextWidths, GetTextGlyphs will correct handle ligatures,
  // contextual alternatives, RTL, and BiDi text.
  std::vector<ScopedCanvas::GlyphAdvance> GetTextGlyphs(
      absl::string_view text, const ScopedCanvas::TextOptions& text_options)
      ABSL_LOCKS_EXCLUDED(platform_source_mutex_);

  // Returns information about the font used for drawing text with the given
  // text options. Useful for laying out text.
  ScopedCanvas::FontInfo GetFontInfo(
      const ScopedCanvas::TextOptions& text_options)
      ABSL_LOCKS_EXCLUDED(platform_source_mutex_);

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
  // TODO: Remove this API in favor of the other overload after
  // fully migrating to OwnedPtr/BorrowedPtr.
  std::unique_ptr<ScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::DrawMode draw_mode = ScopedCanvas::DrawMode::kClear);

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
  std::unique_ptr<ScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
      ScopedCanvas::DrawMode draw_mode = ScopedCanvas::DrawMode::kClear,
      SmallSourceLocation loc = SmallSourceLocation::Current());

 private:
  absl::Mutex platform_source_mutex_;

  // Underlying platform specific implementation of the CanvasSource.
  std::unique_ptr<PlatformCanvasSource> platform_source_
      ABSL_GUARDED_BY(platform_source_mutex_);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_CANVAS_SOURCE_H_
