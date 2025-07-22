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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_PLATFORM_CANVAS_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_PLATFORM_CANVAS_SOURCE_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"

namespace imp {

// Interface for the platform specific implementation of a CanvasSource.
// Each platform should implement this interface.
struct PlatformCanvasSource {
  virtual ~PlatformCanvasSource() = default;

  virtual bool IsFeatureSupported(ScopedCanvas::Feature feature) = 0;

  virtual Texture* GetTexture() = 0;

  virtual Future<absl::Status> PrepareFont(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) = 0;

  virtual ScopedCanvas::TextMetrics GetTextMetrics(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) = 0;

  virtual ScopedCanvas::TextMetrics GetGlyphMetrics(
      ScopedCanvas::GlyphId glyph,
      const ScopedCanvas::TextOptions& text_options) = 0;

  virtual std::vector<ScopedCanvas::GlyphGroup> GetCombinedCharacterGroups(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) = 0;

  virtual std::vector<float> GetTextWidths(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) = 0;

  virtual std::vector<ScopedCanvas::GlyphAdvance> GetTextGlyphs(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) = 0;

  virtual ScopedCanvas::FontInfo GetFontInfo(
      const ScopedCanvas::TextOptions& text_options) = 0;

  virtual std::unique_ptr<ScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size, ScopedCanvas::DrawMode draw_mode) = 0;

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
  virtual std::unique_ptr<ScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
      ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_PLATFORM_CANVAS_SOURCE_H_
