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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_IOS_PLATFORM_CANVAS_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_IOS_PLATFORM_CANVAS_SOURCE_H_

#include <memory>
#include <vector>

#include "core/canvas/platform_canvas_source.h"
#include "core/canvas/scoped_canvas.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"

namespace imp {

// Implementation of CanvasSource for iOS.
class IosPlatformCanvasSource : public PlatformCanvasSource {
 public:
  IosPlatformCanvasSource();
  ~IosPlatformCanvasSource() override;

  bool IsFeatureSupported(ScopedCanvas::Feature feature) override;

  Texture* GetTexture() override;

  Future<absl::Status> PrepareFont(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) override;

  ScopedCanvas::TextMetrics GetTextMetrics(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) override;

  ScopedCanvas::TextMetrics GetGlyphMetrics(
      ScopedCanvas::GlyphId glyph,
      const ScopedCanvas::TextOptions& text_options) override;

  std::vector<ScopedCanvas::GlyphGroup> GetCombinedCharacterGroups(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) override;

  std::vector<float> GetTextWidths(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) override;

  std::vector<ScopedCanvas::GlyphAdvance> GetTextGlyphs(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) override;

  ScopedCanvas::FontInfo GetFontInfo(
      const ScopedCanvas::TextOptions& text_options) override;

  std::unique_ptr<ScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::DrawMode draw_mode) override;

  std::unique_ptr<ScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
      ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) override;

 private:
  enum class TextGlyphsMode { kIncludeLigatures, kExcludeLigatures };

  std::vector<ScopedCanvas::GlyphAdvance> GetTextGlyphsInternal(
      absl::string_view text, const ScopedCanvas::TextOptions& text_options,
      TextGlyphsMode mode);

  // Forward Declaration of Plain Old Data that includes objective-c types that
  // can't be declared in the header file. Unlike IosScopedCanvas, these fields
  // are scoped to the CanvasSource.
  struct PlatformPod;

  // Forward declare the IosScopedCanvas privately because it contains
  // objective-c types that are only usable within the .mm file. This is similar
  // to the PIMPL design pattern.
  class IosScopedCanvas;

  std::unique_ptr<PlatformPod> platform_pod_;

  OwnedTexturePtr texture_;

  float pixels_per_dp_;
};
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_IOS_PLATFORM_CANVAS_SOURCE_H_
