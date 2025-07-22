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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_DESKTOP_PLATFORM_CANVAS_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_DESKTOP_PLATFORM_CANVAS_SOURCE_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/canvas/fonts/desktop_font_holder.h"
#include "core/canvas/platform_canvas_source.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "third_party/skia/HEAD/include/core/SkBitmap.h"
#include "third_party/skia/HEAD/include/core/SkCanvas.h"
#include "third_party/skia/HEAD/include/core/SkRefCnt.h"
#include "third_party/skia/HEAD/modules/skparagraph/include/FontCollection.h"

namespace imp {

// Implementation of CanvasSource for Desktop.
// Uses (broken link).
// TODO: Add desktop support for controlling fonts.
class DesktopPlatformCanvasSource : public PlatformCanvasSource {
 public:
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
  class DesktopScopedCanvas : public ScopedCanvas {
   public:
    DesktopScopedCanvas(DesktopPlatformCanvasSource& source, uint2 pixel_size,
                        bool did_texture_change);
    ~DesktopScopedCanvas() override;

    Texture* GetTexture() override;
    bool DidTextureChange() const override;

    void DrawColor(float3 color) override;
    void DrawColor(float4 color) override;

    void DrawRoundedRect(float3 color, float2 corner_radius,
                         const Rect& rect) override;
    void DrawRoundedRect(float4 color, float2 corner_radius,
                         const Rect& rect) override;

    void DrawText(absl::string_view text, float2 pos,
                  const TextOptions& text_options) override;

    void DrawGlyph(GlyphId glyph, float2 pos,
                   const TextOptions& text_options) override;

    void ClearRect(const Rect& rect) override;

   private:
    DesktopPlatformCanvasSource& source_;

    SkBitmap bitmap_;
    std::unique_ptr<uint8_t[]> pixel_buffer_;
    size_t pixel_buffer_size_;
    bool did_texture_change_;

    std::unique_ptr<SkCanvas> canvas_;
  };

  sk_sp<FontCollection> FontCollectionFromTextOptions(
      const ScopedCanvas::TextOptions& text_options);

  // Creates TextStyle from TextOptions which specifies font properties and
  // coloring.
  skia::textlayout::TextStyle CreateTextStyle(
      const ScopedCanvas::TextOptions& text_options, bool draw_stroke_only);

  // Helper function to get Paragraph from text_options
  std::unique_ptr<skia::textlayout::Paragraph> CreateParagraph(
      absl::string_view text, const ScopedCanvas::TextOptions& text_options,
      bool draw_stroke_only);

  DesktopFontHolder font_holder_;
  OwnedTexturePtr texture_;
  uint2 pixel_size_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_DESKTOP_PLATFORM_CANVAS_SOURCE_H_
