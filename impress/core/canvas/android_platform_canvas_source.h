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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_PLATFORM_CANVAS_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_PLATFORM_CANVAS_H_

#include "core/canvas/android_glyph_source.h"
#include "core/canvas/platform_canvas_source.h"
#include "core/common/context.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/canvas.h"
#include "core/view/platforms/android/wrappers/picture.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/platforms/android/wrappers/surface_texture.h"

namespace imp {

// Implementation of CanvasSource for Android.
class AndroidPlatformCanvasSource : public PlatformCanvasSource {
 public:
  explicit AndroidPlatformCanvasSource(BaseView& view);

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
      uint2 pixel_size, ScopedCanvas::DrawMode draw_mode) override;

  std::unique_ptr<ScopedCanvas> StartDrawing(
      uint2 pixel_size, ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
      ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) override;

 private:
  class AndroidScopedCanvas : public ScopedCanvas {
   public:
    explicit AndroidScopedCanvas(AndroidPlatformCanvasSource& source,
                                 uint2 pixel_size, bool did_texture_change);
    ~AndroidScopedCanvas() override;

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
    AndroidPlatformCanvasSource& source_;

    android::Picture picture_;
    android::Canvas canvas_;
    bool did_texture_change_;
  };

  BaseView& view_;
  const Context& context_;

  android::SurfaceTexture surface_texture_;
  android::Surface surface_;
  android::Paint paint_;
  // Used to render an outstroke for text.
  android::Paint stroke_paint_;

  AndroidGlyphSource glyph_source_;

  OwnedTexturePtr texture_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_PLATFORM_CANVAS_H_
