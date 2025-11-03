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

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/canvas/android_glyph_source.h"
#include "core/canvas/platform_canvas_source.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/context.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/canvas.h"
#include "core/view/platforms/android/wrappers/paint.h"
#include "core/view/platforms/android/wrappers/picture.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/platforms/android/wrappers/surface_texture.h"

namespace imp {

// Implementation of CanvasSource for Android.
class AndroidPlatformCanvasSource : public PlatformCanvasSource {
 public:
  AndroidPlatformCanvasSource(Context context,
                              AndroidGlyphSource::Method glyph_method,
                              bool use_hardware_rendering = true);

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

  // TODO: This function behaves incorrectly for characters outside
  // of the basic multilingual plane (BMP) (e.g. 𨭎). Paint.getTextWidths()
  // returns one entry for each UTF-16 codepoint, NOT for each Unicode
  // codepoint. Since Android supports glyphs, which do properly interpret the
  // Unicode codepoints, we don't ever rely on GetTextWidths() code anywhere in
  // production. That said, the behavior of this function is still incorrect.
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

  void ForceReset() override;

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

  Context context_;

  std::unique_ptr<android::SurfaceTexture> surface_texture_;
  std::unique_ptr<android::Surface> surface_;
  android::Paint paint_;
  // Used to render an outstroke for text.
  android::Paint stroke_paint_;

  AndroidGlyphSource glyph_source_;

  OwnedTexturePtr texture_;
  bool use_hardware_rendering_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_ANDROID_PLATFORM_CANVAS_H_
