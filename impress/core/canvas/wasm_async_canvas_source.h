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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_WASM_ASYNC_CANVAS_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_WASM_ASYNC_CANVAS_SOURCE_H_

#include <cstdint>
#include <memory>
#include <vector>

#include "absl/base/thread_annotations.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "core/async/future.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/text/text_helpers.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/base_view.h"
#include "core/view/platforms/wasm/wasm_canvas_manager.h"

namespace imp {

// Implementation of CanvasSource for Wasm.
// Utilizes the browser canvas to render and measure text, as well as draw basic
// shapes such as the rounded rectangle.
class WasmAsyncCanvasSource : public AsyncCanvasSource {
 public:
  WasmAsyncCanvasSource();

  // Because this class passes a pointer to itself to callbacks, it may not be
  // copied or moved.
  WasmAsyncCanvasSource(const WasmAsyncCanvasSource&) = delete;
  WasmAsyncCanvasSource(WasmAsyncCanvasSource&&) = delete;

  bool IsFeatureSupported(ScopedCanvas::Feature feature) override;

  Texture* GetTexture();

  Future<absl::Status> PrepareFont(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) override;

  Future<std::vector<TextAndFontMetrics>> GetFontAndTextMetrics(
      std::vector<ScopedCanvas::TextToMeasure> texts) override;

  Future<TextMetrics> MeasureGlyph(
      GlyphToMeasure glyph_to_measure,
      ScopedCanvas::TextOptions text_options) override;

  Future<std::vector<TextMetrics>> MeasureGlyphs(
      std::vector<GlyphToMeasure> glyphs_to_measure,
      ScopedCanvas::TextOptions text_options) override;

  Future<std::vector<ScopedCanvas::GlyphGroup>> GetCombinedCharacterGroups(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) override;

  Future<std::vector<std::vector<float>>> GetTextWidths(
      const std::vector<Chunk>& chunks,
      const ScopedCanvas::TextOptions& text_options) override;

  Future<std::unique_ptr<std::vector<ScopedCanvas::GlyphAdvance>>>
  GetTextGlyphs(absl::string_view text,
                const ScopedCanvas::TextOptions& text_options) override;

  Future<FontInfo> GetFontInfo(
      const ScopedCanvas::TextOptions& text_options) override;

  std::unique_ptr<AsyncScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::DrawMode draw_mode =
          ScopedCanvas::DrawMode::kClear) override;

  std::unique_ptr<AsyncScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
      ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) override;

  void OnPixelBufferReady(BaseView& view, uint8_t* data, int length,
                          uint32_t* dirty_rects_data, int dirty_rects_length);

  void OnRectCleared(Rect rect);

  class WasmScopedCanvas : public AsyncScopedCanvas {
   public:
    WasmScopedCanvas(WasmAsyncCanvasSource& source,
                     WasmCanvasManager* platform_canvas_wrapper,
                     uint2 pixel_size, bool did_texture_change);
    ~WasmScopedCanvas() override;

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
    Future<absl::Status> PrepareToUpdateTexture() override;
    bool SupportsSynchronousTextureUpdate() const override;

    Future<absl::Status> PrepareFont(absl::string_view text,
                                     const TextOptions& text_options);
    Future<std::vector<TextMetrics>> MeasureGlyphs(
        std::vector<GlyphToMeasure> glyphs_to_measure,
        ScopedCanvas::TextOptions text_options);
    Future<std::vector<TextMetrics>> MeasureGlyphs(
        std::vector<Chunk> chunks, ScopedCanvas::TextOptions text_options);
    Future<std::vector<std::vector<float>>> GetTextWidths(
        const std::vector<Chunk>& chunks,
        const ScopedCanvas::TextOptions& text_options);
    Future<FontInfo> GetFontInfo(const ScopedCanvas::TextOptions& text_options);
    Future<std::vector<TextAndFontMetrics>> GetFontAndTextMetrics(
        std::vector<ScopedCanvas::TextToMeasure> texts);

    WasmAsyncCanvasSource& GetSource() { return source_; }

   private:
    WasmAsyncCanvasSource& source_;
    WasmCanvasManager* platform_canvas_wrapper_;

    void SetTextOptions(TextOptions text_options);

    // Measures the given chunks and returns the results. For separable chunks,
    // each character in the chunk will be measured, otherwise the full chunk
    // text will be measured as one. If only_widths is false, each chunk
    // measurement will be represented as a tuple of [origin_x, origin_y,
    // size_x, size_y, font_origin_y, font_size_y] otherwise each chunk
    // measurement will be a single width value. The returned vector is a vector
    // of measurements for each chunk. The inner vector is a flattend vector of
    // measurement tuples for each piece of the chunk measured.
    // Note that if only_widths is true, the returned vector will be the
    // advance width of each chunk, not the render bounds.
    Future<std::vector<std::vector<float>>> MeasureTexts(
        const std::vector<Chunk>& chunks, const TextOptions& text_options,
        bool only_widths);

    bool did_texture_change_;

    // Give WasmAsyncCanvasSource access to MeasureTexts
    friend class WasmAsyncCanvasSource;
  };

 private:
  OwnedTexturePtr texture_;
  uint2 pixel_size_;
  absl::Mutex canvas_mutex_;

  // TODO Combine both canvases into one.
  // A wrapper for the web canvas, specifically used for measuring text.
  std::unique_ptr<WasmCanvasManager> measuring_canvas_;
  // A wrapper for the web canvas used for drawing to the glyph atlas canvas.
  // This is separate from the measuring canvas as it has specific fill and
  // stroke colors that remain the same at all times, while the measuring canvas
  // can have its style options changed at any time.
  std::unique_ptr<WasmCanvasManager> drawing_canvas_;
  // A scoped canvas used only to measure text size information.
  WasmScopedCanvas measuring_scoped_canvas_ ABSL_GUARDED_BY(canvas_mutex_);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_WASM_ASYNC_CANVAS_SOURCE_H_
