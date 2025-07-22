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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_CANVAS_SOURCE_WRAPPER_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_CANVAS_SOURCE_WRAPPER_H_

#include <memory>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/canvas_source.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/math/vec.h"
#include "core/text/text_helpers.h"
#include "core/view/base_view.h"

namespace imp {

// A wrapper around a synchronous CanvasSource that makes it implement the
// AsyncCanvasSource interface by wrapping return values as Futures.
class AsyncCanvasSourceWrapper : public AsyncCanvasSource {
 public:
  explicit AsyncCanvasSourceWrapper(std::unique_ptr<CanvasSource> source)
      : source_(std::move(source)) {}

  bool IsFeatureSupported(ScopedCanvas::Feature feature) override;

  Future<absl::Status> PrepareFont(
      absl::string_view text,
      const ScopedCanvas::TextOptions& text_options) override;

  Future<ScopedCanvas::TextMetrics> MeasureGlyph(
      GlyphToMeasure glyph_to_measure,
      ScopedCanvas::TextOptions text_options) override;

  Future<std::vector<ScopedCanvas::TextMetrics>> MeasureGlyphs(
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

  Future<ScopedCanvas::FontInfo> GetFontInfo(
      const ScopedCanvas::TextOptions& text_options) override;

  std::unique_ptr<AsyncScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::DrawMode draw_mode =
          ScopedCanvas::DrawMode::kClear) override;

  std::unique_ptr<AsyncScopedCanvas> StartDrawing(
      BaseView& view, uint2 pixel_size,
      ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
      ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) override;

 private:
  std::unique_ptr<CanvasSource> source_;

  ScopedCanvas::TextMetrics MeasureGlyphSync(
      GlyphToMeasure glyph_to_measure, ScopedCanvas::TextOptions text_options);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_ASYNC_CANVAS_SOURCE_WRAPPER_H_
