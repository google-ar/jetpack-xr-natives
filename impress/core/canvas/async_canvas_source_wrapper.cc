// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/canvas/async_canvas_source_wrapper.h"

#include <memory>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "core/async/future.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/async_scoped_canvas_wrapper.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/text/text_helpers.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/base_view.h"

namespace imp {

bool AsyncCanvasSourceWrapper::IsFeatureSupported(
    ScopedCanvas::Feature feature) {
  return source_->IsFeatureSupported(feature);
};

Future<absl::Status> AsyncCanvasSourceWrapper::PrepareFont(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return source_->PrepareFont(text, text_options);
};

Future<std::vector<TextAndFontMetrics>>
AsyncCanvasSourceWrapper::GetFontAndTextMetrics(
    std::vector<ScopedCanvas::TextToMeasure> texts) {
  std::vector<TextAndFontMetrics> text_and_font_metrics;
  text_and_font_metrics.reserve(texts.size());
  for (const ScopedCanvas::TextToMeasure& text : texts) {
    TextAndFontMetrics metrics;
    *metrics.mutable_text_metrics() =
        MeasureGlyphSync({.glyph = text.text}, text.text_options);
    *metrics.mutable_font_info() = source_->GetFontInfo(text.text_options);
    text_and_font_metrics.push_back(metrics);
  }
  return Future<std::vector<TextAndFontMetrics>>(text_and_font_metrics);
}

Future<TextMetrics> AsyncCanvasSourceWrapper::MeasureGlyph(
    GlyphToMeasure glyph_to_measure, ScopedCanvas::TextOptions text_options) {
  return Future<TextMetrics>(MeasureGlyphSync(glyph_to_measure, text_options));
}

Future<std::vector<TextMetrics>> AsyncCanvasSourceWrapper::MeasureGlyphs(
    std::vector<GlyphToMeasure> glyphs_to_measure,
    ScopedCanvas::TextOptions text_options) {
  std::vector<TextMetrics> text_metrics;
  text_metrics.reserve(glyphs_to_measure.size());
  for (const GlyphToMeasure& glyph : glyphs_to_measure) {
    text_metrics.push_back(MeasureGlyphSync(glyph, text_options));
  }
  return Future<std::vector<TextMetrics>>(text_metrics);
}

Future<std::vector<ScopedCanvas::GlyphGroup>>
AsyncCanvasSourceWrapper::GetCombinedCharacterGroups(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return Future<std::vector<ScopedCanvas::GlyphGroup>>(
      source_->GetCombinedCharacterGroups(text, text_options));
}

Future<std::vector<std::vector<float>>> AsyncCanvasSourceWrapper::GetTextWidths(
    const std::vector<Chunk>& chunks,
    const ScopedCanvas::TextOptions& text_options) {
  std::vector<std::vector<float>> widths;
  widths.reserve(chunks.size());
  for (const Chunk& chunk : chunks) {
    if (chunk.is_separable) {
      widths.push_back(source_->GetTextWidths(chunk.chunk_text, text_options));
    } else {
#if IMP_PLATFORM(DESKTOP)
      // TODO: Relying on the metrics here is incorrect, but the
      // sum of the glyph advances on desktop does not correctly sum to the
      // total advance of the string.
      widths.push_back(
          {source_->GetTextMetrics(chunk.chunk_text, text_options).size_x()});
#else
      widths.push_back(source_->GetTextWidths(chunk.chunk_text, text_options));
#endif
    }
  }
  return Future<std::vector<std::vector<float>>>(widths);
};

Future<std::unique_ptr<std::vector<ScopedCanvas::GlyphAdvance>>>
AsyncCanvasSourceWrapper::GetTextGlyphs(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return Future<std::unique_ptr<std::vector<ScopedCanvas::GlyphAdvance>>>(
      std::make_unique<std::vector<ScopedCanvas::GlyphAdvance>>(
          source_->GetTextGlyphs(text, text_options)));
};

Future<FontInfo> AsyncCanvasSourceWrapper::GetFontInfo(
    const ScopedCanvas::TextOptions& text_options) {
  return Future<FontInfo>(source_->GetFontInfo(text_options));
};

std::unique_ptr<AsyncScopedCanvas> AsyncCanvasSourceWrapper::StartDrawing(
    BaseView& view, uint2 pixel_size, ScopedCanvas::DrawMode draw_mode) {
  return absl::WrapUnique(new AsyncScopedCanvasWrapper(
      source_->StartDrawing(view, pixel_size, draw_mode)));
}

std::unique_ptr<AsyncScopedCanvas> AsyncCanvasSourceWrapper::StartDrawing(
    BaseView& view, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) {
  return absl::WrapUnique(new AsyncScopedCanvasWrapper(source_->StartDrawing(
      view, pixel_size, std::move(on_texture_changed_fn), draw_mode, loc)));
}

void AsyncCanvasSourceWrapper::ForceReset() { source_->ForceReset(); }

TextMetrics AsyncCanvasSourceWrapper::MeasureGlyphSync(
    GlyphToMeasure glyph_to_measure, ScopedCanvas::TextOptions text_options) {
  if (glyph_to_measure.font_override) {
    text_options.font_holder = glyph_to_measure.font_override;
  }
  if (absl::holds_alternative<ScopedCanvas::GlyphId>(glyph_to_measure.glyph)) {
    return source_->GetGlyphMetrics(
        absl::get<ScopedCanvas::GlyphId>(glyph_to_measure.glyph), text_options);
  } else {
    return source_->GetTextMetrics(
        absl::get<absl::string_view>(glyph_to_measure.glyph), text_options);
  }
}

}  // namespace imp
