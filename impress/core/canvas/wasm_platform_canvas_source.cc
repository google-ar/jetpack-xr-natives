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

#include "core/canvas/wasm_platform_canvas_source.h"

#include <memory>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/scoped_canvas.h"
#include "core/canvas/wasm_async_canvas_source.h"
#include "core/common/small_source_location.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/text/text_helpers.h"
#include "core/view/base_view.h"

namespace imp {
bool WasmPlatformCanvasSource::IsFeatureSupported(
    ScopedCanvas::Feature feature) {
  return wrapped_.IsFeatureSupported(feature);
};

Texture* WasmPlatformCanvasSource::GetTexture() {
  return wrapped_.GetTexture();
};

Future<absl::Status> WasmPlatformCanvasSource::PrepareFont(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return wrapped_.PrepareFont(text, text_options);
};

ScopedCanvas::TextMetrics WasmPlatformCanvasSource::GetTextMetrics(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  absl::StatusOr<ScopedCanvas::TextMetrics> result =
      wrapped_
          .MeasureGlyph(AsyncCanvasSource::GlyphToMeasure({.glyph = text}),
                        text_options)
          .Get();

  if (result.ok()) {
    return result.value();
  } else {
    IMP_LOG(imp::FATAL) << "Failed to measure text widths.";
    return ScopedCanvas::TextMetrics();
  }
};

ScopedCanvas::TextMetrics WasmPlatformCanvasSource::GetGlyphMetrics(
    ScopedCanvas::GlyphId glyph,
    const ScopedCanvas::TextOptions& text_options) {
  IMP_LOG(imp::FATAL) << "CanvasSource::GetGlyphMetrics is unavailable on WASM.";
  return {};
}

std::vector<ScopedCanvas::GlyphGroup>
WasmPlatformCanvasSource::GetCombinedCharacterGroups(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  absl::StatusOr<std::vector<ScopedCanvas::GlyphGroup>> result =
      wrapped_.GetCombinedCharacterGroups(text, text_options).Get();
  if (result.ok()) {
    return *result;
  }

  IMP_LOG(imp::FATAL) << "Failed to get combined character groups on WASM";
  return {};
}

std::vector<float> WasmPlatformCanvasSource::GetTextWidths(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  absl::StatusOr<std::vector<std::vector<float>>> result =
      wrapped_.GetTextWidths(GetChunks(text), text_options).Get();
  if (result.ok() && result->size() == 1) {
    return result.value()[0];
  } else {
    IMP_LOG(imp::FATAL) << "Failed to measure text widths.";
    return {};
  }
}

std::vector<ScopedCanvas::GlyphAdvance> WasmPlatformCanvasSource::GetTextGlyphs(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  IMP_LOG(imp::FATAL) << "CanvasSource::GetTextGlyphs is unavailable on WASM.";
  return {};
}

ScopedCanvas::FontInfo WasmPlatformCanvasSource::GetFontInfo(
    const ScopedCanvas::TextOptions& text_options) {
  absl::StatusOr<ScopedCanvas::FontInfo> result =
      wrapped_.GetFontInfo(text_options).Get();

  if (result.ok()) {
    return result.value();
  } else {
    IMP_LOG(imp::FATAL) << "Failed to get font info.";
    return ScopedCanvas::FontInfo();
  }
}

std::unique_ptr<ScopedCanvas> WasmPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size, ScopedCanvas::DrawMode draw_mode) {
  return wrapped_.StartDrawing(view, pixel_size, draw_mode);
}

std::unique_ptr<ScopedCanvas> WasmPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) {
  return wrapped_.StartDrawing(
      view, pixel_size, std::move(on_texture_changed_fn), draw_mode, loc);
}

}  // namespace imp
