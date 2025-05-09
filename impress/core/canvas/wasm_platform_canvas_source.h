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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_WASM_PLATFORM_CANVAS_SOURCE_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_WASM_PLATFORM_CANVAS_SOURCE_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/platform_canvas_source.h"
#include "core/canvas/scoped_canvas.h"
#include "core/canvas/wasm_async_canvas_source.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "imp.h"
namespace imp {

// A synchronous wrapper around a WasmAsyncCanvasSource to be used as a
// PlatformCanvasSource. This assumes the WasmAsyncCanvasSource produces results
// synchronously, which is true as long as the WasmAsyncCanvasSource does not
// have a remote message port.
class WasmPlatformCanvasSource : public PlatformCanvasSource {
 public:
  explicit WasmPlatformCanvasSource(BaseView& view);
  ~WasmPlatformCanvasSource() = default;

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
  std::unique_ptr<WasmAsyncCanvasSource> wrapped_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_WASM_PLATFORM_CANVAS_SOURCE_H_
