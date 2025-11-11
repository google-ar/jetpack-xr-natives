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

#include "core/canvas/wasm_async_canvas_source.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <numeric>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/variant.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/text/text_helpers.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/base_view.h"
#include "core/view/platforms/wasm/wasm_canvas_manager.h"

namespace imp {
namespace {

constexpr float kMaxRGB = 255.0f;
constexpr absl::string_view kDefaultFontName = "sans-serif";
constexpr absl::string_view kFontWeightLight = "300";
constexpr absl::string_view kFontWeightNormal = "normal";
constexpr absl::string_view kFontWeightMedium = "500";
constexpr absl::string_view kFontWeightBold = "bold";

absl::string_view GetFontWeightFromTextOptions(
    const ScopedCanvas::TextOptions& text_options) {
  if (text_options.font_holder) {
    switch (text_options.font_holder->GetFontWeight()) {
      case FontWeight::FONT_WEIGHT_LIGHT:
        return kFontWeightLight;
      case FontWeight::FONT_WEIGHT_NORMAL:
        return kFontWeightNormal;
      case FontWeight::FONT_WEIGHT_MEDIUM:
        return kFontWeightMedium;
      case FontWeight::FONT_WEIGHT_BOLD:
        return kFontWeightBold;
    }
  }
  return "";
}

absl::string_view GetTextStyleFromTextOptions(
    const ScopedCanvas::TextOptions& text_options) {
  if (text_options.font_holder) {
    switch (text_options.font_holder->GetTextStyle()) {
      case TextStyle::TEXT_STYLE_NORMAL:
        return "";
      case TextStyle::TEXT_STYLE_ITALIC:
        return "italic";
    }
  }
  return "";
}

class WasmCanvasManagerCallbacks
    : public WasmCanvasManager::PixelBufferCallbacks {
 public:
  WasmCanvasManagerCallbacks(WasmAsyncCanvasSource& source, BaseView& view)
      : source_(source), view_(view) {};

  void OnPixelBufferUpdated(uint8_t* data, int length,
                            uint32_t* dirty_rects_data,
                            int dirty_rects_length) override {
    source_.OnPixelBufferReady(view_, data, length, dirty_rects_data,
                               dirty_rects_length);
  };

  void OnRectCleared(const Rect& rect) override {
    source_.OnRectCleared(rect);
  };

 private:
  WasmAsyncCanvasSource& source_;
  BaseView& view_;
};

}  // namespace

WasmAsyncCanvasSource::WasmAsyncCanvasSource()
    : measuring_canvas_(
          std::make_unique<WasmCanvasManager>(nullptr, float2(0.0f))),
      measuring_scoped_canvas_(*this, measuring_canvas_.get(), float2(0.0f),
                               false) {}

bool WasmAsyncCanvasSource::IsFeatureSupported(ScopedCanvas::Feature feature) {
  switch (feature) {
    case ScopedCanvas::Feature::kGlyphs:
      return false;
    case ScopedCanvas::Feature::kKeepContents:
      return true;
  }
}

Texture* WasmAsyncCanvasSource::GetTexture() { return texture_.operator->(); }

Future<absl::Status> WasmAsyncCanvasSource::PrepareFont(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  absl::MutexLock lock(&canvas_mutex_);
  absl::string_view font_family = kDefaultFontName;
  absl::string_view font_weight = GetFontWeightFromTextOptions(text_options);
  absl::string_view text_style = GetTextStyleFromTextOptions(text_options);

  if (text_options.font_holder) {
    font_family =
        *static_cast<std::string*>(text_options.font_holder->GetPlatformFont());
  }

  return measuring_canvas_->PrepareFont(text, font_family, font_weight,
                                        text_style, text_options.size_pixels);
}

Future<std::vector<TextAndFontMetrics>>
WasmAsyncCanvasSource::GetFontAndTextMetrics(
    std::vector<ScopedCanvas::TextToMeasure> texts) {
  absl::MutexLock lock(&canvas_mutex_);
  return measuring_scoped_canvas_.GetFontAndTextMetrics(texts);
}

Future<TextMetrics> WasmAsyncCanvasSource::MeasureGlyph(
    GlyphToMeasure glyph_to_measure, ScopedCanvas::TextOptions text_options) {
  absl::MutexLock lock(&canvas_mutex_);

  if (!absl::holds_alternative<absl::string_view>(glyph_to_measure.glyph)) {
    IMP_LOG(imp::ERROR) << "Measuring by GlyphId is not supported on WASM.";
    return {};
  }

  // TODO Don't measure the same text multiple times.
  std::string text =
      std::string(absl::get<absl::string_view>(glyph_to_measure.glyph));
  std::vector<Chunk> typographical_chunks =
      GetChunks(text, text_options.force_non_separable);
  Future<float> typographical_width_future = Future<float>(0.0f);

  // Because measuring the typographical width incurs extra work, by default
  // don't do the calculations unless the user explicitly requires it.
  if (text_options.should_measure_typographical_width) {
    typographical_width_future =
        measuring_scoped_canvas_
            .MeasureTexts(typographical_chunks, text_options,
                          /*only_widths=*/true)
            .Then(
                [](std::vector<std::vector<float>> measurements) {
                  // Compute typographical width by summing the advance widths
                  // of each chunk.
                  float typographical_width = 0.0f;
                  for (const std::vector<float>& measurement : measurements) {
                    typographical_width += std::accumulate(
                        measurement.begin(), measurement.end(), 0.0f);
                  }
                  return typographical_width;
                },
                Executor::Type::kImmediate);
  }

  Future<std::vector<TextMetrics>> measure_text_future =
      measuring_scoped_canvas_.MeasureGlyphs(
          {{.chunk_text = text, .codepoint_count = 1, .is_separable = false}},
          text_options);

  return measure_text_future.Merge(typographical_width_future)
      .Then(
          [should_measure_typographical_width =
               text_options.should_measure_typographical_width](
              std::tuple<std::vector<TextMetrics>, float> tuple)
              -> TextMetrics {
            auto [text_metrics, typographical_width] = tuple;
            if (text_metrics.size() == 1) {
              if (should_measure_typographical_width) {
                text_metrics[0].set_typographical_width(typographical_width);
              }
              return text_metrics[0];
            } else {
              IMP_LOG(imp::ERROR) << "Unexpected text_metrics size "
                         << text_metrics.size();
              return {};
            }
          },
          Executor::Type::kCurrent);
}

Future<std::vector<TextMetrics>> WasmAsyncCanvasSource::MeasureGlyphs(
    std::vector<GlyphToMeasure> glyphs_to_measure,
    ScopedCanvas::TextOptions text_options) {
  absl::MutexLock lock(&canvas_mutex_);
  return measuring_scoped_canvas_.MeasureGlyphs(glyphs_to_measure,
                                                text_options);
}

// TODO: Support combining characters.
Future<std::vector<ScopedCanvas::GlyphGroup>>
WasmAsyncCanvasSource::GetCombinedCharacterGroups(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return Future<std::vector<ScopedCanvas::GlyphGroup>>(
      std::vector<ScopedCanvas::GlyphGroup>());
}

Future<std::vector<std::vector<float>>> WasmAsyncCanvasSource::GetTextWidths(
    const std::vector<Chunk>& chunks,
    const ScopedCanvas::TextOptions& text_options) {
  absl::MutexLock lock(&canvas_mutex_);
  return measuring_scoped_canvas_.GetTextWidths(chunks, text_options);
}

Future<std::unique_ptr<std::vector<ScopedCanvas::GlyphAdvance>>>
WasmAsyncCanvasSource::GetTextGlyphs(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return Future<std::unique_ptr<std::vector<ScopedCanvas::GlyphAdvance>>>(
      absl::UnimplementedError(
          "CanvasSource::GetTextGlyphs is unavailable on WASM."));
}

Future<FontInfo> WasmAsyncCanvasSource::GetFontInfo(
    const ScopedCanvas::TextOptions& text_options) {
  absl::MutexLock lock(&canvas_mutex_);
  return measuring_scoped_canvas_.GetFontInfo(text_options);
}

std::unique_ptr<AsyncScopedCanvas> WasmAsyncCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size, ScopedCanvas::DrawMode draw_mode) {
  absl::MutexLock lock(&canvas_mutex_);
  bool did_texture_change = false;
  if (!texture_ || pixel_size_ != pixel_size) {
    pixel_size_ = pixel_size;
#if IMP_RUNTIME(DEV)
    texture_ = view.GetTextureFactory().CreateTexture(
        pixel_size_.x, pixel_size_.y, filament::Texture::InternalFormat::RGBA8,
        filament::Texture::Usage::COLOR_ATTACHMENT |
            filament::Texture::Usage::BLIT_SRC |
            filament::Texture::Usage::DEFAULT);
#else
    texture_ = view.GetTextureFactory().CreateTexture(
        pixel_size_.x, pixel_size_.y, filament::Texture::InternalFormat::RGBA8);
#endif
    drawing_canvas_ = std::make_unique<WasmCanvasManager>(
        std::make_unique<WasmCanvasManagerCallbacks>(*this, view), pixel_size_);
    did_texture_change = true;
  }

  return std::make_unique<WasmScopedCanvas>(*this, drawing_canvas_.get(),
                                            pixel_size_, did_texture_change);
}

// TODO: Implement kClear mode.
std::unique_ptr<AsyncScopedCanvas> WasmAsyncCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) {
  absl::MutexLock lock(&canvas_mutex_);
  bool did_texture_change = false;
  if (!texture_ || pixel_size_ != pixel_size) {
    pixel_size_ = pixel_size;

    // Don't destroy until after on_texture_changed_fn is called so that the
    // caller has the opportunity to clear references to the old texture.
    OwnedTexturePtr old_texture = std::move(texture_);

#if IMP_RUNTIME(DEV)
    texture_ = view.GetTextureFactory().CreateTexture(
        pixel_size_.x, pixel_size_.y, filament::Texture::InternalFormat::RGBA8,
        filament::Texture::Usage::COLOR_ATTACHMENT |
            filament::Texture::Usage::DEFAULT);
#else
    texture_ = view.GetTextureFactory().CreateTexture(
        pixel_size_.x, pixel_size_.y, filament::Texture::InternalFormat::RGBA8);
#endif
    drawing_canvas_ = std::make_unique<WasmCanvasManager>(
        std::make_unique<WasmCanvasManagerCallbacks>(*this, view), pixel_size_);
    did_texture_change = true;

    on_texture_changed_fn(texture_.Borrow(loc));
  }

  return std::make_unique<WasmScopedCanvas>(*this, drawing_canvas_.get(),
                                            pixel_size_, did_texture_change);
}

void WasmAsyncCanvasSource::OnPixelBufferReady(BaseView& view, uint8_t* data,
                                               int length,
                                               uint32_t* dirty_rects_data,
                                               int dirty_rects_length) {
  filament::backend::PixelDataFormat format = filament::Texture::Format::RGBA;
  filament::backend::PixelDataType type = filament::Texture::Type::UBYTE;

  Future<absl::Status> gpu_upload_future(absl::OkStatus());
  int data_offset = 0;
  for (int i = 0; i < dirty_rects_length; i += 4) {
    const uint32_t x = dirty_rects_data[i];
    const uint32_t y = dirty_rects_data[i + 1];
    const uint32_t width = dirty_rects_data[i + 2];
    const uint32_t height = dirty_rects_data[i + 3];
    const int block_size = (int)(width * height * 4);
    Future<absl::Status> gpu_upload;
    gpu_upload_future = gpu_upload_future.Combine(gpu_upload);
    filament::Texture::PixelBufferDescriptor pixel_buffer =
        filament::Texture::PixelBufferDescriptor::make(
            data + data_offset, block_size, format, type,
            [gpu_upload](void* buffer, size_t size) {
              gpu_upload.Return(absl::OkStatus());
            });
    texture_->GetTexture()->setImage(*BaseView::GetSharedEngine(),
                                     /*level=*/0, x, y, width, height,
                                     std::move(pixel_buffer));
    data_offset += block_size;
  }

  // Free the buffer that was allocated from the JS side.
  // The data buffer contains multiple rects that need to be updated, so once
  // all of them have been uploaded to the gpu we can free the buffer.
  gpu_upload_future
      .Then([data, dirty_rects_data]() {
        free(data);
        free(dirty_rects_data);
      })
      .KeptBy(&view);
}

void WasmAsyncCanvasSource::OnRectCleared(Rect rect) {
  const float x = rect.center.x - rect.half_extent.x;
  const float y = rect.center.y - rect.half_extent.y;
  const float width = rect.half_extent.x * 2;
  const float height = rect.half_extent.y * 2;
  filament::backend::PixelDataFormat format = filament::Texture::Format::RGBA;
  filament::backend::PixelDataType type = filament::Texture::Type::UBYTE;
  std::unique_ptr<uint8_t[]> zero_rect =
      std::make_unique<uint8_t[]>(width * height * 4);
  filament::Texture::PixelBufferDescriptor pixel_buffer =
      filament::Texture::PixelBufferDescriptor::make(
          zero_rect.release(), width * height * 4, format, type,
          [](void* buffer, size_t size) {
            std::unique_ptr<uint8_t[]> buffer_deleter(
                reinterpret_cast<uint8_t*>(buffer));
          });
  texture_->GetTexture()->setImage(*BaseView::GetSharedEngine(),
                                   /*level=*/0, x, y, width, height,
                                   std::move(pixel_buffer));
}

WasmAsyncCanvasSource::WasmScopedCanvas::WasmScopedCanvas(
    WasmAsyncCanvasSource& source, WasmCanvasManager* platform_canvas_wrapper,
    uint2 pixel_size, bool did_texture_change)
    : source_(source),
      platform_canvas_wrapper_(platform_canvas_wrapper),
      did_texture_change_(did_texture_change) {}

WasmAsyncCanvasSource::WasmScopedCanvas::~WasmScopedCanvas() {
  if (this->SupportsSynchronousTextureUpdate()) {
    (void)platform_canvas_wrapper_->RequestDraw(true);

  } else {
    (void)platform_canvas_wrapper_->GetBuffer();
  }
}

Texture* WasmAsyncCanvasSource::WasmScopedCanvas::GetTexture() {
  return source_.texture_.operator->();
}

bool WasmAsyncCanvasSource::WasmScopedCanvas::DidTextureChange() const {
  return did_texture_change_;
}

void WasmAsyncCanvasSource::WasmScopedCanvas::DrawColor(float3 color) {
  DrawColor(float4(color, 1.0f));
}

void WasmAsyncCanvasSource::WasmScopedCanvas::DrawColor(float4 color) {
  float half_width = source_.pixel_size_.x / 2.0f;
  float half_height = source_.pixel_size_.y / 2.0f;
  DrawRoundedRect(
      color, float2(0.0f),
      Rect{float2(half_width, half_height), float2(half_width, half_height)});
}

void WasmAsyncCanvasSource::WasmScopedCanvas::DrawRoundedRect(
    float3 color, float2 corner_radius, const Rect& rect) {
  DrawRoundedRect(float4(color, 1.0f), corner_radius, rect);
}

void WasmAsyncCanvasSource::WasmScopedCanvas::DrawRoundedRect(
    float4 color, float2 corner_radius, const Rect& rect) {
  platform_canvas_wrapper_->SetFillStyle(color);
  platform_canvas_wrapper_->DrawRoundedRect(rect, corner_radius);
}

void WasmAsyncCanvasSource::WasmScopedCanvas::DrawText(
    absl::string_view text, float2 pos, const TextOptions& text_options) {
  SetTextOptions(text_options);
  platform_canvas_wrapper_->DrawText(
      text, pos, text_options.horizontal_alignment,
      text_options.vertical_alignment, text_options.render_scale.x);
}

void WasmAsyncCanvasSource::WasmScopedCanvas::DrawGlyph(
    GlyphId glyph, float2 pos, const TextOptions& text_options) {
  IMP_LOG(imp::FATAL) << "ScopedCanvas::DrawGlyph is unavailable on WASM.";
}

void WasmAsyncCanvasSource::WasmScopedCanvas::ClearRect(const Rect& rect) {
  platform_canvas_wrapper_->ClearRect(rect);
}

Future<absl::Status> WasmAsyncCanvasSource::WasmScopedCanvas::PrepareFont(
    absl::string_view text, const TextOptions& text_options) {
  absl::string_view font_family = kDefaultFontName;
  absl::string_view font_weight = GetFontWeightFromTextOptions(text_options);
  absl::string_view text_style = GetTextStyleFromTextOptions(text_options);
  if (text_options.font_holder) {
    font_family =
        *static_cast<std::string*>(text_options.font_holder->GetPlatformFont());
  }

  return platform_canvas_wrapper_->PrepareFont(
      text, font_family, font_weight, text_style, text_options.size_pixels);
}

Future<std::vector<TextAndFontMetrics>>
WasmAsyncCanvasSource::WasmScopedCanvas::GetFontAndTextMetrics(
    std::vector<ScopedCanvas::TextToMeasure> texts) {
  for (const ScopedCanvas::TextToMeasure& text : texts) {
    SetTextOptions(text.text_options);

    // Because measuring the typographical width incurs extra work, by default
    // don't do the calculations unless the user explicitly requires it.
    if (text.text_options.should_measure_typographical_width) {
      // TODO Consolidate emscripten calls to ensure an atomic
      // operation. This involves marshalling the texts to measure, as well as
      // making this thread safe by potentially associating a future id with the
      // group of texts to be measured.
      platform_canvas_wrapper_->ClearTextToMeasure();
      std::vector<Chunk> typographical_chunks =
          GetChunks(text.text, text.text_options.force_non_separable);
      for (const Chunk& chunk : typographical_chunks) {
        platform_canvas_wrapper_->AddTextToMeasure(chunk.chunk_text,
                                                   chunk.is_separable);
      }
    }
    platform_canvas_wrapper_->CollectGetFontAndTextMetricsInputs(text.text);
  }
  return platform_canvas_wrapper_->GetFontAndTextMetrics().Then(
      [texts = std::move(texts)](
          WasmCanvasManager::FlattenedFontAndTextMetricsResults measurements)
          -> std::vector<TextAndFontMetrics> {
        const std::vector<std::vector<float>>& text_measurements =
            measurements.per_text_measurements;
        const std::vector<std::vector<float>>& glyph_measurements =
            measurements.per_glyph_measurements;
        assert(text_measurements.size() == glyph_measurements.size());
        std::vector<TextAndFontMetrics> text_and_font_metrics;
        text_and_font_metrics.reserve(text_measurements.size());
        for (int i = 0; i < text_measurements.size(); i++) {
          if (text_measurements[i].size() != 9) {
            IMP_LOG(imp::ERROR) << "Unexpected measurement size "
                       << text_measurements[i].size();
            continue;
          }

          float size_pixels =
              static_cast<float>(texts[i].text_options.size_pixels);
          // LINT.IfChange
          TextMetrics text_metrics;
          text_metrics.set_origin_x(text_measurements[i][0]);
          text_metrics.set_origin_y(text_measurements[i][1]);
          text_metrics.set_size_x(text_measurements[i][2]);
          text_metrics.set_size_y(text_measurements[i][3]);
          text_metrics.set_typographical_width(text_measurements[i][4]);
          text_metrics.set_font_origin_y(text_measurements[i][5]);
          text_metrics.set_font_size_y(text_measurements[i][6]);
          FontInfo font_info;
          font_info.set_ascent(text_measurements[i][7]);
          font_info.set_descent(text_measurements[i][8]);
          // TODO: Supply proper
          // values for these two metrics.
          font_info.set_leading(size_pixels);
          font_info.set_line_spacing(size_pixels);
          TextAndFontMetrics metrics;
          *metrics.mutable_text_metrics() = text_metrics;
          *metrics.mutable_font_info() = font_info;

          std::vector<TextMetrics> glyph_metrics;
          if (glyph_measurements[i].size() % 7 == 0) {
            for (int j = 0; j < glyph_measurements[i].size(); j += 7) {
              TextMetrics text_metrics;
              text_metrics.set_origin_x(glyph_measurements[i][j]);
              text_metrics.set_origin_y(glyph_measurements[i][j + 1]);
              text_metrics.set_size_x(glyph_measurements[i][j + 2]);
              text_metrics.set_size_y(glyph_measurements[i][j + 3]);
              text_metrics.set_typographical_width(
                  glyph_measurements[i][j + 4]);
              text_metrics.set_font_origin_y(glyph_measurements[i][j + 5]);
              text_metrics.set_font_size_y(glyph_measurements[i][j + 6]);
              glyph_metrics.push_back(text_metrics);
            }
            metrics.mutable_glyph_metrics()->Add(glyph_metrics.begin(),
                                                 glyph_metrics.end());
          } else {
            IMP_LOG(imp::ERROR) << "Unexpected glyph measurement size "
                        << glyph_measurements[i].size();
          }

          text_and_font_metrics.push_back(metrics);
          // LINT.ThenChange(//depot/google3/third_party/impress/javascript/core/wasm/canvas/wasm_canvas_renderer.ts)
        }
        return text_and_font_metrics;
      });
}

Future<std::vector<TextMetrics>>
WasmAsyncCanvasSource::WasmScopedCanvas::MeasureGlyphs(
    std::vector<GlyphToMeasure> glyphs_to_measure, TextOptions text_options) {
  std::vector<Chunk> chunks;
  chunks.reserve(glyphs_to_measure.size());
  for (const GlyphToMeasure& glyph_to_measure : glyphs_to_measure) {
    if (absl::holds_alternative<absl::string_view>(glyph_to_measure.glyph)) {
      chunks.push_back(Chunk{
          .chunk_text =
              std::string(absl::get<absl::string_view>(glyph_to_measure.glyph)),
          .is_separable = false,
      });
    } else {
      IMP_LOG(imp::ERROR) << "Measuring by GlyphId is unavailable in WASM";
    }
  }
  return MeasureGlyphs(chunks, text_options);
}

Future<std::vector<TextMetrics>>
WasmAsyncCanvasSource::WasmScopedCanvas::MeasureGlyphs(
    std::vector<Chunk> chunks, TextOptions text_options) {
  return MeasureTexts(chunks, text_options, /*only_widths=*/false)
      .Then(
          [](std::vector<std::vector<float>> measurements) {
            std::vector<TextMetrics> text_metrics;
            text_metrics.reserve(measurements.size());
            for (int i = 0; i < measurements.size(); i++) {
              // LINT.IfChange
              if (measurements[i].size() == 7) {
                TextMetrics metrics;
                metrics.set_origin_x(measurements[i][0]);
                metrics.set_origin_y(measurements[i][1]);
                metrics.set_size_x(measurements[i][2]);
                metrics.set_size_y(measurements[i][3]);
                metrics.set_typographical_width(measurements[i][4]);
                metrics.set_font_origin_y(measurements[i][5]);
                metrics.set_font_size_y(measurements[i][6]);
                text_metrics.push_back(metrics);
              } else {
                IMP_LOG(imp::ERROR) << "Unexpected measurement size "
                           << measurements[i].size();
              }
              // LINT.ThenChange(//depot/google3/third_party/impress/javascript/core/wasm/canvas/wasm_canvas_renderer.ts)
            }
            return text_metrics;
          },
          Executor::Type::kImmediate);
}

Future<std::vector<std::vector<float>>>
WasmAsyncCanvasSource::WasmScopedCanvas::GetTextWidths(
    const std::vector<Chunk>& chunks, const TextOptions& text_options) {
  return MeasureTexts(chunks, text_options, /*only_widths=*/true);
}

Future<std::vector<std::vector<float>>>
WasmAsyncCanvasSource::WasmScopedCanvas::MeasureTexts(
    const std::vector<Chunk>& chunks, const TextOptions& text_options,
    bool only_widths) {
  if (chunks.empty()) {
    std::vector<std::vector<float>> widths = {};
    return Future<std::vector<std::vector<float>>>(widths);
  }

  SetTextOptions(text_options);
  // TODO Consolidate emscripten calls to ensure an atomic
  // operation. This involves marshalling the texts to measure, as well as
  // making this thread safe by potentially associating a future id with the
  // group of texts to be measured.
  platform_canvas_wrapper_->ClearTextToMeasure();
  for (const Chunk& chunk : chunks) {
    platform_canvas_wrapper_->AddTextToMeasure(chunk.chunk_text,
                                               chunk.is_separable);
  }

  return platform_canvas_wrapper_->MeasureText(only_widths);
}

Future<absl::Status>
WasmAsyncCanvasSource::WasmScopedCanvas::PrepareToUpdateTexture() {
  return platform_canvas_wrapper_->RequestDraw(false);
}

bool WasmAsyncCanvasSource::WasmScopedCanvas::SupportsSynchronousTextureUpdate()
    const {
  return platform_canvas_wrapper_->SupportsSynchronousTextureUpdate();
}

Future<FontInfo> WasmAsyncCanvasSource::WasmScopedCanvas::GetFontInfo(
    const TextOptions& text_options) {
  SetTextOptions(text_options);
  Future<std::vector<std::vector<float>>> measure_results_future =
      platform_canvas_wrapper_->GetFontInfo();

  float size = static_cast<float>(text_options.size_pixels);

  return measure_results_future.Then(
      [size](std::vector<std::vector<float>> measurements) {
        FontInfo info;
        info.set_ascent(measurements[0][0]);
        info.set_descent(measurements[0][1]);
        // TODO: Supply proper values for these two metrics.
        info.set_leading(size);
        info.set_line_spacing(size);
        return info;
      });
}

void WasmAsyncCanvasSource::WasmScopedCanvas::SetTextOptions(
    TextOptions text_options) {
  platform_canvas_wrapper_->SetFillStyle(text_options.color);
  platform_canvas_wrapper_->SetStrokeStyle(text_options.stroke_width_pixels,
                                           text_options.stroke_color);

  absl::string_view font_family = kDefaultFontName;
  absl::string_view font_weight = GetFontWeightFromTextOptions(text_options);
  absl::string_view text_style = GetTextStyleFromTextOptions(text_options);
  if (text_options.font_holder) {
    font_family =
        *static_cast<std::string*>(text_options.font_holder->GetPlatformFont());
  }

  platform_canvas_wrapper_->SetTextOptions(text_options.size_pixels,
                                           font_family, font_weight, text_style,
                                           text_options.text_tracking);
}

}  // namespace imp
