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

#include "core/canvas/android_platform_canvas_source.h"

#include <jni.h>

#include <memory>
#include <numeric>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/async/future.h"
#include "core/canvas/android_glyph_source.h"
#include "core/canvas/constants.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/canvas.h"
#include "core/view/platforms/android/wrappers/paint.h"
#include "core/view/platforms/android/wrappers/picture.h"
#include "core/view/platforms/android/wrappers/rect.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/platforms/android/wrappers/surface_texture.h"

namespace imp {

android::Paint::Align TextAlignmentToPaintAlign(
    TextHorizontalAlignment horizontal_alignment) {
  switch (horizontal_alignment) {
    case TextHorizontalAlignment::kCenter:
      return android::Paint::Align::kCenter;
    case TextHorizontalAlignment::kLeft:
    case TextHorizontalAlignment::kLeftExtent:
      return android::Paint::Align::kLeft;
    case TextHorizontalAlignment::kRight:
    case TextHorizontalAlignment::kRightExtent:
      return android::Paint::Align::kRight;
  }
}

inline void ConfigurePaintForTextOptionsCommon(
    android::Paint& paint, const ScopedCanvas::TextOptions& text_options,
    bool configure_for_glyphs) {
  paint.SetTextSize(text_options.size_pixels);
  paint.SetLetterSpacing(text_options.text_tracking);

  if (configure_for_glyphs) {
    paint.SetTextAlign(android::Paint::Align::kLeft);
  } else {
    paint.SetTextAlign(
        TextAlignmentToPaintAlign(text_options.horizontal_alignment));
  }

  if (text_options.font_holder &&
      text_options.font_holder->IsAndroidTypeface()) {
    paint.SetTypeface(
        static_cast<jobject>(text_options.font_holder->GetPlatformFont()));
  } else {
    paint.SetTypeface(jobject{});
  }
}

void ConfigurePaintForTextOptions(android::Paint& paint,
                                  const ScopedCanvas::TextOptions& text_options,
                                  bool configure_for_glyphs) {
  ConfigurePaintForTextOptionsCommon(paint, text_options, configure_for_glyphs);
  paint.SetColor(text_options.color);
}

void ConfigurePaintForStrokeTextOptions(
    android::Paint& paint, const ScopedCanvas::TextOptions& text_options,
    bool configure_for_glyphs) {
  ConfigurePaintForTextOptionsCommon(paint, text_options, configure_for_glyphs);
  paint.SetStrokeWidth(text_options.stroke_width_pixels);
  paint.SetColor(text_options.stroke_color);
}

AndroidPlatformCanvasSource::AndroidPlatformCanvasSource(
    Context context, AndroidGlyphSource::Method glyph_method,
    bool use_hardware_rendering, int glyph_cache_size_bytes,
    bool force_individual_glyph_source_instances)
    : context_(context),
      paint_(context),
      stroke_paint_(context),
      glyph_source_(context, glyph_method, glyph_cache_size_bytes,
                    force_individual_glyph_source_instances),
      use_hardware_rendering_(use_hardware_rendering) {
  paint_.SetAntiAlias(true);
  stroke_paint_.SetAntiAlias(true);
  stroke_paint_.SetStyle(android::Paint::Style::kStroke);
}

bool AndroidPlatformCanvasSource::IsFeatureSupported(
    ScopedCanvas::Feature feature) {
  switch (feature) {
    case ScopedCanvas::Feature::kGlyphs:
      return glyph_source_.IsAvailable();
    case ScopedCanvas::Feature::kKeepContents:
      return false;
  }
}

Future<absl::Status> AndroidPlatformCanvasSource::PrepareFont(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return Future<absl::Status>(absl::OkStatus());
}

TextMetrics AndroidPlatformCanvasSource::GetTextMetrics(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  ConfigurePaintForTextOptions(paint_, text_options,
                               /*configure_for_glyphs=*/false);
  // Even when Paint::SetStrokeWidth is set, Android's paint type ignores stroke
  // when calling measure text. Therefore, we need to manually account for it.
  std::unique_ptr<android::Rect> bounds = paint_.GetTextBounds(text);
  std::vector<float> text_widths = GetTextWidths(text, text_options);
  float typographical_width =
      std::accumulate(text_widths.begin(), text_widths.end(), 0.0f);
  TextMetrics text_metrics;
  // Origin is for the fill not the stroke.
  text_metrics.set_origin_x(bounds->GetLeft());
  text_metrics.set_origin_y(-bounds->GetBottom());
  // Size includes the stroke - 1/2 the stroke on either side as it
  // straddles the fill of the font half in and half out.
  text_metrics.set_size_x(bounds->GetWidth() +
                          text_options.stroke_width_pixels);
  text_metrics.set_size_y(bounds->GetHeight() +
                          text_options.stroke_width_pixels);
  text_metrics.set_typographical_width(typographical_width);
  // TODO: Return proper metrics here
  text_metrics.set_font_origin_y(static_cast<float>(-bounds->GetBottom()));
  text_metrics.set_font_size_y(bounds->GetHeight() +
                               text_options.stroke_width_pixels);
  return text_metrics;
}

TextMetrics AndroidPlatformCanvasSource::GetGlyphMetrics(
    ScopedCanvas::GlyphId glyph,
    const ScopedCanvas::TextOptions& text_options) {
  ConfigurePaintForTextOptions(paint_, text_options,
                               /*configure_for_glyphs=*/true);
  return glyph_source_.GetGlyphMetrics(glyph.Get(), text_options.font_holder,
                                       text_options.stroke_width_pixels,
                                       paint_);
}

std::vector<ScopedCanvas::GlyphGroup>
AndroidPlatformCanvasSource::GetCombinedCharacterGroups(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  ConfigurePaintForTextOptions(paint_, text_options,
                               /*configure_for_glyphs=*/true);
  return glyph_source_.GetCombinedCharacterGroups(text, paint_);
}

std::vector<float> AndroidPlatformCanvasSource::GetTextWidths(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  if (text_options.stroke_width_pixels > 0.0f) {
    ConfigurePaintForStrokeTextOptions(stroke_paint_, text_options,
                                       /*configure_for_glyphs=*/false);
    return stroke_paint_.GetTextWidths(text);
  } else {
    ConfigurePaintForTextOptions(paint_, text_options,
                                 /*configure_for_glyphs=*/false);
    return paint_.GetTextWidths(text);
  }
}

std::vector<ScopedCanvas::GlyphAdvance>
AndroidPlatformCanvasSource::GetTextGlyphs(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  ConfigurePaintForTextOptions(paint_, text_options,
                               /*configure_for_glyphs=*/true);
  return glyph_source_.GetTextGlyphs(text, paint_);
}

void AndroidPlatformCanvasSource::ReleaseTextGlyphs(absl::Span<int> glyph_ids) {
  return glyph_source_.ReleaseTextGlyphs(glyph_ids);
}

FontInfo AndroidPlatformCanvasSource::GetFontInfo(
    const ScopedCanvas::TextOptions& text_options) {
  ConfigurePaintForTextOptions(paint_, text_options,
                               /*configure_for_glyphs=*/false);

  std::unique_ptr<android::Paint::FontMetrics> font_metrics =
      paint_.GetFontMetrics();

  FontInfo font_info;
  font_info.set_ascent(paint_.Ascent());
  font_info.set_descent(paint_.Descent());
  font_info.set_leading(font_metrics->Leading());
  font_info.set_line_spacing(paint_.GetFontSpacing());
  return font_info;
}

std::unique_ptr<ScopedCanvas> AndroidPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size, ScopedCanvas::DrawMode draw_mode) {
  bool did_texture_change = false;

  if (!surface_texture_) {
    // TODO: (broken link) - why do we pass 0 as textureId?
    surface_texture_ =
        std::make_unique<android::SurfaceTexture>(context_, 0, false);
    surface_ = std::make_unique<android::Surface>(context_, *surface_texture_);

    texture_ = view.GetTextureFactory().CreateExternalTexture(
        surface_texture_->WeakReference(), pixel_size);
    did_texture_change = true;
  }

  if (absl::Status status = surface_texture_->SetDefaultBufferSize(pixel_size);
      !status.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to set default buffer size: " << status;
  }
  return std::make_unique<AndroidScopedCanvas>(*this, pixel_size,
                                               did_texture_change);
}

std::unique_ptr<ScopedCanvas> AndroidPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) {
  bool did_texture_change = false;
  if (!surface_texture_) {
    // TODO: (broken link) - why do we pass 0 as textureId?
    surface_texture_ =
        std::make_unique<android::SurfaceTexture>(context_, 0, false);
    surface_ = std::make_unique<android::Surface>(context_, *surface_texture_);

    OwnedTexturePtr texture = view.GetTextureFactory().CreateExternalTexture(
        surface_texture_->WeakReference(), pixel_size);
    did_texture_change = true;

    on_texture_changed_fn(texture.Borrow(loc));

    texture_ = std::move(texture);
  }

  if (absl::Status status = surface_texture_->SetDefaultBufferSize(pixel_size);
      !status.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to set default buffer size: " << status;
  }
  return std::make_unique<AndroidScopedCanvas>(*this, pixel_size,
                                               did_texture_change);
}

Texture* AndroidPlatformCanvasSource::GetTexture() {
  return texture_.operator->();
}

AndroidPlatformCanvasSource::AndroidScopedCanvas::AndroidScopedCanvas(
    AndroidPlatformCanvasSource& source, uint2 pixel_size,
    bool did_texture_change)
    : source_(source),
      picture_(source_.context_),
      canvas_(picture_.BeginRecording(pixel_size)),
      did_texture_change_(did_texture_change) {
  canvas_.Clear();
}

AndroidPlatformCanvasSource::AndroidScopedCanvas::~AndroidScopedCanvas() {
  android::Canvas canvas = source_.use_hardware_rendering_
                               ? source_.surface_->LockHardwareCanvas()
                               : source_.surface_->LockCanvas();
  canvas.DrawPicture(picture_.WeakReference());
  source_.surface_->UnlockCanvasAndPost(canvas);
}

Texture* AndroidPlatformCanvasSource::AndroidScopedCanvas::GetTexture() {
  return source_.GetTexture();
}
bool AndroidPlatformCanvasSource::AndroidScopedCanvas::DidTextureChange()
    const {
  return did_texture_change_;
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawColor(float3 color) {
  canvas_.DrawColor(color);
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawColor(float4 color) {
  canvas_.DrawColor(color);
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawRoundedRect(
    float3 color, float2 corner_radius, const Rect& rect) {
  DrawRoundedRect(float4(color, 1.0f), corner_radius, rect);
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawRoundedRect(
    float4 color, float2 corner_radius, const Rect& rect) {
  source_.paint_.SetColor(color);
  canvas_.DrawRoundRect(rect, corner_radius, source_.paint_);
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawText(
    absl::string_view text, float2 pos, const TextOptions& text_options) {
  ConfigurePaintForTextOptions(source_.paint_, text_options,
                               /*configure_for_glyphs=*/false);

  // Offset based on the horizontal text alignment if there is stroke.
  float horizontal_offset = 0.0f;
  switch (text_options.horizontal_alignment) {
    case TextHorizontalAlignment::kLeftExtent:
      horizontal_offset = (text_options.stroke_width_pixels / 2);
      break;
    case TextHorizontalAlignment::kRightExtent:
      horizontal_offset = (-text_options.stroke_width_pixels / 2);
      break;
    default:
      break;
  }

  float vertical_offset = 0.0f;
  switch (text_options.vertical_alignment) {
    case TextVerticalAlignment::kTopExtent:
    case TextVerticalAlignment::kAtlas: {
      std::unique_ptr<android::Rect> bounds =
          source_.paint_.GetTextBounds(text);
      vertical_offset =
          -bounds->GetTop() + (text_options.stroke_width_pixels / 2);
      break;
    }
    case TextVerticalAlignment::kAscent: {
      vertical_offset = -source_.paint_.Ascent();
      break;
    }
    case TextVerticalAlignment::kCenter: {
      float font_height = -source_.paint_.Ascent() + source_.paint_.Descent();
      float half_font_height = font_height * 0.5f;
      vertical_offset = half_font_height - source_.paint_.Descent();
      break;
    }
    case TextVerticalAlignment::kBaseline: {
      // Do Nothing
      break;
    }
    case TextVerticalAlignment::kDescent: {
      vertical_offset = -source_.paint_.Descent();
      break;
    }
    case TextVerticalAlignment::kBottomExtent: {
      std::unique_ptr<android::Rect> bounds =
          source_.paint_.GetTextBounds(text);
      vertical_offset =
          -bounds->GetBottom() - (text_options.stroke_width_pixels / 2);
      break;
    }
  }

  pos.x += horizontal_offset;
  pos.y += vertical_offset;

  if (text_options.stroke_width_pixels > 0 &&
      text_options.stroke_color != kZero4) {
    ConfigurePaintForStrokeTextOptions(source_.stroke_paint_, text_options,
                                       /*configure_for_glyphs=*/false);
    canvas_.DrawText(text, pos, source_.stroke_paint_);
  }
  canvas_.DrawText(text, pos, source_.paint_);
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawGlyph(
    GlyphId glyph, float2 pos, const TextOptions& text_options) {
  if (text_options.stroke_width_pixels > 0.0f) {
    ConfigurePaintForStrokeTextOptions(source_.stroke_paint_, text_options,
                                       /*configure_for_glyphs=*/true);
  }
  ConfigurePaintForTextOptions(source_.paint_, text_options,
                               /*configure_for_glyphs=*/true);
  source_.glyph_source_.DrawGlyph(
      canvas_, glyph.Get(), pos.x, pos.y, text_options.font_holder,
      text_options.stroke_width_pixels, source_.paint_, source_.stroke_paint_);
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::ClearRect(
    const Rect& rect) {
  source_.paint_.SetColor(kZero4);
  canvas_.DrawRect(rect, source_.paint_);
}

void AndroidPlatformCanvasSource::ForceReset() {
  surface_texture_.reset();
  surface_.reset();
}

}  // namespace imp
