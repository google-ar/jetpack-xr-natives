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

#include <memory>
#include <numeric>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/canvas/android_glyph_source.h"
#include "core/canvas/constants.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
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

void ConfigurePaintForTextOptions(
    android::Paint& paint, const ScopedCanvas::TextOptions& text_options) {
  paint.SetTextSize(text_options.size_pixels);
  paint.SetLetterSpacing(text_options.text_tracking);
  paint.SetTextAlign(
      TextAlignmentToPaintAlign(text_options.horizontal_alignment));
  paint.SetColor(text_options.color);
  paint.SetAntiAlias(true);
  if (text_options.font_holder) {
    paint.SetTypeface(
        static_cast<jobject>(text_options.font_holder->GetPlatformFont()));
  } else {
    paint.SetTypeface(jobject{});
  }
}

void ConfigurePaintForStrokeTextOptions(
    android::Paint& paint, const ScopedCanvas::TextOptions& text_options) {
  ConfigurePaintForTextOptions(paint, text_options);
  paint.SetStyle(android::Paint::Style::kStroke);
  paint.SetStrokeWidth(text_options.stroke_width_pixels);
  paint.SetColor(text_options.stroke_color);
  paint.SetAntiAlias(true);
}

AndroidPlatformCanvasSource::AndroidPlatformCanvasSource(Context context)
    : context_(context),
      surface_texture_(context_, false),
      surface_(context_, surface_texture_),
      paint_(context_),
      stroke_paint_(context_),
      glyph_source_(context_, AndroidGlyphSource::Method::kAuto) {}

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

ScopedCanvas::TextMetrics AndroidPlatformCanvasSource::GetTextMetrics(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  ConfigurePaintForTextOptions(paint_, text_options);
  // Even when Paint::SetStrokeWidth is set, Android's paint type ignores stroke
  // when calling measure text. Therefore, we need to manually account for it.
  std::unique_ptr<android::Rect> bounds = paint_.GetTextBounds(text);
  std::vector<float> text_widths = GetTextWidths(text, text_options);
  float typographical_width =
      std::accumulate(text_widths.begin(), text_widths.end(), 0.0f);
  return ScopedCanvas::TextMetrics{
      // Origin is for the fill not the stroke.
      .origin = float2{bounds->GetLeft(), -bounds->GetBottom()},
      // Size includes the stroke - 1/2 the stroke on either side as it
      // straddles the fill of the font half in and half out.
      .size = float2{bounds->GetWidth() + text_options.stroke_width_pixels,
                     bounds->GetHeight() + text_options.stroke_width_pixels},
      .typographical_width = typographical_width,
      // TODO: Return proper metrics here
      .font_origin_y = static_cast<float>(-bounds->GetBottom()),
      .font_size_y = bounds->GetHeight() + text_options.stroke_width_pixels,
  };
}

ScopedCanvas::TextMetrics AndroidPlatformCanvasSource::GetGlyphMetrics(
    ScopedCanvas::GlyphId glyph,
    const ScopedCanvas::TextOptions& text_options) {
  return glyph_source_.GetGlyphMetrics(
      glyph, text_options.font_holder, text_options.size_pixels,
      text_options.stroke_width_pixels, text_options.text_tracking);
}

// TODO: This is needed to properly support path text in Android.
std::vector<ScopedCanvas::GlyphGroup>
AndroidPlatformCanvasSource::GetCombinedCharacterGroups(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return {};
}

std::vector<float> AndroidPlatformCanvasSource::GetTextWidths(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  if (text_options.stroke_width_pixels > 0.0f) {
    ConfigurePaintForStrokeTextOptions(stroke_paint_, text_options);
    return stroke_paint_.GetTextWidths(text);
  } else {
    ConfigurePaintForTextOptions(paint_, text_options);
    return paint_.GetTextWidths(text);
  }
}

std::vector<ScopedCanvas::GlyphAdvance>
AndroidPlatformCanvasSource::GetTextGlyphs(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return glyph_source_.GetTextGlyphs(text, text_options.size_pixels,
                                     text_options.text_tracking);
}

ScopedCanvas::FontInfo AndroidPlatformCanvasSource::GetFontInfo(
    const ScopedCanvas::TextOptions& text_options) {
  ConfigurePaintForTextOptions(paint_, text_options);

  std::unique_ptr<android::Paint::FontMetrics> font_metrics =
      paint_.GetFontMetrics();

  return ScopedCanvas::FontInfo{.ascent = paint_.Ascent(),
                                .descent = paint_.Descent(),
                                .leading = font_metrics->Leading(),
                                .line_spacing = paint_.GetFontSpacing()};
}

std::unique_ptr<ScopedCanvas> AndroidPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size, ScopedCanvas::DrawMode draw_mode) {
  bool did_texture_change = false;
  if (!texture_) {
    texture_ = view.GetTextureFactory().CreateExternalTexture(
        surface_texture_.WeakReference(), pixel_size);
    did_texture_change = true;
  }

  if (absl::Status status = surface_texture_.SetDefaultBufferSize(pixel_size);
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
  if (!texture_) {
    texture_ = view.GetTextureFactory().CreateExternalTexture(
        surface_texture_.WeakReference(), pixel_size);
    did_texture_change = true;

    // On Android the texture is only created once, so no need to give an
    // opportunity for the caller to clear references to the old texture.
    on_texture_changed_fn(texture_.Borrow(loc));
  }

  if (absl::Status status = surface_texture_.SetDefaultBufferSize(pixel_size);
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
  android::Canvas canvas = source_.surface_.LockHardwareCanvas();
  canvas.DrawPicture(picture_.WeakReference());
  source_.surface_.UnlockCanvasAndPost(canvas);
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
  ConfigurePaintForTextOptions(source_.paint_, text_options);

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
    ConfigurePaintForStrokeTextOptions(source_.stroke_paint_, text_options);
    canvas_.DrawText(text, pos, source_.stroke_paint_);
  }
  canvas_.DrawText(text, pos, source_.paint_);
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawGlyph(
    GlyphId glyph, float2 pos, const TextOptions& text_options) {
  source_.glyph_source_.DrawGlyph(
      canvas_, glyph, pos.x, pos.y, text_options.font_holder,
      text_options.size_pixels, text_options.stroke_width_pixels,
      text_options.color, text_options.stroke_color,
      text_options.text_tracking);
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::ClearRect(
    const Rect& rect) {
  source_.paint_.SetColor(kZero4);
  canvas_.DrawRect(rect, source_.paint_);
}

}  // namespace imp
