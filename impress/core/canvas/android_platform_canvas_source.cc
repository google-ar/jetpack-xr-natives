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

#include <android/bitmap.h>
#include <jni.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
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
#include "core/common/jni_helpers.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/render/texture_options.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/base_view.h"
#include "core/view/platforms/android/wrappers/bitmap.h"
#include "core/view/platforms/android/wrappers/canvas.h"
#include "core/view/platforms/android/wrappers/paint.h"
#include "core/view/platforms/android/wrappers/rect.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "core/view/platforms/android/wrappers/surface_texture.h"

namespace imp {
namespace {
// Experimentally derived padding to ensure we mark the entire glyph as dirty.
constexpr float kGlyphPadding = 2.0f;
}  // namespace

class SurfaceTextureSurfaceProvider
    : public AndroidPlatformCanvasSource::SurfaceProvider {
 public:
  SurfaceTextureSurfaceProvider(bool use_hardware_rendering)
      : use_hardware_rendering_(use_hardware_rendering) {}

  ~SurfaceTextureSurfaceProvider() override = default;

  void OnPause(Context& context) override;
  void OnResume(Context& context) override;

  bool StartDrawing(BaseView& view, Context& context,
                    uint2 pixel_size) override;

  bool StartDrawing(BaseView& view, Context& context, uint2 pixel_size,
                    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
                    SmallSourceLocation loc) override;

  Texture* GetTexture() override;

  void ForceReset(Context& context) override;

  bool SupportsKeepContents() const override;

  void ApplyDrawCommands(Context& context, android::Canvas& canvas,
                         const absl::Span<Rect>& dirty_rects) override;

  JniUniquePtr<jobject> GetCanvas(Context& context) override;

 private:
  bool use_hardware_rendering_;
  OwnedTexturePtr texture_;
  std::unique_ptr<android::SurfaceTexture> surface_texture_;
  std::unique_ptr<android::Surface> surface_;
};

struct PixelPair {
  uint32_t count;
  uint32_t value;
};

// TODO: Add unit tests for this class.
class BitmapSurfaceProvider
    : public AndroidPlatformCanvasSource::SurfaceProvider {
 public:
  BitmapSurfaceProvider(Context& context);
  ~BitmapSurfaceProvider() override = default;

  void OnPause(Context& context) override;
  void OnResume(Context& context) override;

  bool StartDrawing(BaseView& view, Context& context,
                    uint2 pixel_size) override;

  bool StartDrawing(BaseView& view, Context& context, uint2 pixel_size,
                    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
                    SmallSourceLocation loc) override;

  Texture* GetTexture() override;

  void ForceReset(Context& context) override;

  bool SupportsKeepContents() const override;

  void ApplyDrawCommands(Context& context, android::Canvas& canvas,
                         const absl::Span<Rect>& dirty_rects) override;

  JniUniquePtr<jobject> GetCanvas(Context& context) override;

  void InitializeBitmap(Context& context, uint2 pixel_size);

 private:
  OwnedTexturePtr texture_;
  std::optional<android::Bitmap> bitmap_;
  uint2 bitmap_size_ = uint2{0, 0};
  std::vector<PixelPair> preserved_pixels_;

  // Cache the Canvas class and constructor for later use.
  jclass canvas_class_;
  jmethodID canvas_ctor_;
};

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
    bool force_individual_glyph_source_instances,
    bool use_bitmap_surface_provider)
    : context_(context),
      paint_(context),
      stroke_paint_(context),
      clear_paint_(context),
      glyph_source_(context, glyph_method, glyph_cache_size_bytes,
                    force_individual_glyph_source_instances) {
  paint_.SetAntiAlias(true);
  stroke_paint_.SetAntiAlias(true);
  stroke_paint_.SetStyle(android::Paint::Style::kStroke);
  clear_paint_.SetColor(kZero4);
  clear_paint_.SetXfermodeClear(true);

  if (use_bitmap_surface_provider) {
    surface_provider_ = std::make_unique<BitmapSurfaceProvider>(context);
  } else {
    surface_provider_ =
        std::make_unique<SurfaceTextureSurfaceProvider>(use_hardware_rendering);
  }
}

void AndroidPlatformCanvasSource::OnResume() {
  surface_provider_->OnResume(context_);
}

void AndroidPlatformCanvasSource::OnPause() {
  surface_provider_->OnPause(context_);
}

bool AndroidPlatformCanvasSource::IsFeatureSupported(
    ScopedCanvas::Feature feature) {
  switch (feature) {
    case ScopedCanvas::Feature::kGlyphs:
      return glyph_source_.IsAvailable();
    case ScopedCanvas::Feature::kKeepContents:
      return surface_provider_->SupportsKeepContents();
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
  bool did_texture_change =
      surface_provider_->StartDrawing(view, context_, pixel_size);

  return std::make_unique<AndroidScopedCanvas>(*this, pixel_size,
                                               did_texture_change, draw_mode);
}

std::unique_ptr<ScopedCanvas> AndroidPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) {
  bool did_texture_change = surface_provider_->StartDrawing(
      view, context_, pixel_size, std::move(on_texture_changed_fn), loc);

  return std::make_unique<AndroidScopedCanvas>(*this, pixel_size,
                                               did_texture_change, draw_mode);
}

Texture* AndroidPlatformCanvasSource::GetTexture() {
  return surface_provider_->GetTexture();
}

AndroidPlatformCanvasSource::AndroidScopedCanvas::AndroidScopedCanvas(
    AndroidPlatformCanvasSource& source, uint2 pixel_size,
    bool did_texture_change, ScopedCanvas::DrawMode draw_mode)
    : source_(source),
      canvas_(source.context_.GetJniEnv(),
              source.surface_provider_->GetCanvas(source.context_)),
      paint_(source.context_),
      stroke_paint_(source.context_),
      did_texture_change_(did_texture_change),
      pixel_size_(pixel_size) {
  paint_.SetAntiAlias(true);
  stroke_paint_.SetAntiAlias(true);
  stroke_paint_.SetStyle(android::Paint::Style::kStroke);
  if (did_texture_change || draw_mode == ScopedCanvas::DrawMode::kClear) {
    canvas_.Clear();
    AddDirtyRect(Rect{float2(pixel_size_.x * 0.5f, pixel_size_.y * 0.5f),
                      float2(pixel_size_.x * 0.5f, pixel_size_.y * 0.5f)});
  }
}

AndroidPlatformCanvasSource::AndroidScopedCanvas::~AndroidScopedCanvas() {
  source_.surface_provider_->ApplyDrawCommands(source_.context_, canvas_,
                                               absl::MakeSpan(dirty_rects_));
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::AddDirtyRect(
    const Rect& rect) {
  dirty_rects_.push_back(rect);
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
  AddDirtyRect(Rect{float2(pixel_size_.x * 0.5f, pixel_size_.y * 0.5f),
                    float2(pixel_size_.x * 0.5f, pixel_size_.y * 0.5f)});
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawColor(float4 color) {
  canvas_.DrawColor(color);
  AddDirtyRect(Rect{float2(pixel_size_.x * 0.5f, pixel_size_.y * 0.5f),
                    float2(pixel_size_.x * 0.5f, pixel_size_.y * 0.5f)});
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawRoundedRect(
    float3 color, float2 corner_radius, const Rect& rect) {
  DrawRoundedRect(float4(color, 1.0f), corner_radius, rect);
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawRoundedRect(
    float4 color, float2 corner_radius, const Rect& rect) {
  paint_.SetColor(color);
  canvas_.DrawRoundRect(rect, corner_radius, paint_);
  AddDirtyRect(rect);
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawText(
    absl::string_view text, float2 pos, const TextOptions& text_options) {
  ConfigurePaintForTextOptions(paint_, text_options,
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
      std::unique_ptr<android::Rect> bounds = paint_.GetTextBounds(text);
      vertical_offset =
          -bounds->GetTop() + (text_options.stroke_width_pixels / 2);
      break;
    }
    case TextVerticalAlignment::kAscent: {
      vertical_offset = -paint_.Ascent();
      break;
    }
    case TextVerticalAlignment::kCenter: {
      float font_height = -paint_.Ascent() + paint_.Descent();
      float half_font_height = font_height * 0.5f;
      vertical_offset = half_font_height - paint_.Descent();
      break;
    }
    case TextVerticalAlignment::kBaseline: {
      // Do Nothing
      break;
    }
    case TextVerticalAlignment::kDescent: {
      vertical_offset = -paint_.Descent();
      break;
    }
    case TextVerticalAlignment::kBottomExtent: {
      std::unique_ptr<android::Rect> bounds = paint_.GetTextBounds(text);
      vertical_offset =
          -bounds->GetBottom() - (text_options.stroke_width_pixels / 2);
      break;
    }
  }

  pos.x += horizontal_offset;
  pos.y += vertical_offset;

  if (text_options.stroke_width_pixels > 0 &&
      text_options.stroke_color != kZero4) {
    ConfigurePaintForStrokeTextOptions(stroke_paint_, text_options,
                                       /*configure_for_glyphs=*/false);
    canvas_.DrawText(text, pos, stroke_paint_);
  }
  canvas_.DrawText(text, pos, paint_);

  std::unique_ptr<android::Rect> bounds = paint_.GetTextBounds(text);
  float stroke = text_options.stroke_width_pixels + kGlyphPadding;
  float2 min_p = {pos.x + bounds->GetLeft() - stroke,
                  pos.y + bounds->GetTop() - stroke};
  float2 max_p = {pos.x + bounds->GetRight() + stroke,
                  pos.y + bounds->GetBottom() + stroke};
  AddDirtyRect(Rect{(min_p + max_p) * 0.5f, (max_p - min_p) * 0.5f});
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::DrawGlyph(
    GlyphId glyph, float2 pos, const TextOptions& text_options,
    const TextMetrics* pre_cached_metrics) {
  if (text_options.stroke_width_pixels > 0.0f) {
    ConfigurePaintForStrokeTextOptions(stroke_paint_, text_options,
                                       /*configure_for_glyphs=*/true);
  }
  ConfigurePaintForTextOptions(paint_, text_options,
                               /*configure_for_glyphs=*/true);
  source_.glyph_source_.DrawGlyph(
      canvas_, glyph.Get(), pos.x, pos.y, text_options.font_holder,
      text_options.stroke_width_pixels, paint_, stroke_paint_);

  TextMetrics metrics_storage;
  if (!pre_cached_metrics) {
    metrics_storage = source_.GetGlyphMetrics(glyph, text_options);
    pre_cached_metrics = &metrics_storage;
  }
  float stroke = text_options.stroke_width_pixels + kGlyphPadding;
  float min_x = pos.x + pre_cached_metrics->origin_x() - stroke;
  float min_y = pos.y + pre_cached_metrics->origin_y() - stroke;
  float max_x = min_x + pre_cached_metrics->size_x() + 2.f * stroke;
  float max_y = min_y + pre_cached_metrics->font_size_y() + 2.f * stroke;
  AddDirtyRect(Rect{float2((min_x + max_x) * 0.5f, (min_y + max_y) * 0.5f),
                    float2((max_x - min_x) * 0.5f, (max_y - min_y) * 0.5f)});
}

void AndroidPlatformCanvasSource::AndroidScopedCanvas::ClearRect(
    const Rect& rect) {
  canvas_.DrawRect(rect, source_.clear_paint_);
  AddDirtyRect(rect);
}

void AndroidPlatformCanvasSource::ForceReset() {
  surface_provider_->ForceReset(context_);
}

void SurfaceTextureSurfaceProvider::OnPause(Context& context) {}
void SurfaceTextureSurfaceProvider::OnResume(Context& context) {}

bool SurfaceTextureSurfaceProvider::StartDrawing(BaseView& view,
                                                 Context& context,
                                                 uint2 pixel_size) {
  bool did_texture_change = false;
  if (!surface_texture_) {
    // TODO: (broken link) - why do we pass 0 as textureId?
    surface_texture_ =
        std::make_unique<android::SurfaceTexture>(context, 0, false);
    surface_ = std::make_unique<android::Surface>(context, *surface_texture_);

    texture_ = view.GetTextureFactory().CreateExternalTexture(
        surface_texture_->WeakReference(), pixel_size);
    did_texture_change = true;
  }

  if (absl::Status status = surface_texture_->SetDefaultBufferSize(pixel_size);
      !status.ok()) {
    IMP_LOG(imp::FATAL) << "Failed to set default buffer size: " << status;
  }

  return did_texture_change;
}

bool SurfaceTextureSurfaceProvider::StartDrawing(
    BaseView& view, Context& context, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    SmallSourceLocation loc) {
  bool did_texture_change = false;
  if (!surface_texture_) {
    // TODO: (broken link) - why do we pass 0 as textureId?
    surface_texture_ =
        std::make_unique<android::SurfaceTexture>(context, 0, false);
    surface_ = std::make_unique<android::Surface>(context, *surface_texture_);

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
  return did_texture_change;
}

Texture* SurfaceTextureSurfaceProvider::GetTexture() {
  return texture_.operator->();
}

void SurfaceTextureSurfaceProvider::ForceReset(Context& context) {
  surface_texture_.reset();
  surface_.reset();
}

bool SurfaceTextureSurfaceProvider::SupportsKeepContents() const {
  return false;
}

void SurfaceTextureSurfaceProvider::ApplyDrawCommands(
    Context& context, android::Canvas& canvas,
    const absl::Span<Rect>& dirty_rects) {
  surface_->UnlockCanvasAndPost(canvas);
}

JniUniquePtr<jobject> SurfaceTextureSurfaceProvider::GetCanvas(
    Context& context) {
  android::Canvas canvas = use_hardware_rendering_
                               ? surface_->LockHardwareCanvas()
                               : surface_->LockCanvas();
  return WrapJni(context.GetJniEnv(), canvas.Reference());
}

void BitmapSurfaceProvider::OnPause(Context& context) {
  if (!bitmap_) return;

  absl::StatusOr<AndroidBitmapInfo> info = bitmap_->GetBitmapInfo();
  if (!info.ok()) return;

  absl::StatusOr<void*> pixels = bitmap_->LockPixels();
  if (!pixels.ok()) return;

  const uint32_t* ptr = static_cast<const uint32_t*>(pixels.value());
  size_t num_pixels = (info->stride * info->height) / sizeof(uint32_t);

  if (num_pixels > 0) {
    uint32_t current_pixel = ptr[0];
    uint32_t count = 1;
    for (size_t i = 1; i < num_pixels; ++i) {
      if (ptr[i] == current_pixel) {
        count++;
      } else {
        preserved_pixels_.push_back(PixelPair{count, current_pixel});
        current_pixel = ptr[i];
        count = 1;
      }
    }
    preserved_pixels_.push_back(PixelPair{count, current_pixel});
  }

  bitmap_->UnlockPixels().IgnoreError();
}

void BitmapSurfaceProvider::OnResume(Context& context) {
  if (!bitmap_) return;

  if (preserved_pixels_.empty()) return;

  absl::StatusOr<AndroidBitmapInfo> info = bitmap_->GetBitmapInfo();
  if (!info.ok()) return;

  absl::StatusOr<void*> pixels = bitmap_->LockPixels();
  if (!pixels.ok()) return;

  uint32_t* ptr = static_cast<uint32_t*>(pixels.value());
  size_t num_pixels = (info->stride * info->height) / sizeof(uint32_t);
  size_t idx = 0;

  for (const auto& p : preserved_pixels_) {
    if (idx + p.count <= num_pixels) {
      std::fill_n(ptr + idx, p.count, p.value);
      idx += p.count;
    }
  }
  preserved_pixels_.clear();

  bitmap_->UnlockPixels().IgnoreError();
}

BitmapSurfaceProvider::BitmapSurfaceProvider(Context& context) {
  JNIEnv* env = context.GetJniEnv();
  jclass local_class = env->FindClass("android/graphics/Canvas");
  canvas_class_ = (jclass)env->NewGlobalRef(local_class);
  canvas_ctor_ =
      env->GetMethodID(canvas_class_, "<init>", "(Landroid/graphics/Bitmap;)V");
}

void BitmapSurfaceProvider::InitializeBitmap(Context& context,
                                             uint2 pixel_size) {
  JNIEnv* env = context.GetJniEnv();

  JniUniquePtr<jclass> bitmap_class = FindClass(env, "android/graphics/Bitmap");
  jmethodID create_bitmap = env->GetStaticMethodID(
      bitmap_class.get(), "createBitmap",
      "(IILandroid/graphics/Bitmap$Config;)Landroid/graphics/Bitmap;");
  JniUniquePtr<jclass> config_class =
      FindClass(env, "android/graphics/Bitmap$Config");
  jfieldID argb8888_id = env->GetStaticFieldID(
      config_class.get(), "ARGB_8888", "Landroid/graphics/Bitmap$Config;");
  jobject argb8888 = env->GetStaticObjectField(config_class.get(), argb8888_id);

  bitmap_.emplace(
      env, env->CallStaticObjectMethod(bitmap_class.get(), create_bitmap,
                                       pixel_size.x, pixel_size.y, argb8888));
  bitmap_size_ = pixel_size;
}

bool BitmapSurfaceProvider::StartDrawing(BaseView& view, Context& context,
                                         uint2 pixel_size) {
  bool did_texture_change = false;

  if (!bitmap_ || bitmap_size_ != pixel_size) {
    if (bitmap_) {
      bitmap_.reset();
    }

    InitializeBitmap(context, pixel_size);

    texture_ = view.GetTextureFactory().CreateTexture(
        TextureFactory::TextureCreationSettings{
            .width = static_cast<uint32_t>(pixel_size.x),
            .height = static_cast<uint32_t>(pixel_size.y),
            .format = TextureFactory::Format::RGBA8,
            .usage = TextureFactory::Usage::DEFAULT,
            .sampler_options = TextureSamplerOptions{
                .wrap_mode = TextureFactory::WrapMode::CLAMP_TO_EDGE,
                .mag_filter = TextureFactory::MagFilter::LINEAR,
                .min_filter = TextureFactory::MinFilter::LINEAR}});
    did_texture_change = true;
  }

  return did_texture_change;
}

bool BitmapSurfaceProvider::StartDrawing(
    BaseView& view, Context& context, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    SmallSourceLocation loc) {
  bool did_texture_change = false;

  if (!bitmap_ || bitmap_size_ != pixel_size) {
    if (bitmap_) {
      bitmap_.reset();
    }

    InitializeBitmap(context, pixel_size);

    OwnedTexturePtr texture = view.GetTextureFactory().CreateTexture(
        TextureFactory::TextureCreationSettings{
            .width = static_cast<uint32_t>(pixel_size.x),
            .height = static_cast<uint32_t>(pixel_size.y),
            .format = TextureFactory::Format::RGBA8,
            .usage = TextureFactory::Usage::DEFAULT,
            .sampler_options = TextureSamplerOptions{
                .wrap_mode = TextureFactory::WrapMode::CLAMP_TO_EDGE,
                .mag_filter = TextureFactory::MagFilter::LINEAR,
                .min_filter = TextureFactory::MinFilter::LINEAR}});
    did_texture_change = true;

    on_texture_changed_fn(texture.Borrow(loc));

    texture_ = std::move(texture);
  }
  return did_texture_change;
}

Texture* BitmapSurfaceProvider::GetTexture() { return texture_.operator->(); }

void BitmapSurfaceProvider::ForceReset(Context& context) {
  if (bitmap_.has_value()) {
    bitmap_.reset();
    bitmap_size_ = {0, 0};
  }
}
bool BitmapSurfaceProvider::SupportsKeepContents() const { return true; }

void BitmapSurfaceProvider::ApplyDrawCommands(
    Context& context, android::Canvas& canvas,
    const absl::Span<Rect>& dirty_rects) {
  if (!bitmap_) return;

  absl::StatusOr<AndroidBitmapInfo> info = bitmap_->GetBitmapInfo();
  if (!info.ok()) return;

  absl::StatusOr<void*> pixels = bitmap_->LockPixels();
  if (!pixels.ok()) return;

  struct IntRect {
    int2 min;
    int2 max;
  };
  int2 bounds_min = {0, 0};
  int2 bounds_max = {static_cast<int>(info->width),
                     static_cast<int>(info->height)};

  std::vector<IntRect> active_rects;
  active_rects.reserve(dirty_rects.size());
  float2 half_size = float2(info->width * 0.5f, info->height * 0.5f);
  if (dirty_rects.empty()) {
    // If there are no dirty rects, we just copy the entire bitmap.
    active_rects.push_back(IntRect{bounds_min, bounds_max});
  } else {
    auto check_proximity = [](const IntRect& a, const IntRect& b,
                              int threshold) {
      return !(a.max.x + threshold < b.min.x || a.min.x - threshold > b.max.x ||
               a.max.y + threshold < b.min.y || a.min.y - threshold > b.max.y);
    };
    auto union_rect = [](const IntRect& a, const IntRect& b) {
      int2 min_val = {std::min(a.min.x, b.min.x), std::min(a.min.y, b.min.y)};
      int2 max_val = {std::max(a.max.x, b.max.x), std::max(a.max.y, b.max.y)};
      return IntRect{min_val, max_val};
    };

    for (const auto& r : dirty_rects) {
      IntRect current = {int2(floor(r.GetMin())), int2(ceil(r.GetMax()))};
      bool merged = true;
      while (merged) {
        merged = false;
        for (auto it = active_rects.begin(); it != active_rects.end(); ++it) {
          // Use a 16-pixel threshold to merge disjoint but nearby rects
          if (check_proximity(current, *it, 16)) {
            current = union_rect(current, *it);
            active_rects.erase(it);
            merged = true;
            break;
          }
        }
      }
      active_rects.push_back(current);
    }
  }

  const uint8_t* src = static_cast<const uint8_t*>(pixels.value());
  uint8_t pixel_size = 4;

  for (const auto& rect : active_rects) {
    int2 rect_min = clamp(rect.min, bounds_min, bounds_max);
    int2 rect_max = clamp(rect.max, bounds_min, bounds_max);

    if (rect_min.x >= rect_max.x || rect_min.y >= rect_max.y) continue;

    size_t src_stride = info->stride;
    int2 delta = rect_max - rect_min;

    size_t buffer_stride = delta.x * pixel_size;
    size_t buffer_size = buffer_stride * delta.y;

    void* buffer = malloc(buffer_size);
    uint8_t* dst = static_cast<uint8_t*>(buffer);

    for (int y = 0; y < delta.y; ++y) {
      size_t src_offset =
          (rect_min.y + y) * src_stride + rect_min.x * pixel_size;
      std::memcpy(dst + y * buffer_stride, src + src_offset, buffer_stride);
    }

    auto descriptor = filament::Texture::PixelBufferDescriptor(
        buffer, buffer_size, filament::backend::PixelDataFormat::RGBA,
        filament::backend::PixelDataType::UBYTE, pixel_size, 0, 0, delta.x,
        [](void* buffer, size_t size, void* user) { free(buffer); });

    texture_->GetTexture()->setImage(*BaseView::GetSharedEngine(), 0,
                                     rect_min.x, rect_min.y, delta.x, delta.y,
                                     std::move(descriptor));
  }

  bitmap_->UnlockPixels().IgnoreError();
}

JniUniquePtr<jobject> BitmapSurfaceProvider::GetCanvas(Context& context) {
  JNIEnv* env = context.GetJniEnv();
  return WrapJni(env, env->NewObject(canvas_class_, canvas_ctor_,
                                     bitmap_->WeakReference()));
}

}  // namespace imp
