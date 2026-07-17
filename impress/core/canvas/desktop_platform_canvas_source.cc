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

#include "core/canvas/desktop_platform_canvas_source.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/canvas/constants.h"
#include "core/canvas/fonts/desktop_font_holder.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/small_source_location.h"
#include "core/config.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/render/texture_factory.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/base_view.h"
#include "third_party/skia/HEAD/include/core/SkAlphaType.h"
#include "third_party/skia/HEAD/include/core/SkColor.h"
#include "third_party/skia/HEAD/include/core/SkColorType.h"
#include "third_party/skia/HEAD/include/core/SkFont.h"
#include "third_party/skia/HEAD/include/core/SkFontMetrics.h"
#include "third_party/skia/HEAD/include/core/SkFontStyle.h"
#include "third_party/skia/HEAD/include/core/SkImageInfo.h"
#include "third_party/skia/HEAD/include/core/SkPaint.h"
#include "third_party/skia/HEAD/include/core/SkRefCnt.h"
#include "third_party/skia/HEAD/include/core/SkScalar.h"
#include "third_party/skia/HEAD/include/core/SkString.h"
#include "third_party/skia/HEAD/modules/skparagraph/include/FontCollection.h"
#include "third_party/skia/HEAD/modules/skparagraph/include/Metrics.h"
#include "third_party/skia/HEAD/modules/skparagraph/include/Paragraph.h"
#include "third_party/skia/HEAD/modules/skparagraph/include/ParagraphBuilder.h"
#include "third_party/skia/HEAD/modules/skparagraph/include/ParagraphStyle.h"
#include "third_party/skia/HEAD/modules/skunicode/include/SkUnicode_icu.h"
#include "util/utf8/public/unicodetext.h"
#if IMP_PLATFORM(LINUX)
#include "absl/debugging/leak_check.h"
#endif

namespace imp {

namespace {
using skia::textlayout::LineMetrics;
using skia::textlayout::Paragraph;
using skia::textlayout::ParagraphBuilder;
using skia::textlayout::ParagraphStyle;

U8CPU ColorComponentRatioToByte(float color_component_ratio) {
  return round(color_component_ratio * 255.0f);
}

SkColor ToSkColor(float4 color) {
  return SkColorSetARGB(
      ColorComponentRatioToByte(color.a), ColorComponentRatioToByte(color.r),
      ColorComponentRatioToByte(color.g), ColorComponentRatioToByte(color.b));
}
}  // namespace

bool DesktopPlatformCanvasSource::IsFeatureSupported(
    ScopedCanvas::Feature feature) {
  switch (feature) {
    case ScopedCanvas::Feature::kGlyphs:
      return false;
    case ScopedCanvas::Feature::kKeepContents:
      return false;
  }
}

Future<absl::Status> DesktopPlatformCanvasSource::PrepareFont(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return Future<absl::Status>(absl::OkStatus());
}

TextMetrics DesktopPlatformCanvasSource::GetTextMetrics(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  std::unique_ptr<Paragraph> paragraph =
      CreateParagraph(text, text_options, /*draw_stroke_only=*/false);
  LineMetrics metrics;
  paragraph->getLineMetricsAt(0, &metrics);
  // The stroke straddles the font, half in and half out.
  float stroke_padding = text_options.stroke_width_pixels;
  TextMetrics text_metrics;
  // Origin does not include the stroke width.
  text_metrics.set_origin_x(0.0f);
  text_metrics.set_origin_y(-metrics.fDescent);
  // Size does include the stroke width.
  text_metrics.set_size_x(metrics.fWidth + stroke_padding);
  text_metrics.set_size_y(metrics.fHeight + stroke_padding);
  text_metrics.set_typographical_width(0.0f);
  // TODO: Return proper metrics here
  // Font origin does not include the stroke width.
  text_metrics.set_font_origin_y(static_cast<float>(-metrics.fDescent));
  text_metrics.set_font_size_y(
      static_cast<float>(metrics.fHeight + stroke_padding));
  return text_metrics;
}

TextMetrics DesktopPlatformCanvasSource::GetGlyphMetrics(
    ScopedCanvas::GlyphId glyph,
    const ScopedCanvas::TextOptions& text_options) {
  IMP_LOG(imp::FATAL) << "CanvasSource::GetGlyphMetrics is unavailable on Desktop.";
  return {};
}

// TODO: Either implement this for desktop if glyph support is
// added, or find another way to support combining characters.
std::vector<ScopedCanvas::GlyphGroup>
DesktopPlatformCanvasSource::GetCombinedCharacterGroups(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return {};
}

std::vector<float> DesktopPlatformCanvasSource::GetTextWidths(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  /**
   * If the original text was utf16 but we are passed a substring that can be
   * represented in ASCII, we can wrap it in a string to force the characters to
   * be 1 byte each.
   */
  std::unique_ptr<Paragraph> paragraph = CreateParagraph(
      std::string(text), text_options, /*draw_stroke_only=*/false);
  skia::textlayout::Paragraph::GlyphClusterInfo cluster_info;
  size_t i = 0;
  UnicodeText unicode_text;
  unicode_text.PointToUTF8(text);
  auto itr = unicode_text.begin();
  /**
   * getGlyphClusterAt will merge any ligatures together. For example, with
   * "soufflé", "ffl" get merged into a single glyph cluster, meaning we would
   * have 5 text widths instead of the 7 that the font atlas is expecting. To
   * account for this, we need to add 0 widths to the vector, but we determine
   * if characters were merged by comparing them to the UnicodeText iterator. We
   * cannot simply compare them to the characters in the text_view because é has
   * a cluster text range of 2 which the text_view does not reveal. Thus, we
   * should only append spare 0's when the character matches the UnicodeText
   * iterator.
   */
  std::vector<float> result;
  while (i < text.size() && itr != unicode_text.end()) {
    if (paragraph->getGlyphClusterAt(i, &cluster_info)) {
      result.push_back(cluster_info.fBounds.width());
      ++itr;
      while (++i < cluster_info.fClusterTextRange.end && i < text.size()) {
        if (itr != unicode_text.end() && text[i] == *(itr.utf8_data())) {
          ++itr;
          result.push_back(0);
        }
      }
    } else {
      ++i;
    }
  }
  return result;
}

std::vector<ScopedCanvas::GlyphAdvance>
DesktopPlatformCanvasSource::GetTextGlyphs(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  IMP_LOG(imp::FATAL) << "CanvasSource::GetTextGlyphs is unavailable on Desktop.";
  return {};
}

FontInfo DesktopPlatformCanvasSource::GetFontInfo(
    const ScopedCanvas::TextOptions& text_options) {
  std::unique_ptr<Paragraph> paragraph =
      CreateParagraph(" ", text_options, /*draw_stroke_only=*/false);
  SkFont font = paragraph->getFontAt(0);
  SkFontMetrics font_metrics;
  font.getMetrics(&font_metrics);

  FontInfo font_info;
  font_info.set_ascent(font_metrics.fAscent);
  font_info.set_descent(font_metrics.fDescent);
  font_info.set_leading(font_metrics.fLeading);
  font_info.set_line_spacing(-font_metrics.fAscent + font_metrics.fDescent);
  return font_info;
}

void DesktopPlatformCanvasSource::ReleaseTextGlyphs(absl::Span<int> glyph_ids) {
  // No-op on desktop.
}

std::unique_ptr<ScopedCanvas> DesktopPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size, ScopedCanvas::DrawMode draw_mode) {
  bool did_texture_change = false;
  if (!texture_ || pixel_size_ != pixel_size) {
    pixel_size_ = pixel_size;
#if IMP_RUNTIME(DEV)
    texture_ = view.GetTextureFactory().CreateTexture(
        imp::TextureFactory::TextureCreationSettings{
            .width = pixel_size_.x,
            .height = pixel_size_.y,
            .format = filament::Texture::InternalFormat::RGBA8,
            .usage = filament::Texture::Usage::COLOR_ATTACHMENT |
                     filament::Texture::Usage::BLIT_SRC |
                     filament::Texture::Usage::DEFAULT,
        });
#else
    texture_ = view.GetTextureFactory().CreateTexture(
        imp::TextureFactory::TextureCreationSettings{
            .width = pixel_size_.x,
            .height = pixel_size_.y,
            .format = filament::Texture::InternalFormat::RGBA8,
        });
#endif
    did_texture_change = true;
  }

  return std::make_unique<DesktopScopedCanvas>(*this, pixel_size_,
                                               did_texture_change);
}

std::unique_ptr<ScopedCanvas> DesktopPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) {
  bool did_texture_change = false;
  if (!texture_ || pixel_size_ != pixel_size) {
    pixel_size_ = pixel_size;

    // Don't destroy until after on_texture_changed_fn is called so that the
    // caller has the opportunity to clear references to the old texture.
    OwnedTexturePtr old_texture = std::move(texture_);

#if IMP_RUNTIME(DEV)
    texture_ = view.GetTextureFactory().CreateTexture(
        imp::TextureFactory::TextureCreationSettings{
            .width = pixel_size_.x,
            .height = pixel_size_.y,
            .format = filament::Texture::InternalFormat::RGBA8,
            .usage = filament::Texture::Usage::COLOR_ATTACHMENT |
                     filament::Texture::Usage::DEFAULT,
        });
#else
    texture_ = view.GetTextureFactory().CreateTexture(
        imp::TextureFactory::TextureCreationSettings{
            .width = pixel_size_.x,
            .height = pixel_size_.y,
            .format = filament::Texture::InternalFormat::RGBA8,
        });
#endif
    did_texture_change = true;

    on_texture_changed_fn(texture_.Borrow(loc));
  }

  return std::make_unique<DesktopScopedCanvas>(*this, pixel_size_,
                                               did_texture_change);
}

Texture* DesktopPlatformCanvasSource::GetTexture() {
  return texture_.operator->();
}

sk_sp<FontCollection>
DesktopPlatformCanvasSource::FontCollectionFromTextOptions(
    const ScopedCanvas::TextOptions& text_options) {
  if (text_options.font_holder) {
    return sk_ref_sp<FontCollection>(static_cast<FontCollection*>(
        text_options.font_holder->GetPlatformFont()));
  }
  return sk_ref_sp<FontCollection>(
      static_cast<FontCollection*>(font_holder_.GetPlatformFont()));
}

skia::textlayout::TextStyle DesktopPlatformCanvasSource::CreateTextStyle(
    const ScopedCanvas::TextOptions& text_options, bool draw_stroke_only) {
  skia::textlayout::TextStyle text_style;
  text_style.setFontSize(text_options.size_pixels);
  if (text_options.font_holder) {
    text_style.setFontFamilies(
        {SkString(text_options.font_holder->GetFontName())});
  } else {
    text_style.setFontFamilies({SkString(font_holder_.GetFontName())});
  }
  SkPaint paint;
  paint.setAntiAlias(true);
  if (draw_stroke_only) {
    paint.setColor(ToSkColor(text_options.stroke_color));
    paint.setStyle(SkPaint::Style::kStroke_Style);
    paint.setStrokeWidth(text_options.stroke_width_pixels);
  } else {
    paint.setColor(ToSkColor(text_options.color));
    paint.setStyle(SkPaint::Style::kFill_Style);
  }
  text_style.setForegroundPaint(paint);
  text_style.setLetterSpacing(text_options.text_tracking *
                              text_options.size_pixels);
  return text_style;
}

std::unique_ptr<skia::textlayout::Paragraph>
DesktopPlatformCanvasSource::CreateParagraph(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options,
    bool draw_stroke_only) {
  sk_sp<FontCollection> font_collection =
      FontCollectionFromTextOptions(text_options);
  skia::textlayout::TextStyle sk_text_style =
      CreateTextStyle(text_options, draw_stroke_only);
  ParagraphStyle paragraph_style;
  FontHolder* font_holder = &font_holder_;
  if (text_options.font_holder) {
    font_holder = text_options.font_holder;
  }
  FontWeight font_weight = font_holder->GetFontWeight();
  int weight = 0;
  switch (font_weight) {
    case FontWeight::FONT_WEIGHT_LIGHT:
      weight = SkFontStyle::kLight_Weight;
      break;
    case FontWeight::FONT_WEIGHT_NORMAL:
      weight = SkFontStyle::kNormal_Weight;
      break;
    case FontWeight::FONT_WEIGHT_MEDIUM:
      weight = SkFontStyle::kMedium_Weight;
      break;
    case FontWeight::FONT_WEIGHT_BOLD:
      weight = SkFontStyle::kBold_Weight;
      break;
  }

  imp::TextStyle text_style = font_holder->GetTextStyle();
  SkFontStyle::Slant slant = text_style == imp::TextStyle::TEXT_STYLE_ITALIC
                                 ? SkFontStyle::kItalic_Slant
                                 : SkFontStyle::kUpright_Slant;
  SkFontStyle font_style(weight, SkFontStyle::kNormal_Width, slant);
  sk_text_style.setFontStyle(font_style);

  paragraph_style.setTextStyle(sk_text_style);
  std::unique_ptr<ParagraphBuilder> paragraph_builder = ParagraphBuilder::make(
      paragraph_style, font_collection, SkUnicodes::ICU::Make());
  paragraph_builder->addText(text.data(), text.size());

  std::unique_ptr<Paragraph> paragraph = paragraph_builder->Build();
#if IMP_PLATFORM(LINUX)
  // FontConfig has a memory leak we should suppress.
  // (broken link)
  absl::LeakCheckDisabler disabler;
#endif
  paragraph->layout(SK_ScalarMax);

  return paragraph;
}

DesktopPlatformCanvasSource::DesktopScopedCanvas::DesktopScopedCanvas(
    DesktopPlatformCanvasSource& source, uint2 pixel_size,
    bool did_texture_change)
    : source_(source), did_texture_change_(did_texture_change) {
  // Compute the byte counts required to allocate the buffer then allocate and
  // setup the `ImageData`.
  SkImageInfo image_info = SkImageInfo::Make(
      pixel_size.x, pixel_size.y, SkColorType::kRGBA_8888_SkColorType,
      SkAlphaType::kUnpremul_SkAlphaType);

  pixel_buffer_size_ = image_info.computeMinByteSize();
  assert(pixel_buffer_size_ != SIZE_MAX);

  const size_t row_byte_count = image_info.minRowBytes();
  assert(row_byte_count != 0);

  // Allocate the buffer for the image and then zero out the buffer.
  pixel_buffer_ = std::make_unique<uint8_t[]>(pixel_buffer_size_);
  std::fill_n(pixel_buffer_.get(), pixel_buffer_size_, 0);

  if (!bitmap_.installPixels(image_info, pixel_buffer_.get(), row_byte_count)) {
    IMP_LOG(imp::FATAL) << "Unable to install pixels.";
  }

  canvas_ = std::make_unique<SkCanvas>(bitmap_);
}

DesktopPlatformCanvasSource::DesktopScopedCanvas::~DesktopScopedCanvas() {
  if (pixel_buffer_size_ == 0) {
    pixel_buffer_.reset();
    return;
  }

  filament::backend::PixelDataFormat format = filament::Texture::Format::RGBA;
  filament::backend::PixelDataType type = filament::Texture::Type::UBYTE;

  // The pixel_buffer_ is released so that it isn't destroyed until filament
  // finishes uploading the data to the GPU.
  filament::Texture::PixelBufferDescriptor pixel_buffer =
      filament::Texture::PixelBufferDescriptor(
          pixel_buffer_.release(), pixel_buffer_size_, format, type,
          [](void* buffer, size_t size, void* user) {
            // Called after filament finishes uploading the data to the GPU.
            // Converts the buffer back into a unique_ptr to destroy it,
            // preventing the data from leaking.
            std::unique_ptr<uint8_t[]> pixel_buffer(
                reinterpret_cast<uint8_t*>(buffer));
          },
          nullptr);

  source_.texture_->GetTexture()->setImage(
      *BaseView::GetSharedEngine(), /*level=*/0, std::move(pixel_buffer));
}

Texture* DesktopPlatformCanvasSource::DesktopScopedCanvas::GetTexture() {
  return source_.GetTexture();
}
bool DesktopPlatformCanvasSource::DesktopScopedCanvas::DidTextureChange()
    const {
  return did_texture_change_;
}

void DesktopPlatformCanvasSource::DesktopScopedCanvas::DrawColor(float3 color) {
  DrawColor(float4(color, 1.0f));
}

void DesktopPlatformCanvasSource::DesktopScopedCanvas::DrawColor(float4 color) {
  canvas_->drawColor(ToSkColor(color));
}

void DesktopPlatformCanvasSource::DesktopScopedCanvas::DrawRoundedRect(
    float3 color, float2 corner_radius, const Rect& rect) {
  DrawRoundedRect(float4(color, 1.0f), corner_radius, rect);
}

void DesktopPlatformCanvasSource::DesktopScopedCanvas::DrawRoundedRect(
    float4 color, float2 corner_radius, const Rect& rect) {
  SkPaint paint;
  paint.setColor(ToSkColor(color));

  float2 rect_min = rect.GetMin();
  float2 rect_max = rect.GetMax();

  SkRect sk_rect{.fLeft = rect_min.x,
                 .fTop = rect_min.y,
                 .fRight = rect_max.x,
                 .fBottom = rect_max.y};

  canvas_->drawRoundRect(sk_rect, corner_radius.x, corner_radius.y, paint);
}

void DesktopPlatformCanvasSource::DesktopScopedCanvas::DrawText(
    absl::string_view text, float2 pos, const TextOptions& text_options) {
  std::unique_ptr<Paragraph> paragraph =
      source_.CreateParagraph(text, text_options, /*draw_stroke_only=*/false);
  LineMetrics metrics;
  paragraph->getLineMetricsAt(0, &metrics);
  float text_width = ceil(paragraph->getMaxIntrinsicWidth());

  float half_stroke_width = text_options.stroke_width_pixels / 2.0f;

  // Offset based on the horizontal text alignment.
  float horizontal_offset = 0.0f;
  switch (text_options.horizontal_alignment) {
    case TextHorizontalAlignment::kLeftExtent: {
      horizontal_offset = half_stroke_width;
      break;
    }
    case TextHorizontalAlignment::kCenter: {
      horizontal_offset = -text_width / 2.0f;
      break;
    }
    case TextHorizontalAlignment::kLeft: {
      break;
    }
    case TextHorizontalAlignment::kRight: {
      horizontal_offset = -text_width;
      break;
    }
    case TextHorizontalAlignment::kRightExtent: {
      horizontal_offset = -text_width - half_stroke_width;
      break;
    }
  }

  // Offset based on the vertical text alignment.
  float vertical_offset = 0.0f;
  switch (text_options.vertical_alignment) {
    case TextVerticalAlignment::kAtlas:
    case TextVerticalAlignment::kTopExtent: {
      vertical_offset = half_stroke_width;
      break;
    }
    case TextVerticalAlignment::kAscent: {
      break;
    }
    case TextVerticalAlignment::kCenter: {
      vertical_offset = (-metrics.fBaseline - metrics.fDescent) / 2.0f;
      break;
    }
    case TextVerticalAlignment::kBaseline: {
      vertical_offset = -metrics.fBaseline;
      break;
    }
    case TextVerticalAlignment::kDescent: {
      vertical_offset = -metrics.fBaseline - metrics.fDescent;
      break;
    }
    case TextVerticalAlignment::kBottomExtent: {
      vertical_offset = -metrics.fHeight - half_stroke_width;
      break;
    }
  }

  if (text_options.stroke_width_pixels > 0 &&
      text_options.stroke_color != kZero4) {
    std::unique_ptr<Paragraph> paragraph_stroke =
        source_.CreateParagraph(text, text_options, /*draw_stroke_only=*/true);
    paragraph_stroke->layout(text_width);
    paragraph_stroke->paint(canvas_.get(), pos.x + horizontal_offset,
                            pos.y + vertical_offset);
  }

  paragraph->layout(text_width);
  paragraph->paint(canvas_.get(), pos.x + horizontal_offset,
                   pos.y + vertical_offset);
}

void DesktopPlatformCanvasSource::DesktopScopedCanvas::DrawGlyph(
    GlyphId glyph, float2 pos, const TextOptions& text_options,
    const TextMetrics* pre_cached_metrics) {
  IMP_LOG(imp::FATAL) << "ScopedCanvas::DrawGlyph is unavailable on Desktop.";
}

void DesktopPlatformCanvasSource::DesktopScopedCanvas::ClearRect(
    const Rect& rect) {
  SkPaint paint;
  paint.setColor(0);

  float2 rect_min = rect.GetMin();
  float2 rect_max = rect.GetMax();

  SkRect sk_rect{.fLeft = rect_min.x,
                 .fTop = rect_min.y,
                 .fRight = rect_max.x,
                 .fBottom = rect_max.y};

  canvas_->drawRect(sk_rect, paint);
}

}  // namespace imp
