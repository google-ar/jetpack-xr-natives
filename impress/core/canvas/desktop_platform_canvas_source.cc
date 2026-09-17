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
#include <cstring>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Texture.h"
#include "unicode/uchar.h"
#include "unicode/utf8.h"
#include "unicode/utypes.h"
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
#include "third_party/skia/HEAD/include/core/SkTypeface.h"
#include "third_party/skia/HEAD/include/core/SkTypes.h"
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

constexpr char kNotoSansFontName[] = "Noto Sans";
constexpr char kGoogleSansFontName[] = "Google Sans";
constexpr char kNotoSansSymbolsFontName[] = "Noto Sans Symbols";
constexpr char kNotoSansSymbols2FontName[] = "Noto Sans Symbols2";

constexpr char kNotoColorEmojiFontName[] = "Noto Color Emoji";
constexpr char kAppleColorEmojiFontName[] = "Apple Color Emoji";
constexpr char kSegoeUiEmojiFontName[] = "Segoe UI Emoji";
constexpr char kColorEmojiFontName[] = "Color Emoji";
constexpr char kEmojiFontName[] = "Emoji";

U8CPU ColorComponentRatioToByte(float color_component_ratio) {
  return round(color_component_ratio * 255.0f);
}

SkColor ToSkColor(float4 color) {
  return SkColorSetARGB(
      ColorComponentRatioToByte(color.a), ColorComponentRatioToByte(color.r),
      ColorComponentRatioToByte(color.g), ColorComponentRatioToByte(color.b));
}

SkFontStyle::Weight ToSkFontStyleWeight(FontWeight weight) {
  switch (weight) {
    case FontWeight::FONT_WEIGHT_LIGHT:
      return SkFontStyle::Weight::kLight_Weight;
    case FontWeight::FONT_WEIGHT_NORMAL:
      return SkFontStyle::Weight::kNormal_Weight;
    case FontWeight::FONT_WEIGHT_MEDIUM:
      return SkFontStyle::Weight::kMedium_Weight;
    case FontWeight::FONT_WEIGHT_BOLD:
      return SkFontStyle::Weight::kBold_Weight;
    default:
      return SkFontStyle::Weight::kNormal_Weight;
  }
}

SkFontStyle::Slant ToSkFontStyleSlant(TextStyle style) {
  switch (style) {
    case TextStyle::TEXT_STYLE_ITALIC:
      return SkFontStyle::Slant::kItalic_Slant;
    case TextStyle::TEXT_STYLE_NORMAL:
    default:
      return SkFontStyle::Slant::kUpright_Slant;
  }
}

FontWeight FromSkWeightToFontWeight(int weight) {
  if (weight <= SkFontStyle::kLight_Weight)
    return FontWeight::FONT_WEIGHT_LIGHT;
  if (weight <= SkFontStyle::kNormal_Weight)
    return FontWeight::FONT_WEIGHT_NORMAL;
  if (weight <= SkFontStyle::kMedium_Weight)
    return FontWeight::FONT_WEIGHT_MEDIUM;
  return FontWeight::FONT_WEIGHT_BOLD;
}

TextStyle FromSkFontStyleToTextStyle(SkFontStyle::Slant slant) {
  if (slant == SkFontStyle::kItalic_Slant) return TextStyle::TEXT_STYLE_ITALIC;
  return TextStyle::TEXT_STYLE_NORMAL;
}

class EmojiScanner {
 public:
  explicit EmojiScanner(absl::string_view text) : text_(text), index_(0) {}

  enum class SpanType {
    kText,
    kEmoji,
    kSymbol,
  };

  struct Span {
    size_t start;
    size_t end;
    SpanType type;
  };

  std::vector<Span> Scan() {
    std::vector<Span> spans;
    if (text_.empty()) return spans;

    size_t run_start = 0;
    std::optional<SpanType> in_type;

    while (index_ < text_.length()) {
      size_t char_start = index_;
      size_t char_len = 0;
      SpanType type = NextEmojiType(&char_len);

      if (!in_type.has_value()) {
        in_type = type;
      } else if (type != *in_type) {
        spans.push_back({run_start, char_start, *in_type});
        run_start = char_start;
        in_type = type;
      }

      index_ += char_len;
    }

    if (in_type.has_value()) {
      spans.push_back({run_start, text_.length(), *in_type});
    }
    return spans;
  }

 private:
  absl::string_view text_;
  size_t index_;

  SpanType NextEmojiType(size_t* length) const {
    if (index_ >= text_.length()) {
      *length = 0;
      return SpanType::kText;
    }

    int32_t temp_index = index_;
    UChar32 c;
    U8_NEXT(text_.data(), temp_index, text_.length(), c);
    size_t first_char_len = temp_index - index_;

    if (c < 0) {
      *length = first_char_len;
      return SpanType::kText;
    }

    if (IsKeycapBase(c)) {
      size_t char_len1 = 0;
      UChar32 next1 = PeekCodepoint(temp_index, &char_len1);
      if (next1 == 0xFE0F) {
        size_t char_len2 = 0;
        UChar32 next2 = PeekCodepoint(temp_index + char_len1, &char_len2);
        if (next2 == 0x20E3) {
          *length = first_char_len + char_len1 + char_len2;
          return SpanType::kEmoji;
        }
      } else if (next1 == 0x20E3) {
        *length = first_char_len + char_len1;
        return SpanType::kEmoji;
      }
      *length = first_char_len;
      return SpanType::kText;
    }

    if (IsRegionalIndicator(c)) {
      size_t char_len1 = 0;
      UChar32 next1 = PeekCodepoint(temp_index, &char_len1);
      if (IsRegionalIndicator(next1)) {
        *length = first_char_len + char_len1;
        return SpanType::kEmoji;
      }
      *length = first_char_len;
      return SpanType::kEmoji;
    }

    bool is_emoji = u_hasBinaryProperty(c, UCHAR_EMOJI);
    bool is_emoji_presentation =
        u_hasBinaryProperty(c, UCHAR_EMOJI_PRESENTATION);

    if (is_emoji) {
      size_t char_len1 = 0;
      UChar32 next1 = PeekCodepoint(temp_index, &char_len1);
      if (next1 == 0xFE0E) {  // VS15
        *length = first_char_len + char_len1;
        return SpanType::kSymbol;
      }
      if (next1 == 0xFE0F) {  // VS16
        *length = first_char_len + char_len1;
        size_t seq_len = 0;
        if (IsSequencedEmoji(temp_index + char_len1, &seq_len)) {
          *length += seq_len;
        }
        return SpanType::kEmoji;
      }

      *length = first_char_len;
      size_t modifier_len = 0;
      UChar32 next_c = PeekCodepoint(temp_index, &modifier_len);
      if (u_hasBinaryProperty(next_c, UCHAR_EMOJI_MODIFIER)) {
        *length += modifier_len;
        temp_index += modifier_len;
      }

      size_t seq_len = 0;
      if (IsSequencedEmoji(temp_index, &seq_len)) {
        *length += seq_len;
      }

      return is_emoji_presentation ? SpanType::kEmoji : SpanType::kSymbol;
    }

    *length = first_char_len;
    return SpanType::kText;
  }

  UChar32 PeekCodepoint(size_t byte_idx, size_t* len) const {
    if (byte_idx >= text_.length()) {
      *len = 0;
      return -1;
    }
    int32_t temp_idx = byte_idx;
    UChar32 c;
    U8_NEXT(text_.data(), temp_idx, text_.length(), c);
    *len = temp_idx - byte_idx;
    return c;
  }

  bool IsSequencedEmoji(size_t byte_idx, size_t* length) const {
    size_t total_len = 0;
    size_t current_idx = byte_idx;
    while (true) {
      size_t char_len = 0;
      UChar32 c = PeekCodepoint(current_idx, &char_len);
      if (c == 0x200D) {  // ZWJ
        size_t next_emoji_len = 0;
        bool next_is_emoji =
            NextEmojiLengthInternal(current_idx + char_len, &next_emoji_len);
        if (next_is_emoji) {
          total_len += char_len + next_emoji_len;
          current_idx += char_len + next_emoji_len;
        } else {
          break;
        }
      } else {
        break;
      }
    }
    if (total_len > 0) {
      *length = total_len;
      return true;
    }
    *length = 0;
    return false;
  }

  bool NextEmojiLengthInternal(size_t byte_idx, size_t* length) const {
    size_t char_len = 0;
    UChar32 c = PeekCodepoint(byte_idx, &char_len);
    if (c < 0) return false;

    if (u_hasBinaryProperty(c, UCHAR_EMOJI)) {
      *length = char_len;
      size_t modifier_len = 0;
      UChar32 next_c = PeekCodepoint(byte_idx + char_len, &modifier_len);
      if (u_hasBinaryProperty(next_c, UCHAR_EMOJI_MODIFIER)) {
        *length += modifier_len;
      }
      return true;
    }
    return false;
  }

  bool IsKeycapBase(UChar32 c) const {
    return (c >= '0' && c <= '9') || c == '#' || c == '*';
  }

  bool IsRegionalIndicator(UChar32 c) const {
    return c >= 0x1F1E6 && c <= 0x1F1FF;
  }
};

bool IsEmojiTypeface(SkTypeface* typeface) {
  if (!typeface) return false;
  SkString family_name;
  typeface->getFamilyName(&family_name);
  return family_name.contains("Emoji") || family_name.contains("emoji");
}
}  // namespace

bool DesktopPlatformCanvasSource::IsFeatureSupported(
    ScopedCanvas::Feature feature) {
  switch (feature) {
    case ScopedCanvas::Feature::kGlyphs:
      return true;
    case ScopedCanvas::Feature::kKeepContents:
      return true;
  }
}

void DesktopPlatformCanvasSource::EnsurePixelBuffer(uint2 pixel_size) {
  if (bitmap_.isNull() || bitmap_.width() != pixel_size.x ||
      bitmap_.height() != pixel_size.y) {
    SkImageInfo image_info = SkImageInfo::Make(
        pixel_size.x, pixel_size.y, SkColorType::kRGBA_8888_SkColorType,
        SkAlphaType::kUnpremul_SkAlphaType);
    if (!bitmap_.tryAllocPixels(image_info)) {
      IMP_LOG(imp::FATAL) << "Could not allocate pixel buffer.";
    }
  }
}

bool DesktopPlatformCanvasSource::EnsureTextureAndBuffer(
    BaseView& view, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    SmallSourceLocation loc) {
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

    if (on_texture_changed_fn) {
      on_texture_changed_fn(texture_.Borrow(loc));
    }
  }

  EnsurePixelBuffer(pixel_size);

  return did_texture_change;
}

Future<absl::Status> DesktopPlatformCanvasSource::PrepareFont(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return Future<absl::Status>(absl::OkStatus());
}

TextMetrics DesktopPlatformCanvasSource::GetTextMetrics(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  LineMetrics metrics;
  {
    // Lock the paragraph mutex to ensure thread-safe access to the Skia state.
    absl::MutexLock lock(paragraph_mutex_);
    std::unique_ptr<Paragraph> paragraph =
        CreateParagraph(text, text_options, /*draw_stroke_only=*/false);
    paragraph->getLineMetricsAt(0, &metrics);
  }
  std::vector<float> text_widths = GetTextWidths(text, text_options);
  float typographical_width =
      std::accumulate(text_widths.begin(), text_widths.end(), 0.0f);
  // The stroke straddles the font, half in and half out.
  float stroke_padding = text_options.stroke_width_pixels;
  TextMetrics text_metrics;
  // Origin does not include the stroke width.
  text_metrics.set_origin_x(0.0f);
  text_metrics.set_origin_y(-metrics.fDescent);
  // Size does include the stroke width.
  text_metrics.set_size_x(metrics.fWidth + stroke_padding);
  text_metrics.set_size_y(metrics.fHeight + stroke_padding);
  text_metrics.set_typographical_width(typographical_width);
  // Font origin does not include the stroke width.
  text_metrics.set_font_origin_y(static_cast<float>(-metrics.fDescent));
  text_metrics.set_font_size_y(
      static_cast<float>(metrics.fHeight + stroke_padding));
  return text_metrics;
}

TextMetrics DesktopPlatformCanvasSource::GetGlyphMetrics(
    ScopedCanvas::GlyphId glyph,
    const ScopedCanvas::TextOptions& text_options) {
  sk_sp<SkTypeface> typeface;
  {
    absl::MutexLock lock(paragraph_mutex_);
    typeface = GetTypeface(text_options);
  }
  if (!typeface) {
    IMP_LOG(imp::ERROR) << "Failed to get typeface for glyph metrics.";
    return TextMetrics();
  }

  SkFont font(typeface, text_options.size_pixels);
  font.setSubpixel(true);
  font.setHinting(SkFontHinting::kSlight);
  SkGlyphID sk_glyph = static_cast<SkGlyphID>(glyph.Get());

  SkRect bounds = font.getBounds(sk_glyph, nullptr);

  SkScalar width;
  font.getWidths(SkSpan(&sk_glyph, 1), SkSpan(&width, 1));

  SkFontMetrics font_metrics;
  font.getMetrics(&font_metrics);

  float max_font_descent = std::max(bounds.fBottom, font_metrics.fDescent);
  float max_font_ascent = -std::min(bounds.fTop, font_metrics.fAscent);
  float padding = text_options.stroke_width_pixels;

  float glyph_width = bounds.width();
  float glyph_height = bounds.height();
  float origin_x = bounds.fLeft;
  float origin_y = -bounds.fBottom;
  if (glyph_width == 0.0f && width > 0.0f) {
    glyph_width = width;
    origin_x = 0.0f;
  }
  if (glyph_height == 0.0f) {
    glyph_height = max_font_ascent + max_font_descent;
    origin_y = -max_font_descent;
  }

  TextMetrics metrics;
  metrics.set_origin_x(origin_x);
  metrics.set_origin_y(origin_y);
  metrics.set_size_x(glyph_width + padding);
  metrics.set_size_y(glyph_height + padding);
  metrics.set_typographical_width(width);
  metrics.set_font_origin_y(-max_font_descent);
  metrics.set_font_size_y(max_font_ascent + max_font_descent + padding);

  return metrics;
}

std::vector<ScopedCanvas::GlyphGroup>
DesktopPlatformCanvasSource::GetCombinedCharacterGroups(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  std::unique_ptr<Paragraph> paragraph;
  {
    absl::MutexLock lock(paragraph_mutex_);
    paragraph = CreateParagraph(text, text_options, /*draw_stroke_only=*/false);
  }

  std::vector<ScopedCanvas::GlyphGroup> result;
  ScopedCanvas::GlyphGroup current_group = 0;
  paragraph->visit([&result, &current_group](
                       int line_number, const Paragraph::VisitorInfo* info) {
    if (!info) return;
    result.reserve(result.size() + info->count);
    for (int i = 0; i < info->count; ++i) {
      float current_x = info->positions[i].x();
      float next_x;
      if (i < info->count - 1) {
        next_x = info->positions[i + 1].x();
      } else {
        next_x = info->advanceX - info->origin.x();
      }

      if (!result.empty() && current_x < next_x) {
        current_group++;
      }
      result.push_back(current_group);
    }
  });

  return result;
}

std::vector<float> DesktopPlatformCanvasSource::GetTextWidths(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  std::unique_ptr<Paragraph> paragraph;
  {
    // Lock the paragraph mutex to ensure thread-safe access to the Skia state.
    absl::MutexLock lock(paragraph_mutex_);
    /**
     * If the original text was utf16 but we are passed a substring that can be
     * represented in ASCII, we can wrap it in a string to force the characters
     * to be 1 byte each.
     */
    paragraph = CreateParagraph(std::string(text), text_options,
                                /*draw_stroke_only=*/false);
  }
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
  std::unique_ptr<Paragraph> paragraph;
  sk_sp<SkTypeface> expected_typeface;
  {
    absl::MutexLock lock(paragraph_mutex_);
    paragraph = CreateParagraph(text, text_options, /*draw_stroke_only=*/false);
    expected_typeface = GetTypeface(text_options);
  }

  std::vector<ScopedCanvas::GlyphAdvance> glyph_advances;
  paragraph->visit([&glyph_advances, &expected_typeface](
                       int line_number, const Paragraph::VisitorInfo* info) {
    if (!info) return;

    SkTypeface* run_typeface = info->font.getTypeface();

    std::unique_ptr<DesktopFontHolder> run_fallback_font = nullptr;
    if (run_typeface &&
        !SkTypeface::Equal(run_typeface, expected_typeface.get())) {
      SkString family_name;
      run_typeface->getFamilyName(&family_name);
      SkFontStyle style = run_typeface->fontStyle();
      run_fallback_font = std::make_unique<DesktopFontHolder>(
          family_name.c_str(), FromSkWeightToFontWeight(style.weight()),
          FromSkFontStyleToTextStyle(style.slant()));
    }

    bool is_emoji = IsEmojiTypeface(run_typeface);

    glyph_advances.reserve(glyph_advances.size() + info->count);

    std::vector<int> sorted_indices(info->count);
    std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
    std::stable_sort(sorted_indices.begin(), sorted_indices.end(),
                     [info](int a, int b) {
                       return info->positions[a].x() < info->positions[b].x();
                     });

    for (int k = 0; k < info->count; ++k) {
      int i = sorted_indices[k];
      float advance;
      if (k < info->count - 1) {
        int next_i = sorted_indices[k + 1];
        advance = info->positions[next_i].x() - info->positions[i].x();
      } else {
        advance = (info->advanceX - info->origin.x()) - info->positions[i].x();
      }

      std::unique_ptr<FontHolder> glyph_fallback_font = nullptr;
      if (run_fallback_font) {
        glyph_fallback_font = std::make_unique<DesktopFontHolder>(
            run_fallback_font->GetFontName(),
            run_fallback_font->GetFontWeight(),
            run_fallback_font->GetTextStyle(),
            run_fallback_font->GetFontCollection());
      }

      glyph_advances.push_back({
          ScopedCanvas::GlyphId(info->glyphs[i]),
          advance,
          std::move(glyph_fallback_font),
          is_emoji,
      });
    }
  });

  if (glyph_advances.empty() && !text.empty()) {
    std::vector<float> widths = GetTextWidths(text, text_options);
    glyph_advances.reserve(widths.size());
    sk_sp<SkTypeface> typeface;
    {
      absl::MutexLock lock(paragraph_mutex_);
      typeface = GetTypeface(text_options);
    }
    SkFont font(typeface, text_options.size_pixels);
    int text_idx = 0;
    for (float width : widths) {
      char32_t unichar = (text_idx < text.size()) ? text[text_idx] : ' ';
      SkGlyphID gid = font.unicharToGlyph(unichar);
      glyph_advances.push_back({
          ScopedCanvas::GlyphId(gid),
          width,
          nullptr,
          /*is_emoji=*/false,
      });
      text_idx++;
    }
  }

  return glyph_advances;
}

FontInfo DesktopPlatformCanvasSource::GetFontInfo(
    const ScopedCanvas::TextOptions& text_options) {
  SkFont font;
  {
    // Lock the paragraph mutex to ensure thread-safe access to the Skia state.
    absl::MutexLock lock(paragraph_mutex_);
    std::unique_ptr<Paragraph> paragraph =
        CreateParagraph(" ", text_options, /*draw_stroke_only=*/false);
    font = paragraph->getFontAt(0);
  }
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
  bool did_texture_change = EnsureTextureAndBuffer(view, pixel_size);

  if (draw_mode == ScopedCanvas::DrawMode::kClear && !did_texture_change) {
    bitmap_.eraseColor(0);
  }

  return std::make_unique<DesktopScopedCanvas>(*this, pixel_size_,
                                               did_texture_change);
}

std::unique_ptr<ScopedCanvas> DesktopPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size,
    ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) {
  bool did_texture_change = EnsureTextureAndBuffer(
      view, pixel_size, std::move(on_texture_changed_fn), loc);

  if (draw_mode == ScopedCanvas::DrawMode::kClear && !did_texture_change) {
    bitmap_.eraseColor(0);
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

sk_sp<SkTypeface> DesktopPlatformCanvasSource::GetTypeface(
    const ScopedCanvas::TextOptions& text_options) {
#if IMP_PLATFORM(LINUX)
  // FontConfig has a memory leak we should suppress.
  // (broken link)
  absl::LeakCheckDisabler disabler;
#endif
  sk_sp<FontCollection> font_collection =
      FontCollectionFromTextOptions(text_options);
  const FontHolder* font_holder =
      text_options.font_holder ? text_options.font_holder : &font_holder_;
  absl::string_view family_name = font_holder->GetFontName();

  SkFontStyle font_style(ToSkFontStyleWeight(font_holder->GetFontWeight()),
                         SkFontStyle::Width::kNormal_Width,
                         ToSkFontStyleSlant(font_holder->GetTextStyle()));

  sk_sp<SkTypeface> resolved_typeface = nullptr;

  if (family_name.empty()) {
    sk_sp<SkFontMgr> fm = font_collection->getFallbackManager();
    if (fm) {
      resolved_typeface = fm->legacyMakeTypeface(nullptr, font_style);
      if (!resolved_typeface && fm->countFamilies() > 0) {
        SkString first_family;
        fm->getFamilyName(0, &first_family);
        resolved_typeface =
            fm->legacyMakeTypeface(first_family.c_str(), font_style);
      }
    }
  } else {
    std::vector<SkString> families = {
        SkString(family_name.data(), family_name.size()),
        SkString(kNotoSansFontName), SkString(kGoogleSansFontName),
        SkString(kNotoSansSymbolsFontName),
        SkString(kNotoSansSymbols2FontName)};
    std::vector<sk_sp<SkTypeface>> typefaces =
        font_collection->findTypefaces(families, font_style);
    if (!typefaces.empty()) {
      resolved_typeface = typefaces[0];
    }
  }

  if (!resolved_typeface) {
    return nullptr;
  }

  return resolved_typeface;
}

SkPaint DesktopPlatformCanvasSource::CreatePaint(
    const ScopedCanvas::TextOptions& text_options, bool draw_stroke_only) {
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
  return paint;
}

skia::textlayout::TextStyle DesktopPlatformCanvasSource::CreateTextStyle(
    const ScopedCanvas::TextOptions& text_options, bool draw_stroke_only) {
  skia::textlayout::TextStyle text_style;
  text_style.setFontSize(text_options.size_pixels);
  std::vector<SkString> font_families;
  if (text_options.font_holder) {
    font_families.push_back(SkString(text_options.font_holder->GetFontName()));
  } else {
    font_families.push_back(SkString(font_holder_.GetFontName()));
  }
  font_families.push_back(SkString(kNotoSansFontName));
  font_families.push_back(SkString(kGoogleSansFontName));
  font_families.push_back(SkString(kNotoSansSymbolsFontName));
  font_families.push_back(SkString(kNotoSansSymbols2FontName));
  text_style.setFontFamilies(font_families);
  SkPaint paint = CreatePaint(text_options, draw_stroke_only);
  text_style.setForegroundPaint(paint);
  text_style.setLetterSpacing(text_options.text_tracking *
                              text_options.size_pixels);
  text_style.setSubpixel(true);
  text_style.setFontHinting(SkFontHinting::kSlight);
  if (text_options.locale.has_value()) {
    text_style.setLocale(SkString(*text_options.locale));
  }
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

  if (IsFeatureSupported(ScopedCanvas::Feature::kGlyphs)) {
    EmojiScanner scanner(text);
    std::vector<EmojiScanner::Span> spans = scanner.Scan();
    for (const auto& span : spans) {
      absl::string_view run_text =
          text.substr(span.start, span.end - span.start);
      if (span.type == EmojiScanner::SpanType::kEmoji) {
        skia::textlayout::TextStyle emoji_style = sk_text_style;
        emoji_style.setFontFamilies({
            SkString(kNotoColorEmojiFontName),
            SkString(kAppleColorEmojiFontName),
            SkString(kSegoeUiEmojiFontName),
            SkString(kColorEmojiFontName),
            SkString(kEmojiFontName),
        });
        paragraph_builder->pushStyle(emoji_style);
        paragraph_builder->addText(run_text.data(), run_text.size());
        paragraph_builder->pop();
      } else if (span.type == EmojiScanner::SpanType::kSymbol) {
        skia::textlayout::TextStyle symbol_style = sk_text_style;
        std::vector<SkString> families = {SkString(font_holder->GetFontName())};
        families.push_back(SkString(kNotoSansSymbolsFontName));
        families.push_back(SkString(kNotoSansSymbols2FontName));
        symbol_style.setFontFamilies(families);

        paragraph_builder->pushStyle(symbol_style);
        paragraph_builder->addText(run_text.data(), run_text.size());
        paragraph_builder->pop();
      } else {
        paragraph_builder->addText(run_text.data(), run_text.size());
      }
    }
  } else {
    paragraph_builder->addText(text.data(), text.size());
  }

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
    : source_(source),
      bitmap_(source.bitmap_),
      did_texture_change_(did_texture_change) {
  canvas_ = std::make_unique<SkCanvas>(bitmap_);
}

DesktopPlatformCanvasSource::DesktopScopedCanvas::~DesktopScopedCanvas() {
  size_t byte_size = bitmap_.computeByteSize();
  if (byte_size == 0) {
    return;
  }

  filament::backend::PixelDataFormat format = filament::Texture::Format::RGBA;
  filament::backend::PixelDataType type = filament::Texture::Type::UBYTE;

  // The persistent buffer should stay alive in the CanvasSource, so we must
  // copy the pixels to a temporary buffer for Filament to upload and release.
  void* temp_buffer = malloc(byte_size);
  std::memcpy(temp_buffer, bitmap_.getPixels(), byte_size);

  filament::Texture::PixelBufferDescriptor pixel_buffer =
      filament::Texture::PixelBufferDescriptor(
          temp_buffer, byte_size, format, type,
          [](void* buffer, size_t size, void* user) { free(buffer); }, nullptr);

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
  std::unique_ptr<Paragraph> paragraph;
  {
    // Lock the paragraph mutex to ensure thread-safe access to the Skia state.
    absl::MutexLock lock(source_.paragraph_mutex_);
    paragraph =
        source_.CreateParagraph(text, text_options, /*draw_stroke_only=*/false);
  }
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

  {
    // Lock the paragraph mutex for CreateParagraph() and layout() calls.
    absl::MutexLock lock(source_.paragraph_mutex_);
    if (text_options.stroke_width_pixels > 0 &&
        text_options.stroke_color != kZero4) {
      std::unique_ptr<Paragraph> paragraph_stroke = source_.CreateParagraph(
          text, text_options, /*draw_stroke_only=*/true);
      paragraph_stroke->layout(text_width);
      paragraph_stroke->paint(canvas_.get(), pos.x + horizontal_offset,
                              pos.y + vertical_offset);
    }

    paragraph->layout(text_width);
    paragraph->paint(canvas_.get(), pos.x + horizontal_offset,
                     pos.y + vertical_offset);
  }
}

void DesktopPlatformCanvasSource::DesktopScopedCanvas::DrawGlyph(
    GlyphId glyph, float2 pos, const TextOptions& text_options,
    const TextMetrics* pre_cached_metrics) {
  sk_sp<SkTypeface> typeface;
  {
    absl::MutexLock lock(source_.paragraph_mutex_);
    typeface = source_.GetTypeface(text_options);
  }
  if (!typeface) {
    IMP_LOG(imp::ERROR) << "Failed to get typeface for DrawGlyph.";
    return;
  }

  TextMetrics metrics_storage;
  if (!pre_cached_metrics) {
    metrics_storage = source_.GetGlyphMetrics(glyph, text_options);
    pre_cached_metrics = &metrics_storage;
  }

  SkFont font(typeface, text_options.size_pixels);
  font.setSubpixel(true);
  font.setHinting(SkFontHinting::kSlight);
  SkGlyphID sk_glyph = static_cast<SkGlyphID>(glyph.Get());

  float half_stroke = text_options.stroke_width_pixels / 2.0f;
  float max_font_ascent = pre_cached_metrics->font_size_y() +
                          pre_cached_metrics->font_origin_y() -
                          text_options.stroke_width_pixels;

  SkPoint sk_pos =
      SkPoint::Make(pos.x + half_stroke - pre_cached_metrics->origin_x(),
                    pos.y + half_stroke + max_font_ascent);

  if (text_options.stroke_width_pixels > 0 &&
      text_options.stroke_color != kZero4) {
    SkPaint stroke_paint =
        source_.CreatePaint(text_options, /*draw_stroke_only=*/true);
    canvas_->drawGlyphs(SkSpan(&sk_glyph, 1), SkSpan(&sk_pos, 1),
                        /*origin=*/{0, 0}, font, stroke_paint);
  }

  SkPaint fill_paint =
      source_.CreatePaint(text_options, /*draw_stroke_only=*/false);
  canvas_->drawGlyphs(SkSpan(&sk_glyph, 1), SkSpan(&sk_pos, 1),
                      /*origin=*/{0, 0}, font, fill_paint);
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
