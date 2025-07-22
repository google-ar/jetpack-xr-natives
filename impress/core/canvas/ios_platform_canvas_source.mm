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

#include "core/canvas/ios_platform_canvas_source.h"

#include <CoreText/CoreText.h>
#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#include "core/common/log.h"

#include <algorithm>
#include <iterator>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "third_party/absl/memory/memory.h"
#include "core/canvas/constants.h"
#include "core/canvas/fonts/ios_font_holder.h"
#include "core/geometry/shapes/rect.h"
#include "core/render/texture_factory.h"

namespace imp {

namespace {

UIFont* FontFromTextOptions(const ScopedCanvas::TextOptions& text_options, float pixels_per_dp) {
  if (text_options.font_holder) {
    UIFont* ui_font = (__bridge UIFont*)text_options.font_holder->GetPlatformFont();
    return [ui_font fontWithSize:(text_options.size_pixels / pixels_per_dp)];
  }

  // No font specified, make a default font.
  return [UIFont systemFontOfSize:(text_options.size_pixels / pixels_per_dp)];
}

// Returns a float in the range [0.0, 100.0] that is a percentage of the stroke width with respect
// to the font size, and is meant to be used as value of  NSStrokeWidthAttributeName.
float GetStrokeWidthAsPercentageOfFontSize(const ScopedCanvas::TextOptions& text_options) {
  // Default value of zero will not enable outstroke.
  // https://developer.apple.com/documentation/uikit/nsstrokewidthattributename
  return 100.0f * (text_options.stroke_width_pixels / text_options.size_pixels);
}

NSDictionary<NSAttributedStringKey, id>* GetTextAttributesForTextMetrics(
    const ScopedCanvas::TextOptions& text_options, float pixels_per_dp) {
  UIFont* text_font = FontFromTextOptions(text_options, pixels_per_dp);
  float text_tracking = text_options.text_tracking * text_options.size_pixels / pixels_per_dp;
  // Use kerning instead of tracking if the later is not supported. See:
  // https://developer.apple.com/documentation/uikit/nskernattributename
  // https://developer.apple.com/documentation/uikit/nstrackingattributename
  if (@available(iOS 14, *)) {
    return @{
      NSFontAttributeName : text_font,
      NSTrackingAttributeName : [NSNumber numberWithFloat:text_tracking],
    };
  } else {
    return @{
      NSFontAttributeName : text_font,
      NSKernAttributeName : [NSNumber numberWithFloat:text_tracking],
    };
  }
}

}  // namespace

struct IosPlatformCanvasSource::PlatformPod {
  CGRect pixel_bounds;
  CVPixelBufferRef pixel_buffer = nil;
};

class IosPlatformCanvasSource::IosScopedCanvas : public ScopedCanvas {
 public:
  explicit IosScopedCanvas(IosPlatformCanvasSource& source, uint2 pixel_size,
                           bool did_texture_change, ScopedCanvas::DrawMode draw_mode);
  ~IosScopedCanvas() override;

  Texture* GetTexture() override;
  bool DidTextureChange() const override;

  void DrawColor(float3 color) override;
  void DrawColor(float4 color) override;
  void DrawRoundedRect(float3 color, float2 corner_radius, const Rect& rect) override;
  void DrawRoundedRect(float4 color, float2 corner_radius, const Rect& rect) override;
  void DrawText(absl::string_view text, float2 pos, const TextOptions& text_options) override;
  void DrawGlyph(GlyphId glyph, float2 pos, const TextOptions& text_options) override;
  void ClearRect(const Rect& rect) override;

 private:
  IosPlatformCanvasSource& source_;

  CGColorSpaceRef device_colors_ = nil;
  CGContextRef context_ = nil;
  bool did_texture_change_;
};

IosPlatformCanvasSource::IosPlatformCanvasSource()
    : platform_pod_(std::make_unique<PlatformPod>()) {
  pixels_per_dp_ = [UIScreen mainScreen].scale;
}

// Destructor must be defined after PlatformPod so that unique_ptr can delete it.
IosPlatformCanvasSource::~IosPlatformCanvasSource() {
  if (platform_pod_->pixel_buffer) {
    CVPixelBufferRelease(platform_pod_->pixel_buffer);
  }
}

bool IosPlatformCanvasSource::IsFeatureSupported(ScopedCanvas::Feature feature) {
  switch (feature) {
    case ScopedCanvas::Feature::kGlyphs:
      return true;
    case ScopedCanvas::Feature::kKeepContents:
      return true;
  }
}

Future<absl::Status> IosPlatformCanvasSource::PrepareFont(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return Future<absl::Status>(absl::OkStatus());
}

ScopedCanvas::TextMetrics IosPlatformCanvasSource::GetTextMetrics(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  @autoreleasepool {
    NSDictionary<NSAttributedStringKey, id>* text_attributes =
        GetTextAttributesForTextMetrics(text_options, pixels_per_dp_);

    NSString* string = @(std::string(text).c_str());
    NSAttributedString* attributed_string =
        [[NSAttributedString alloc] initWithString:string attributes:text_attributes];

    CGSize text_size = [attributed_string size];
    CGRect text_rect = [attributed_string boundingRectWithSize:text_size
                                                       options:NSStringDrawingUsesDeviceMetrics
                                                       context:nullptr];

    // Note: NSAttributedString's size measurements return the size/shape of a bounding box around
    // the font's stroke path; it does not include any stroke width in NSStrokeWidthAttributeName.
    // The stroke straddles the text, so half of the stroke width needs to be added to either side.
    // https://developer.apple.com/documentation/coregraphics/1454679-cgcontextstrokerectwithwidth?language=objc.

    // Stroke_width_pixels is already in pixels, so no need to multiply by pixels_per_dp_.
    float stroke_padding = text_options.stroke_width_pixels;
    return {
        // Origin refers to the starting location of the fill, not the stroke.
        .origin = float2{text_rect.origin.x * pixels_per_dp_, text_rect.origin.y * pixels_per_dp_},
        // The size includes the stroke.
        .size = float2{text_rect.size.width * pixels_per_dp_ + stroke_padding,
                       text_rect.size.height * pixels_per_dp_ + stroke_padding},
        .typographical_width = static_cast<float>(text_size.width * pixels_per_dp_),
        // TODO: Return proper metrics here. Though, given that
        // iOS supports per-glyph rendering, these metrics are never used
        // anywhere.
        .font_origin_y = static_cast<float>(text_rect.origin.y * pixels_per_dp_),
        .font_size_y = static_cast<float>(text_size.height * pixels_per_dp_)};
  }
}

ScopedCanvas::TextMetrics IosPlatformCanvasSource::GetGlyphMetrics(
    ScopedCanvas::GlyphId glyph, const ScopedCanvas::TextOptions& text_options) {
  UIFont* text_font = FontFromTextOptions(text_options, pixels_per_dp_);

  CGGlyph cg_glyph = static_cast<CGGlyph>(glyph);

  // Get the bounds of the glyph.
  CGRect glyph_rect = CTFontGetBoundingRectsForGlyphs(
      (CTFontRef)text_font, kCTFontOrientationDefault, &cg_glyph, nullptr, 1);

  // Stroke_width_pixels is already in pixels, so no need to multiply by pixels_per_dp_.
  float stroke_padding = text_options.stroke_width_pixels;

  // TODO: We should be aligning to the line height of the specific
  // font used to render this specific glyph, not the line height of the default
  // font. By doing it like below, we risk creating a "jagged baseline" for any
  // script which regularly exceeds the height of the default font.
  float font_origin_y;
  float font_size_y;
  // Some override fonts, like KohinoorTelugu-Regular, regularly exceed the line height and draw
  // the stroke into the leading, and therefore need the leading included in the font height.
  // See (broken link) for more information.
  // Note that many default fonts such as GoogleSans-Regular have a leading of 0, but they are
  // overridden when the requested glyph is not available in that font set.
  float text_font_total_line_height = text_font.lineHeight + text_font.leading;
  if (glyph_rect.size.height > text_font_total_line_height) {
    font_origin_y = glyph_rect.origin.y * pixels_per_dp_;
    font_size_y = glyph_rect.size.height * pixels_per_dp_ + stroke_padding;
  } else {
    font_origin_y = (text_font.descender - text_font.leading) * pixels_per_dp_;
    font_size_y = text_font_total_line_height * pixels_per_dp_ + stroke_padding;
  }

  return {
      // The origin is the starting location of the fill, not the stroke.
      .origin = float2{glyph_rect.origin.x * pixels_per_dp_, glyph_rect.origin.y * pixels_per_dp_},
      // The size includes the stroke.
      .size = float2{glyph_rect.size.width * pixels_per_dp_ + stroke_padding,
                     glyph_rect.size.height * pixels_per_dp_ + stroke_padding},
      // Font origin is the starting location of the fill, not the stroke.
      .font_origin_y = font_origin_y,
      .font_size_y = font_size_y};
}

std::vector<ScopedCanvas::GlyphGroup> IosPlatformCanvasSource::GetCombinedCharacterGroups(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  std::vector<ScopedCanvas::GlyphGroup> result;
  @autoreleasepool {
    // Get the font based on the text options.
    UIFont* text_font = FontFromTextOptions(text_options, pixels_per_dp_);

    // Convert the text and the font into an NSAttributedString.
    NSDictionary<NSAttributedStringKey, id>* text_attributes =
        @{NSFontAttributeName : text_font,
          NSLigatureAttributeName : @1};
    NSString* string = @(std::string(text).c_str());
    CFStringRef cf_string = (__bridge CFStringRef)string;
    NSAttributedString* attributed_string =
        [[NSAttributedString alloc] initWithString:string attributes:text_attributes];

    // Create the CoreText line from the NSAttributedString.
    CTLineRef ct_line = CTLineCreateWithAttributedString((CFAttributedStringRef)attributed_string);

    CFArrayRef runs = CTLineGetGlyphRuns(ct_line);
    CFIndex num_runs = CFArrayGetCount(runs);

    for (CFIndex run_index = 0; run_index < num_runs; run_index++) {
      CTRunRef run = (CTRunRef)CFArrayGetValueAtIndex(runs, run_index);
      CFIndex glyph_count = CTRunGetGlyphCount(run);

      for (CFIndex glyph_index = 0; glyph_index < glyph_count; glyph_index++) {
        CFIndex string_index;
        CTRunGetStringIndices(run, CFRangeMake(glyph_index, 1), &string_index);

        CFRange range = CFStringGetRangeOfComposedCharactersAtIndex(cf_string, string_index);
        result.push_back(range.location);
      }
    }
    CFRelease(ct_line);
  }

  return result;
}

std::vector<float> IosPlatformCanvasSource::GetTextWidths(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  // Get the Text Glyphs.
  std::vector<ScopedCanvas::GlyphAdvance> text_glyphs =
      GetTextGlyphsInternal(text, text_options, TextGlyphsMode::kExcludeLigatures);

  std::vector<float> result;
  result.reserve(text_glyphs.size());

  // Convert the glyphs to just the widths.
  std::transform(text_glyphs.begin(), text_glyphs.end(), std::back_inserter(result),
                 [](const ScopedCanvas::GlyphAdvance& glyph) { return glyph.width; });

  return result;
}

std::vector<ScopedCanvas::GlyphAdvance> IosPlatformCanvasSource::GetTextGlyphs(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options) {
  return GetTextGlyphsInternal(text, text_options, TextGlyphsMode::kIncludeLigatures);
}

std::vector<ScopedCanvas::GlyphAdvance> IosPlatformCanvasSource::GetTextGlyphsInternal(
    absl::string_view text, const ScopedCanvas::TextOptions& text_options, TextGlyphsMode mode) {
  @autoreleasepool {
    // Get the font based on the text options.
    UIFont* text_font = FontFromTextOptions(text_options, pixels_per_dp_);
    // Be consistent, draw with the same attributes as the text metrics was measured with.
    NSMutableDictionary<NSAttributedStringKey, id>* text_attributes = [NSMutableDictionary
        dictionaryWithDictionary:GetTextAttributesForTextMetrics(text_options, pixels_per_dp_)];
    text_attributes[NSLigatureAttributeName] =
        (mode == TextGlyphsMode::kIncludeLigatures) ? @1 : @0;
    NSString* string = @(std::string(text).c_str());
    NSAttributedString* attributed_string =
        [[NSAttributedString alloc] initWithString:string attributes:text_attributes];

    // Create the CoreText line from the NSAttributedString.
    CTLineRef ct_line = CTLineCreateWithAttributedString((CFAttributedStringRef)attributed_string);

    // Get the runs from the line.
    // A line may have multiple lines when the multiple fonts are used within the line.
    // This occurs when the primary font selected doesn't contain a needed glyph and the system
    // picks a fallback font. This is can happen with text mixing different languages. For example,
    // CJK characters mixed with latin characters.
    CFArrayRef runs = CTLineGetGlyphRuns(ct_line);
    CFIndex num_runs = CFArrayGetCount(runs);

    // Stores the results.
    std::vector<ScopedCanvas::GlyphAdvance> result;

    for (CFIndex run_index = 0; run_index < num_runs; run_index++) {
      CTRunRef run = (CTRunRef)CFArrayGetValueAtIndex(runs, run_index);

      // Get the glyphs for this run.
      CFIndex glyph_count = CTRunGetGlyphCount(run);
      std::vector<CGGlyph> glyphs(glyph_count);
      CTRunGetGlyphs(run, CFRangeMake(0, glyph_count), glyphs.data());

      // Get the font for this run. This may be different from the font used by the
      // NSAttributedString if the system picked a fallback font due to missing glyphs.
      NSDictionary<NSString*, id>* run_attributes = (__bridge NSDictionary*)CTRunGetAttributes(run);
      UIFont* run_font = (UIFont*)[run_attributes objectForKey:NSFontAttributeName];
      CTFontRef ct_text_font = (__bridge CTFontRef)run_font;
      bool font_has_colorized_glyphs =
          (CTFontGetSymbolicTraits(ct_text_font) & kCTFontTraitColorGlyphs) != 0;

      // Get the advances for the glyphs.
      std::vector<CGSize> advances(glyph_count);
      CTRunGetAdvances(run, CFRangeMake(0, glyph_count), advances.data());

      // Add the glyphs from this run to the result.
      for (CFIndex glyph_index = 0; glyph_index < glyph_count; glyph_index++) {
        // If a fallback font is required for this glyph, then we must report that so that it can be
        // used when drawing the glyph later.
        std::unique_ptr<FontHolder> fallback_font;
        if (run_font != text_font) {
          fallback_font = std::make_unique<IosFontHolder>(run_font);
        }

        // Check if this glyph is colored (e.g. emoji). Creating a path will fail for such glyphs.
        // (Unfortunately there doesn't seem to be a better API to determine this.)
        bool has_color = font_has_colorized_glyphs &&
                         (CGPathRef(CTFontCreatePathForGlyph(ct_text_font, glyphs.at(glyph_index),
                                                             nullptr)) == nullptr);

        result.push_back(ScopedCanvas::GlyphAdvance{
            .glyph = glyphs.at(glyph_index),
            .width = (static_cast<float>(advances.at(glyph_index).width) * pixels_per_dp_),
            .fallback_font = std::move(fallback_font),
            .is_emoji = has_color});
      }
    }
    CFRelease(ct_line);

    return result;
  }
}

ScopedCanvas::FontInfo IosPlatformCanvasSource::GetFontInfo(
    const ScopedCanvas::TextOptions& text_options) {
  UIFont* text_font = FontFromTextOptions(text_options, pixels_per_dp_);
  return ScopedCanvas::FontInfo{
      .ascent = static_cast<float>(-text_font.ascender) * pixels_per_dp_,
      .descent = static_cast<float>(-text_font.descender) * pixels_per_dp_,
      .leading = static_cast<float>(text_font.leading) * pixels_per_dp_,
      .line_spacing = static_cast<float>(text_font.lineHeight) * pixels_per_dp_};
}

std::unique_ptr<ScopedCanvas> IosPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size, ScopedCanvas::DrawMode draw_mode) {
  bool did_texture_change = false;
  if (!texture_) {
    texture_ = view.GetTextureFactory().CreateExternalTexture(pixel_size);
    did_texture_change = true;
  }

  return absl::make_unique<IosScopedCanvas>(*this, pixel_size, did_texture_change, draw_mode);
}

std::unique_ptr<ScopedCanvas> IosPlatformCanvasSource::StartDrawing(
    BaseView& view, uint2 pixel_size, ScopedCanvas::OnTextureChangedFn on_texture_changed_fn,
    ScopedCanvas::DrawMode draw_mode, SmallSourceLocation loc) {
  bool did_texture_change = false;
  if (!texture_) {
    texture_ = view.GetTextureFactory().CreateExternalTexture(pixel_size);
    did_texture_change = true;

    // On iOS the texture is only created once, so no need to give an opportunity for the caller to
    // clear references to the old texture.
    on_texture_changed_fn(texture_.Borrow(loc));
  }

  return absl::make_unique<IosScopedCanvas>(*this, pixel_size, did_texture_change, draw_mode);
}

Texture* IosPlatformCanvasSource::GetTexture() { return texture_.operator->(); }

IosPlatformCanvasSource::IosScopedCanvas::IosScopedCanvas(IosPlatformCanvasSource& source,
                                                          uint2 pixel_size, bool did_texture_change,
                                                          ScopedCanvas::DrawMode draw_mode)
    : source_(source), did_texture_change_(did_texture_change) {
  @autoreleasepool {
    PlatformPod& platform_pod = *source_.platform_pod_;

    // Either we haven't made the pixel buffer yet or the size has changed, so create it.
    if (!platform_pod.pixel_buffer || platform_pod.pixel_bounds.size.width != pixel_size.x ||
        platform_pod.pixel_bounds.size.height != pixel_size.y) {
      if (platform_pod.pixel_buffer) {
        CVPixelBufferRelease(platform_pod.pixel_buffer);
      }

      NSDictionary<NSString*, NSNumber*>* pixelBufferAttributes;
      if (BaseView::GetSharedEngine()->getBackend() == filament::Engine::Backend::METAL) {
        pixelBufferAttributes = @{(id)kCVPixelBufferMetalCompatibilityKey : @(YES)};
      } else {
        pixelBufferAttributes = @{(id)kCVPixelBufferOpenGLCompatibilityKey : @(YES)};
      }

      CVReturn cvret = CVPixelBufferCreate(
          kCFAllocatorDefault, pixel_size.x, pixel_size.y, kCVPixelFormatType_32BGRA,
          (__bridge CFDictionaryRef)pixelBufferAttributes, &platform_pod.pixel_buffer);
      if (cvret != kCVReturnSuccess) {
        IMP_LOG(imp::FATAL) << "Unable to create pixel buffer for CanvasSource: " << cvret;
      }

      platform_pod.pixel_bounds = CGRectMake(0, 0, pixel_size.x, pixel_size.y);
    }

    // Populate pixel data.
    CVReturn cvret = CVPixelBufferLockBaseAddress(platform_pod.pixel_buffer, 0);
    if (cvret != kCVReturnSuccess) {
      IMP_LOG(imp::FATAL) << "Unable to lock pixel buffer for CanvasSource: " << cvret;
    }
    // Set up a Core Graphics bitmap context pointing at the base address.
    device_colors_ = CGColorSpaceCreateDeviceRGB();
    assert(device_colors_ != nil);

    context_ = CGBitmapContextCreate(
        CVPixelBufferGetBaseAddress(platform_pod.pixel_buffer), pixel_size.x, pixel_size.y, 8,
        CVPixelBufferGetBytesPerRow(platform_pod.pixel_buffer), device_colors_,
        (CGBitmapInfo)kCGBitmapByteOrder32Little | kCGImageAlphaPremultipliedFirst);
    assert(context_ != nil);

    // Metal expects textures to be bottom-up, while Core Graphics expects bitmaps to be
    // top-down. Apply a linear transform to flip all draw calls upside down and scale
    // them appropriately.
    CGContextTranslateCTM(context_, 0, platform_pod.pixel_bounds.size.height);
    CGContextScaleCTM(context_, source_.pixels_per_dp_, -source_.pixels_per_dp_);

    // Push the Core Graphics onto the UIKit stack and actually draw the |view|.
    UIGraphicsPushContext(context_);

    if (did_texture_change_ || draw_mode == DrawMode::kClear) {
      CGContextClearRect(context_, platform_pod.pixel_bounds);
    }
  }
}

IosPlatformCanvasSource::IosScopedCanvas::~IosScopedCanvas() {
  PlatformPod& platform_pod = *source_.platform_pod_;

  UIGraphicsPopContext();

  // Clean up.
  CGContextRelease(context_);
  CGColorSpaceRelease(device_colors_);

  CVReturn cvret = CVPixelBufferUnlockBaseAddress(platform_pod.pixel_buffer, 0);
  if (cvret != kCVReturnSuccess) {
    IMP_LOG(imp::FATAL) << "Unable to unlock pixel buffer for CanvasSource: " << cvret;
  }

  // The Impress texture must be updated with the image from the pixel buffer.
  source_.texture_->GetTexture()->setExternalImage(*BaseView::GetSharedEngine(),
                                                   platform_pod.pixel_buffer);
}

Texture* IosPlatformCanvasSource::IosScopedCanvas::GetTexture() { return source_.GetTexture(); }

bool IosPlatformCanvasSource::IosScopedCanvas::DidTextureChange() const {
  return did_texture_change_;
}

void IosPlatformCanvasSource::IosScopedCanvas::DrawColor(float3 color) {
  DrawColor(float4(color, 1.0f));
}

void IosPlatformCanvasSource::IosScopedCanvas::DrawColor(float4 color) {
  CGContextSetFillColorWithColor(
      context_, [UIColor colorWithRed:color.x green:color.y blue:color.z alpha:color.w].CGColor);
  CGContextFillRect(context_, source_.platform_pod_->pixel_bounds);
}

void IosPlatformCanvasSource::IosScopedCanvas::DrawRoundedRect(float3 color, float2 corner_radius,
                                                               const Rect& rect) {
  DrawRoundedRect(float4(color, 1.0f), corner_radius, rect);
}

void IosPlatformCanvasSource::IosScopedCanvas::DrawRoundedRect(float4 color, float2 corner_radius,
                                                               const Rect& rect) {
  float2 extents = rect.half_extent * 2.0f / source_.pixels_per_dp_;
  float2 origins = rect.GetMin() / source_.pixels_per_dp_;
  float2 corners = corner_radius / source_.pixels_per_dp_;
  CGRect cg_rect = CGRectMake(origins.x, origins.y, extents.x, extents.y);
  CGPathRef path = CGPathCreateWithRoundedRect(cg_rect, corners.x, corners.y, NULL);
  CGContextSetFillColorWithColor(
      context_, [UIColor colorWithRed:color.x green:color.y blue:color.z alpha:color.w].CGColor);
  CGContextAddPath(context_, path);
  CGContextDrawPath(context_, kCGPathFill);
  CGPathRelease(path);
}

void IosPlatformCanvasSource::IosScopedCanvas::DrawText(absl::string_view text, float2 pos,
                                                        const TextOptions& text_options) {
  UIFont* text_font = FontFromTextOptions(text_options, source_.pixels_per_dp_);
  UIColor* text_color = [UIColor colorWithRed:text_options.color.x
                                        green:text_options.color.y
                                         blue:text_options.color.z
                                        alpha:text_options.color.w];

  UIColor* stroke_color = [UIColor colorWithRed:text_options.stroke_color.x
                                          green:text_options.stroke_color.y
                                           blue:text_options.stroke_color.z
                                          alpha:text_options.stroke_color.w];

  NSDictionary<NSAttributedStringKey, id>* text_attributes = @{
    NSFontAttributeName : text_font,
    NSForegroundColorAttributeName : text_color,
    NSStrokeWidthAttributeName : @(GetStrokeWidthAsPercentageOfFontSize(text_options)),
    NSStrokeColorAttributeName : stroke_color
  };
  // In order to make sure this can render the outstroke correctly, outstroke is drawn first and
  // then the fill.
  NSDictionary<NSAttributedStringKey, id>* fill_text_attributes = @{
    NSFontAttributeName : text_font,
    NSForegroundColorAttributeName : text_color,
  };

  NSString* string = @(std::string(text).c_str());
  NSAttributedString* attributed_string =
      [[NSAttributedString alloc] initWithString:string attributes:text_attributes];
  NSAttributedString* fill_attributed_string =
      [[NSAttributedString alloc] initWithString:string attributes:fill_text_attributes];

  CGSize text_size = [attributed_string size];

  CGFloat stroke_width = text_options.stroke_width_pixels / source_.pixels_per_dp_;
  // The stroke follows the path of the outside of the fill, half in and half out.
  // https://developer.apple.com/documentation/coregraphics/1454679-cgcontextstrokerectwithwidth?language=objc.
  CGFloat half_stroke_width = stroke_width / 2.0f;

  // Convert horizontal alignment to an offset. -drawWithRect interprets its X arg as the text's
  // desired 'left' position, so offset to the right (positive) for left-extent, and offset
  // to the left (negative) for right/right-extent.
  CGFloat horizontal_offset_dp = 0.0f;
  switch (text_options.horizontal_alignment) {
    case TextHorizontalAlignment::kLeftExtent: {
      horizontal_offset_dp = half_stroke_width;
      break;
    }
    case TextHorizontalAlignment::kLeft: {
      // Do nothing, leave at 0 offset.
      break;
    }
    case TextHorizontalAlignment::kCenter: {
      horizontal_offset_dp = -text_size.width / 2.0f;
      break;
    }
    case TextHorizontalAlignment::kRight: {
      horizontal_offset_dp = -text_size.width;
      break;
    }
    case TextHorizontalAlignment::kRightExtent: {
      horizontal_offset_dp = -text_size.width - half_stroke_width;
      break;
    }
  }

  // Convert vertical alignment to an offset. -drawWithRect interprets its Y arg as the text's
  // desired baseline position, so offset down (positive) for top/ascent, and offset up (negative)
  // for descent/bottom.
  //
  // Note that `ascender` and `descender` reflect the highest possible ascender/descender of ALL
  // glyphs in the font, while `text_size` reflects the specific text being drawn. If you
  // need to precisely position some text vertically, pre-measure it with GetTextMetrics() and
  // use the `origin.y` value to determine the actual distance from baseline.
  CGFloat vertical_offset_dp = 0.0f;
  switch (text_options.vertical_alignment) {
    case TextVerticalAlignment::kAtlas:
    case TextVerticalAlignment::kTopExtent: {
      vertical_offset_dp = text_font.ascender + half_stroke_width;
      break;
    }
    case TextVerticalAlignment::kAscent: {
      vertical_offset_dp = text_font.ascender;
      break;
    }
    case TextVerticalAlignment::kCenter: {
      float font_height = text_font.lineHeight - text_font.leading;
      float half_font_height = font_height * 0.5f;
      vertical_offset_dp = half_font_height + text_font.descender;
      break;
    }
    case TextVerticalAlignment::kBaseline: {
      // Do nothing, leave at 0 offset.
      break;
    }
    case TextVerticalAlignment::kDescent: {
      vertical_offset_dp = text_font.descender;  // Descender should already be negative.
      break;
    }
    case TextVerticalAlignment::kBottomExtent: {
      vertical_offset_dp = text_font.descender - half_stroke_width;
      break;
    }
  }

  // -drawWithRect: doesn't actually draw in the rect if you don't use the UsesLineFragmentOrigin
  // option -- it instead interprets the rect's origin as the left (non-extent) and baseline, so
  // most of the text goes _above_ the rect. See comments above for offset values.
  constexpr NSStringDrawingOptions kDrawingOptions = NSStringDrawingUsesDeviceMetrics;
  CGRect baseline_rect =
      CGRectMake((pos.x / source_.pixels_per_dp_) + horizontal_offset_dp,
                 (pos.y / source_.pixels_per_dp_) + vertical_offset_dp, CGFLOAT_MAX, CGFLOAT_MAX);

  [attributed_string drawWithRect:baseline_rect options:kDrawingOptions context:nil];
  [fill_attributed_string drawWithRect:baseline_rect options:kDrawingOptions context:nil];
}

void IosPlatformCanvasSource::IosScopedCanvas::DrawGlyph(GlyphId glyph, float2 pos,
                                                         const TextOptions& text_options) {
  @autoreleasepool {
    UIFont* text_font = FontFromTextOptions(text_options, source_.pixels_per_dp_);
    UIColor* text_color = [UIColor colorWithRed:text_options.color.x
                                          green:text_options.color.y
                                           blue:text_options.color.z
                                          alpha:text_options.color.w];

    UIColor* stroke_color = [UIColor colorWithRed:text_options.stroke_color.x
                                            green:text_options.stroke_color.y
                                             blue:text_options.stroke_color.z
                                            alpha:text_options.stroke_color.w];

    CGGlyph cg_glyph = static_cast<CGGlyph>(glyph);

    CGRect glyph_rect = CTFontGetBoundingRectsForGlyphs(
        (CTFontRef)text_font, kCTFontOrientationDefault, &cg_glyph, nullptr, 1);
    CGSize text_size = glyph_rect.size;

    // TODO: We should be aligning to the line height of the
    // specific font used to render this specific glyph, not the line height of
    // the default font. By doing it like below, we risk creating a "jagged
    // baseline" for any script which regularly exceeds the height of the
    // default font.
    float y_offset;
    if (glyph_rect.size.height > text_font.lineHeight) {
      y_offset = glyph_rect.origin.y + text_size.height;
    } else {
      y_offset = text_font.descender + text_font.lineHeight;
    }

    float half_stroke_width = text_options.stroke_width_pixels / 2.0f;

    // Glyph is positioned from the top-left, but we draw it upside-down, so
    // center it at the bottom-left instead.
    CGPoint position =
        CGPointMake((pos.x + half_stroke_width) / source_.pixels_per_dp_ - glyph_rect.origin.x,
                    (pos.y + half_stroke_width) / source_.pixels_per_dp_ + y_offset);

    CGContextSaveGState(context_);

    CGContextSetFillColorWithColor(context_, text_color.CGColor);

    // CTFontDrawGlyphs draws the glyph upside down so we need to flip it.
    CGContextScaleCTM(context_, 1.0, -1.0);
    position.y = -position.y;

    // Actually draws the glyph.
    if (text_options.stroke_width_pixels > 0.0f) {
      CGContextSetLineWidth(context_, text_options.stroke_width_pixels / source_.pixels_per_dp_);
      CGContextSetLineJoin(context_, kCGLineJoinRound);

      CGContextSetTextDrawingMode(context_, kCGTextStroke);
      CGContextSetStrokeColorWithColor(context_, stroke_color.CGColor);
      CTFontDrawGlyphs((CTFontRef)text_font, &cg_glyph, &position, 1, context_);

      CGContextSetTextDrawingMode(context_, kCGTextFillStroke);
      CGContextSetStrokeColorWithColor(context_, [UIColor clearColor].CGColor);
      CTFontDrawGlyphs((CTFontRef)text_font, &cg_glyph, &position, 1, context_);
    } else {
      CGContextSetTextDrawingMode(context_, kCGTextFill);
      CTFontDrawGlyphs((CTFontRef)text_font, &cg_glyph, &position, 1, context_);
    }

    CGContextRestoreGState(context_);
  }
}  // namespace imp

void IosPlatformCanvasSource::IosScopedCanvas::ClearRect(const Rect& rect) {
  float2 size = rect.half_extent * 2.0f / source_.pixels_per_dp_;
  float2 origins = rect.GetMin() / source_.pixels_per_dp_;
  CGRect cg_rect = CGRectMake(origins.x, origins.y, size.x, size.y);
  CGContextClearRect(context_, cg_rect);
}

}  // namespace imp
