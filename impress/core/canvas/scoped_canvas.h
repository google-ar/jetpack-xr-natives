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

#ifndef THIRD_PARTY_IMPRESS_CORE_CANVAS_SCOPED_CANVAS_H_
#define THIRD_PARTY_IMPRESS_CORE_CANVAS_SCOPED_CANVAS_H_

#include <sys/types.h>

#include <cstdint>
#include <memory>
#include <optional>

#include "absl/strings/string_view.h"
#include "core/canvas/constants.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/common/invocable.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/text/text_metrics.proto.h"

namespace imp {

// Interface for a canvas provided by CanvasSource::StartDrawing to draw to the
// texture provided by CanvasSource.
//
// The ScopedCanvas records Draw calls and then applies them to the
// CanvasSource's texture when it is destructed.
struct ScopedCanvas {
  // Features that aren't supported on all platforms.
  enum class Feature {
    // Indicates support for Glyph related methods.
    // Currently only supported on iOS.
    // TODO: Support glyphs on Android API 31+ and Desktop.
    kGlyphs,
    // Indicates support for keeping the contents of the texture between calls
    // to StartDrawing. Currently, only supported on iOS. Android is likely to
    // never support this since it disabled hardware acceleration when doing the
    // drawing.
    kKeepContents
  };

  // Determines the draw mode when calling CanvasSource::StartDrawing.
  enum class DrawMode { kClear, kKeepContents };

  // Represents a glyph within a font, which is a single representation of a
  // unicode character.
  //
  // There may be multiple glyphs for a single character, where a different
  // glyph is picked based on adjacent characters. Sometimes multiple characters
  // can get combined into a single glyph.
  // Each font may have its own range of GlyphIds, which means GlyphIds are
  // only guaranteed to be unique only within the space of a font.
  using GlyphId = int32_t;

  // Represents a group of glyphs that should be rendered together, such as in
  // the case of combining characters where all glyphs in a combined character
  // should be transformed or rendered together. Each platform may have its own
  // representation of GlyphGroups, which means each GlyphGroup is only
  // guaranteed to be unique within the space of the string used to request the
  // GlyphGroups for.
  using GlyphGroup = int32_t;

  // Result of CanvasSource::GetTextGlyphs which is used to determine what
  // sequence of glyphs should be used to display a particular text string.
  struct GlyphAdvance {
    // The glyph to draw.
    GlyphId glyph;
    // The horizontal width between this glyph and the next glyph in the
    // sequence for correct rendering. Dependent on adjacent glyphs, this is not
    // the same as the horizontal size of the glyph itself.
    float width;
    // If this glyph was not available in the font provided to GetTextGlyphs,
    // then this is the fallback font that must be used to draw the glyph.
    // This typically occurs for text that mixes CJK languages with latin
    // languages.
    std::unique_ptr<FontHolder> fallback_font;
    // If this glyph is an emoji it should be colorized.
    bool is_emoji;
  };

  // Controls how text will be drawn when DrawText is called.
  struct TextOptions {
    int size_pixels = 200;
    TextHorizontalAlignment horizontal_alignment =
        TextHorizontalAlignment::kLeft;
    TextVerticalAlignment vertical_alignment = TextVerticalAlignment::kBaseline;
    float4 color = float4(0.0f, 0.0f, 0.0f, 1.0f);

    // Optionally, specify which font to use for the text.
    // If unassigned, then a default font is used for the current platform.
    //
    // Android Default:
    //  Typically, Roboto
    //
    // iOS Default:
    //  System Font, typically SF Pro
    //
    // Desktop Default:
    //  On Mac: Helvetica
    //  On Linux: Serif
    //  On WASM: Google Sans
    FontHolder* font_holder = nullptr;

    // Optionally, specify the width of the stroke to outline the text.
    // If assigned, the text will be outlined using stroke_color.
    // While a value of zero could be passed to Android and Desktop for a
    // Hairline (single pixel), a value of zero on iOS would not render
    // outstroke. To avoid an api mismatch, the outstroke will not be rendered
    // if set to zero.
    float stroke_width_pixels = 0.0f;
    float4 stroke_color = float4(0.0f, 0.0f, 0.0f, 1.0f);

    // The amount of additional space as a percentage of the font size that
    // should be added to each glyph after layout. Default value of 0 adds no
    // additional tracking. Value of 0.5 adds half the font size of space
    // between each glyph, and -0.5 would subtract half the font size of space
    // between each glyph, leading to the glyphs overlapping each other.
    float text_tracking = 0.0f;

    // On wasm, enable or disable the extra work needing to be done to measure
    // the typographical width of text. On wasm, by default the typographical
    // width of the entire text is measured instead of breaking it up into
    // glyphs and summing each glyph's typographical width.
    bool should_measure_typographical_width = false;

    // On wasm, we use the render scale to draw the text at 2x the size and
    // super sample it if the window.devicePixelRatio property is less than 2.0.
    // Note that we only use the render scale for supersampling in the x
    // direction.
    // TODO (broken link) Support supersampling in the y direction as well.
    float2 render_scale = float2{1.0f};

    // If possible (i.e. if not on a path), disable splitting this text into
    // individual glyphs when rendering. This may improve legibility at the cost
    // of increased glyph atlas usage.
    bool force_non_separable = false;

    std::optional<TextAndFontMetrics> precomputed_metrics = std::nullopt;
  };

  // The inputs necessary to measure text.
  struct TextToMeasure {
    absl::string_view text;
    ScopedCanvas::TextOptions text_options;
  };

  // EXPERIMENTAL
  //
  // Callback passed into StartDrawing to access the Impress texture that the
  // canvas draws into that can be assigned to an Impress material.
  //
  // The texture persists until either the CanvasSource is destroyed or
  // the next time StartDrawing creates a new texture. In practice, StartDrawing
  // will create the texture the first time it's called and if the size changes
  // (on Desktop and Web). On iOS and Android, the texture is never re-created
  // after the first call.
  //
  // When writing a material to use the texture, it is recommended to specify
  // the sampler type as "{SAMPLER_EXTERNAL}". On Android, samplerExternal is
  // required, on Desktop, sampler2D is required. On iOS, either type works. By
  // specifying "{SAMPLER_EXTERNAL}", the sampler type is automatically changed
  // to the correct type based on the platform.
  //
  // *IMPORTANT* All BorrowedTexturePtr objects referencing an old texture from
  // a prior call to StartDrawing must be gone by the time the callback returns,
  // because at that point the old texture will be destroyed.
  using OnTextureChangedFn = Invocable<void(BorrowedTexturePtr)>;

  virtual ~ScopedCanvas() = default;

  // Provides access to the Impress texture that the canvas draws into that can
  // be assigned to an Impress material.
  //
  // The texture persists until either the CanvasSource is destroyed or
  // CanvasSource::StartDrawing returns a ScopedCanvas where
  // ScopedCanvas::DidTextureChange is true.
  //
  // When writing a material to use the texture, it is recommended to specify
  // the sampler type as "{SAMPLER_EXTERNAL}". On Android, samplerExternal is
  // required, on Desktop, sampler2D is required. On iOS, either type works. By
  // specifying "{SAMPLER_EXTERNAL}", the sampler type is automatically changed
  // to the correct type based on the platform.
  // TODO: Remove this API after fully migrating to
  // OwnedPtr/BorrowedPtr.
  virtual Texture* GetTexture() = 0;

  // Returns true if the texture was created or re-created when StartDrawing was
  // called.
  //
  // This will always be true the first time StartDrawing is called. On
  // subsequent times, it depends on the platform.
  //
  // On Desktop, the texture changes each time the size changes. Otherwise, it
  // never changes.
  // TODO: Remove this API after fully migrating to
  // OwnedPtr/BorrowedPtr.
  virtual bool DidTextureChange() const = 0;

  virtual void DrawColor(float3 color) = 0;
  virtual void DrawColor(float4 color) = 0;

  virtual void DrawRoundedRect(float3 color, float2 corner_radius,
                               const Rect& rect) = 0;
  virtual void DrawRoundedRect(float4 color, float2 corner_radius,
                               const Rect& rect) = 0;

  virtual void DrawText(absl::string_view text, float2 pos,
                        const TextOptions& text_options) = 0;

  // Draws the glyph from the top-left. Alignment options are ignored.
  virtual void DrawGlyph(GlyphId glyph, float2 pos,
                         const TextOptions& text_options) = 0;

  // Clears a region of the texture as specified by rect.
  // TODO (broken link) The web implementation of this API always calls ClearRect
  // before any other draw call, which is incorrect behavior. This means that if
  // a clear rect happens after a draw call, because the clear happens first,
  // the drawings that could have been cleared would not be cleared.
  virtual void ClearRect(const Rect& rect) = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_CANVAS_CANVAS_H_
