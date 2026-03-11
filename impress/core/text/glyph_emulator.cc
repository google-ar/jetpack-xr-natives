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

#include "core/text/glyph_emulator.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/variant.h"
#include "unicode/umachine.h"
#include "unicode/utf8.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/constants.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/canvas/fonts/system_font_provider.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/context.h"
#include "core/common/hash.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/text/text_helpers.h"
#include "core/text/text_metrics.proto.h"

namespace imp {
namespace {

#if IMP_RUNTIME(DEV)
bool RectContainsPoint(const Rect& rect, const float2& point) {
  return ((std::abs(rect.center.x - point.x) <= (rect.half_extent.x)) &&
          (std::abs(rect.center.y - point.y) <= (rect.half_extent.y)));
}
#endif

#if IMP_PLATFORM(WASM)
constexpr float kSuperSampleThreshold = 2.0f;
#endif

const ScopedCanvas::TextOptions kTextOptions{
    .size_pixels = GlyphEmulator::kDefaultFontSizePixels,
    .horizontal_alignment = TextHorizontalAlignment::kLeftExtent,
    .vertical_alignment = TextVerticalAlignment::kAtlas,
    .color = float4(0.0f, 0.0f, 0.0f, 1.0f),
    .stroke_width_pixels = 0.0f,
    .stroke_color = float4(0.0f, 0.0f, 0.0f, 1.0f),
    .text_tracking = 0.0f,
    .should_measure_typographical_width = false,
};

// For canvas sources that do not natively support glyphs, GlyphEmulator will
// approximate the computation of glyphs from a string by doing the following:
//
// A. Breaking the string into chunks of separable and non-separable text
//    chunks.
// B. Measure the advance width of the glyphs in each chunk. Separable chunks
//    are subdivided into glyphs along codepoint boundaries. Non-separable
//    chunks are treated as one glyph.
// C. Flatten the computed advance widths from (B) into a vector of
//    GlyphEmulator::Glyphs.
//
// GetChunks in text_helpers.h is the step (A) above.
// This is step (C).
//
// Note: there may be fewer Glyphs added than there are widths if any widths are
// 0. In that case the character is merged with the previous character to create
// one Glyphs.
void GetGlyphsForChunk(const Chunk& chunk,
                       const std::vector<float>& advance_widths,
                       const ScopedCanvas::TextOptions& canvas_options,
                       std::vector<GlyphEmulator::Glyph>& out_glyphs) {
  if (!chunk.is_separable) {
    out_glyphs.push_back(GlyphEmulator::Glyph{
        .glyph = std::string(chunk.chunk_text),
        .advance_width = advance_widths[0] * canvas_options.render_scale.x,
        .is_emoji = canvas_options.force_non_separable ||
                    ContainsEmoji(chunk.chunk_text),
        .contains_non_separable_script =
            canvas_options.force_non_separable ||
            ContainsNonSeparableScript(chunk.chunk_text)});
    return;
  }

  // TODO: If this code path triggers, its probably because you're
  // on Android, didn't compile in glyph support, and we haven't fixed the
  // behavior of GetTextWidths() yet. See the android_platform_canvas_source.h
  // for more details.
  if (chunk.codepoint_count != advance_widths.size()) {
    IMP_LOG(imp::FATAL) << "GlyphEmulator is unable to generate glyphs for text "
               << chunk.chunk_text << ". Detected " << chunk.codepoint_count
               << " unicode characters but only " << advance_widths.size()
               << " glyph advance widths.";
  }

  const char* s = chunk.chunk_text.data();
  int32_t length = chunk.chunk_text.length();
  int32_t start = 0;
  int32_t si = 0;
  int i = 0;
  while (si < length) {
    UChar32 c;
    U8_NEXT(s, si, length, c);

    float advance_width = advance_widths[i];
    bool is_emoji = IsEmoji(c);

    // Essential ligatures like 'ß' will already be encoded as a single
    // character in the UTF8 text. However, non-essential ones *might* be turned
    // into a ligature by the font. For instance, the adjacent characters "fi"
    // may be turned into the ligature 'ﬁ'. However, not all fonts have glyphs
    // for the non-essential ligatures. For example, Roboto does, but Google
    // Sans doesn't.
    //
    // Below, we're able to detect this case by checking for when the next
    // character has an advance width of zero, in which case we can combine
    // the characters together and treat them as one glyph to draw the
    // ligature.
    //
    // This still doesn't work for languages like arabic since the
    // alternative glyphs don't combine adjacent characters.
    //
    // Read more:
    // (broken link)
    // (broken link)
    while (i < advance_widths.size() - 1 && advance_widths[i + 1] == 0.0f) {
      U8_FWD_1(s, si, length);
      i++;
    }

    std::string utf8_char = chunk.chunk_text.substr(start, si - start);
    start = si;

    out_glyphs.push_back(GlyphEmulator::Glyph{
        .glyph = std::move(utf8_char),
        .advance_width = advance_width * canvas_options.render_scale.x,
        .is_emoji = is_emoji});
    i++;
  }
}

}  // namespace

GlyphEmulator::GlyphEmulator(Context context) : context_(context) {}

void GlyphEmulator::AddFont(absl::string_view font_name,
                            std::unique_ptr<FontHolder> font_holder) {
  absl::MutexLock lock(fonts_mutex_);
  fonts_.emplace(font_name, std::move(font_holder));
}

Future<absl::Status> GlyphEmulator::PrepareFont(
    absl::string_view text, const ScopedCanvas::TextOptions& options,
    AsyncCanvasSource& canvas_source) {
  return canvas_source.PrepareFont(text, options);
}

Future<std::vector<ScopedCanvas::GlyphGroup>>
GlyphEmulator::GetCombinedCharacterGroups(
    absl::string_view text, const ScopedCanvas::TextOptions& options,
    AsyncCanvasSource& canvas_source) {
  if (canvas_source.IsFeatureSupported(ScopedCanvas::Feature::kGlyphs)) {
    return canvas_source.GetCombinedCharacterGroups(text, options);
  } else {
    return Future<std::vector<ScopedCanvas::GlyphGroup>>(
        std::vector<ScopedCanvas::GlyphGroup>());
  }
}

Future<TextMetrics> GlyphEmulator::GetTextMetrics(
    absl::string_view text, const ScopedCanvas::TextOptions& options,
    AsyncCanvasSource& canvas_source) {
  return canvas_source.PrepareFont(text, options)
      .Then(
          [&canvas_source, text = std::string(text), options]() {
            return canvas_source.MeasureGlyph(
                AsyncCanvasSource::GlyphToMeasure({text}), options);
          },
          Executor::Type::kCurrent);
}

Future<FontInfo> GlyphEmulator::GetFontInfo(
    const ScopedCanvas::TextOptions& options,
    AsyncCanvasSource& canvas_source) {
  return canvas_source.PrepareFont(" ", options)
      .Then([options,
             &canvas_source]() { return canvas_source.GetFontInfo(options); },
            Executor::Type::kCurrent);
}

Future<std::vector<TextAndFontMetrics>> GlyphEmulator::GetFontAndTextMetrics(
    std::vector<ScopedCanvas::TextToMeasure> texts,
    AsyncCanvasSource& canvas_source) {
  std::vector<Future<absl::Status>> prepare_futures;
  prepare_futures.reserve(texts.size());
  for (int i = 0; i < texts.size(); ++i) {
    prepare_futures.push_back(
        canvas_source.PrepareFont(texts[i].text, texts[i].text_options));
  }
  return Future<absl::Status>::CombineList(prepare_futures)
      .Then(
          [&canvas_source, texts = std::move(texts)]()
              -> Future<std::vector<TextAndFontMetrics>> {
            return canvas_source.GetFontAndTextMetrics(texts);
          },
          Executor::Type::kCurrent);
}

Future<std::unique_ptr<std::vector<GlyphEmulator::Glyph>>>
GlyphEmulator::GetGlyphs(absl::string_view text,
                         const ScopedCanvas::TextOptions& options,
                         AsyncCanvasSource& canvas_source) {
  if (text.empty()) {
    return Future<std::unique_ptr<std::vector<Glyph>>>(
        std::make_unique<std::vector<Glyph>>());
  }

  Future<absl::Status> prepare_font_future =
      options.precomputed_metrics.has_value()
          ? Future<absl::Status>(absl::OkStatus())
          : canvas_source.PrepareFont(text, options);

  bool reverse_order = ContainsRtl(text);

  Future<std::unique_ptr<std::vector<Glyph>>> glyphs_future;
  glyphs_future =
      prepare_font_future
          .Then(
              [this, &canvas_source, text = std::string(text), options]() {
                return BreakIntoGlyphs(text, options, canvas_source);
              },
              Executor::Type::kCurrent)
          .Then(
              [this, reverse_order, &canvas_source,
               options](std::unique_ptr<std::vector<Glyph>> glyphs) mutable {
                if (!options.precomputed_metrics.has_value() ||
                    options.precomputed_metrics->glyph_metrics().size() !=
                        glyphs->size()) {
                  if (options.precomputed_metrics.has_value()) {
                    IMP_LOG(imp::ERROR)
                        << "Number of precomputed glyph metrics does not "
                           "match number of glyphs";
                  }
                  return MeasureGlyphs(std::move(glyphs), options,
                                       canvas_source);
                }
                int size = options.precomputed_metrics->glyph_metrics().size();
                for (int i = 0; i < size; i++) {
                  Glyph& glyph = (*glyphs)[reverse_order ? size - 1 - i : i];
                  glyph.metrics =
                      options.precomputed_metrics->glyph_metrics()[i];
                }
                return Future<std::unique_ptr<std::vector<Glyph>>>(
                    std::move(glyphs));
              },
              Executor::Type::kCurrent);

  if (options.render_scale.x > 1.0 || options.render_scale.y > 1.0) {
    glyphs_future =
        glyphs_future.Then([render_scale = options.render_scale](
                               std::unique_ptr<std::vector<Glyph>> glyphs) {
          for (auto& glyph : *glyphs) {
            glyph.metrics.set_origin_x(glyph.metrics.origin_x() *
                                       render_scale.x);
            glyph.metrics.set_origin_y(glyph.metrics.origin_y() *
                                       render_scale.y);
            glyph.metrics.set_size_x(glyph.metrics.size_x() * render_scale.x);
            glyph.metrics.set_size_y(glyph.metrics.size_y() * render_scale.y);
          }
          return glyphs;
        });
  }

  return glyphs_future;
}

Future<std::unique_ptr<std::vector<GlyphEmulator::Glyph>>>
GlyphEmulator::BreakIntoGlyphs(absl::string_view text,
                               const ScopedCanvas::TextOptions& canvas_options,
                               AsyncCanvasSource& canvas_source) {
  // If CanvasSource supports glyphs on this platform / OS then use it.
  // Otherwise, fall back to representing glyphs as characters.
  //
  // There isn't always a 1:1 mapping between a character in a string and a
  // glyph in a font for rendering, so the fallback won't correctly handle all
  // cases of ligatures, contextual alternatives, RTL, and BiDi text.
  //
  // TODO: Detect which fallback cases won't work based on the
  // unicode characters in the string and fallback to drawing the entire
  // string in the atlas instead of individual glyphs.
  if (canvas_source.IsFeatureSupported(ScopedCanvas::Feature::kGlyphs)) {
    absl::string_view font_holder_name = "";
    if (canvas_options.font_holder) {
      font_holder_name = canvas_options.font_holder->GetFontName();
    }
    return canvas_source.GetTextGlyphs(text, canvas_options)
        .Then(
            [font_holder_name = std::string(font_holder_name)](
                std::unique_ptr<std::vector<ScopedCanvas::GlyphAdvance>>
                    canvas_glyph_advances) {
              auto glyphs = std::make_unique<std::vector<Glyph>>();
              glyphs->reserve(canvas_glyph_advances->size());
              for (ScopedCanvas::GlyphAdvance& canvas_glyph_advance :
                   *canvas_glyph_advances) {
                absl::string_view font = "";
                if (canvas_glyph_advance.fallback_font) {
                  font = canvas_glyph_advance.fallback_font->GetFontName();
                } else if (!font_holder_name.empty()) {
                  font = font_holder_name;
                }
                glyphs->push_back(GlyphEmulator::Glyph{
                    .glyph = GlyphKey{.glyph_id = canvas_glyph_advance.glyph,
                                      .font_id = Hash(font)},
                    .advance_width = canvas_glyph_advance.width,
                    .is_emoji = canvas_glyph_advance.is_emoji,
                    .fallback_font =
                        std::move(canvas_glyph_advance.fallback_font),
                });
              }
              return glyphs;
            },
            Executor::Type::kCurrent);
  }

  // A string or string_view is really just an array of bytes. A byte is only
  // big enough to represent ASCII characters. Standardized across google,
  // strings in C++ and protos are encoded as UTF8 to support localization,
  // ligatures, and accented characters. In UTF8, a character is a variable
  // number of bytes (1-4). If it's 1 byte, it's ASCII.
  //
  // Learn more at (broken link).

  // Determines how the text needs to be laid out by getting the width of each
  // character within the string. This isn't the same as the width of the
  // character itself, since the layout width is impacted by adjacent
  // characters.
  //
  // Note, using characters as glyphs is NOT correct in all cases. For
  // example, in arabic the correct glyph for a character is determined by
  // adjacent characters. The best way to handle this is to correctly convert
  // the text string into glyphs from the font, and then draw each individual
  // glyph as a separate entry in the atlas.
  //
  // As an intermediate solution, for texts that are rendered RTL or contain
  // non-separable texts, we will try to separate them on separable characters
  // and render each portion as a single glyph. This means that if the entire
  // text is non-separable, then we will render the entire text as a single
  // chunk.
  //
  // However, The Android PositionedGlyphs API that is required to implement
  // this on Android isn't available until API level 31 which we cannot rely
  // on.
  //
  // TODO: Add support for atlasing individual glyphs for text
  // with accents/ligatures when running on Desktop.
  std::vector<Chunk> chunks =
      GetChunks(text, canvas_options.force_non_separable);
  Future<std::vector<std::vector<float>>> widths;

  if (canvas_options.precomputed_metrics.has_value()) {
    // We need to check if the number of precomputed metrics matches the total
    // glyph count summed across all chunks. If it doesn't match then we would
    // have an array out of bounds error.
    size_t chunk_glyph_count = 0;
    for (const Chunk& chunk : chunks) {
      chunk_glyph_count += chunk.codepoint_count;
    }
    if (canvas_options.precomputed_metrics->glyph_metrics().size() ==
        chunk_glyph_count) {
      std::vector<std::vector<float>> chunk_widths;
      int idx = 0;
      for (int chunk_idx = 0; chunk_idx < chunks.size(); ++chunk_idx) {
        const Chunk& chunk = chunks[chunk_idx];
        chunk_widths.emplace_back(chunk.codepoint_count);
        for (int i = 0; i < chunk.codepoint_count; ++i) {
          chunk_widths[chunk_idx][i] =
              canvas_options.precomputed_metrics->glyph_metrics()[idx++]
                  .typographical_width();
        }
      }
      widths.Return(chunk_widths);
    } else {
      IMP_LOG(imp::ERROR) << "Number of precomputed glyph metrics does not match total "
                     "number of glyphs in chunks. Expected "
                  << chunk_glyph_count << " but got "
                  << canvas_options.precomputed_metrics->glyph_metrics().size();
      widths = canvas_source.GetTextWidths(chunks, canvas_options);
    }
  } else {
    widths = canvas_source.GetTextWidths(chunks, canvas_options);
  }
  bool contains_rtl = ContainsRtl(text);
  return widths.Then(
      [chunks = std::move(chunks), contains_rtl,
       canvas_options](std::vector<std::vector<float>> widths) {
        auto glyphs = std::make_unique<std::vector<Glyph>>();
        for (int i = 0; i < chunks.size(); i++) {
          GetGlyphsForChunk(chunks[i], widths[i], canvas_options, *glyphs);
        }

        // Because we tokenize the text and try to split it on separable
        // delimiters, if the text was rtl then we need to reverse the order
        // of the tokens if it is rtl.
        if (contains_rtl) {
          absl::c_reverse(*glyphs);
        }
        return glyphs;
      },
      Executor::Type::kCurrent);
}

Future<std::unique_ptr<std::vector<GlyphEmulator::Glyph>>>
GlyphEmulator::MeasureGlyphs(std::unique_ptr<std::vector<Glyph>> glyphs,
                             ScopedCanvas::TextOptions canvas_options,
                             AsyncCanvasSource& canvas_source) {
  std::vector<AsyncCanvasSource::GlyphToMeasure> glyphs_to_measure;
  glyphs_to_measure.reserve(glyphs->size());
  absl::c_transform(
      *glyphs, std::back_inserter(glyphs_to_measure),
      [](const Glyph& glyph) -> AsyncCanvasSource::GlyphToMeasure {
        std::variant<absl::string_view, ScopedCanvas::GlyphId> glyph_key;
        if (absl::holds_alternative<GlyphKey>(glyph.glyph)) {
          glyph_key = absl::get<GlyphKey>(glyph.glyph).glyph_id;
        } else {
          glyph_key = absl::get<std::string>(glyph.glyph);
        }
        AsyncCanvasSource::GlyphToMeasure glyph_to_measure(
            {.glyph = glyph_key});
        if (glyph.fallback_font) {
          glyph_to_measure.font_override = glyph.fallback_font.get();
        }
        return glyph_to_measure;
      });

  return canvas_source.MeasureGlyphs(glyphs_to_measure, canvas_options)
      .Then(
          [glyphs = std::move(glyphs)](
              std::vector<TextMetrics> text_metrics) mutable {
            Glyph* glyph_data = glyphs->data();
            for (int i = 0; i < text_metrics.size(); i++) {
              glyph_data[i].metrics = text_metrics[i];
            }
            return std::move(glyphs);
          },
          Executor::Type::kCurrent);
};

absl::StatusOr<ScopedCanvas::TextOptions>
GlyphEmulator::CanvasOptionsFromGlyphEmulatorOptions(
    const TextOptions& options, std::optional<float2> subpixel_render_ratio) {
  FontHolder* font_holder = nullptr;

  ScopedCanvas::TextOptions canvas_options = kTextOptions;
  if (std::holds_alternative<std::string>(options.font_params)) {
    absl::string_view font_name = std::get<std::string>(options.font_params);
    if (!font_name.empty()) {
      absl::MutexLock lock(fonts_mutex_);
      auto font_itr = fonts_.find(font_name);
      if (font_itr != fonts_.end()) {
        font_holder = font_itr.value().get();
      } else {
        return absl::InvalidArgumentError(absl::StrFormat(
            "Failed to get glyphs. Missing font %s", font_name));
      }
    }
  } else if (std::holds_alternative<SystemFontParams>(options.font_params)) {
    const SystemFontParams font_params =
        std::get<SystemFontParams>(options.font_params);
    absl::MutexLock lock(system_fonts_mutex_);
    auto system_font_iter = system_fonts_.find(font_params);
    if (system_font_iter != system_fonts_.end()) {
      font_holder = system_font_iter->second.get();
    } else {
      font_holder =
          system_fonts_
              .emplace(font_params, LoadSystemFont(context_, font_params))
              .first->second.get();
    }
  }

  // Convert the text options into the canvas options actually used for
  // drawing.
  canvas_options.stroke_width_pixels = options.stroke_width_pixels;
  if (options.font_size_pixels) {
    canvas_options.size_pixels = *options.font_size_pixels;
  }
  if (font_holder != nullptr) {
    canvas_options.font_holder = font_holder;
  }
  canvas_options.color = options.color;
  canvas_options.stroke_color = options.stroke_color;

  canvas_options.text_tracking = options.text_tracking;
  canvas_options.should_measure_typographical_width =
      options.should_measure_typographical_width;
  canvas_options.render_scale = subpixel_render_ratio.value_or(float2{1.0f});
  canvas_options.force_non_separable = options.force_non_separable;
  canvas_options.precomputed_metrics = options.precomputed_metrics;
  return canvas_options;
}

GlyphEmulator::SuperSampleInfo GlyphEmulator::GetSuperSampleInfo(
    float2 physical_pixel_ratio, bool force_off) {
#if IMP_PLATFORM(WASM)
  bool should_super_sample =
      !force_off && physical_pixel_ratio.x < kSuperSampleThreshold;
  float2 subpixel_render_ratio =
      should_super_sample ? float2{kSuperSampleThreshold, 1.0f} : float2{1.0f};
  return SuperSampleInfo{
      .should_super_sample = should_super_sample,
      .subpixel_render_ratio = subpixel_render_ratio,
  };
#else
  return SuperSampleInfo{
      .should_super_sample = false,
      .subpixel_render_ratio = float2{1.0f},
  };
#endif
}

void GlyphEmulator::DrawGlyph(ScopedCanvas& canvas,
                              const GlyphEmulator::GlyphKeyOrGlyphString& glyph,
                              float2 position,
                              const ScopedCanvas::TextOptions& canvas_options) {
  if (absl::holds_alternative<GlyphEmulator::GlyphKey>(glyph)) {
    canvas.DrawGlyph(std::get<GlyphEmulator::GlyphKey>(glyph).glyph_id,
                     position, canvas_options);
  } else {
    canvas.DrawText(std::get<std::string>(glyph), position, canvas_options);
  }
}

}  // namespace imp
