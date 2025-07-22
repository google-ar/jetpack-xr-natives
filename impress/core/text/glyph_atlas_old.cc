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

#include "core/text/glyph_atlas_old.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/base/nullability.h"
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
#include "core/atlas/atlas_packer.h"
#include "core/atlas/shelf_atlas_packer.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_canvas_source_factory.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/constants.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/canvas/fonts/system_font_provider.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/hash.h"
#include "core/common/ref_counter.h"
#include "core/config.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/render/texture.h"
#include "core/text/glyph_atlas.h"
#include "core/text/glyph_emulator.h"
#include "core/text/text_helpers.h"
#include "core/view/base_view.h"
#include "core/view/utils/device.h"
#include "core/view/view_events.h"
#if IMP_PLATFORM(WASM)
#include "emscripten/emscripten.h"
#endif

#if IMP_RUNTIME(DEV)
#include "core/editor/editor.h"
#include "core/editor/layout/editor_panel_ids.h"
#include "core/editor/widget_ui_system.h"
#include "core/text/editor/glyph_atlas_visualizer.h"
#endif

namespace imp {
namespace {

#if IMP_RUNTIME(DEV)
bool RectContainsPoint(const Rect& rect, const float2& point) {
  return ((std::abs(rect.center.x - point.x) <= (rect.half_extent.x)) &&
          (std::abs(rect.center.y - point.y) <= (rect.half_extent.y)));
}
#endif

constexpr uint8_t kPadding = 8;
constexpr uint8_t kHalfPadding = kPadding / 2;
constexpr float4 kStrokeIdentifierColor = float4(1.0f, 0.0f, 0.0f, 1.0f);
constexpr float4 kFillIdentifierColor = float4(1.0f, 1.0f, 0.0f, 1.0f);
#if IMP_PLATFORM(WASM)
constexpr float kSuperSampleThreshold = 2.0f;
#endif

constexpr ScopedCanvas::TextOptions kTextOptions{
    .size_pixels = GlyphEmulator::kDefaultFontSizePixels,
    .horizontal_alignment = TextHorizontalAlignment::kLeftExtent,
    .vertical_alignment = TextVerticalAlignment::kAtlas,
    .color = float4(0.0f, 0.0f, 0.0f, 1.0f),
    .stroke_width_pixels = 0.0f,
    .stroke_color = float4(0.0f, 0.0f, 0.0f, 1.0f),
    .text_tracking = 0.0f,
    .should_measure_typographical_width = false,
};

float2 GetTextureSize(GlyphAtlas::TextureSize texture_size) {
  switch (texture_size) {
    case GlyphAtlas::TextureSize::k2048:
      return {2048.0f, 2048.0f};
    case GlyphAtlas::TextureSize::k2048_4096:
      return {2048.0f, 4096.0f};
    case GlyphAtlas::TextureSize::k4096:
      return {4096.0f, 4096.0f};
  }
}

}  // namespace

AsyncScopedCanvas* GlyphAtlasOld::GetOrStartDrawing(
    ScopedCanvas::DrawMode draw_mode) {
  if (Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "GetOrStartDrawing makes calls to Filament and may only be "
                  "called on the foreground executor.";
  }

  if (canvas_) {
    return canvas_.get();
  }

  canvas_ = canvas_source_->StartDrawing(view_, atlas_texture_size_, draw_mode);
  if (canvas_->DidTextureChange()) {
    texture_ = canvas_->GetTexture();
  }

  return canvas_.get();
}

GlyphAtlasOld::GlyphAtlasOld(BaseView& view, Config config)
    : GlyphAtlasOld::GlyphAtlasOld(
          view, AsyncCanvasSourceFactory::Create(view.GetContext()), config) {}

GlyphAtlasOld::GlyphAtlasOld(BaseView& view,
                             std::unique_ptr<AsyncCanvasSource> canvas_source,
                             Config config)
    : view_(view),
      atlas_texture_size_(GetTextureSize(config.texture_size)),
      canvas_source_(std::move(canvas_source)),
      atlas_packer_(atlas_texture_size_) {
  view.GetDispatcher().Connect(
      [this](const imp::ViewPostFrameUpdateEvent& event) mutable {
        EndFrame();
      },
      this);
  if (view.GetDevice().IsPhysicalPixelRatioAvailable()) {
    physical_pixel_ratio_available_.Return(absl::OkStatus());
  } else {
    view_.GetDispatcher().Connect(
        [this](const ViewSizeChangedEvent& event) {
          event.Disconnect();
          physical_pixel_ratio_available_.Return(absl::OkStatus());
        },
        this);
  }

#if IMP_RUNTIME(DEV)
  if (auto editor = view.GetRegistry().Get<editor::Editor>(); editor.ok()) {
    editor->get().GetWidgetUiSystem().AddWidget<editor::GlyphAtlasVisualizer>(
        editor::WidgetLayoutInfo(editor::panel_ids::kTabBar), view,
        editor::GlyphAtlasVisualizer::AtlasDataProvider{
            .get_texture_func = [this]() { return GetTexture(); },
            .get_glyph_info_func =
                [this](float2 uv) { return GetGlyphAtlasInfoAt(uv); },
            .get_utilization_func = [this]() { return GetAtlasUtilization(); },
            .get_all_glyph_info_func = [this]() { return GetAllGlyphInfo(); }});
  }
#endif
}

GlyphAtlasOld::~GlyphAtlasOld() {
  // Wait until destruction-blocking blocks across all threads have been
  // completed.
  absl::MutexLock lock(&completion_gate_->mutex_);

  completion_gate_->complete_ = true;

  {
    absl::MutexLock lock(&canvas_mutex_);
    canvas_.reset();
  }
}

void GlyphAtlasOld::EndFrame() {
  canvas_mutex_.Lock();
  if (texture_status_ == TextureStatus::kStable ||
      texture_status_ == TextureStatus::kPreparingToUpdateTexture || !canvas_) {
    canvas_mutex_.Unlock();
    return;
  }

  // There are new glyphs that are ready to be drawn. For clients that do
  // not support partial updates to the texture, the entire atlas must be
  // redrawn. This is because on Android you can't do partial updates to the
  // texture when drawing it with the GPU.
  // TODO: Investigate performance characteristics for how the
  // atlas is drawn on each platform.
  // TODO: Do async drawing of glyphs for platforms that don't
  // support kKeepContents
  if (texture_status_ == TextureStatus::kHasNewGlyphs &&
      !canvas_source_->IsFeatureSupported(
          ScopedCanvas::Feature::kKeepContents)) {
    // Unlock the mutex for DrawAllGlyphsToCanvas to hold.
    canvas_mutex_.Unlock();
    DrawAllGlyphsToCanvas();
    canvas_mutex_.Lock();
    texture_status_ = TextureStatus::kReadyToApplyDrawCommands;
  }

  if (texture_status_ == TextureStatus::kReadyToApplyDrawCommands) {
    PrepareToUpdateTexture();
  }

  if (texture_status_ == TextureStatus::kReadyToRelease) {
    texture_status_ = TextureStatus::kStable;
    for (const auto& future : texture_update_futures_) {
      future.Return(absl::OkStatus());
    }
    canvas_.reset();
  }
  canvas_mutex_.Unlock();
}

void GlyphAtlasOld::AddFont(absl::string_view font_name,
                            std::unique_ptr<FontHolder> font_holder) {
  {
    absl::MutexLock lock(&fonts_mutex_);
    fonts_.emplace(font_name, std::move(font_holder));
  }
}

Future<absl::Status> GlyphAtlasOld::PrepareFont(absl::string_view text,
                                                const TextOptions& options) {
  return CanvasOptionsFromGlyphAtlasOptions(options).Then(
      [this, text = std::string(text)](
          absl::StatusOr<ScopedCanvas::TextOptions> canvas_options) {
        if (!canvas_options.ok()) {
          return Future<absl::Status>(canvas_options.status());
        }

        return canvas_source_->PrepareFont(text, *canvas_options);
      },
      Executor::Type::kCurrent);
}

Future<std::vector<ScopedCanvas::GlyphGroup>>
GlyphAtlasOld::GetCombinedCharacterGroups(absl::string_view text,
                                          const TextOptions& options) {
  return CanvasOptionsFromGlyphAtlasOptions(options).Then(
      [this, text = std::string(text)](
          absl::StatusOr<ScopedCanvas::TextOptions> canvas_options) {
        if (!canvas_options.ok()) {
          return Future<std::vector<ScopedCanvas::GlyphGroup>>(
              canvas_options.status());
        }

        if (canvas_source_->IsFeatureSupported(
                ScopedCanvas::Feature::kGlyphs)) {
          return canvas_source_->GetCombinedCharacterGroups(text,
                                                            *canvas_options);
        } else {
          return Future<std::vector<ScopedCanvas::GlyphGroup>>(
              std::vector<ScopedCanvas::GlyphGroup>());
        }
      },
      Executor::Type::kCurrent);
}

Future<ScopedCanvas::TextMetrics> GlyphAtlasOld::GetTextMetrics(
    absl::string_view text, const TextOptions& options) {
  return CanvasOptionsFromGlyphAtlasOptions(options).Then(
      [this, text = std::string(text)](
          absl::StatusOr<ScopedCanvas::TextOptions> canvas_options) {
        if (!canvas_options.ok()) {
          return Future<ScopedCanvas::TextMetrics>(canvas_options.status());
        }

        return canvas_source_->PrepareFont(text, *canvas_options)
            .Then(
                [this, text = std::string(text),
                 canvas_options = *canvas_options]() {
                  AsyncCanvasSource::GlyphToMeasure glyph_to_measure(
                      {.glyph = text});
                  return canvas_source_
                      ->MeasureGlyph(glyph_to_measure, canvas_options)
                      .Then(
                          [this](ScopedCanvas::TextMetrics metrics) {
                            metrics.size /= GetSubpixelRenderRatio();
                            metrics.typographical_width /=
                                GetSubpixelRenderRatio().x;
                            metrics.origin /= GetSubpixelRenderRatio();
                            return metrics;
                          },
                          Executor::Type::kCurrent);
                },
                Executor::Type::kCurrent);
      },
      Executor::Type::kCurrent);
}

Future<ScopedCanvas::FontInfo> GlyphAtlasOld::GetFontInfo(
    const TextOptions& options) {
  return CanvasOptionsFromGlyphAtlasOptions(options).Then(
      [this](absl::StatusOr<ScopedCanvas::TextOptions> canvas_options) {
        if (!canvas_options.ok()) {
          return Future<ScopedCanvas::FontInfo>(canvas_options.status());
        }
        return canvas_source_->PrepareFont(" ", *canvas_options)
            .Then(
                [this, canvas_options = *canvas_options]() {
                  return canvas_source_->GetFontInfo(canvas_options);
                },
                Executor::Type::kCurrent);
      },
      Executor::Type::kCurrent);
}

Future<std::vector<GlyphAtlas::Glyph>> GlyphAtlasOld::GetGlyphs(
    absl::string_view text, const GlyphAtlas::TextOptions& options) {
  // GetGlyphs must run on the foreground Executor since we make calls to
  // Filament through GetOrStartDrawing().
  if (Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    return Future<std::vector<GlyphAtlas::Glyph>>::Schedule(
        [this, text = std::string(text),
         options = GlyphAtlas::TextOptions(options)]() {
          return GetGlyphs(text, options);
        });
  }

  if (text.empty()) {
    return Future<std::vector<Glyph>>(std::vector<Glyph>());
  }

  // Convert the text options into the canvas options actually used for drawing.
  return CanvasOptionsFromGlyphAtlasOptions(options).Then(
      [this, text = std::string(text),
       options](absl::StatusOr<ScopedCanvas::TextOptions> canvas_options) {
        if (!canvas_options.ok()) {
          return Future<std::vector<Glyph>>(canvas_options.status());
        }

        return PrepareFont(text, options)
            .Then([this, text = std::string(text),
                   canvas_options = *canvas_options]() {
              return BreakIntoGlyphs(text, canvas_options)
                  .Then([this, canvas_options](
                            std::unique_ptr<std::vector<GlyphAdvance>>
                                glyph_advances) -> Future<std::vector<Glyph>> {
                    Future<absl::flat_hash_map<
                        GlyphAtlasOld::GlyphKeyOrGlyphString,
                        ScopedCanvas::TextMetrics>>
                        measurements =
                            MeasureGlyphs(*glyph_advances, canvas_options);
                    return measurements.Then(
                        [this, canvas_options,
                         glyph_advances = std::move(glyph_advances)](
                            absl::flat_hash_map<GlyphKeyOrGlyphString,
                                                ScopedCanvas::TextMetrics>
                                measurements) {
                          auto pending_canvas_glyphs = std::make_unique<
                              std::vector<CanvasOptionsGlyphKey>>();
                          std::vector<Glyph> result;
                          AddGlyphs(std::move(*glyph_advances), canvas_options,
                                    measurements, result,
                                    *pending_canvas_glyphs);
                          if (pending_canvas_glyphs->empty()) {
                            return Future<std::vector<Glyph>>(
                                std::move(result));
                          }

                          // Whether updating the texture can happen
                          // synchronously and therefore is guaranteed to
                          // happen on the next EndFrame after all draw
                          // commands are issued.
                          bool synchronous_texture_update;
                          Future<absl::Status> result_future;
                          {
                            if (canvas_source_->IsFeatureSupported(
                                    ScopedCanvas::Feature::kKeepContents)) {
                              {
                                absl::MutexLock lock(&canvas_mutex_);
                                GetOrStartDrawing(
                                    ScopedCanvas::DrawMode::kKeepContents);
                              }

                              // Schedule the glyphs that are newly added to
                              // the atlas to be drawn to the canvas
                              // asynchronously.
                              result_future = DrawGlyphsToCanvasAsync(
                                  std::move(pending_canvas_glyphs));
                            } else {
                              absl::MutexLock lock(&canvas_mutex_);
                              // Set the texture_status_ to kHasNewGlyphs to
                              // ensure that the glyphs will be drawn on the
                              // next frame.
                              GetOrStartDrawing(ScopedCanvas::DrawMode::kClear);
                              texture_status_ = TextureStatus::kHasNewGlyphs;
                              result_future.Return(absl::OkStatus());
                            }
                            absl::MutexLock lock(&canvas_mutex_);
                            synchronous_texture_update =
                                canvas_->SupportsSynchronousTextureUpdate();
                          }
                          return result_future
                              .Then([this, synchronous_texture_update]() {
                                if (synchronous_texture_update) {
                                  // If synchronous_texture_update is true,
                                  // the texture is guaranteed to update
                                  // synchronously on the next EndFrame
                                  // after the result_future resolves, so no
                                  // need to block this future until the
                                  // texture is updated.
                                  return Future<absl::Status>(absl::OkStatus());
                                } else {
                                  // Block resolution of the glyphs until
                                  // the texture has been updated and the
                                  // glyphs can be safely used.
                                  Future<absl::Status> pending_glyph_future;
                                  texture_update_futures_.push_back(
                                      pending_glyph_future);
                                  return pending_glyph_future;
                                }
                              })
                              .Then([glyphs = std::move(result)]() {
                                return glyphs;
                              });
                        });
                  });
            });
      },
      Executor::Type::kCurrent);
}

Future<std::unique_ptr<std::vector<GlyphAtlasOld::GlyphAdvance>>>
GlyphAtlasOld::BreakIntoGlyphs(
    absl::string_view text, const ScopedCanvas::TextOptions& canvas_options) {
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
  if (canvas_source_->IsFeatureSupported(ScopedCanvas::Feature::kGlyphs)) {
    absl::string_view font_holder_name = "";
    if (canvas_options.font_holder) {
      font_holder_name = canvas_options.font_holder->GetFontName();
    }
    return canvas_source_->GetTextGlyphs(text, canvas_options)
        .Then([font_holder_name](
                  std::unique_ptr<std::vector<ScopedCanvas::GlyphAdvance>>
                      canvas_glyph_advances)
                  -> std::unique_ptr<std::vector<GlyphAdvance>> {
          std::unique_ptr<std::vector<GlyphAdvance>> glyph_advances =
              std::make_unique<std::vector<GlyphAdvance>>();
          glyph_advances->reserve(canvas_glyph_advances->size());
          for (ScopedCanvas::GlyphAdvance& canvas_glyph_advance :
               *canvas_glyph_advances) {
            absl::string_view font = "";
            if (!font_holder_name.empty()) {
              font = font_holder_name;
            }
            if (canvas_glyph_advance.fallback_font) {
              font = canvas_glyph_advance.fallback_font->GetFontName();
            }
            glyph_advances->push_back(GlyphAtlasOld::GlyphAdvance{
                .glyph = GlyphKey{.glyph_id = canvas_glyph_advance.glyph,
                                  .font_id = Hash(font)},
                .width = canvas_glyph_advance.width,
                .fallback_font = std::move(canvas_glyph_advance.fallback_font),
                .is_emoji = canvas_glyph_advance.is_emoji});
          }
          return glyph_advances;
        });
  } else {
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
    // with accents/ligatures when running on Android API 31, iOS, and Desktop.
    std::vector<Chunk> chunks = GetChunks(text);
    auto widths = canvas_source_->GetTextWidths(chunks, canvas_options);
    bool contains_rtl = ContainsRtl(text);
    return widths.Then(
        [chunks = std::move(chunks), contains_rtl,
         canvas_options](std::vector<std::vector<float>> widths) {
          std::unique_ptr<std::vector<GlyphAdvance>> glyph_advances =
              std::make_unique<std::vector<GlyphAdvance>>();
          for (int i = 0; i < chunks.size(); i++) {
            GlyphAtlasOld::GetGlyphsForChunk(chunks[i], widths[i],
                                             canvas_options, *glyph_advances);
          }

          // Because we tokenize the text and try to split it on separable
          // delimiters, if the text was rtl then we need to reverse the order
          // of the tokens if it is rtl.
          if (contains_rtl) {
            absl::c_reverse(*glyph_advances);
          }
          return glyph_advances;
        });
  }
}

void GlyphAtlasOld::GetGlyphsForChunk(
    const Chunk& chunk, const std::vector<float>& advance_widths,
    const ScopedCanvas::TextOptions& canvas_options,
    std::vector<GlyphAtlasOld::GlyphAdvance>& out_glyph_advances) {
  if (!chunk.is_separable) {
    out_glyph_advances.push_back(
        GlyphAdvance{.glyph = std::string(chunk.chunk_text),
                     .width = advance_widths[0],
                     .is_emoji = ContainsEmoji(chunk.chunk_text),
                     .contains_non_separable_script =
                         ContainsNonSeparableScript(chunk.chunk_text)});
    return;
  }

  // TODO: This assumption is *incorrect on Android* for characters
  // outside of the basic multilingual plane (BMP); for example, 𨭎. On Android,
  // Paint.getTextWidths() returns one entry for each UTF-16 codepoint, NOT for
  // each Unicode codepoint. This is different than emoji, which can be made up
  // of several independent Unicode codepoints (e.g. for skin color).
  if (chunk.codepoint_count != advance_widths.size()) {
    IMP_LOG(imp::FATAL) << "GlyphAtlasOld is unable to generate glyphs for text "
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
    // character in the UTF8 text. However, non-essential ones *might* be
    // turned into a ligature by the font. For instance, the adjacent
    // characters "fi" may be turned into the ligature 'ﬁ'. However, not all
    // fonts have glyphs for the non-essential ligatures. For example,
    // Roboto does, but Google Sans doesn't.
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

    out_glyph_advances.push_back(GlyphAdvance{.glyph = std::move(utf8_char),
                                              .width = advance_width,
                                              .is_emoji = is_emoji});
    i++;
  }
}

const GlyphAtlasOld::GlyphInfo* /*absl_nullable*/ GlyphAtlasOld::GetGlyphInfo(
    const CanvasOptionsGlyphKey& glyph_key) {
  absl::MutexLock glyph_map_lock(&glyph_map_mutex_);
  // Try to find the already cached main glyph entry for this text.
  auto itr = glyph_map_.find(glyph_key);
  if (itr != glyph_map_.end()) {
    return &itr->second;
  }
  return nullptr;
}

Future<absl::flat_hash_map<GlyphAtlasOld::GlyphKeyOrGlyphString,
                           ScopedCanvas::TextMetrics>>
GlyphAtlasOld::MeasureGlyphs(const std::vector<GlyphAdvance>& glyphs,
                             ScopedCanvas::TextOptions canvas_options) {
  std::vector<AsyncCanvasSource::GlyphToMeasure> glyphs_to_measure;
  std::vector<GlyphKeyOrGlyphString> glyph_keys_to_measure;

  absl::flat_hash_map<GlyphAtlasOld::GlyphKeyOrGlyphString,
                      ScopedCanvas::TextMetrics>
      measurements;
  for (const GlyphAdvance& glyph_advance : glyphs) {
    const GlyphAtlasOld::GlyphInfo* /*absl_nullable*/ glyph_info = GetGlyphInfo(
        {.canvas_options = canvas_options, .glyph = glyph_advance.glyph});
    if (glyph_info == nullptr) {
      std::variant<absl::string_view, ScopedCanvas::GlyphId> glyph;
      if (absl::holds_alternative<GlyphKey>(glyph_advance.glyph)) {
        glyph = absl::get<GlyphKey>(glyph_advance.glyph).glyph_id;
      } else {
        glyph = absl::get<std::string>(glyph_advance.glyph);
      }
      AsyncCanvasSource::GlyphToMeasure glyph_to_measure({.glyph = glyph});
      if (glyph_advance.fallback_font) {
        glyph_to_measure.font_override = glyph_advance.fallback_font.get();
      }
      glyphs_to_measure.push_back(glyph_to_measure);
      glyph_keys_to_measure.push_back(glyph_advance.glyph);
    } else {
      measurements.emplace(glyph_advance.glyph, glyph_info->measurements);
    }
  }

  return canvas_source_->MeasureGlyphs(glyphs_to_measure, canvas_options)
      .Then([measurements = std::move(measurements),
             glyph_keys_to_measure = std::move(glyph_keys_to_measure)](
                std::vector<ScopedCanvas::TextMetrics> text_metrics) mutable {
        for (int i = 0; i < text_metrics.size(); i++) {
          measurements.emplace(glyph_keys_to_measure[i], text_metrics[i]);
        }
        return measurements;
      });
};

void GlyphAtlasOld::AddGlyphs(
    std::vector<GlyphAdvance> glyphs,
    const ScopedCanvas::TextOptions& canvas_options,
    const absl::flat_hash_map<GlyphKeyOrGlyphString, ScopedCanvas::TextMetrics>&
        measurement_map,
    std::vector<Glyph>& result,
    std::vector<CanvasOptionsGlyphKey>& pending_canvas_glyphs) {
  result.reserve(glyphs.size());
  for (GlyphAdvance& glyph_advance : glyphs) {
    // For non-separable texts, we render the entire thing as a single glyph. If
    // the text happens to contain an emoji, then either the emoji would be
    // colored wrong, or the text would be colored wrong. To address this, we
    // should check if the text contains both emojis and non-emoji characters,
    // then colorize it in the requested color in the glyph atlas. Otherwise, we
    // can colorize it with the stroke and fill identifier colors for handling
    // in the shade.
    const bool is_emoji = glyph_advance.is_emoji;
    const bool contains_non_emoji_characters =
        glyph_advance.contains_non_separable_script;
    CanvasOptionsGlyphKey pending_glyph = {
        .canvas_options = canvas_options,
        .glyph = glyph_advance.glyph,
        .requires_color_key = (is_emoji && contains_non_emoji_characters)};
    if (!pending_glyph.requires_color_key) {
      // In the text shader we can figure out the stroke and fill by checking
      // the value of the R and G channel that contain the stroke and fill.
      pending_glyph.canvas_options.stroke_color =
          pending_glyph.canvas_options.stroke_width_pixels > 0
              ? kStrokeIdentifierColor
              : kZero4;
      pending_glyph.canvas_options.color = kFillIdentifierColor;
    }
    bool needs_render = GetGlyphInfo(pending_glyph) == nullptr;
    if (needs_render) {
      pending_canvas_glyphs.push_back(pending_glyph);
    }
    const float width = glyph_advance.width;
    const GlyphInfo& glyph_info = GetOrAddGlyphInfo(
        std::move(glyph_advance), pending_glyph, measurement_map);
    Glyph glyph = GlyphInfoToGlyph(glyph_info, width, GetSubpixelRenderRatio(),
                                   atlas_texture_size_, is_emoji);
    result.push_back(glyph);
  }
}

const GlyphAtlasOld::GlyphInfo& GlyphAtlasOld::GetOrAddGlyphInfo(
    GlyphAdvance glyph_advance, const CanvasOptionsGlyphKey& glyph_key,
    const absl::flat_hash_map<GlyphKeyOrGlyphString, ScopedCanvas::TextMetrics>&
        measurement_map) {
  const GlyphAtlasOld::GlyphInfo* /*absl_nullable*/ glyph_info =
      GetGlyphInfo(glyph_key);
  if (glyph_info != nullptr) {
    return *glyph_info;
  }

  auto itr = measurement_map.find(glyph_advance.glyph);
  if (itr == measurement_map.end()) {
    IMP_LOG(imp::ERROR) << "Missing measurement for glyph \""
               << ToString(glyph_advance.glyph) << "\"";
    return kEmptyGlyphInfo;
  }
  const ScopedCanvas::TextMetrics& measurements = itr->second;

  // Atlas entry is integer precision with some padding.
  uint2 atlas_entry_size = {
      static_cast<int>(std::ceil(measurements.size.x)) + kPadding,
      static_cast<int>(std::ceil(measurements.font_size_y)) + kPadding};

  std::optional<AtlasPacker::ScopedAtlasEntry> atlas_entry =
      TryAddAtlasEntry(atlas_entry_size);

  // TODO: Add support for automatically expanding to multiple
  // textures if we run out of space.
  if (!atlas_entry) {
    IMP_LOG(imp::ERROR) << "Unable to find space in Glyph Atlas for glyph \""
               << ToString(glyph_advance.glyph)
               << "\" with size x=" << atlas_entry_size.x
               << ", y=" << atlas_entry_size.y;
    return kEmptyGlyphInfo;
  }

  absl::MutexLock glyph_map_lock(&glyph_map_mutex_);
  return glyph_map_
      .emplace(glyph_key,
               GlyphInfo{
                   .atlas_entry = std::move(*atlas_entry),
                   .measurements = measurements,
                   .fallback_font = std::move(glyph_advance.fallback_font),
#if IMP_RUNTIME(DEV)
                   .stroke_width = glyph_key.canvas_options.stroke_width_pixels,
#endif
               })
      .first->second;
}

void GlyphAtlasOld::DrawAllGlyphsToCanvas() {
  ScopedCanvas* canvas;
  {
    absl::MutexLock lock(&canvas_mutex_);
    canvas = GetOrStartDrawing(ScopedCanvas::DrawMode::kClear);
    // TODO : Canvas can may be dirty due to async drawing of
    // glyphs; investigate how to prevent that from happening.
    canvas->ClearRect({.center = atlas_texture_size_ / 2.0f,
                       .half_extent = atlas_texture_size_ / 2.0f});
  }

  absl::MutexLock glyph_map_lock(&glyph_map_mutex_);
  for (auto& [glyph, glyph_info] : glyph_map_) {
    // Lock inside the loop since we want to allow background threads to
    // access canvas_ in between iterations.
    absl::MutexLock canvas_lock(&canvas_mutex_);
    // We don't need to check for an invalid canvas here since this function
    // and the releasing of canvas_ both occur synchronously on the foreground
    // thread.
    DrawGlyphToCanvas(*canvas, glyph, glyph_info);
  }
}

Future<absl::Status> GlyphAtlasOld::DrawGlyphsToCanvasAsync(
    std::unique_ptr<std::vector<CanvasOptionsGlyphKey>> glyphs) {
  if (glyphs->empty()) {
    return Future<absl::Status>(absl::OkStatus());
  }

  if (!canvas_source_->IsFeatureSupported(
          ScopedCanvas::Feature::kKeepContents)) {
    return Future<absl::Status>(
        absl::AbortedError("kKeepContents not supported"));
  }

  return Future<std::unique_ptr<std::vector<CanvasOptionsGlyphKey>>>::Schedule(
             [this, gate = completion_gate_,
              glyphs = std::move(glyphs)]() mutable
                 -> absl::StatusOr<
                     std::unique_ptr<std::vector<CanvasOptionsGlyphKey>>> {
               absl::ReaderMutexLock lock(&gate->mutex_);
               if (gate->complete_) {
                 glyphs->clear();
                 return absl::AbortedError("Glyph drawing aborted");
               }
               (void)DrawGlyphsToCanvas(*glyphs);
               return std::move(glyphs);
             },
             Executor::Type::kBackground)
      .Then([this](std::unique_ptr<std::vector<CanvasOptionsGlyphKey>> glyphs)
                -> Future<absl::Status> {
        {
          absl::MutexLock canvas_lock(&canvas_mutex_);
          // This means all the glyphs newly added to the atlas has now been
          // drawn to the canvas, and so the only thing left to do is to release
          // the pixel buffer in the canvas.
          if (glyphs->empty()) {
            texture_status_ = TextureStatus::kReadyToApplyDrawCommands;
            return Future<absl::Status>(absl::OkStatus());
          }
          // If the status is an error, it means that the glyphs haven't
          // completed drawing to the canvas, which indicates the canvas was
          // destroyed somewhere during the process. Recreate the canvas and try
          // again.
          GetOrStartDrawing(ScopedCanvas::DrawMode::kKeepContents);
        }
        return DrawGlyphsToCanvasAsync(std::move(glyphs));
      });
}

absl::Status GlyphAtlasOld::DrawGlyphsToCanvas(
    std::vector<CanvasOptionsGlyphKey>& glyphs) {
  while (!glyphs.empty()) {
    const CanvasOptionsGlyphKey& glyph = glyphs.back();
    absl::MutexLock glyph_map_lock(&glyph_map_mutex_);
    auto itr = glyph_map_.find(glyph);
    if (itr != glyph_map_.end()) {
      absl::MutexLock canvas_lock(&canvas_mutex_);
      // As the canvas may be destroyed at any point in this loop due to the
      // async nature of drawing glyphs to the canvas, so we want to check
      // that it's still valid. If it's not, we stop here, and
      // DrawGlyphsToCanvasAsync will recreate the canvas and reschedule the
      // rest of the glyphs after that.
      if (!canvas_) {
        return absl::AbortedError("Canvas unavailable");
      }
      DrawGlyphToCanvas(*canvas_, glyph, itr->second);
    }
    glyphs.pop_back();
  }

  return absl::OkStatus();
}

std::optional<AtlasPacker::ScopedAtlasEntry> GlyphAtlasOld::TryAddAtlasEntry(
    uint2 atlas_entry_size) {
  absl::MutexLock lock(&atlas_packer_mutex_);
  std::optional<AtlasPacker::ScopedAtlasEntry> atlas_entry =
      atlas_packer_.AddEntry(atlas_entry_size);

  // The atlas was full. Clear unused glyphs and try again.
  if (!atlas_entry) {
    ClearUnusedGlyphs();
    atlas_entry = atlas_packer_.AddEntry(atlas_entry_size);
  }

  return atlas_entry;
}

void GlyphAtlasOld::ClearUnusedGlyphs() {
  absl::MutexLock glyph_map_lock(&glyph_map_mutex_);
  absl::MutexLock canvas_lock(&canvas_mutex_);
  // Remove each glyph that is unused. Detect if it's unused if the ref
  // counter is at zero.
  for (auto glyph_itr = glyph_map_.begin(); glyph_itr != glyph_map_.end();) {
    auto copy_glyph_itr = glyph_itr++;

    if (copy_glyph_itr->second.ref_counter.GetCount() == 0) {
      // If we are retaining the canvas's content then we need to ensure
      // that the old entries are cleared from the canvas for when the
      // texture memory is re-used in the future.
      if (canvas_source_->IsFeatureSupported(
              ScopedCanvas::Feature::kKeepContents)) {
        ScopedCanvas* canvas =
            GetOrStartDrawing(ScopedCanvas::DrawMode::kKeepContents);

        const GlyphAtlasOld::GlyphInfo& glyph_info = copy_glyph_itr->second;
        const AtlasPacker::ScopedAtlasEntry& atlas_entry =
            glyph_info.atlas_entry;
        float2 half_extent =
            (atlas_entry.GetBottomRight() - atlas_entry.GetTopLeft()) / 2.0f;
        Rect rect{.center = atlas_entry.GetTopLeft() + half_extent,
                  .half_extent = half_extent};
        canvas->ClearRect(rect);
      }

      glyph_map_.erase(copy_glyph_itr);
    }
  }
}

size_t GlyphAtlasOld::GetNumCachedGlyphs() const {
  absl::MutexLock glyph_map_lock(&glyph_map_mutex_);
  return glyph_map_.size();
}

GlyphAtlas::Glyph GlyphAtlasOld::GlyphInfoToGlyph(
    const GlyphAtlasOld::GlyphInfo& glyph_info, float advance_width,
    float2 pixel_ratio_scale, float2 glyph_atlas_size, bool is_emoji) {
  Glyph glyph{.glyph_ref = glyph_info.ref_counter.Retain()};
  glyph.actual_origin = glyph_info.measurements.origin / pixel_ratio_scale;
  glyph.actual_size = glyph_info.measurements.size / pixel_ratio_scale;
  glyph.atlas_size = (glyph_info.atlas_entry.GetBottomRight() -
                      glyph_info.atlas_entry.GetTopLeft()) /
                     pixel_ratio_scale;
  glyph.advance_width = advance_width / pixel_ratio_scale.x;
  glyph.atlas_origin = (float2{glyph_info.measurements.origin.x,
                               glyph_info.measurements.font_origin_y} -
                        float2{kHalfPadding}) /
                       pixel_ratio_scale;
  glyph.uv_top_left = glyph_info.atlas_entry.GetTopLeft() / glyph_atlas_size;
  glyph.uv_size = (glyph_info.atlas_entry.GetBottomRight() -
                   glyph_info.atlas_entry.GetTopLeft()) /
                  glyph_atlas_size;
  glyph.is_emoji = is_emoji;

  return glyph;
}

void GlyphAtlasOld::DrawGlyphToCanvas(ScopedCanvas& canvas,
                                      CanvasOptionsGlyphKey glyph,
                                      const GlyphInfo& glyph_info) {
  if (glyph_info.fallback_font) {
    glyph.canvas_options.font_holder = glyph_info.fallback_font.get();
  }
  float2 glyph_top_left =
      glyph_info.atlas_entry.GetTopLeft() + float2{kHalfPadding};
  DrawGlyphKeyOrGlyphString(canvas, glyph.glyph, glyph_top_left,
                            glyph.canvas_options);
}

void GlyphAtlasOld::DrawGlyphKeyOrGlyphString(
    ScopedCanvas& canvas, const GlyphKeyOrGlyphString& glyph, float2 position,
    const ScopedCanvas::TextOptions& canvas_options) {
  if (absl::holds_alternative<GlyphKey>(glyph)) {
    canvas.DrawGlyph(std::get<GlyphKey>(glyph).glyph_id, position,
                     canvas_options);
  } else {
    canvas.DrawText(std::get<std::string>(glyph), position, canvas_options);
  }
}

Future<ScopedCanvas::TextOptions>
GlyphAtlasOld::CanvasOptionsFromGlyphAtlasOptions(const TextOptions& options) {
  return physical_pixel_ratio_available_.Then(
      [this, options]() -> absl::StatusOr<ScopedCanvas::TextOptions> {
        FontHolder* font_holder = nullptr;
        ScopedCanvas::TextOptions canvas_options = kTextOptions;
        if (std::holds_alternative<std::string>(options.font_params)) {
          absl::string_view font_name =
              std::get<std::string>(options.font_params);
          if (!font_name.empty()) {
            absl::MutexLock lock(&fonts_mutex_);
            auto font_itr = fonts_.find(font_name);
            if (font_itr != fonts_.end()) {
              font_holder = font_itr->second.get();
            } else {
              return absl::InvalidArgumentError(absl::StrFormat(
                  "Failed to get glyphs. Missing font %s", font_name));
            }
          }
        } else if (std::holds_alternative<SystemFontParams>(
                       options.font_params)) {
          const SystemFontParams font_params =
              std::get<SystemFontParams>(options.font_params);
          absl::MutexLock lock(&system_fonts_mutex_);
          auto system_font_iter = system_fonts_.find(font_params);
          if (system_font_iter != system_fonts_.end()) {
            font_holder = system_font_iter->second.get();
          } else {
            font_holder =
                system_fonts_
                    .emplace(font_params,
                             LoadSystemFont(view_.GetContext(), font_params))
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
        canvas_options.render_scale = GetSubpixelRenderRatio().x;
        return canvas_options;
      },
      Executor::Type::kCurrent);
}

std::string GlyphAtlasOld::ToString(const GlyphKeyOrGlyphString& glyph) {
  if (absl::holds_alternative<GlyphKey>(glyph)) {
    GlyphKey glyph_key = absl::get<GlyphKey>(glyph);
    return absl::StrFormat("Id=%i, Font=%i", glyph_key.glyph_id,
                           glyph_key.font_id);
  } else {
    return absl::get<std::string>(glyph);
  }
}

Texture* GlyphAtlasOld::GetTexture() {
  if (Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "GetTexture may only be called from the foreground "
                  "thread.";
  }
  return texture_;
}

void GlyphAtlasOld::PrepareToUpdateTexture() {
  if (canvas_->SupportsSynchronousTextureUpdate()) {
    texture_status_ = TextureStatus::kReadyToRelease;
  } else {
    texture_status_ = TextureStatus::kPreparingToUpdateTexture;
    // If the previous future hasn't resolved yet, cancel it.
    if (!pending_prepare_future_.Ready()) {
      pending_prepare_future_.Cancel();
    }
    pending_prepare_future_ = canvas_->PrepareToUpdateTexture();

    // Unlock the mutex so that it is not held when the Then callback is
    // called, regardless of whether it is called synchronously or
    // asynchronously.
    canvas_mutex_.Unlock();
    pending_prepare_future_ =
        pending_prepare_future_.Then([this](absl::Status status) {
          if (!status.ok()) {
            // When new glyphs are added, any pending PrepareToUpdateTexture
            // futures will be cancelled. If however it is not a cancelled
            // future, we should log the error.
            if (!absl::IsCancelled(status)) {
              IMP_LOG(imp::ERROR) << status;
            }
            return;
          }
          absl::MutexLock lock(&canvas_mutex_);
          // If the status is no longer kPreparingToUpdateTexture, new
          // glyphs were likely added while preparing the canvas for
          // release. We'll have to draw those glyphs and reprepare in that
          // case.
          if (texture_status_ == TextureStatus::kPreparingToUpdateTexture) {
            texture_status_ = TextureStatus::kReadyToRelease;
          }
        });
    canvas_mutex_.Lock();
  }
}

// Copied from GlyphAtlasNew.
Future<GlyphEmulator::SuperSampleInfo> GlyphAtlasOld::GetSuperSampleInfo()
    const {
#if IMP_PLATFORM(WASM)
  if (view_.GetDevice().IsPhysicalPixelRatioAvailable()) {
    return Future<GlyphEmulator::SuperSampleInfo>(
        GlyphEmulator::GetSuperSampleInfo(
            view_.GetDevice().GetPhysicalPixelRatio()));
  }
  return physical_pixel_ratio_available_.Then([this]() {
    return GlyphEmulator::GetSuperSampleInfo(
        view_.GetDevice().GetPhysicalPixelRatio());
  });
#else
  return Future<GlyphEmulator::SuperSampleInfo>(GlyphEmulator::SuperSampleInfo{
      .should_super_sample = false,
      .subpixel_render_ratio = float2{1.0f},
  });
#endif
}

bool GlyphAtlasOld::ShouldSuperSample() const {
#if IMP_PLATFORM(WASM)
  return view_.GetDevice().IsPhysicalPixelRatioAvailable() &&
         view_.GetDevice().GetPhysicalPixelRatio().x < kSuperSampleThreshold;
#endif
  return false;
}

float2 GlyphAtlasOld::GetSubpixelRenderRatio() const {
#if IMP_PLATFORM(WASM)
  return ShouldSuperSample() ? float2{kSuperSampleThreshold, 1.0f}
                             : float2{1.0f};
#endif
  return float2{1.0f};
}

float GlyphAtlasOld::GetAtlasUtilization() const {
  float utilization;
  {
    absl::MutexLock lock(&atlas_packer_mutex_);
    utilization = atlas_packer_.GetUtilization();
  }
  return utilization;
}

#if IMP_RUNTIME(DEV)
std::optional<editor::GlyphAtlasVisualizer::GlyphAtlasInfo>
GlyphAtlasOld::GetGlyphAtlasInfoAt(const float2& uv) const {
  float2 texture_point = uv * atlas_texture_size_;

  {
    absl::MutexLock lock(&glyph_map_mutex_);
    for (auto& [glyph, glyph_info] : glyph_map_) {
      const AtlasPacker::ScopedAtlasEntry& atlas_entry = glyph_info.atlas_entry;
      float2 half_extent =
          (atlas_entry.GetBottomRight() - atlas_entry.GetTopLeft()) / 2.0f;
      Rect rect{.center = atlas_entry.GetTopLeft() + half_extent,
                .half_extent = half_extent};
      if (RectContainsPoint(rect, texture_point)) {
        rect.half_extent /= atlas_texture_size_;
        rect.center /= atlas_texture_size_;
        float half_stroke_width = glyph_info.stroke_width / 2.0f;
        float2 origin =
            float2(atlas_entry.GetTopLeft().x +
                       glyph_info.measurements.origin.x + half_stroke_width +
                       kHalfPadding,
                   // +Y is up. Start counting from the bottom of the glyph.
                   atlas_entry.GetBottomRight().y +
                       glyph_info.measurements.font_origin_y -
                       half_stroke_width - kHalfPadding) /
            atlas_texture_size_;
        return editor::GlyphAtlasVisualizer::GlyphAtlasInfo{
            .glyph = ToString(glyph.glyph),
            .is_stroke = false,
            .origin = origin,
            .uv = rect};
      }
    }
  }
  return std::nullopt;
}

std::vector<editor::GlyphAtlasVisualizer::GlyphAtlasInfo>
GlyphAtlasOld::GetAllGlyphInfo() const {
  std::vector<editor::GlyphAtlasVisualizer::GlyphAtlasInfo> result;
  {
    absl::MutexLock lock(&glyph_map_mutex_);
    for (auto& [glyph, glyph_info] : glyph_map_) {
      const AtlasPacker::ScopedAtlasEntry& atlas_entry = glyph_info.atlas_entry;
      float2 half_extent =
          (atlas_entry.GetBottomRight() - atlas_entry.GetTopLeft()) / 2.0f;
      Rect rect{.center = atlas_entry.GetTopLeft() + half_extent,
                .half_extent = half_extent};
      rect.half_extent /= atlas_texture_size_;
      rect.center /= atlas_texture_size_;
      float half_stroke_width = glyph_info.stroke_width / 2.0f;
      float2 origin =
          float2(atlas_entry.GetTopLeft().x + glyph_info.measurements.origin.x +
                     half_stroke_width + kHalfPadding,
                 // +Y is up. Start counting from the bottom of the glyph.
                 atlas_entry.GetBottomRight().y +
                     glyph_info.measurements.font_origin_y - half_stroke_width -
                     kHalfPadding) /
          atlas_texture_size_;
      result.push_back(editor::GlyphAtlasVisualizer::GlyphAtlasInfo{
          .glyph = ToString(glyph.glyph),
          .is_stroke = false,
          .origin = origin,
          .uv = rect});
    }
  }
  return result;
}
#endif

}  // namespace imp
