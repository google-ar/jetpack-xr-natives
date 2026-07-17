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

#include "core/text/glyph_atlas.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#if IMP_PLATFORM(WASM)
#include <emscripten.h>
#include <emscripten/em_asm.h>
#endif  // IMP_PLATFORM(WASM)

#include "absl/base/nullability.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/variant.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/atlas/atlas_packer.h"
#include "core/atlas/shelf_atlas_packer.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_canvas_source_factory.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/ref_counter.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/render/texture.h"
#include "core/text/glyph_emulator.h"
#include "core/text/text_glyphs.h"
#include "core/view/base_view.h"
#include "core/view/utils/device.h"
#include "core/view/view_events.h"

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

AsyncScopedCanvas* GlyphAtlas::GetOrStartDrawing(
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
    view_.GetDispatcher().Send(TextureChangedEvent());
  }

  return canvas_.get();
}

GlyphAtlas::GlyphAtlas(BaseView& view, Config config)
    : GlyphAtlas::GlyphAtlas(
          view,
          AsyncCanvasSourceFactory::Create(
              view.GetContext(), config.use_hardware_rendering,
              config.force_auto_method_rendering,
              config.force_individual_glyph_source_instances,
              view.GetConfig()
                  .experimental_feature_flags->enable_label_prep_profile_logging
                  .Value(),
              config.use_bitmap_surface_provider),
          config) {}

GlyphAtlas::GlyphAtlas(BaseView& view,
                       std::unique_ptr<AsyncCanvasSource> canvas_source,
                       Config config)
    : view_(view),
      atlas_texture_size_(GetTextureSize(config.texture_size)),
      canvas_source_(std::move(canvas_source)),
      glyph_emulator_(view.GetContext()),
      atlas_packer_(atlas_texture_size_) {
  view.GetDispatcher().Connect(
      [this](const imp::ViewPostFrameUpdateEvent& event) mutable {
        EndFrame();
      },
      this);

  if (config.use_bitmap_surface_provider) {
    view.GetDispatcher().Connect(
        [this](const ViewPausedEvent& event) { canvas_source_->OnPause(); },
        this);
    view.GetDispatcher().Connect(
        [this](const ViewResumedEvent& event) { canvas_source_->OnResume(); },
        this);
  } else {
    // Prevents an issue where the texture can get cleared and needs to be
    // redrawn when the view is resumed.
    if (config.force_reset_on_view_resumed) {
      view.GetDispatcher().Connect(
          [this](const ViewResumedEvent& event) {
            absl::MutexLock lock(canvas_mutex_);
            // Cannot force reset if there is already an active canvas.
            if (canvas_ == nullptr) {
              canvas_source_->ForceReset();
            }
            texture_status_ = TextureStatus::kHasNewGlyphs;
          },
          this);
    }
  }

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
        editor::WidgetLayoutInfo(editor::PanelId::kTabBar,
                                 editor::WidgetPresence::kOnlyIn2DLargeScreen,
                                 editor::WidgetVisibility::kHidden),
        view,
        editor::GlyphAtlasVisualizer::AtlasDataProvider{
            .get_texture_func = [this]() { return GetTexture(); },
            .get_glyph_info_func =
                [this](float2 uv) { return GetGlyphAtlasInfoAt(uv); },
            .get_utilization_func = [this]() { return GetAtlasUtilization(); },
            .get_all_glyph_info_func = [this]() { return GetAllGlyphInfo(); }});
  }
#endif
}

GlyphAtlas::~GlyphAtlas() {
  ClearRemembered();
  // Wait until destruction-blocking blocks across all threads have been
  // completed.
  absl::MutexLock lock(completion_gate_->mutex);

  completion_gate_->complete = true;

  {
    absl::MutexLock lock(glyph_map_mutex_);
    glyph_map_.clear();
  }

  {
    absl::MutexLock lock(canvas_mutex_);
    canvas_.reset();
  }

#if IMP_RUNTIME(DEV)
  if (auto editor = view_.GetRegistry().Get<editor::Editor>(); editor.ok()) {
    editor->get()
        .GetWidgetUiSystem()
        .RemoveWidget<editor::GlyphAtlasVisualizer>();
  }
#endif
}

void GlyphAtlas::EndFrame() {
  IMP_TRACE();
  canvas_mutex_.lock();
  if (texture_status_ == TextureStatus::kStable ||
      texture_status_ == TextureStatus::kPreparingToUpdateTexture) {
    canvas_mutex_.unlock();
    return;
  }

  if (!canvas_ && canvas_source_->IsFeatureSupported(
                      ScopedCanvas::Feature::kKeepContents)) {
    canvas_mutex_.unlock();
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
    canvas_mutex_.unlock();
    DrawAllGlyphsToCanvas();
    canvas_mutex_.lock();
    texture_status_ = TextureStatus::kReadyToApplyDrawCommands;
  }

  if (texture_status_ == TextureStatus::kReadyToApplyDrawCommands) {
    UpdateTexture();
  }
  canvas_mutex_.unlock();
}

void GlyphAtlas::AddFont(absl::string_view font_name,
                         std::unique_ptr<FontHolder> font_holder) {
  glyph_emulator_.AddFont(font_name, std::move(font_holder));
}

Future<absl::Status> GlyphAtlas::PrepareFont(absl::string_view text,
                                             const TextOptions& options) {
  absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
      glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(options);
  if (!canvas_options.ok()) {
    return Future<absl::Status>(canvas_options.status());
  }
  return glyph_emulator_.PrepareFont(text, *canvas_options, *canvas_source_);
}

Future<std::vector<ScopedCanvas::GlyphGroup>>
GlyphAtlas::GetCombinedCharacterGroups(absl::string_view text,
                                       const TextOptions& options) {
  absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
      glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(options);
  if (!canvas_options.ok()) {
    return Future<std::vector<ScopedCanvas::GlyphGroup>>(
        canvas_options.status());
  }
  return glyph_emulator_.GetCombinedCharacterGroups(text, *canvas_options,
                                                    *canvas_source_);
}

Future<TextMetrics> GlyphAtlas::GetTextMetrics(absl::string_view text,
                                               const TextOptions& options) {
  absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
      glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(options);
  if (!canvas_options.ok()) {
    return Future<TextMetrics>(canvas_options.status());
  }
  return glyph_emulator_.GetTextMetrics(text, *canvas_options, *canvas_source_);
}

Future<FontInfo> GlyphAtlas::GetFontInfo(const TextOptions& options) {
  absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
      glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(options);
  if (!canvas_options.ok()) {
    return Future<FontInfo>(canvas_options.status());
  }
  return glyph_emulator_.GetFontInfo(*canvas_options, *canvas_source_);
}

Future<TextGlyphs> GlyphAtlas::GetTextGlyphs(
    absl::string_view text, const GlyphEmulator::TextOptions& options) {
  return GetGlyphs(text, options)
      .Then([](std::vector<GlyphAtlas::Glyph> glyphs) {
        // The sign bit of the v texture coordinate is used by materials to
        // detect emoji.
        constexpr float4 kEmojiScale = float4(1.f, -1.f, 1.f, -1.f);
        TextGlyphs text_glyphs;
        text_glyphs.Reserve(glyphs.size());
        for (const auto& glyph : glyphs) {
          const RefCounter::Ref& glyph_ref = glyph.glyph_ref;
          float4 atlas_origin_and_size =
              float4(glyph.atlas_origin, glyph.atlas_size);
          float4 actual_origin_and_size =
              float4(glyph.actual_origin, glyph.actual_size);
          float4 uv_origin_and_size = float4(glyph.uv_top_left, glyph.uv_size);
          float advance_width = glyph.advance_width;

          if (glyph.is_emoji) uv_origin_and_size *= kEmojiScale;

          text_glyphs.PushBack(glyph_ref, atlas_origin_and_size,
                               actual_origin_and_size, uv_origin_and_size,
                               advance_width);
        }
        return text_glyphs;
      });
}

Future<std::vector<GlyphAtlas::Glyph>> GlyphAtlas::GetGlyphs(
    absl::string_view text, const GlyphEmulator::TextOptions& options) {
  if (text.empty()) {
    return Future<std::vector<Glyph>>(std::vector<Glyph>());
  }
  double wasm_start_time = 0.0;
  bool enable_label_prep_profile_logging = false;
#if IMP_PLATFORM(WASM)
  if (view_.GetConfig().experimental_feature_flags) {
    enable_label_prep_profile_logging =
        view_.GetConfig()
            .experimental_feature_flags->enable_label_prep_profile_logging
            .Value();
  }
  if (enable_label_prep_profile_logging) {
    wasm_start_time = EM_ASM_DOUBLE({ return performance.now(); });
  }
#endif  // IMP_PLATFORM(WASM)

  // GetGlyphs must run on the foreground Executor since we make calls to
  // Filament through GetOrStartDrawing().
  if (Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    return Future<std::vector<Glyph>>::Schedule(
        [this, text = std::string(text),
         options = GlyphEmulator::TextOptions(options)]() {
          return GetGlyphs(text, options);
        },
        Executor::Type::kForeground);
  }

  // Convert the text options into the canvas options actually used for drawing.
  return GetSuperSampleInfo(options.force_non_separable)
      .Then(
          [this, text = std::string(text), options = options, wasm_start_time,
           enable_label_prep_profile_logging](
              GlyphEmulator::SuperSampleInfo super_sample_info) {
            absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
                glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(
                    options, super_sample_info.subpixel_render_ratio);
            if (!canvas_options.ok()) {
              return Future<std::vector<Glyph>>(canvas_options.status());
            }

            double break_start_time = 0.0;
            (void)break_start_time;
#if IMP_PLATFORM(WASM)
            break_start_time = EM_ASM_DOUBLE({ return performance.now(); });
#endif  // IMP_PLATFORM(WASM)

            return glyph_emulator_
                .GetGlyphs(text, *canvas_options, *canvas_source_)
                .Then([break_start_time, enable_label_prep_profile_logging](
                          std::unique_ptr<std::vector<GlyphEmulator::Glyph>>
                              glyphs) {
#if IMP_PLATFORM(WASM)
                  if (enable_label_prep_profile_logging) {
                    EM_ASM(
                        {
                          try {
                            performance.measure(
                                'Label Prep: GetGlyphs: Break Text',
                                {start : $0});
                          } catch (e) {
                          }
                        },
                        break_start_time);
                  }
#endif  // IMP_PLATFORM(WASM)
                  return glyphs;
                })
                .Then([this, canvas_options = *canvas_options,
                       super_sample_info, wasm_start_time,
                       enable_label_prep_profile_logging](
                          std::unique_ptr<std::vector<GlyphEmulator::Glyph>>
                              glyphs) {
                  auto pending_canvas_glyphs =
                      std::make_unique<std::vector<CanvasOptionsGlyphKey>>();

                  std::vector<Glyph> result;
                  AddGlyphs(std::move(*glyphs), canvas_options, result,
                            *pending_canvas_glyphs,
                            super_sample_info.subpixel_render_ratio);
                  if (pending_canvas_glyphs->empty()) {
                    return Future<std::vector<Glyph>>(std::move(result));
                  }

                  // Whether updating the texture can happen synchronously and
                  // therefore is guaranteed to happen on the next EndFrame
                  // after all draw commands are issued.
                  bool synchronous_texture_update;
                  Future<absl::Status> result_future;
                  {
                    if (canvas_source_->IsFeatureSupported(
                            ScopedCanvas::Feature::kKeepContents)) {
                      {
                        absl::MutexLock lock(canvas_mutex_);
                        GetOrStartDrawing(
                            ScopedCanvas::DrawMode::kKeepContents);
                      }

                      // Schedule the glyphs that are newly added to the atlas
                      // to be drawn to the canvas asynchronously.
                      // To ensure glyphs are always drawn once their space is
                      // reserved, the ownership of the DrawGlyphsToCanvasAsync
                      // future is transferred to the GlyphAtlas. This ties the
                      // future's lifecycle to the atlas itself, rather than the
                      // calling method.
                      // This is to avoid the specific situation where the
                      // caller is destroyed before the future is resolved, thus
                      // cancelling this future, but the atlas space is still
                      // reserved as other callers may have made references to
                      // those glyphs in the meantime, preventing the now
                      // invalid atlas space from being released.
                      // TODO: Have the future's lifetime be tied
                      // to the glyphs's space reservation instead of the atlas
                      // itself.
                      DrawGlyphsToCanvasAsync(std::move(pending_canvas_glyphs))
                          .Then(
                              [result_future](absl::Status status) {
                                result_future.Return(status);
                                return absl::OkStatus();
                              },
                              Executor::Type::kCurrent)
                          .KeptBy(this);
                    } else {
                      absl::MutexLock lock(canvas_mutex_);
                      // Set the texture_status_ to kHasNewGlyphs to ensure that
                      // the glyphs will be drawn on the next frame.
                      GetOrStartDrawing(ScopedCanvas::DrawMode::kClear);
                      texture_status_ = TextureStatus::kHasNewGlyphs;
                      result_future.Return(absl::OkStatus());
                    }
                    absl::MutexLock lock(canvas_mutex_);
                    synchronous_texture_update =
                        canvas_->SupportsSynchronousTextureUpdate();
                  }
                  return result_future
                      .Then([this, synchronous_texture_update]() {
                        bool canvas_is_null;
                        {
                          absl::MutexLock lock(canvas_mutex_);
                          canvas_is_null = canvas_ == nullptr;
                          if (canvas_is_null) {
                            texture_status_ = TextureStatus::kStable;
                          }
                        }
                        if (synchronous_texture_update || canvas_is_null) {
                          // If synchronous_texture_update is true, the texture
                          // is guaranteed to update synchronously on the next
                          // EndFrame after the result_future resolves, so no
                          // need to block this future until the texture is
                          // updated.

                          // If the canvas is null, that means the texture has
                          // already been updated after the draw, but before
                          // this code is executed. Theres no more work to do
                          // so return immediately.
                          return Future<absl::Status>(absl::OkStatus());
                        } else {
                          // Block resolution of the glyphs until the texture
                          // has been updated and the glyphs can be safely used.
                          Future<absl::Status> pending_glyph_future;
                          texture_update_futures_.push_back(
                              pending_glyph_future);
                          return pending_glyph_future;
                        }
                      })
                      .Then([glyphs = std::move(result), wasm_start_time,
                             enable_label_prep_profile_logging]() {
#if IMP_PLATFORM(WASM)
                        if (enable_label_prep_profile_logging) {
                          EM_ASM(
                              {
                                try {
                                  performance.measure('Label Prep: GetGlyphs',
                                                      {start : $0});
                                } catch (e) {
                                }
                              },
                              wasm_start_time);
                        }
#endif  // IMP_PLATFORM(WASM)
                        return glyphs;
                      });
                });
          },
          Executor::Type::kCurrent);
}

const GlyphAtlas::GlyphInfo* /*absl_nullable*/  GlyphAtlas::GetGlyphInfo(
    const CanvasOptionsGlyphKey& glyph_key) {
  absl::MutexLock glyph_map_lock(glyph_map_mutex_);
  // Try to find the already cached main glyph entry for this text.
  auto itr = glyph_map_.find(glyph_key);
  if (itr != glyph_map_.end()) {
    return &itr->second;
  }
  return nullptr;
}

void GlyphAtlas::AddGlyphs(
    std::vector<GlyphEmulator::Glyph> glyphs,
    const ScopedCanvas::TextOptions& canvas_options, std::vector<Glyph>& result,
    std::vector<CanvasOptionsGlyphKey>& pending_canvas_glyphs,
    float2 subpixel_render_ratio) {
  result.reserve(glyphs.size());
  for (GlyphEmulator::Glyph& glyph : glyphs) {
    // For non-separable texts, we render the entire thing as a single glyph. If
    // the text happens to contain an emoji, then either the emoji would be
    // colored wrong, or the text would be colored wrong. To address this, we
    // should check if the text contains both emojis and non-emoji characters,
    // then colorize it in the requested color in the glyph atlas. Otherwise, we
    // can colorize it with the stroke and fill identifier colors for handling
    // in the shade.
    const bool is_emoji = glyph.is_emoji;
    const bool contains_non_emoji_characters =
        glyph.contains_non_separable_script;
    CanvasOptionsGlyphKey pending_glyph = {
        .canvas_options = canvas_options,
        .glyph = glyph.glyph,
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
    const float width = glyph.advance_width;
    const GlyphInfo* glyph_info = GetOrAddGlyphInfo(glyph, pending_glyph);
    if (glyph_info != nullptr && needs_render) {
      pending_canvas_glyphs.push_back(pending_glyph);
    }
    Glyph atlas_glyph = GlyphInfoToGlyph(
        glyph_info == nullptr ? kEmptyGlyphInfo : *glyph_info, width,
        subpixel_render_ratio, atlas_texture_size_, is_emoji);
    result.push_back(atlas_glyph);
  }
}

const GlyphAtlas::GlyphInfo* GlyphAtlas::GetOrAddGlyphInfo(
    GlyphEmulator::Glyph& glyph, const CanvasOptionsGlyphKey& glyph_key) {
  const GlyphAtlas::GlyphInfo* /*absl_nullable*/  glyph_info =
      GetGlyphInfo(glyph_key);
  if (glyph_info != nullptr) {
    return glyph_info;
  }

  // Atlas entry is integer precision with some padding.
  uint2 atlas_entry_size = {
      static_cast<int>(std::ceil(glyph.metrics.size_x())) + kPadding,
      static_cast<int>(std::ceil(glyph.metrics.font_size_y())) + kPadding};

  std::optional<AtlasPacker::ScopedAtlasEntry> atlas_entry =
      TryAddAtlasEntry(atlas_entry_size);

  // TODO: Add support for automatically expanding to multiple
  // textures if we run out of space.
  if (!atlas_entry) {
    IMP_LOG(imp::ERROR) << "Unable to find space in Glyph Atlas for glyph \""
               << ToString(glyph.glyph)
               << "\" with size x=" << atlas_entry_size.x
               << ", y=" << atlas_entry_size.y;
    return nullptr;
  }

  absl::MutexLock glyph_map_lock(glyph_map_mutex_);
  return &glyph_map_
              .emplace(glyph_key,
                       GlyphInfo{
                           .atlas_entry = std::move(*atlas_entry),
                           .measurements = glyph.metrics,
                           .fallback_font = std::move(glyph.fallback_font),
#if IMP_RUNTIME(DEV)
                           .stroke_width =
                               glyph_key.canvas_options.stroke_width_pixels,
#endif
                       })
              .first->second;
}

void GlyphAtlas::DrawAllGlyphsToCanvas() {
  ScopedCanvas* canvas;
  {
    absl::MutexLock lock(canvas_mutex_);
    canvas = GetOrStartDrawing(ScopedCanvas::DrawMode::kClear);
    // TODO : Canvas can may be dirty due to async drawing of
    // glyphs; investigate how to prevent that from happening.
    canvas->ClearRect({.center = atlas_texture_size_ / 2.0f,
                       .half_extent = atlas_texture_size_ / 2.0f});
  }

  absl::MutexLock glyph_map_lock(glyph_map_mutex_);
  for (auto& [glyph, glyph_info] : glyph_map_) {
    // Lock inside the loop since we want to allow background threads to
    // access canvas_ in between iterations.
    absl::MutexLock canvas_lock(canvas_mutex_);
    // We don't need to check for an invalid canvas here since this function
    // and the releasing of canvas_ both occur synchronously on the foreground
    // thread.
    DrawGlyphToCanvas(*canvas, glyph, glyph_info);
  }
}

Future<absl::Status> GlyphAtlas::DrawGlyphsToCanvasAsync(
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
               absl::ReaderMutexLock lock(gate->mutex);
               if (gate->complete) {
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
          absl::MutexLock canvas_lock(canvas_mutex_);
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

absl::Status GlyphAtlas::DrawGlyphsToCanvas(
    std::vector<CanvasOptionsGlyphKey>& glyphs) {
  while (!glyphs.empty()) {
    const CanvasOptionsGlyphKey& glyph = glyphs.back();
    absl::MutexLock glyph_map_lock(glyph_map_mutex_);
    auto itr = glyph_map_.find(glyph);
    if (itr != glyph_map_.end()) {
      absl::MutexLock canvas_lock(canvas_mutex_);
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

std::optional<AtlasPacker::ScopedAtlasEntry> GlyphAtlas::TryAddAtlasEntry(
    uint2 atlas_entry_size) {
  absl::MutexLock lock(atlas_packer_mutex_);
  std::optional<AtlasPacker::ScopedAtlasEntry> atlas_entry =
      atlas_packer_.AddEntry(atlas_entry_size);

  // The atlas was full. Clear unused glyphs and try again.
  if (!atlas_entry) {
    ClearUnusedGlyphs();
    atlas_entry = atlas_packer_.AddEntry(atlas_entry_size);
  }

  return atlas_entry;
}

void GlyphAtlas::ClearUnusedGlyphs() {
  absl::MutexLock glyph_map_lock(glyph_map_mutex_);
  absl::MutexLock canvas_lock(canvas_mutex_);
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

        const GlyphAtlas::GlyphInfo& glyph_info = copy_glyph_itr->second;
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

size_t GlyphAtlas::GetNumCachedGlyphs() const {
  absl::MutexLock glyph_map_lock(glyph_map_mutex_);
  return glyph_map_.size();
}

GlyphAtlas::Glyph GlyphAtlas::GlyphInfoToGlyph(const GlyphInfo& glyph_info,
                                               float advance_width,
                                               float2 pixel_ratio_scale,
                                               float2 glyph_atlas_size,
                                               bool is_emoji) {
  Glyph glyph{.glyph_ref = glyph_info.ref_counter.Retain()};
  glyph.actual_origin = float2(glyph_info.measurements.origin_x(),
                               glyph_info.measurements.origin_y()) /
                        pixel_ratio_scale;
  glyph.actual_size = float2(glyph_info.measurements.size_x(),
                             glyph_info.measurements.size_y()) /
                      pixel_ratio_scale;
  glyph.atlas_size = (glyph_info.atlas_entry.GetBottomRight() -
                      glyph_info.atlas_entry.GetTopLeft()) /
                     pixel_ratio_scale;
  glyph.advance_width = advance_width / pixel_ratio_scale.x;
  glyph.atlas_origin = (float2{glyph_info.measurements.origin_x(),
                               glyph_info.measurements.font_origin_y()} -
                        float2{kHalfPadding}) /
                       pixel_ratio_scale;
  glyph.uv_top_left = glyph_info.atlas_entry.GetTopLeft() / glyph_atlas_size;
  glyph.uv_size = (glyph_info.atlas_entry.GetBottomRight() -
                   glyph_info.atlas_entry.GetTopLeft()) /
                  glyph_atlas_size;
  glyph.is_emoji = is_emoji;

  return glyph;
}

void GlyphAtlas::DrawGlyphToCanvas(ScopedCanvas& canvas,
                                   CanvasOptionsGlyphKey glyph,
                                   const GlyphInfo& glyph_info) {
  if (glyph_info.fallback_font) {
    glyph.canvas_options.font_holder = glyph_info.fallback_font.get();
  }
  float2 glyph_top_left =
      glyph_info.atlas_entry.GetTopLeft() + float2{kHalfPadding};
  GlyphEmulator::DrawGlyph(canvas, glyph.glyph, glyph_top_left,
                           glyph.canvas_options, &glyph_info.measurements);
}

std::string GlyphAtlas::ToString(
    const GlyphEmulator::GlyphKeyOrGlyphString& glyph) {
  if (absl::holds_alternative<GlyphEmulator::GlyphKey>(glyph)) {
    GlyphEmulator::GlyphKey glyph_key =
        absl::get<GlyphEmulator::GlyphKey>(glyph);
    return absl::StrFormat("Id=%i, Font=%i", glyph_key.glyph_id.Get(),
                           glyph_key.font_id);
  } else {
    return absl::get<std::string>(glyph);
  }
}

Texture* GlyphAtlas::GetTexture() {
  if (Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "GetTexture may only be called from the foreground "
                  "thread.";
  }
  return texture_;
}

void GlyphAtlas::UpdateTexture() {
  if (canvas_->SupportsSynchronousTextureUpdate()) {
    UpdateTextureSync();
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
    canvas_mutex_.unlock();
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
          absl::MutexLock lock(canvas_mutex_);
          // If the status is no longer kPreparingToUpdateTexture, new
          // glyphs were likely added while preparing the canvas for
          // release. We'll have to draw those glyphs and reprepare in that
          // case.
          if (texture_status_ == TextureStatus::kPreparingToUpdateTexture) {
            UpdateTextureSync();
          }
        });
    canvas_mutex_.lock();
  }
}

void GlyphAtlas::UpdateTextureSync() {
  texture_status_ = TextureStatus::kStable;
  for (const auto& future : texture_update_futures_) {
    future.Return(absl::OkStatus());
  }
  texture_update_futures_.clear();
  canvas_.reset();
}

Future<GlyphEmulator::SuperSampleInfo> GlyphAtlas::GetSuperSampleInfo(
    bool force_off) const {
#if IMP_PLATFORM(WASM)
  if (view_.GetDevice().IsPhysicalPixelRatioAvailable()) {
    return Future<GlyphEmulator::SuperSampleInfo>(
        GlyphEmulator::GetSuperSampleInfo(
            view_.GetDevice().GetPhysicalPixelRatio(), force_off));
  }
  return physical_pixel_ratio_available_.Then([this, force_off]() {
    return GlyphEmulator::GetSuperSampleInfo(
        view_.GetDevice().GetPhysicalPixelRatio(), force_off);
  });
#else
  return Future<GlyphEmulator::SuperSampleInfo>(GlyphEmulator::SuperSampleInfo{
      .should_super_sample = false,
      .subpixel_render_ratio = float2{1.0f},
  });
#endif
}

float GlyphAtlas::GetAtlasUtilization() const {
  float utilization;
  {
    absl::MutexLock lock(atlas_packer_mutex_);
    utilization = atlas_packer_.GetUtilization();
  }
  return utilization;
}

GlyphEmulator& GlyphAtlas::GetGlyphEmulator() { return glyph_emulator_; }

#if IMP_RUNTIME(DEV)
std::optional<editor::GlyphAtlasVisualizer::GlyphAtlasInfo>
GlyphAtlas::GetGlyphAtlasInfoAt(const float2& uv) const {
  float2 texture_point = uv * atlas_texture_size_;

  {
    absl::MutexLock lock(glyph_map_mutex_);
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
                       glyph_info.measurements.origin_x() + half_stroke_width +
                       kHalfPadding,
                   // +Y is up. Start counting from the bottom of the glyph.
                   atlas_entry.GetBottomRight().y +
                       glyph_info.measurements.font_origin_y() -
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
GlyphAtlas::GetAllGlyphInfo() const {
  std::vector<editor::GlyphAtlasVisualizer::GlyphAtlasInfo> result;
  {
    absl::MutexLock lock(glyph_map_mutex_);
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
          float2(atlas_entry.GetTopLeft().x +
                     glyph_info.measurements.origin_x() + half_stroke_width +
                     kHalfPadding,
                 // +Y is up. Start counting from the bottom of the glyph.
                 atlas_entry.GetBottomRight().y +
                     glyph_info.measurements.font_origin_y() -
                     half_stroke_width - kHalfPadding) /
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
