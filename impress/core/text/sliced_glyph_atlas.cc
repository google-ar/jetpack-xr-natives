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

#include "core/text/sliced_glyph_atlas.h"

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
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
#include "core/common/typed_span.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/render/texture.h"
#include "core/text/glyph_emulator.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/base_view.h"
#include "core/view/utils/device.h"
#include "core/view/view_events.h"

#if IMP_RUNTIME(DEV)
#include "core/editor/editor.h"
#include "core/editor/layout/editor_panel_ids.h"
#include "core/editor/widget_ui_system.h"
#include "core/text/editor/sliced_glyph_atlas_visualizer.h"
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
constexpr auto kInvalidSliceId = SlicedGlyphAtlas::SliceId{
    std::numeric_limits<SlicedGlyphAtlas::SliceId::ValueType>::max()};

float2 GetTextureSize(SlicedGlyphAtlas::TextureSize texture_size) {
  switch (texture_size) {
    case SlicedGlyphAtlas::TextureSize::k2048:
      return {2048.0f, 2048.0f};
    case SlicedGlyphAtlas::TextureSize::k2048_4096:
      return {2048.0f, 4096.0f};
    case SlicedGlyphAtlas::TextureSize::k4096:
      return {4096.0f, 4096.0f};
  }
}

size_t GetTextureDepth(SlicedGlyphAtlas::TextureSize texture_size) {
  switch (texture_size) {
    case SlicedGlyphAtlas::TextureSize::k2048:
      return 1;
    case SlicedGlyphAtlas::TextureSize::k2048_4096:
      return 1;
    case SlicedGlyphAtlas::TextureSize::k4096:
      return 1;
  }
}

}  // namespace

SlicedGlyphAtlas::SlicedGlyphAtlas(BaseView& view, Config config)
    : SlicedGlyphAtlas::SlicedGlyphAtlas(
          view,
          AsyncCanvasSourceFactory::Create(view.GetContext(),
                                           config.use_hardware_rendering),
          config) {}

SlicedGlyphAtlas::SlicedGlyphAtlas(
    BaseView& view, std::unique_ptr<AsyncCanvasSource> canvas_source,
    Config config)
    : view_(view),
      atlas_texture_size_(GetTextureSize(config.texture_size)),
      atlas_texture_depth_(GetTextureDepth(config.texture_size)),
      canvas_source_(std::move(canvas_source)),
      glyph_emulator_(view.GetContext(), *canvas_source_) {
  slice_storage_ =
      static_cast<Slice*>(malloc(sizeof(Slice) * atlas_texture_depth_));
  for (size_t i = 0; i < atlas_texture_depth_; ++i) {
    std::construct_at(slice_storage_ + i, atlas_texture_size_);
  }
  slices_ = TypedSpan<Slice>(slice_storage_, atlas_texture_depth_);

  view.GetDispatcher().Connect(
      [this](const imp::ViewPostFrameUpdateEvent& event) mutable {
        EndFrame();
      },
      this);

  // Prevents an issue where the texture can get cleared and needs to be redrawn
  // when the view is resumed.
  if (config.force_reset_on_view_resumed) {
    view.GetDispatcher().Connect(
        [this](const ViewResumedEvent& event) {
          // Cannot force reset if there is already an active canvas.
          if (absl::c_all_of(slices_, [](const auto& slice) {
                absl::MutexLock lock(slice.canvas_mutex_);
                return slice.canvas_ == nullptr;
              })) {
            canvas_source_->ForceReset();
          }
          for (auto& slice : slices_) {
            slice.OnViewResumed();
          }
        },
        this);
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
    editor->get()
        .GetWidgetUiSystem()
        .AddWidget<editor::SlicedGlyphAtlasVisualizer>(
            editor::WidgetLayoutInfo(editor::PanelId::kTabBar), view,
            editor::SlicedGlyphAtlasVisualizer::AtlasDataProvider{
                .get_texture_func = [this]() { return GetTexture(); },
                .get_glyph_info_func =
                    [this](float2 uv) { return GetSlicedGlyphAtlasInfoAt(uv); },
                .get_utilization_func =
                    [this]() { return GetAtlasUtilization(); },
                .get_all_glyph_info_func =
                    [this]() { return GetAllGlyphInfo(); }});
  }
#endif
}

SlicedGlyphAtlas::~SlicedGlyphAtlas() {
  ClearRemembered();
  for (auto& slice : slices_) {
    {
      // Wait until destruction-blocking blocks across all threads have been
      // completed.
      absl::MutexLock lock(slice.completion_gate_->mutex);
      slice.completion_gate_->complete = true;
    }

    std::destroy_at(&slice);
  }
  free(slice_storage_);
}

void SlicedGlyphAtlas::EndFrame() {
  for (auto& slice : slices_) {
    slice.EndFrame(view_, *canvas_source_);
  }
}

void SlicedGlyphAtlas::AddFont(absl::string_view font_name,
                               std::unique_ptr<FontHolder> font_holder) {
  glyph_emulator_.AddFont(font_name, std::move(font_holder));
}

Future<absl::Status> SlicedGlyphAtlas::PrepareFont(absl::string_view text,
                                                   const TextOptions& options) {
  absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
      glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(options,
                                                            float2(1.0f));
  if (!canvas_options.ok()) {
    return Future<absl::Status>(canvas_options.status());
  }
  return glyph_emulator_.PrepareFont(text, *canvas_options);
}

Future<std::vector<ScopedCanvas::GlyphGroup>>
SlicedGlyphAtlas::GetCombinedCharacterGroups(absl::string_view text,
                                             const TextOptions& options) {
  absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
      glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(options,
                                                            float2(1.0f));
  if (!canvas_options.ok()) {
    return Future<std::vector<ScopedCanvas::GlyphGroup>>(
        canvas_options.status());
  }
  return glyph_emulator_.GetCombinedCharacterGroups(text, *canvas_options);
}

Future<TextMetrics> SlicedGlyphAtlas::GetTextMetrics(
    absl::string_view text, const TextOptions& options) {
  absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
      glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(options,
                                                            float2(1.0f));
  if (!canvas_options.ok()) {
    return Future<TextMetrics>(canvas_options.status());
  }
  return glyph_emulator_.GetTextMetrics(text, *canvas_options);
}

Future<FontInfo> SlicedGlyphAtlas::GetFontInfo(const TextOptions& options) {
  absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
      glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(options);
  if (!canvas_options.ok()) {
    return Future<FontInfo>(canvas_options.status());
  }
  return glyph_emulator_.GetFontInfo(*canvas_options);
}

Future<std::vector<SlicedGlyphAtlas::Glyph>> SlicedGlyphAtlas::GetGlyphs(
    absl::string_view text, const GlyphEmulator::TextOptions& options) {
  if (text.empty()) {
    return Future<std::vector<Glyph>>(std::vector<Glyph>());
  }

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
          [this, text = std::string(text), options = options](
              GlyphEmulator::SuperSampleInfo super_sample_info) {
            absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
                glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(
                    options, super_sample_info.subpixel_render_ratio);
            if (!canvas_options.ok()) {
              return Future<std::vector<Glyph>>(canvas_options.status());
            }

            return glyph_emulator_.GetGlyphs(text, *canvas_options)
                .Then([this, canvas_options = *canvas_options,
                       super_sample_info](
                          std::unique_ptr<std::vector<GlyphEmulator::Glyph>>
                              glyphs) {
                  auto pending_canvas_glyphs =
                      std::make_unique<PendingCanvasGlyphs>();

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
                  std::vector<Future<absl::Status>> entry_futures;
                  for (auto& entry : pending_canvas_glyphs->entries) {
                    Future<absl::Status> entry_future;
                    auto& slice = slices_[entry.slice];
                    if (canvas_source_->IsFeatureSupported(
                            ScopedCanvas::Feature::kKeepContents)) {
                      {
                        absl::MutexLock lock(slice.canvas_mutex_);
                        slice.GetOrStartDrawing(
                            view_, *canvas_source_,
                            ScopedCanvas::DrawMode::kKeepContents);
                      }

                      // Schedule the glyphs that are newly added to the atlas
                      // to be drawn to the canvas asynchronously.
                      // To ensure glyphs are always drawn once their space is
                      // reserved, the ownership of the DrawGlyphsToCanvasAsync
                      // future is transferred to the SlicedGlyphAtlas. This
                      // ties the future's lifecycle to the atlas itself, rather
                      // than the calling method. This is to avoid the specific
                      // situation where the caller is destroyed before the
                      // future is resolved, thus cancelling this future, but
                      // the atlas space is still reserved as other callers may
                      // have made references to those glyphs in the meantime,
                      // preventing the now invalid atlas space from being
                      // released.
                      // TODO: Have the future's lifetime be tied
                      // to the glyphs's space reservation instead of the atlas
                      // itself.
                      slice
                          .DrawGlyphsToCanvasAsync(view_, *canvas_source_,
                                                   std::move(entry.glyphs))
                          .Then(
                              [entry_future](absl::Status status) {
                                entry_future.Return(status);
                                return absl::OkStatus();
                              },
                              Executor::Type::kCurrent)
                          .KeptBy(this);
                    } else {
                      absl::MutexLock lock(slice.canvas_mutex_);
                      // Set the texture_status_ to kHasNewGlyphs to ensure that
                      // the glyphs will be drawn on the next frame.
                      slice.GetOrStartDrawing(view_, *canvas_source_,
                                              ScopedCanvas::DrawMode::kClear);
                      slice.texture_status_ = TextureStatus::kHasNewGlyphs;
                      entry_future.Return(absl::OkStatus());
                    }
                    absl::MutexLock lock(slice.canvas_mutex_);
                    synchronous_texture_update =
                        slice.canvas_->SupportsSynchronousTextureUpdate();
                    entry_futures.push_back(entry_future);
                  }
                  result_future =
                      Future<absl::Status>::CombineList(entry_futures);
                  return result_future
                      .Then([this, synchronous_texture_update]() {
                        std::vector<Future<absl::Status>> slice_futures;
                        for (auto& slice : slices_) {
                          bool canvas_is_null;
                          {
                            absl::MutexLock lock(slice.canvas_mutex_);
                            canvas_is_null = slice.canvas_ == nullptr;
                            if (canvas_is_null) {
                              slice.texture_status_ = TextureStatus::kStable;
                            }
                          }
                          if (synchronous_texture_update || canvas_is_null) {
                            // If synchronous_texture_update is true, the
                            // texture is guaranteed to update synchronously on
                            // the next EndFrame after the result_future
                            // resolves, so no need to block this future until
                            // the texture is updated.

                            // If the canvas is null, that means the texture has
                            // already been updated after the draw, but before
                            // this code is executed. Theres no more work to do
                            // so return immediately.
                            slice_futures.push_back(
                                Future<absl::Status>(absl::OkStatus()));
                          } else {
                            // Block resolution of the glyphs until the texture
                            // has been updated and the glyphs can be safely
                            // used.
                            Future<absl::Status> pending_glyph_future;
                            slice.texture_update_futures_.push_back(
                                pending_glyph_future);
                            slice_futures.push_back(pending_glyph_future);
                          }
                        }
                        return Future<absl::Status>::CombineList(slice_futures);
                      })
                      .Then([glyphs = std::move(result)]() { return glyphs; });
                });
          },
          Executor::Type::kCurrent);
}

std::optional<SlicedGlyphAtlas::SlicedGlyphInfo> SlicedGlyphAtlas::GetGlyphInfo(
    const CanvasOptionsGlyphKey& key) {
  for (auto& slice : slices_) {
    const GlyphInfo* glyph_info = slice.GetGlyphInfo(key);
    if (glyph_info != nullptr) {
      return SlicedGlyphInfo{.slice = slices_.IdOf(slice), .info = *glyph_info};
    }
  }
  return std::nullopt;
}

void SlicedGlyphAtlas::AddGlyphs(
    std::vector<GlyphEmulator::Glyph> glyphs,
    const ScopedCanvas::TextOptions& canvas_options, std::vector<Glyph>& result,
    PendingCanvasGlyphs& pending_canvas_glyphs, float2 subpixel_render_ratio) {
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
    bool needs_render = !GetGlyphInfo(pending_glyph).has_value();
    const float width = glyph.advance_width;

    std::optional<SlicedGlyphInfo> sliced_glyph_info =
        GetOrAddGlyphInfo(glyph, pending_glyph);
    if (sliced_glyph_info.has_value() && needs_render) {
      pending_canvas_glyphs.Get(sliced_glyph_info->slice)
          .push_back(pending_glyph);
    }

    Glyph atlas_glyph = GlyphInfoToGlyph(
        sliced_glyph_info.has_value() ? sliced_glyph_info->info
                                      : kEmptyGlyphInfo,
        sliced_glyph_info.has_value() ? sliced_glyph_info->slice
                                      : kInvalidSliceId,
        width, subpixel_render_ratio, atlas_texture_size_, is_emoji);
    result.push_back(atlas_glyph);
  }
}

std::optional<SlicedGlyphAtlas::SlicedGlyphInfo>
SlicedGlyphAtlas::GetOrAddGlyphInfo(GlyphEmulator::Glyph& glyph,
                                    const CanvasOptionsGlyphKey& glyph_key) {
  std::optional<SlicedGlyphInfo> glyph_info = GetGlyphInfo(glyph_key);
  if (glyph_info.has_value()) {
    return *glyph_info;
  }

  // Atlas entry is integer precision with some padding.
  uint2 atlas_entry_size = {
      static_cast<int>(std::ceil(glyph.metrics.size_x())) + kPadding,
      static_cast<int>(std::ceil(glyph.metrics.font_size_y())) + kPadding};

  std::optional<SlicedAtlasEntry> atlas_entry =
      TryAddAtlasEntry(atlas_entry_size);

  // TODO: Add support for automatically expanding to multiple
  // textures if we run out of space.
  if (!atlas_entry) {
    IMP_LOG(imp::ERROR) << "Unable to find space in Glyph Atlas for glyph \""
               << Slice::ToString(glyph.glyph)
               << "\" with size x=" << atlas_entry_size.x
               << ", y=" << atlas_entry_size.y;
    return std::nullopt;
  }

  auto& slice = slices_[atlas_entry->slice];
  absl::MutexLock glyph_map_lock(slice.glyph_map_mutex_);
  const GlyphInfo* info =
      &slice.glyph_map_
           .emplace(
               glyph_key,
               GlyphInfo{
                   .atlas_entry = std::move(atlas_entry->entry),
                   .measurements = glyph.metrics,
                   .fallback_font = std::move(glyph.fallback_font),
#if IMP_RUNTIME(DEV)
                   .stroke_width = glyph_key.canvas_options.stroke_width_pixels,
#endif
               })
           .first->second;
  return SlicedGlyphInfo{.slice = atlas_entry->slice, .info = *info};
}

std::optional<SlicedGlyphAtlas::SlicedAtlasEntry>
SlicedGlyphAtlas::TryAddAtlasEntry(uint2 atlas_entry_size) {
  for (auto& slice : slices_) {
    absl::MutexLock lock(slice.atlas_packer_mutex_);
    std::optional<AtlasPacker::ScopedAtlasEntry> atlas_entry =
        slice.atlas_packer_.AddEntry(atlas_entry_size);
    if (atlas_entry) {
      return SlicedAtlasEntry{.slice = slices_.IdOf(slice),
                              .entry = std::move(*atlas_entry)};
    }
  }

  // The atlas was full. Clear unused glyphs and try again.
  for (auto& slice : slices_) {
    absl::MutexLock lock(slice.atlas_packer_mutex_);
    slice.ClearUnusedGlyphs(view_, *canvas_source_);

    std::optional<AtlasPacker::ScopedAtlasEntry> atlas_entry =
        slice.atlas_packer_.AddEntry(atlas_entry_size);
    if (atlas_entry) {
      return SlicedAtlasEntry{.slice = slices_.IdOf(slice),
                              .entry = std::move(*atlas_entry)};
    }
  }
  return std::nullopt;
}

size_t SlicedGlyphAtlas::GetNumCachedGlyphs() const {
  return absl::c_accumulate(slices_, 0, [](size_t sum, const Slice& slice) {
    return sum + slice.GetNumCachedGlyphs();
  });
}

SlicedGlyphAtlas::Glyph SlicedGlyphAtlas::GlyphInfoToGlyph(
    const GlyphInfo& glyph_info, SliceId slice, float advance_width,
    float2 pixel_ratio_scale, float2 glyph_atlas_size, bool is_emoji) {
  Glyph glyph{.slice = slice, .glyph_ref = glyph_info.ref_counter.Retain()};
  glyph.actual_origin = float2(glyph_info.measurements.origin_x(),
                               glyph_info.measurements.font_origin_y()) /
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

Texture* SlicedGlyphAtlas::GetTexture() {
  if (Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "GetTexture may only be called from the foreground "
                  "thread.";
  }
  return slices_.front().texture_;
}

Future<GlyphEmulator::SuperSampleInfo> SlicedGlyphAtlas::GetSuperSampleInfo(
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

float SlicedGlyphAtlas::GetAtlasUtilization() const {
  float utilization =
      absl::c_accumulate(slices_, 0.f, [](float sum, const Slice& slice) {
        return sum + slice.GetAtlasUtilization();
      });
  return utilization / slices_.size();
}

GlyphEmulator& SlicedGlyphAtlas::GetGlyphEmulator() { return glyph_emulator_; }

#if IMP_RUNTIME(DEV)
std::optional<editor::SlicedGlyphAtlasVisualizer::SlicedGlyphAtlasInfo>
SlicedGlyphAtlas::GetSlicedGlyphAtlasInfoAt(const float2& uv) const {
  for (auto& slice : slices_) {
    auto result = slice.GetSlicedGlyphAtlasInfoAt(uv);
    if (result.has_value()) {
      return result;
    }
  }

  return std::nullopt;
}

std::vector<editor::SlicedGlyphAtlasVisualizer::SlicedGlyphAtlasInfo>
SlicedGlyphAtlas::GetAllGlyphInfo() const {
  std::vector<editor::SlicedGlyphAtlasVisualizer::SlicedGlyphAtlasInfo> result;
  for (auto& slice : slices_) {
    auto slice_result = slice.GetAllGlyphInfo();
    result.insert(result.end(), slice_result.begin(), slice_result.end());
  }
  return result;
}
#endif

}  // namespace imp
