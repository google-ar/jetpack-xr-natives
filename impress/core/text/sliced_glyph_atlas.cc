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
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "filament/filament/include/filament/Renderer.h"
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
#include "core/text/sliced_glyph_atlas_helpers.h"
#include "core/text/sliced_glyph_texture_manager.h"
#include "core/text/text_glyphs.h"
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
    case SlicedGlyphAtlas::TextureSize::k256_256_8_8:
      return {256.0f, 256.0f};
  }
}

uint2 GetGridSize(SlicedGlyphAtlas::TextureSize texture_size) {
  switch (texture_size) {
    case SlicedGlyphAtlas::TextureSize::k2048:
      return {1, 1};
    case SlicedGlyphAtlas::TextureSize::k2048_4096:
      return {1, 1};
    case SlicedGlyphAtlas::TextureSize::k4096:
      return {1, 1};
    case SlicedGlyphAtlas::TextureSize::k256_256_8_8:
      return {8, 8};
  }
}

}  // namespace

SlicedGlyphAtlas::SlicedGlyphAtlas(BaseView& view, Config config)
    : SlicedGlyphAtlas::SlicedGlyphAtlas(
          view,
          [&view, &config]() -> std::unique_ptr<AsyncCanvasSource> {
            return AsyncCanvasSourceFactory::Create(
                view.GetContext(), config.use_hardware_rendering,
                /*force_auto_method_rendering=*/false,
                config.force_individual_glyph_source_instances);
          },
          config) {}

SlicedGlyphAtlas::SlicedGlyphAtlas(
    BaseView& view,
    std::function<std::unique_ptr<AsyncCanvasSource>()>
        canvas_source_factory_fn,
    Config config)
    : view_(view),
      atlas_texture_size_(GetTextureSize(config.texture_size)),
      atlas_grid_size_(GetGridSize(config.texture_size)),
      glyph_emulator_(view.GetContext()),
      addition_cursor_(SliceId::At(0)) {
  size_t slice_count = atlas_grid_size_.x * atlas_grid_size_.y;
  slice_storage_ = static_cast<Slice*>(malloc(sizeof(Slice) * slice_count));
  for (size_t i = 0; i < slice_count; ++i) {
    std::construct_at(slice_storage_ + i, atlas_texture_size_,
                      canvas_source_factory_fn());
  }
  slices_ = TypedSpan<Slice>(slice_storage_, slice_count);
  pending_renders_.Resize(slice_count);

  shared_canvas_source_ = canvas_source_factory_fn();

  // Create the composite texture that slices will blit into.
  composite_texture_ = TextureManager::CreateCompositeTexture(
      view, atlas_texture_size_, atlas_grid_size_);

  // This could be absent if there is only one slice, but on Android would
  // require conditionally swapping materials based on the number of slices.
  TextureManager::CreateAsync(view, atlas_texture_size_, atlas_grid_size_,
                              composite_texture_.get())
      .Then([this](std::unique_ptr<TextureManager> texture_manager) {
        texture_manager_ = std::move(texture_manager);
        pending_renders_.ForEachBit<SliceId>([this](SliceId slice) {
          texture_manager_->PrepareBlit(slice, slices_[slice].texture_);
        });
      })
      .KeptBy(this);

  view.GetDispatcher().Connect(
      [this](const imp::ViewPostFrameUpdateEvent& event) mutable {
        EndFrame();
      },
      this);

  // Hook up to 'PreRender' to handle blitting slices into the composite
  // texture.  This happens late in the impress frame, after updates and
  // beginFrame, but before the main frame is submitted to filament.
  view.GetDispatcher().Connect(
      [this](const ViewPreRenderEvent& pre_render_event) {
        RenderBlits(*pre_render_event.GetRenderer());
      },
      this);

  // Prevents an issue where the texture can get cleared and needs to be redrawn
  // when the view is resumed. This is only relevant to the sliced atlas if
  // there is only one slice. With multiple slices, the external texture is
  // not sampled directly.
  if (config.force_reset_on_view_resumed && (slice_count == 1)) {
    view.GetDispatcher().Connect(
        [this](const ViewResumedEvent& event) {
          // Cannot force reset if there is already an active canvas.
          for (auto& slice : slices_) {
            absl::MutexLock lock(slice.canvas_mutex_);
            if (slice.canvas_ != nullptr) {
              continue;
            }
            slice.canvas_source_->ForceReset();
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
                .get_atlas_info_func =
                    [this]() {
                      std::vector<Texture*> slice_textures;
                      for (auto& slice : slices_) {
                        slice_textures.push_back(slice.texture_);
                      }

                      return editor::SlicedGlyphAtlasVisualizer::AtlasInfo{
                          .texture = GetTexture(),
                          .texture_size = atlas_texture_size_,
                          .grid_size = atlas_grid_size_,
                          .slice_texture_size =
                              atlas_texture_size_ / atlas_grid_size_,
                          .slice_textures = std::move(slice_textures),
                      };
                    },
                .get_glyph_info_func =
                    [this](float2 uv) { return GetGlyphInfoAt(uv); },
                .get_utilization_func =
                    [this]() { return GetAtlasUtilization(); },
                .get_all_glyph_info_func =
                    [this]() { return GetAllGlyphInfo(); }});
  }
#endif
}

void SlicedGlyphAtlas::RenderBlits(filament::Renderer& renderer) {
  if (!texture_manager_) {
    return;
  }

#if IMP_PLATFORM(ANDROID_X86_64) || IMP_PLATFORM(ANDROID_X86_32)
  // A very specific fix for running on Android emulators.
  //
  // The short version is the emulator does not deliver external textures
  // updates reliably (It may contain an old version of the texture, even after
  // updateTexImage() has been called). This is an annoyance for the old atlas
  // (which may return futures a frame or two early before the glyphs are
  // technically visible) but catastrophic for the sliced atlas (which only
  // considers the textures when blitting to the composite texture used in
  // rendering).
  //
  // The workaround is that once a surface as been blitted, we continue to blit
  // it every frame.  This causes the new atlas to render the same way the old
  // atlas did.
  for (auto slice : slices_.Ids()) {
    if (slices_[slice].texture_ != nullptr && !pending_renders_.Get(slice)) {
      // Has a texture; ensure we won't signal early.
      if (slices_[slice].texture_blit_futures_.empty()) {
        pending_renders_.Set(slice, true);
      }
    }
  }
#endif  // IMP_PLATFORM(ANDROID_X86_64) || IMP_PLATFORM(ANDROID_X86_32)

  size_t rendered_count = 0;
  filament::Renderer::ClearOptions previous_clear_options;
  pending_renders_.ForEachBit<SliceId>([&rendered_count, &renderer,
                                        &previous_clear_options,
                                        this](SliceId slice) {
    if (!rendered_count) {
      // Since blitting only affects a portion of the render target, we need to
      // ensure discard/clear are disabled for the target prior to rendering.
      previous_clear_options = renderer.getClearOptions();
      renderer.setClearOptions(filament::Renderer::ClearOptions{
          .clear = false,
          .discard = false,
      });
    }
    texture_manager_->RenderSlice(renderer, slice);
    slices_[slice].OnBlitCompleted();
    ++rendered_count;
  });
  if (rendered_count > 0) {
    renderer.setClearOptions(previous_clear_options);
    pending_renders_.SetAll(false);
  }
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

  shared_canvas_source_.reset();

  texture_manager_.reset();
  composite_texture_.reset();

#if IMP_RUNTIME(DEV)
  if (auto editor = view_.GetRegistry().Get<editor::Editor>(); editor.ok()) {
    editor->get()
        .GetWidgetUiSystem()
        .RemoveWidget<editor::SlicedGlyphAtlasVisualizer>();
  }
#endif
}

void SlicedGlyphAtlas::EndFrame() {
  // A slice requests a blit because glyphs were added to it, but this is a
  // heavyweight operation.  To prevent jank, we limit the number of slices
  // that can request blits in each frame.
  constexpr size_t kMaxSlicesRenderedCount = 2;
  size_t slices_rendered_count = 0;
  for (auto& slice : slices_) {
    Slice::EndFrameResult result = slice.EndFrame(view_);
    if (result == Slice::EndFrameResult::kBlitRequired) {
      // Schedule a blit.
      pending_renders_.Set(slices_.IdOf(slice), true);
      if (texture_manager_) {
        texture_manager_->PrepareBlit(slices_.IdOf(slice), slice.texture_);
      }
      if (++slices_rendered_count >= kMaxSlicesRenderedCount) {
        break;
      }
    }
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
  return glyph_emulator_.PrepareFont(text, *canvas_options,
                                     *shared_canvas_source_);
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
  return glyph_emulator_.GetCombinedCharacterGroups(text, *canvas_options,
                                                    *shared_canvas_source_);
}

Future<TextMetrics> SlicedGlyphAtlas::GetTextMetrics(
    absl::string_view text, const TextOptions& options) {
  absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
      glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(options,
                                                            float2(1.0f));
  if (!canvas_options.ok()) {
    return Future<TextMetrics>(canvas_options.status());
  }
  return glyph_emulator_.GetTextMetrics(text, *canvas_options,
                                        *shared_canvas_source_);
}

Future<FontInfo> SlicedGlyphAtlas::GetFontInfo(const TextOptions& options) {
  absl::StatusOr<ScopedCanvas::TextOptions> canvas_options =
      glyph_emulator_.CanvasOptionsFromGlyphEmulatorOptions(options);
  if (!canvas_options.ok()) {
    return Future<FontInfo>(canvas_options.status());
  }
  return glyph_emulator_.GetFontInfo(*canvas_options, *shared_canvas_source_);
}

Future<TextGlyphs> SlicedGlyphAtlas::GetTextGlyphs(
    absl::string_view text, const GlyphEmulator::TextOptions& options) {
  return GetGlyphs(text, options)
      .Then([this](std::vector<SlicedGlyphAtlas::Glyph> glyphs) {
        // The sign bit of the v texture coordinate is used by materials to
        // detect emoji.
        constexpr float4 kEmojiScale = float4(1.f, -1.f, 1.f, -1.f);
        TextGlyphs text_glyphs;
        text_glyphs.Reserve(glyphs.size());
        for (const auto& glyph : glyphs) {
          float2 slice_offset, slice_scale;
          GetSliceOffsetAndScale(glyph.slice, &slice_offset, &slice_scale);
          const RefCounter::Ref& glyph_ref = glyph.glyph_ref;
          float4 atlas_origin_and_size =
              float4(glyph.atlas_origin, glyph.atlas_size);
          float4 actual_origin_and_size =
              float4(glyph.actual_origin, glyph.actual_size);
          float4 uv_origin_and_size =
              float4(slice_offset + glyph.uv_top_left * slice_scale,
                     glyph.uv_size * slice_scale);
          float advance_width = glyph.advance_width;

          if (glyph.is_emoji) uv_origin_and_size *= kEmojiScale;

          text_glyphs.PushBack(glyph_ref, atlas_origin_and_size,
                               actual_origin_and_size, uv_origin_and_size,
                               advance_width);
        }
        return text_glyphs;
      });
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

            return glyph_emulator_
                .GetGlyphs(text, *canvas_options, *shared_canvas_source_)
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
                  Future<absl::Status> result_future;
                  std::vector<Future<absl::Status>> entry_futures;
                  for (auto& entry : pending_canvas_glyphs->entries) {
                    Future<absl::Status> entry_future;
                    auto& slice = slices_[entry.slice];
                    if (slice.canvas_source_->IsFeatureSupported(
                            ScopedCanvas::Feature::kKeepContents)) {
                      {
                        absl::MutexLock lock(slice.canvas_mutex_);
                        slice.GetOrStartDrawing(
                            view_, ScopedCanvas::DrawMode::kKeepContents);
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
                          .DrawGlyphsToCanvasAsync(view_,
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
                      slice.GetOrStartDrawing(view_,
                                              ScopedCanvas::DrawMode::kClear);
                      slice.texture_status_ = TextureStatus::kHasNewGlyphs;
                      entry_future.Return(absl::OkStatus());
                    }
                    absl::MutexLock lock(slice.canvas_mutex_);
                    entry_futures.push_back(entry_future);
                  }
                  result_future =
                      Future<absl::Status>::CombineList(entry_futures);
                  return result_future
                      .Then([this, pending_canvas_glyphs =
                                       std::move(pending_canvas_glyphs)]() {
                        std::vector<Future<absl::Status>> slice_futures;
                        for (auto& entry : pending_canvas_glyphs->entries) {
                          auto& slice = slices_[entry.slice];
                          bool canvas_is_null;
                          {
                            absl::MutexLock lock(slice.canvas_mutex_);
                            canvas_is_null = slice.canvas_ == nullptr;
                            if (canvas_is_null) {
                              slice.texture_status_ = TextureStatus::kStable;
                            }
                          }

                          // Block resolution of the glyphs until the texture
                          // has been updated and the glyphs can be safely used.
                          // With the SlicedGlyphAtlas this always blocks, since
                          // the glyphs are not available until the external
                          // texture has been blitted to the composite texture.
                          Future<absl::Status> pending_glyph_future;
                          slice.texture_update_futures_.push_back(
                              pending_glyph_future);
                          slice_futures.push_back(pending_glyph_future);
                        }
                        return Future<absl::Status>::CombineList(slice_futures);
                      })
                      .Then([glyphs = std::move(result)]() { return glyphs; });
                });
          },
          Executor::Type::kCurrent);
}

std::optional<SlicedGlyphAtlas::SlicedGlyphInfo> SlicedGlyphAtlas::GetGlyphInfo(
    const CanvasOptionsGlyphKey& key, const CanvasOptionsInfo& info) {
  std::optional<SlicedGlyphAtlas::SlicedGlyphInfo> result = std::nullopt;
  info.active_slices.ForEachBit<SliceId>([this, &key, &result](SliceId slice) {
    const GlyphInfo* glyph_info = slices_[slice].GetGlyphInfo(key);
    if (glyph_info != nullptr) {
      result.emplace(SlicedGlyphInfo{.slice = slice, .info = *glyph_info});
    }
  });
  return result;
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
    bool needs_render = false;
    const float width = glyph.advance_width;

    std::optional<SlicedGlyphInfo> sliced_glyph_info =
        GetOrAddGlyphInfo(glyph, pending_glyph, needs_render);
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

SlicedGlyphAtlas::CanvasOptionsInfo& SlicedGlyphAtlas::GetCanvasOptionsInfo(
    const CanvasOptionsKey& canvas_options_key) {
  absl::MutexLock lock(canvas_options_map_mutex_);
  SlicedGlyphAtlas::CanvasOptionsInfo& result =
      canvas_options_map_[canvas_options_key];
  if (result.active_slices.empty()) {
    result.active_slices.Resize(slices_.size());
    result.glyph_count.resize(slices_.size(), 0);
  }
  return result;
}

std::optional<SlicedGlyphAtlas::SlicedGlyphInfo>
SlicedGlyphAtlas::GetOrAddGlyphInfo(GlyphEmulator::Glyph& glyph,
                                    const CanvasOptionsGlyphKey& glyph_key,
                                    bool& out_added_new_glyph) {
  CanvasOptionsInfo& canvas_options_info = GetCanvasOptionsInfo(
      CanvasOptionsKey{.canvas_options = glyph_key.canvas_options,
                       .requires_color_key = glyph_key.requires_color_key});

  std::optional<SlicedGlyphInfo> glyph_info =
      GetGlyphInfo(glyph_key, canvas_options_info);
  if (glyph_info.has_value()) {
    out_added_new_glyph = false;
    return *glyph_info;
  }

  // Atlas entry is integer precision with some padding.
  uint2 atlas_entry_size = {
      static_cast<int>(std::ceil(glyph.metrics.size_x())) + kPadding,
      static_cast<int>(std::ceil(glyph.metrics.font_size_y())) + kPadding};

  std::optional<SlicedAtlasEntry> atlas_entry =
      TryAddAtlasEntry(atlas_entry_size, canvas_options_info, glyph_key);

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
  out_added_new_glyph = true;
  return SlicedGlyphInfo{.slice = atlas_entry->slice, .info = *info};
}

std::optional<SlicedGlyphAtlas::SlicedAtlasEntry>
SlicedGlyphAtlas::TryAddAtlasEntry(uint2 atlas_entry_size,
                                   CanvasOptionsInfo& info,
                                   const CanvasOptionsGlyphKey& glyph_key) {
  std::optional<SlicedGlyphAtlas::SlicedAtlasEntry> result = std::nullopt;

  // Prefer slices that already contain these options.  This groups glyphs with
  // the same canvas options together, which not only minimizes the number of
  // slices we need to check when searching for a glyph, but also improves the
  // performance of rendering a slice by minimizing JNI state changes on the
  // canvas paint objects.
  info.active_slices.ForEachBit<SliceId>(
      [this, &atlas_entry_size, &result](SliceId slice) {
        absl::MutexLock lock(slices_[slice].atlas_packer_mutex_);
        std::optional<AtlasPacker::ScopedAtlasEntry> atlas_entry =
            slices_[slice].atlas_packer_.AddEntry(atlas_entry_size);
        if (atlas_entry) {
          result.emplace(SlicedAtlasEntry{.slice = slice,
                                          .entry = std::move(*atlas_entry)});
        }
      });

  if (result.has_value()) {
    ++info.glyph_count[result->slice];
    return result;
  }

  SliceId begin_slice = SliceId::At(0);
  SliceId end_slice = SliceId::At(slices_.size());
  SliceId cursor = addition_cursor_;

  // Attempt to add to a new slice starting from the (rotating) addition cursor.
  do {
    // Skip slices we've already checked.
    if (!info.active_slices.Get(cursor)) {
      absl::MutexLock lock(slices_[cursor].atlas_packer_mutex_);
      std::optional<AtlasPacker::ScopedAtlasEntry> atlas_entry =
          slices_[cursor].atlas_packer_.AddEntry(atlas_entry_size);
      if (atlas_entry) {
        result.emplace(SlicedAtlasEntry{.slice = cursor,
                                        .entry = std::move(*atlas_entry)});
      }
    }
    // Handle wrapping the id as we cross the end.
    if (++cursor == end_slice) cursor = begin_slice;
  } while (!result.has_value() && cursor != addition_cursor_);

  // Rotate the addition cursor after every add.
  if (++addition_cursor_ == end_slice) addition_cursor_ = begin_slice;

  if (result.has_value()) {
    if (!(info.glyph_count[result->slice]++))
      info.active_slices.Set(result->slice);
    return result;
  }

  // The atlas was full. Clear unused glyphs and try again.
  ClearUnusedGlyphs();

  // Clearing glyphs can invalidate the info handle; refresh the reference.
  info = GetCanvasOptionsInfo(
      CanvasOptionsKey{.canvas_options = glyph_key.canvas_options,
                       .requires_color_key = glyph_key.requires_color_key});

  for (auto& slice : slices_) {
    absl::MutexLock lock(slice.atlas_packer_mutex_);

    std::optional<AtlasPacker::ScopedAtlasEntry> atlas_entry =
        slice.atlas_packer_.AddEntry(atlas_entry_size);
    if (atlas_entry) {
      result.emplace(SlicedAtlasEntry{.slice = slices_.IdOf(slice),
                                      .entry = std::move(*atlas_entry)});
      break;
    }
  }

  if (result.has_value()) {
    if (!(info.glyph_count[result->slice]++))
      info.active_slices.Set(result->slice);
    return result;
  } else {
    IMP_LOG(imp::ERROR) << "Unable to find space in Glyph Atlas for glyph with size x="
               << atlas_entry_size.x << ", y=" << atlas_entry_size.y;
  }

  return std::nullopt;
}

void SlicedGlyphAtlas::ClearUnusedGlyphs() {
  for (auto& slice : slices_) {
    absl::MutexLock lock(slice.atlas_packer_mutex_);
    slice.ClearUnusedGlyphs(view_, [this, id = slices_.IdOf(slice)](
                                       const CanvasOptionsGlyphKey& glyph_key) {
      CanvasOptionsInfo& canvas_options_info = GetCanvasOptionsInfo(
          CanvasOptionsKey{.canvas_options = glyph_key.canvas_options,
                           .requires_color_key = glyph_key.requires_color_key});
      if (!--canvas_options_info.glyph_count[id]) {
        canvas_options_info.active_slices.Set(id, false);
      }
    });
  }

  // Holding the mutex, clear all infos that are now empty.
  {
    absl::MutexLock lock(canvas_options_map_mutex_);

    absl::erase_if(canvas_options_map_, [](const auto& it) {
      return !it.second.active_slices.Any();
    });
  }
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

imp::Texture* SlicedGlyphAtlas::GetTexture() {
  if (Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "GetTexture may only be called from the foreground "
                  "thread.";
  }
  // If the grid size is 1x1, then we don't need to composite the textures, we
  // just return the external texture.
  if (atlas_grid_size_.x == 1 && atlas_grid_size_.y == 1) {
    return slices_.front().texture_;
  } else {
    return composite_texture_.get();
  }
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

void SlicedGlyphAtlas::GetSliceOffsetAndScale(SliceId slice, float2* offset,
                                              float2* scale) const {
  return sliced_glyph_atlas::GetGridSliceOffsetAndScale(atlas_grid_size_, slice,
                                                        offset, scale);
}

#if IMP_RUNTIME(DEV)
std::optional<editor::SlicedGlyphAtlasVisualizer::GlyphInfo>
SlicedGlyphAtlas::GetGlyphInfoAt(const float2& uv) const {
  for (auto& slice : slices_) {
    float2 slice_offset, slice_scale;
    GetSliceOffsetAndScale(slices_.IdOf(slice), &slice_offset, &slice_scale);
    auto result = slice.GetGlyphInfoAt(uv, slice_offset, slice_scale);
    if (result.has_value()) {
      return result;
    }
  }

  return std::nullopt;
}

std::vector<editor::SlicedGlyphAtlasVisualizer::GlyphInfo>
SlicedGlyphAtlas::GetAllGlyphInfo() const {
  std::vector<editor::SlicedGlyphAtlasVisualizer::GlyphInfo> result;
  for (auto& slice : slices_) {
    float2 slice_offset, slice_scale;
    GetSliceOffsetAndScale(slices_.IdOf(slice), &slice_offset, &slice_scale);
    auto slice_result = slice.GetAllGlyphInfo(slice_offset, slice_scale);
    result.insert(result.end(), slice_result.begin(), slice_result.end());
  }
  return result;
}
#endif

}  // namespace imp
