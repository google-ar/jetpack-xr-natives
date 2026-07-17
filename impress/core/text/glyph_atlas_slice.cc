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

#include "core/text/glyph_atlas_slice.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iterator>
#include <memory>
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
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/atlas/atlas_packer.h"
#include "core/atlas/shelf_atlas_packer.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/ref_counter.h"
#include "core/common/small_source_location.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/geometry/shapes/rect.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/render/texture.h"
#include "core/text/glyph_emulator.h"
#include "core/view/base_view.h"

#if IMP_RUNTIME(DEV)
#include "core/editor/editor.h"
#include "core/editor/layout/editor_panel_ids.h"
#include "core/editor/widget_ui_system.h"
#include "core/text/editor/sliced_glyph_atlas_visualizer.h"
#endif

namespace imp {
namespace sliced_glyph_atlas {
namespace {

#if IMP_RUNTIME(DEV)
bool RectContainsPoint(const Rect& rect, const float2& point) {
  return ((std::abs(rect.center.x - point.x) <= (rect.half_extent.x)) &&
          (std::abs(rect.center.y - point.y) <= (rect.half_extent.y)));
}
#endif

constexpr uint8_t kPadding = 8;
constexpr uint8_t kHalfPadding = kPadding / 2;

}  // namespace

AsyncScopedCanvas* Slice::GetOrStartDrawing(imp::BaseView& view,
                                            ScopedCanvas::DrawMode draw_mode,
                                            bool single_slice) {
  if (canvas_) {
    return canvas_.get();
  }

  canvas_ = canvas_source_->StartDrawing(
      view, texture_size_,
      [this](BorrowedTexturePtr texture) { borrowed_texture_ = texture; },
      draw_mode, SmallSourceLocation::Current());
  if (canvas_->DidTextureChange()) {
    texture_ = canvas_->GetTexture();
    if (single_slice) {
      view.GetDispatcher().Send(TextureChangedEvent());
    }
  }

  return canvas_.get();
}

Slice::Slice(uint2 texture_size,
             std::unique_ptr<AsyncCanvasSource> canvas_source)
    : texture_size_(texture_size),
      atlas_packer_(texture_size),
      canvas_source_(std::move(canvas_source)) {}

Slice::~Slice() {
  {
    absl::MutexLock lock(glyph_map_mutex_);
    glyph_map_.clear();
  }

  {
    absl::MutexLock lock(canvas_mutex_);
    borrowed_texture_ = nullptr;
    canvas_.reset();
  }
}

Slice::EndFrameResult Slice::EndFrame(imp::BaseView& view, bool single_slice) {
  IMP_TRACE();
  canvas_mutex_.lock();
  if (texture_status_ == TextureStatus::kStable ||
      texture_status_ == TextureStatus::kPreparingToUpdateTexture) {
    canvas_mutex_.unlock();
    return EndFrameResult::kStable;
  }

  // Intentionally check for kReadyToBlit first, so that it gets acted on the
  // frame after it is set.  This prevents occasional issues where blitting
  // occurs before the external texture has been updated
  if (texture_status_ == TextureStatus::kReadyToBlit) {
    canvas_mutex_.unlock();
    return EndFrameResult::kBlitRequired;
  }

  if (!canvas_ &&
      canvas_source_->IsFeatureSupported(
          ScopedCanvas::Feature::kKeepContents) &&
      texture_status_ != TextureStatus::kReadyToApplyDrawCommands) {
    canvas_mutex_.unlock();
    return EndFrameResult::kStable;
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
    DrawAllGlyphsToCanvas(view, single_slice);
    canvas_mutex_.lock();
    texture_status_ = TextureStatus::kReadyToApplyDrawCommands;
  }

  if (texture_status_ == TextureStatus::kReadyToApplyDrawCommands) {
    if (!canvas_) {
      GetOrStartDrawing(view, ScopedCanvas::DrawMode::kKeepContents,
                        single_slice);
    }
    UpdateTexture();
  }

  auto result = EndFrameResult::kStable;

  canvas_mutex_.unlock();

  return result;
}

const GlyphInfo* /*absl_nullable*/  Slice::GetGlyphInfo(
    const CanvasOptionsGlyphKey& glyph_key) {
  absl::MutexLock glyph_map_lock(glyph_map_mutex_);
  // Try to find the already cached main glyph entry for this text.
  auto itr = glyph_map_.find(glyph_key);
  if (itr != glyph_map_.end()) {
    return &itr->second;
  }
  return nullptr;
}

void Slice::DrawAllGlyphsToCanvas(imp::BaseView& view, bool single_slice) {
  ScopedCanvas* canvas;
  {
    absl::MutexLock lock(canvas_mutex_);
    canvas =
        GetOrStartDrawing(view, ScopedCanvas::DrawMode::kClear, single_slice);
    // TODO : Canvas can may be dirty due to async drawing of
    // glyphs; investigate how to prevent that from happening.
    canvas->ClearRect(
        {.center = texture_size_ / 2.0f, .half_extent = texture_size_ / 2.0f});
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

Future<absl::Status> Slice::DrawGlyphsToCanvasAsync(
    imp::BaseView& view,
    std::unique_ptr<std::vector<CanvasOptionsGlyphKey>> glyphs,
    bool single_slice) {
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
      .Then([this, single_slice,
             &view](std::unique_ptr<std::vector<CanvasOptionsGlyphKey>> glyphs)
                -> Future<absl::Status> {
        {
          absl::MutexLock canvas_lock(canvas_mutex_);
          // This means all the glyphs newly added to the atlas has now been
          // drawn to the canvas, and so the only thing left to do is to release
          // the pixel buffer in the canvas.
          if (glyphs->empty()) {
            if (texture_status_ == TextureStatus::kStable) {
              texture_status_ = TextureStatus::kReadyToApplyDrawCommands;
            } else {
              // Defer the state transition until the current update/blit cycle
              // completes.
              has_pending_draw_commands_ = true;
            }
            return Future<absl::Status>(absl::OkStatus());
          }
          // If the status is an error, it means that the glyphs haven't
          // completed drawing to the canvas, which indicates the canvas was
          // destroyed somewhere during the process. Recreate the canvas and try
          // again.
          GetOrStartDrawing(view, ScopedCanvas::DrawMode::kKeepContents,
                            single_slice);
        }
        return DrawGlyphsToCanvasAsync(view, std::move(glyphs), single_slice);
      });
}

absl::Status Slice::DrawGlyphsToCanvas(
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

void Slice::ClearUnusedGlyphs(
    imp::BaseView& view, bool single_slice,
    std::function<void(const CanvasOptionsGlyphKey&)> glyph_cleared_fn) {
  absl::MutexLock glyph_map_lock(glyph_map_mutex_);
  // Remove each glyph that is unused. Detect if it's unused if the ref
  // counter is at zero.

  absl::erase_if(glyph_map_, [this, single_slice, &view,
                              &glyph_cleared_fn](const auto& it) {
    if (it.second.ref_counter.GetCount() != 0) {
      return false;
    }

    // If we are retaining the canvas's content then we need to ensure
    // that the old entries are cleared from the canvas for when the
    // texture memory is re-used in the future.
    if (canvas_source_->IsFeatureSupported(
            ScopedCanvas::Feature::kKeepContents)) {
      absl::MutexLock canvas_lock(canvas_mutex_);
      const GlyphInfo& glyph_info = it.second;
      ScopedCanvas* canvas = GetOrStartDrawing(
          view, ScopedCanvas::DrawMode::kKeepContents, single_slice);

      const AtlasPacker::ScopedAtlasEntry& atlas_entry = glyph_info.atlas_entry;
      float2 half_extent =
          (atlas_entry.GetBottomRight() - atlas_entry.GetTopLeft()) / 2.0f;
      Rect rect{.center = atlas_entry.GetTopLeft() + half_extent,
                .half_extent = half_extent};
      canvas->ClearRect(rect);
    }
    // Immediately before the glyph actually gets removed, inform the caller
    // about the removed glyph via the callback function.
    glyph_cleared_fn(it.first);
    return true;
  });
}

size_t Slice::GetNumCachedGlyphs() const {
  absl::MutexLock glyph_map_lock(glyph_map_mutex_);
  return glyph_map_.size();
}

void Slice::DrawGlyphToCanvas(ScopedCanvas& canvas, CanvasOptionsGlyphKey glyph,
                              const GlyphInfo& glyph_info) {
  if (glyph_info.fallback_font) {
    glyph.canvas_options.font_holder = glyph_info.fallback_font.get();
  }
  float2 glyph_top_left =
      glyph_info.atlas_entry.GetTopLeft() + float2{kHalfPadding};
  GlyphEmulator::DrawGlyph(canvas, glyph.glyph, glyph_top_left,
                           glyph.canvas_options, &glyph_info.measurements);
}

void Slice::UpdateTexture() {
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

void Slice::UpdateTextureSync() {
  texture_status_ = TextureStatus::kReadyToBlit;
  canvas_.reset();
#if IMP_PLATFORM(WASM)
  // TODO: Properly resolve this race condition.
  for (const auto& future : texture_update_futures_) {
    future.Return(absl::OkStatus());
  }
#else
  absl::c_move(texture_update_futures_,
               std::back_inserter(texture_blit_futures_));
#endif  // IMP_PLATFORM(WASM)
  texture_update_futures_.clear();
}

void Slice::OnBlitCompleted() {
  absl::MutexLock lock(canvas_mutex_);
  for (const auto& future : texture_blit_futures_) {
    future.Return(absl::OkStatus());
  }
  texture_blit_futures_.clear();
  // Only progress the state forward if we are still in the state that requested
  // the blit (and populated the blit futures). This prevents stomping out of
  // e.g. kHasNewGlyphs which can happen if new glyphs are added to the slice
  // while waiting for the blit to complete.
  if (texture_status_ == TextureStatus::kReadyToBlit) {
    if (has_pending_draw_commands_) {
      texture_status_ = TextureStatus::kReadyToApplyDrawCommands;
      has_pending_draw_commands_ = false;
    } else {
      texture_status_ = TextureStatus::kStable;
    }
  }
}

float Slice::GetAtlasUtilization() const {
  float utilization;
  {
    absl::MutexLock lock(atlas_packer_mutex_);
    utilization = atlas_packer_.GetUtilization();
  }
  return utilization;
}

void Slice::OnViewResumed() {
  absl::MutexLock lock(canvas_mutex_);
  texture_status_ = TextureStatus::kHasNewGlyphs;
}

std::string Slice::ToString(const GlyphEmulator::GlyphKeyOrGlyphString& glyph) {
  if (absl::holds_alternative<GlyphEmulator::GlyphKey>(glyph)) {
    GlyphEmulator::GlyphKey glyph_key =
        absl::get<GlyphEmulator::GlyphKey>(glyph);
    return absl::StrFormat("Id=%i, Font=%i", glyph_key.glyph_id.Get(),
                           glyph_key.font_id);
  } else {
    return absl::get<std::string>(glyph);
  }
}

#if IMP_RUNTIME(DEV)
std::optional<editor::SlicedGlyphAtlasVisualizer::GlyphInfo>
Slice::GetGlyphInfoAt(const float2& uv, const float2& slice_offset,
                      const float2& slice_scale) const {
  float2 slice_uv = (uv - slice_offset) / slice_scale;
  float2 texture_point = slice_uv * texture_size_;

  {
    absl::MutexLock lock(glyph_map_mutex_);
    for (auto& [glyph, glyph_info] : glyph_map_) {
      const AtlasPacker::ScopedAtlasEntry& atlas_entry = glyph_info.atlas_entry;
      float2 half_extent =
          (atlas_entry.GetBottomRight() - atlas_entry.GetTopLeft()) / 2.0f;
      Rect rect{.center = atlas_entry.GetTopLeft() + half_extent,
                .half_extent = half_extent};

      if (RectContainsPoint(rect, texture_point)) {
        rect.half_extent /= texture_size_;
        rect.center /= texture_size_;

        // Adjust for slice.
        rect.center = slice_offset + rect.center * slice_scale;
        rect.half_extent = rect.half_extent * slice_scale;

        float half_stroke_width = glyph_info.stroke_width / 2.0f;
        float2 origin =
            float2(atlas_entry.GetTopLeft().x +
                       glyph_info.measurements.origin_x() + half_stroke_width +
                       kHalfPadding,
                   // +Y is up. Start counting from the bottom of the glyph.
                   atlas_entry.GetBottomRight().y +
                       glyph_info.measurements.font_origin_y() -
                       half_stroke_width - kHalfPadding) /
            texture_size_;
        return editor::SlicedGlyphAtlasVisualizer::GlyphInfo{
            .glyph = ToString(glyph.glyph),
            .is_stroke = false,
            .origin = origin,
            .uv = rect};
      }
    }
  }
  return std::nullopt;
}

std::vector<editor::SlicedGlyphAtlasVisualizer::GlyphInfo>
Slice::GetAllGlyphInfo(const float2& slice_offset,
                       const float2& slice_scale) const {
  std::vector<editor::SlicedGlyphAtlasVisualizer::GlyphInfo> result;
  {
    absl::MutexLock lock(glyph_map_mutex_);
    for (auto& [glyph, glyph_info] : glyph_map_) {
      const AtlasPacker::ScopedAtlasEntry& atlas_entry = glyph_info.atlas_entry;
      float2 half_extent =
          (atlas_entry.GetBottomRight() - atlas_entry.GetTopLeft()) / 2.0f;
      Rect rect{.center = atlas_entry.GetTopLeft() + half_extent,
                .half_extent = half_extent};
      rect.half_extent /= texture_size_;
      rect.center /= texture_size_;
      rect.center = slice_offset + rect.center * slice_scale;
      rect.half_extent = rect.half_extent * slice_scale;
      float half_stroke_width = glyph_info.stroke_width / 2.0f;
      float2 origin =
          float2(atlas_entry.GetTopLeft().x +
                     glyph_info.measurements.origin_x() + half_stroke_width +
                     kHalfPadding,
                 // +Y is up. Start counting from the bottom of the glyph.
                 atlas_entry.GetBottomRight().y +
                     glyph_info.measurements.font_origin_y() -
                     half_stroke_width - kHalfPadding) /
          texture_size_;
      result.push_back(editor::SlicedGlyphAtlasVisualizer::GlyphInfo{
          .glyph = ToString(glyph.glyph),
          .is_stroke = false,
          .origin = origin,
          .uv = rect});
    }
  }
  return result;
}
#endif

}  // namespace sliced_glyph_atlas
}  // namespace imp
