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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_SLICE_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_SLICE_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/synchronization/mutex.h"
#include "core/async/future.h"
#include "core/atlas/atlas_packer.h"
#include "core/atlas/shelf_atlas_packer.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/ref_counter.h"
#include "core/common/typed_id.h"
#include "core/config.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/render/texture.h"
#include "core/text/glyph_emulator.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/base_view.h"

#if IMP_RUNTIME(DEV)
#include "core/text/editor/sliced_glyph_atlas_visualizer.h"
#endif

namespace imp {
namespace sliced_glyph_atlas {
struct Slice;

using SliceId = TypedId<Slice, uint8_t>;

// Event that is sent when the texture changes.
// Users of the SlicedGlyphAtlas should use this event to update the texture
// assignments on their materials.
class TextureChangedEvent : public Event {};

// The size of the texture to use for the glyph atlas.
enum class TextureSize {
  // Uses a 2048x2048 texture for the glyph atlas.
  k2048,
  // Uses a 2048x4096 texture for the glyph atlas.
  k2048_4096,
  // Uses a 4096x4096 texture for the glyph atlas.
  k4096,
  // Experimental: 256x256 textures in an 8x8 grid.
  k256_256_8_8,
};

// Information about a glyph needed to render it and lay it out relative to
// other glyphs in a string.
struct Glyph {
  // The origin of the font as relative to the actual, visible size. Used to
  // calculate bounds of the TextRenderer.
  float2 actual_origin;
  // The actual, visible size of the glyph. Used to calculate bounds of the
  // TextRenderer.
  float2 actual_size;
  // The size in pixels of the glyph in the atlas.
  // Used to correctly scale the mesh used to render the glyph.
  float2 atlas_size;
  // The width of the glyph within the layout of a string.
  // This is not the same thing as size.x, since the advance_width is
  // dependent on adjacent glyphs in the overall string being rendered.
  float advance_width;
  // The origin point of the glyph in the atlas. The origin normally moves
  // along the baseline to the next advance width so we need this to offset
  // the draw position.
  float2 atlas_origin;
  // The top left of the glyph entry on the texture in normalized uv
  // coordinates.
  float2 uv_top_left;
  // The size of the glyph entry on the texture in normalized uv coordinates.
  // uv_bottom_right would be uv_top_left + uv_size.
  float2 uv_size;
  // If the glyph is an emoji then we should render its color from the atlas.
  bool is_emoji;
  // The slice that this glyph is on.
  SliceId slice;
  // Used to track how many references to this glyph there are.
  RefCounter::Ref glyph_ref;
};

// Like CanvasOptionsGlyphKey, but without the glyph.
struct CanvasOptionsKey {
  // Options that control how the text is drawn.
  ScopedCanvas::TextOptions canvas_options;
  // If the glyph contains an emoji and non-separable script, then we need to
  // color it in the user's requested color. We need to update the key so that
  // the color is part of the key, because each instance of the same glyph
  // text will have to be rendered independently.
  bool requires_color_key;

  // Needed for using this type as a key within a map.
  bool operator==(const CanvasOptionsKey& other) const {
    bool eq = (canvas_options.stroke_width_pixels ==
                   other.canvas_options.stroke_width_pixels &&
               canvas_options.font_holder == other.canvas_options.font_holder &&
               canvas_options.size_pixels == other.canvas_options.size_pixels &&
               // text_tracking can affect the width of cursive glyphs.
               AlmostEqual(canvas_options.text_tracking,
                           other.canvas_options.text_tracking));
    if (requires_color_key) {
      return eq && canvas_options.color == other.canvas_options.color &&
             canvas_options.stroke_color == other.canvas_options.stroke_color;
    }
    return eq;
  }

  // Needed for using this type as a key within a map.
  template <typename H>
  friend H AbslHashValue(H hash, const CanvasOptionsKey& canvas_options_key) {
    if (canvas_options_key.requires_color_key) {
      return H::combine(std::move(hash),
                        canvas_options_key.canvas_options.stroke_width_pixels,
                        canvas_options_key.canvas_options.font_holder,
                        canvas_options_key.canvas_options.size_pixels,
                        canvas_options_key.canvas_options.text_tracking,
                        canvas_options_key.canvas_options.color.x,
                        canvas_options_key.canvas_options.color.y,
                        canvas_options_key.canvas_options.color.z,
                        canvas_options_key.canvas_options.color.w,
                        canvas_options_key.canvas_options.stroke_color.x,
                        canvas_options_key.canvas_options.stroke_color.y,
                        canvas_options_key.canvas_options.stroke_color.z,
                        canvas_options_key.canvas_options.stroke_color.w);
    }
    return H::combine(std::move(hash),
                      canvas_options_key.canvas_options.stroke_width_pixels,
                      canvas_options_key.canvas_options.font_holder,
                      canvas_options_key.canvas_options.size_pixels,
                      canvas_options_key.canvas_options.text_tracking);
  }
};

// Type that can be used as the key for a map and contains the
// CanvasSource options for drawing glyphs derived from a particular
// TextOption and the glyph itself. It's guaranteed that no two glyphs fetched
// with different CanvasOptionsGlyphKey will look the same when drawn.
struct CanvasOptionsGlyphKey {
  // Options that control how the text is drawn.
  ScopedCanvas::TextOptions canvas_options;
  GlyphEmulator::GlyphKeyOrGlyphString glyph;
  // If the glyph contains an emoji and non-separable script, then we need to
  // color it in the user's requested color. We need to update the key so that
  // the color is part of the key, because each instance of the same glyph
  // text will have to be rendered independently.
  bool requires_color_key;

  // Needed for using this type as a key within a map.
  bool operator==(const CanvasOptionsGlyphKey& other) const {
    bool eq = (glyph == other.glyph &&
               canvas_options.stroke_width_pixels ==
                   other.canvas_options.stroke_width_pixels &&
               canvas_options.font_holder == other.canvas_options.font_holder &&
               canvas_options.size_pixels == other.canvas_options.size_pixels &&
               // text_tracking can affect the width of cursive glyphs.
               AlmostEqual(canvas_options.text_tracking,
                           other.canvas_options.text_tracking));
    if (requires_color_key) {
      return eq && canvas_options.color == other.canvas_options.color &&
             canvas_options.stroke_color == other.canvas_options.stroke_color;
    }
    return eq;
  }

  // Needed for using this type as a key within a map.
  template <typename H>
  friend H AbslHashValue(H hash,
                         const CanvasOptionsGlyphKey& canvas_options_key) {
    if (canvas_options_key.requires_color_key) {
      return H::combine(std::move(hash), canvas_options_key.glyph,
                        canvas_options_key.canvas_options.stroke_width_pixels,
                        canvas_options_key.canvas_options.font_holder,
                        canvas_options_key.canvas_options.size_pixels,
                        canvas_options_key.canvas_options.text_tracking,
                        canvas_options_key.canvas_options.color.x,
                        canvas_options_key.canvas_options.color.y,
                        canvas_options_key.canvas_options.color.z,
                        canvas_options_key.canvas_options.color.w,
                        canvas_options_key.canvas_options.stroke_color.x,
                        canvas_options_key.canvas_options.stroke_color.y,
                        canvas_options_key.canvas_options.stroke_color.z,
                        canvas_options_key.canvas_options.stroke_color.w);
    }
    return H::combine(std::move(hash), canvas_options_key.glyph,
                      canvas_options_key.canvas_options.stroke_width_pixels,
                      canvas_options_key.canvas_options.font_holder,
                      canvas_options_key.canvas_options.size_pixels,
                      canvas_options_key.canvas_options.text_tracking);
  }
};

// A single glyph may contain a separate atlas entry for the main part of the
// text and the stroke.
struct GlyphInfo {
  AtlasPacker::ScopedAtlasEntry atlas_entry;

  // When drawing the glyph to the atlas it gets drawn relative to the origin
  // which we want to get rid of for some glyphs with massive origin offsets.
  TextMetrics measurements;

  // If this glyph was not available in the font from CanvasOptionsKey,
  // then this is the fallback font that must be used to draw the glyph.
  // This typically occurs for text that mixes CJK languages with latin
  // languages.
  std::unique_ptr<FontHolder> fallback_font;
  // Tracks how many usages of this glyph there are.
  RefCounter ref_counter;

#if IMP_RUNTIME(DEV)
  // Used by the glyph atlas visualizer to account for the extra space around
  // glyphs when drawing origins and visual bounds.
  float stroke_width;
#endif
};

// Maps a unique identifier for a glyph to information about that glyph in the
// atlas.
using GlyphMap = absl::flat_hash_map<CanvasOptionsGlyphKey, GlyphInfo>;

// The status of canvas_ texture.
enum TextureStatus {
  // All glyphs in the atlas are currently in the texture.
  kStable,

  // There are glyphs in the atlas that are not yet drawn.
  kHasNewGlyphs,

  // There are pending draw commands that are ready to be applied to the
  // texture.
  kReadyToApplyDrawCommands,

  // There is an active asynchronous request to prepare the texture's pixels
  // to be released. This stage only occurs for canvases that do not support
  // synchronous texture updates.
  kPreparingToUpdateTexture,

  // The external texture is ready for blitting to the composite texture.
  // This only occurs when there is more than one slice.
  kReadyToBlit,
};

struct PendingCanvasGlyphs {
  struct Entry {
    SliceId slice;
    std::unique_ptr<std::vector<CanvasOptionsGlyphKey>> glyphs;
  };
  std::vector<Entry> entries;

  bool empty() const { return entries.empty(); }
  std::vector<CanvasOptionsGlyphKey>& Get(SliceId slice) {
    for (Entry& entry : entries) {
      if (entry.slice == slice) {
        return *entry.glyphs;
      }
    }
    entries.push_back(Entry{
        .slice = slice,
        .glyphs = std::make_unique<std::vector<CanvasOptionsGlyphKey>>()});
    return *entries.back().glyphs;
  }
};

// Creates a thread-safe flag that is set to true when the SlicedGlyphAtlas is
// being destroyed.
struct CompletionGate {
  absl::Mutex mutex;
  bool complete ABSL_GUARDED_BY(mutex) = false;
};

// A single slice of the glyph atlas representing a single canvas source and
// its backing external texture. The atlas may use multiple slices (compositing
// them into a single texture) or may run in "single slice mode" where there is
// only one slice and no compositing is needed.
class Slice {
 public:
  Slice() = default;
  explicit Slice(uint2 texture_size,
                 std::unique_ptr<AsyncCanvasSource> canvas_source);
  ~Slice();

  // Movable but not copyable
  Slice(Slice&& other) = default;
  Slice& operator=(Slice&& other) = default;
  Slice(const Slice&) = delete;
  Slice& operator=(const Slice&) = delete;

  uint2 texture_size_;

  mutable absl::Mutex canvas_mutex_;
  // Created lazily when a glyph is added, and released when UpdateTextureSync
  // is called.
  std::unique_ptr<AsyncScopedCanvas> canvas_ ABSL_GUARDED_BY(canvas_mutex_);

  mutable absl::Mutex atlas_packer_mutex_;
  ShelfAtlasPacker atlas_packer_ ABSL_GUARDED_BY(atlas_packer_mutex_);

  mutable absl::Mutex glyph_map_mutex_ ABSL_ACQUIRED_BEFORE(canvas_mutex_);
  GlyphMap glyph_map_ ABSL_GUARDED_BY(glyph_map_mutex_);

  TextureStatus texture_status_ ABSL_GUARDED_BY(canvas_mutex_) = kStable;

  // A set of futures which move to the blit list when the texture is updated.
  std::vector<Future<absl::Status>> texture_update_futures_;
  // A set of futures which will resolve the next time the texture is uploaded.
  std::vector<Future<absl::Status>> texture_blit_futures_;

  // Whether there are any pending draw commands that have not been applied to
  // the canvas.
  bool has_pending_draw_commands_ ABSL_GUARDED_BY(canvas_mutex_) = false;

  // The currently pending prepare request. This may be a completed future if
  // there is no active request. This is not thread-safe but does not need
  // a mutex since it is only accessed within PrepareToUpdateTexture. See
  // PrepareToUpdateTexture docs for more details.
  Future<absl::Status> pending_prepare_future_;

  // CanvasSource is thread-safe.
  std::unique_ptr<AsyncCanvasSource> canvas_source_;

  // Represents the texture of the ScopedCanvas.
  Texture* texture_ = nullptr;
  BorrowedTexturePtr borrowed_texture_ = nullptr;

  std::shared_ptr<CompletionGate> completion_gate_ =
      std::make_shared<CompletionGate>();

  enum class EndFrameResult {
    kStable,
    kBlitRequired,
  };

  EndFrameResult EndFrame(imp::BaseView& view, bool single_slice)
      ABSL_LOCKS_EXCLUDED(canvas_mutex_);

  static std::string ToString(
      const GlyphEmulator::GlyphKeyOrGlyphString& glyph);

  AsyncScopedCanvas* GetOrStartDrawing(imp::BaseView& view,
                                       ScopedCanvas::DrawMode draw_mode,
                                       bool single_slice)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  // Updates the actual texture atlas to include all the glyphs.
  void DrawAllGlyphsToCanvas(imp::BaseView& view, bool single_slice)
      ABSL_LOCKS_EXCLUDED(canvas_mutex_, glyph_map_mutex_);

  // Updates the current texture based on what has been drawn to the current
  // canvas_. This may be async in cases where the canvas does not support
  // synchronous texture updates.
  // This function is not thread-safe since it accesses
  // pending_prepare_future_ without acquiring a mutex lock. This is fine
  // because it is only called from EndFrame which is only called from the
  // thread in which the glyph atlas is created.
  void UpdateTexture() ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  // Updates the current texture based on what has been drawn to the current
  // canvas_. This assumes any async prep work to update the texture has been
  // completed.
  void UpdateTextureSync() ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  // Called when the texture has been successfully blitted to the composite
  // texture.
  void OnBlitCompleted();

  // Updates the glyphs that are in the slice to the canvas
  // asynchronously
  Future<absl::Status> DrawGlyphsToCanvasAsync(
      imp::BaseView& view,
      std::unique_ptr<std::vector<CanvasOptionsGlyphKey>> glyphs,
      bool single_slice) ABSL_LOCKS_EXCLUDED(canvas_mutex_);

  // Gets a cached GlyphInfo by key or a nullptr if the glyph info is not yet
  // added to this slice.
  const GlyphInfo* /*absl_nullable*/  GetGlyphInfo(const CanvasOptionsGlyphKey& key);

  void DrawGlyphToCanvas(ScopedCanvas& canvas, CanvasOptionsGlyphKey glyph,
                         const GlyphInfo& glyph_info)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  // Updates the glyphs that are in the slice to the canvas
  absl::Status DrawGlyphsToCanvas(std::vector<CanvasOptionsGlyphKey>& glyphs)
      ABSL_LOCKS_EXCLUDED(glyph_map_mutex_, canvas_mutex_);

  // Remove all unused glyphs from the slice. `glyph_cleared_fn` is called for
  // each glyph that is removed.
  void ClearUnusedGlyphs(
      imp::BaseView& view, bool single_slice,
      std::function<void(const CanvasOptionsGlyphKey&)> glyph_cleared_fn)
      ABSL_LOCKS_EXCLUDED(glyph_map_mutex_, canvas_mutex_);

  // Returns the number of glyphs currently in the slice.
  //
  // This function is thread-safe.
  size_t GetNumCachedGlyphs() const;

  // Provides the percentage of the slice that is currently occupied.
  //
  // This function is thread-safe.
  float GetAtlasUtilization() const;

  void OnViewResumed();

#if IMP_RUNTIME(DEV)
  // Provides information of the glyph stored at the uv coordinate specified.
  //
  // This function is thread-safe.
  std::optional<editor::SlicedGlyphAtlasVisualizer::GlyphInfo> GetGlyphInfoAt(
      const float2& uv, const float2& slice_offset,
      const float2& slice_scale) const;
  // Provides information for all glyphs
  //
  // This function is thread-safe.
  std::vector<editor::SlicedGlyphAtlasVisualizer::GlyphInfo> GetAllGlyphInfo(
      const float2& slice_offset, const float2& slice_scale) const;
#endif
};

}  // namespace sliced_glyph_atlas
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_SLICE_H_
