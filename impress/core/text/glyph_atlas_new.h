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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_NEW_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_NEW_H_

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/base/nullability.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "core/async/future.h"
#include "core/atlas/atlas_packer.h"
#include "core/atlas/shelf_atlas_packer.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/ref_counter.h"
#include "core/common/rememberer.h"
#include "core/config.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/text/glyph_atlas.h"
#include "core/text/glyph_emulator.h"
#include "core/view/base_view.h"

#if IMP_RUNTIME(DEV)
#include "core/text/editor/glyph_atlas_visualizer.h"
#endif

namespace imp {

// Represents an atlas of glyphs from fonts.
//
// This is used to cache the glyph images from fonts in a texture so each glyph
// can be laid out and rendered on an individual quad instead of drawing an
// entire string onto a texture.
//
// Text is added to the atlas through a 3 stage process on call to GetGlyphs:
//
// Stage 1: Break up text into glyphs and determine their metrics.
//    This step is delegated to GlyphEmulator.
// Stage 2: Allocate entries in the atlas for each new glyph.
// Stage 3: Use the canvas source to draw any new glyphs to the atlas texture
//    in the space allocated for them in stage (2).
//
// Note on thread-safety: Most of GlyphAtlasNew functionality is thread-safe,
// but certain functions, namely GetTexture(), may only be called from the
// foreground thread since they interact with Filament. See per-function
// comments.
class GlyphAtlasNew : public GlyphAtlas, public Rememberer {
 public:
  GlyphAtlasNew(BaseView& view, Config config = kDefaultConfig);
  GlyphAtlasNew(BaseView& view,
                std::unique_ptr<AsyncCanvasSource> canvas_source,
                Config config = kDefaultConfig);
  ~GlyphAtlasNew();

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::AddFont()")
  void AddFont(absl::string_view font_name,
               std::unique_ptr<FontHolder> font_holder) override;

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::PrepareFont()")
  Future<absl::Status> PrepareFont(absl::string_view text,
                                   const TextOptions& options) override;

  ABSL_DEPRECATED(
      "GetGlyphEmulator() GlyphEmulator::GetCombinedCharacterGroups()")
  Future<std::vector<ScopedCanvas::GlyphGroup>> GetCombinedCharacterGroups(
      absl::string_view text, const TextOptions& options) override;

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::GetTextMetrics()")
  Future<ScopedCanvas::TextMetrics> GetTextMetrics(
      absl::string_view text, const TextOptions& options) override;

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::GetFontInfo()")
  Future<ScopedCanvas::FontInfo> GetFontInfo(
      const TextOptions& options) override;

  // For the given text, returns a list of glyphs used to render the text.
  //
  // The text must be in UTF8 encoding.
  //
  // Returns an error status if unable to get the glyphs. This could happen
  // if the options specify an invalid font name, or if GlyphAtlasNew is being
  // destructed.
  //
  // Calling this method will add glyphs to the atlas on-demand.
  // If the atlas runs out of space to add a new glyph, it will attempt to
  // clear unused glyphs (tracked using reference counting) to make room.
  //
  // This function is thread-safe.
  Future<std::vector<Glyph>> GetGlyphs(
      absl::string_view text, const GlyphEmulator::TextOptions& options)
      ABSL_LOCKS_EXCLUDED(canvas_mutex_) override;

  // Returns a raw pointer to the Texture that will be used to draw the glyphs.
  // Warning: the Texture* may become invalid if GlyphAtlasNew recreates the
  // canvas.
  //
  // Note: This function is not thread-safe. Because Texture interacts with
  // Filament, this function may only be called from the foreground thread.
  //
  // TODO: Add support for automatically expanding to multiple
  // textures if we run out of space.
  Texture* GetTexture() override;

  // Returns the number of glyphs currently in the atlas.
  //
  // This function is thread-safe.
  size_t GetNumCachedGlyphs() const
      ABSL_LOCKS_EXCLUDED(glyph_map_mutex_) override;

  // Get the super sample info based on the View, when it's ready.
  //
  // See also GlyphEmulator::GetSuperSampleInfo().
  Future<GlyphEmulator::SuperSampleInfo> GetSuperSampleInfo(
      bool force_off) const override;

  // Provides the percentage of the atlas that is currently occupied
  //
  // This function is thread-safe.
  float GetAtlasUtilization() const override;

  // Returns the backing GlyphEmulator used by this atlas.
  GlyphEmulator& GetGlyphEmulator();

#if IMP_RUNTIME(DEV)
  // Provides information of the glyph stored at the uv coordinate specified.
  //
  // This function is thread-safe.
  std::optional<editor::GlyphAtlasVisualizer::GlyphAtlasInfo>
  GetGlyphAtlasInfoAt(const float2& uv) const override;
  // Provides information for all glyphs
  //
  // This function is thread-safe.
  std::vector<editor::GlyphAtlasVisualizer::GlyphAtlasInfo> GetAllGlyphInfo()
      const override;
#endif

 private:
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
      bool eq =
          (glyph == other.glyph &&
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
    ScopedCanvas::TextMetrics measurements;

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

  // Gets the glyph info for the given glyph advance.
  // Note the GlyphAdvance is passed by value as the process of getting the
  // GlyphInfo can be destructive to the GlyphAdvance.
  const GlyphInfo& GetOrAddGlyphInfo(GlyphEmulator::Glyph& glyph,
                                     const CanvasOptionsGlyphKey& glyph_key)
      ABSL_LOCKS_EXCLUDED(glyph_map_mutex_);

  // Updates the actual texture atlas to include all the glyphs.
  void DrawAllGlyphsToCanvas()
      ABSL_LOCKS_EXCLUDED(canvas_mutex_, glyph_map_mutex_);

  void EndFrame() ABSL_LOCKS_EXCLUDED(canvas_mutex_);

  // Updates the glyphs that are in the glyph atlas to the canvas asynchronously
  Future<absl::Status> DrawGlyphsToCanvasAsync(
      std::unique_ptr<std::vector<CanvasOptionsGlyphKey>> glyphs)
      ABSL_LOCKS_EXCLUDED(canvas_mutex_);

  // Updates the glyphs that are in the glyph atlas to the canvas
  absl::Status DrawGlyphsToCanvas(std::vector<CanvasOptionsGlyphKey>& glyphs)
      ABSL_LOCKS_EXCLUDED(glyph_map_mutex_, canvas_mutex_);

  // Converts the internal structure of GlyphInfo used to track glyphs to the
  // external structure Glyph that is provided by GlyphAtlasNew to actually
  // render and layout a Glyph for a string.
  static Glyph GlyphInfoToGlyph(const GlyphInfo& glyph_info,
                                float advance_width, float2 pixel_ratio_scale,
                                float2 glyph_atlas_size, bool is_emoji);

  void DrawGlyphToCanvas(ScopedCanvas& canvas, CanvasOptionsGlyphKey glyph,
                         const GlyphInfo& glyph_info)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  // Helper that draws a glyph or string in the canvas source.
  void DrawGlyphKeyOrGlyphString(
      ScopedCanvas& canvas, const GlyphEmulator::GlyphKeyOrGlyphString& glyph,
      float2 position, const ScopedCanvas::TextOptions& canvas_options)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  static std::string ToString(
      const GlyphEmulator::GlyphKeyOrGlyphString& glyph);

  AsyncScopedCanvas* GetOrStartDrawing(ScopedCanvas::DrawMode draw_mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  std::optional<AtlasPacker::ScopedAtlasEntry> TryAddAtlasEntry(
      uint2 atlas_entry_size) ABSL_LOCKS_EXCLUDED(atlas_packer_mutex_);

  // Gets a cached GlyphInfo by key or a nullptr if the glyph info is not yet
  // added to this atlas.
  const GlyphInfo* /*absl_nullable*/ GetGlyphInfo(const CanvasOptionsGlyphKey& key);

  // Stage 2b of the process of adding glyphs to the atlas described in the
  // GlyphAtlasNew class documentation described above.
  // Adds the given glyphs to the atlas. Note: The GlyphAdvances are passed by
  // value as the process of adding glyphs is destructive to the GlyphAdvances.
  void AddGlyphs(std::vector<GlyphEmulator::Glyph> glyphs,
                 const ScopedCanvas::TextOptions& canvas_options,
                 std::vector<Glyph>& result,
                 std::vector<CanvasOptionsGlyphKey>& pending_canvas_glyphs,
                 float2 subpixel_render_ratio);

  // Does any preparation work to synchronously update the current texture
  // based on what has been drawn to the current canvas_.
  // This function is not thread-safe since it accesses pending_prepare_future_
  // without acquiring a mutex lock. This is fine because it is only called from
  // EndFrame which is only called from the thread in which the glyph atlas is
  // created.
  void PrepareToUpdateTexture() ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  // Clears all the glyphs in the atlas that are currently unused.
  //
  // The glyphs are tracked via reference counting.
  void ClearUnusedGlyphs() ABSL_LOCKS_EXCLUDED(glyph_map_mutex_, canvas_mutex_);

  BaseView& view_;

  // The size of the texture used for the glyph atlas in pixels.
  float2 atlas_texture_size_ = {2048.0f, 2048.0f};

  // CanvasSource is thread-safe.
  std::unique_ptr<AsyncCanvasSource> canvas_source_;

  GlyphEmulator glyph_emulator_;

  mutable absl::Mutex canvas_mutex_;
  // Created lazily when a glyph is added, and released at the end of each
  // frame when texture status is kReadyToRelease.
  std::unique_ptr<AsyncScopedCanvas> canvas_ ABSL_GUARDED_BY(canvas_mutex_);

  // The currently pending prepare request. This may be a completed future if
  // there is no active request. This is not thread-safe but does not need
  // a mutex since it is only accessed within PrepareToUpdateTexture. See
  // PrepareToUpdateTexture docs for more details.
  Future<absl::Status> pending_prepare_future_;

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

    // The canvas_ texture's pixels can be synchronously released, and will
    // be synchronously released on the next EndFrame.
    kReadyToRelease,
  };
  TextureStatus texture_status_ ABSL_GUARDED_BY(canvas_mutex_) = kStable;

  // A set of futures which will resolve the next time the texture is updated.
  std::vector<Future<absl::Status>> texture_update_futures_;

  // Future that is marked as ready when the physical pixel ratio has been set.
  Future<absl::Status> physical_pixel_ratio_available_;

  // Represents the texture of the ScopedCanvas.
  Texture* texture_ = nullptr;

  mutable absl::Mutex atlas_packer_mutex_;
  ShelfAtlasPacker atlas_packer_ ABSL_GUARDED_BY(atlas_packer_mutex_);

  mutable absl::Mutex glyph_map_mutex_ ABSL_ACQUIRED_BEFORE(canvas_mutex_);
  GlyphMap glyph_map_ ABSL_GUARDED_BY(glyph_map_mutex_);

  const ScopedCanvas::TextMetrics kEmptyMeasurements{.origin = {0, 0},
                                                     .size = {0, 0}};
  const GlyphAtlasNew::GlyphInfo kEmptyGlyphInfo{
      .atlas_entry = AtlasPacker::ScopedAtlasEntry::Empty(),
      .measurements = kEmptyMeasurements,
      .fallback_font = nullptr};

  // Creates a thread-safe flag that is set to true when the GlyphAtlasNew is
  // being destroyed.
  struct CompletionGate {
    absl::Mutex mutex_;
    bool complete_ ABSL_GUARDED_BY(mutex_) = false;
  };

  std::shared_ptr<CompletionGate> completion_gate_ =
      std::make_shared<CompletionGate>();
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_NEW_H_
