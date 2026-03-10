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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_SLICED_GLYPH_ATLAS_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_SLICED_GLYPH_ATLAS_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Renderer.h"
#include "core/async/future.h"
#include "core/atlas/atlas_packer.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/bit_vector.h"
#include "core/common/paired_vector.h"
#include "core/common/rememberer.h"
#include "core/common/typed_span.h"
#include "core/config.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/text/glyph_atlas_slice.h"
#include "core/text/glyph_emulator.h"
#include "core/text/sliced_glyph_texture_manager.h"
#include "core/text/text_glyphs.h"
#include "core/text/text_metrics.proto.h"
#include "core/view/base_view.h"

#if IMP_RUNTIME(DEV)
#include "core/text/editor/sliced_glyph_atlas_visualizer.h"
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
// Note on thread-safety: Most of SlicedGlyphAtlas functionality is thread-safe,
// but certain functions, namely GetTexture(), may only be called from the
// foreground thread since they interact with Filament. See per-function
// comments.
class SlicedGlyphAtlas : public Rememberer {
  using Slice = sliced_glyph_atlas::Slice;

 public:
  using TextureSize = sliced_glyph_atlas::TextureSize;
  using TextureChangedEvent = sliced_glyph_atlas::TextureChangedEvent;
  using TextureManager = sliced_glyph_atlas::SlicedGlyphTextureManager;

  // Configuration options for the glyph atlas.
  struct Config {
    // The size of the texture to use for the glyph atlas. The default is a
    // 2048x2048 texture.
    TextureSize texture_size = TextureSize::k2048;
    // If true, the glyph atlas will use hardware rendering of its canvas
    // instead of software. This will reduce the amount of work done on the CPU,
    // but disables the ability to update specific regions of the atlas. Note:
    // This is currently only used on Android.
    bool use_hardware_rendering = true;
    // If true, this will call CanvasSource::ForceReset() and then re-draw all
    // the glyphs when the view is resumed. This will also cause the texture to
    // be changed.
    //
    // ForceReset only has an effect on Android at the moment. This is a
    // workaround for device specific android bugs (i.e. on Samsung Galaxy S24)
    // where an Android Surface can become corrupted after backgrounding and
    // resuming. See (broken link) for more details.
    bool force_reset_on_view_resumed = true;
    // If true, this will force the glyph atlas to use the auto method for
    // rendering. This will attempt to use shaper based rendering if supported,
    // otherwise fallback to path based rendering.
    bool force_auto_method_rendering = false;
    // If true, this will force each canvas source to use its own individual
    // glyph source instance, rather than sharing a single instance.
    bool force_individual_glyph_source_instances = false;
  };

  using SliceId = sliced_glyph_atlas::SliceId;

  constexpr static Config kDefaultConfig = {
      .texture_size = TextureSize::k2048,
      .use_hardware_rendering = true,
      .force_reset_on_view_resumed = true,
      .force_auto_method_rendering = false,
      .force_individual_glyph_source_instances = false};

  using Glyph = sliced_glyph_atlas::Glyph;

  explicit SlicedGlyphAtlas(BaseView& view, Config config = kDefaultConfig);
  SlicedGlyphAtlas(BaseView& view,
                   std::function<std::unique_ptr<AsyncCanvasSource>()>
                       canvas_source_factory_fn,
                   Config config = kDefaultConfig);
  ~SlicedGlyphAtlas();

  using TextOptions ABSL_DEPRECATED("GlyphEmulator::TextOptions") =
      GlyphEmulator::TextOptions;

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::AddFont()")
  void AddFont(absl::string_view font_name,
               std::unique_ptr<FontHolder> font_holder);

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::PrepareFont()")
  Future<absl::Status> PrepareFont(absl::string_view text,
                                   const TextOptions& options);

  ABSL_DEPRECATED(
      "GetGlyphEmulator() GlyphEmulator::GetCombinedCharacterGroups()")
  Future<std::vector<ScopedCanvas::GlyphGroup>> GetCombinedCharacterGroups(
      absl::string_view text, const TextOptions& options);

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::GetTextMetrics()")
  Future<TextMetrics> GetTextMetrics(absl::string_view text,
                                     const TextOptions& options);

  ABSL_DEPRECATED("GetGlyphEmulator() GlyphEmulator::GetFontInfo()")
  Future<FontInfo> GetFontInfo(const TextOptions& options);

  // For the given text, returns a list of glyphs used to render the text.
  //
  // The text must be in UTF8 encoding.
  //
  // Returns an error status if unable to get the glyphs. This could happen
  // if the options specify an invalid font name, or if SlicedGlyphAtlas is
  // being destructed.
  //
  // Calling this method will add glyphs to the atlas on-demand.
  // If the atlas runs out of space to add a new glyph, it will attempt to
  // clear unused glyphs (tracked using reference counting) to make room.
  //
  // This function is thread-safe.
  Future<std::vector<Glyph>> GetGlyphs(
      absl::string_view text, const GlyphEmulator::TextOptions& options);

  // Converts a GetGlyphs call to a TextGlyphs object.
  Future<TextGlyphs> GetTextGlyphs(absl::string_view text,
                                   const GlyphEmulator::TextOptions& options);

  // Returns a raw pointer to the Texture that will be used to draw the glyphs.
  // Warning: the Texture* may become invalid if SlicedGlyphAtlas recreates the
  // canvas.
  //
  // Note: This function is not thread-safe. Because Texture interacts with
  // Filament, this function may only be called from the foreground thread.
  //
  // TODO: Add support for automatically expanding to multiple
  // textures if we run out of space.
  Texture* GetTexture();

  // Returns the number of glyphs currently in the atlas.
  //
  // This function is thread-safe.
  size_t GetNumCachedGlyphs() const;

  // Get the super sample info based on the View, when it's ready.
  //
  // See also GlyphEmulator::GetSuperSampleInfo().
  Future<GlyphEmulator::SuperSampleInfo> GetSuperSampleInfo(
      bool force_off) const;

  // Provides the percentage of the atlas that is currently occupied
  //
  // This function is thread-safe.
  float GetAtlasUtilization() const;

  // Returns the backing GlyphEmulator used by this atlas.
  GlyphEmulator& GetGlyphEmulator();

  void GetSliceOffsetAndScale(SliceId slice, float2* offset,
                              float2* scale) const;

#if IMP_RUNTIME(DEV)
  // Provides information of the glyph stored at the uv coordinate specified.
  //
  // This function is thread-safe.
  std::optional<editor::SlicedGlyphAtlasVisualizer::GlyphInfo> GetGlyphInfoAt(
      const float2& uv) const;
  // Provides information for all glyphs
  //
  // This function is thread-safe.
  std::vector<editor::SlicedGlyphAtlasVisualizer::GlyphInfo> GetAllGlyphInfo()
      const;
#endif

 private:
  // Type that can be used as the key for a map and contains the
  // CanvasSource options for drawing glyphs derived from a particular
  // TextOption and the glyph itself. It's guaranteed that no two glyphs fetched
  // with different CanvasOptionsGlyphKey will look the same when drawn.
  using CanvasOptionsGlyphKey = sliced_glyph_atlas::CanvasOptionsGlyphKey;
  using CanvasOptionsKey = sliced_glyph_atlas::CanvasOptionsKey;

  struct CanvasOptionsInfo {
    // Slices that have at least one glyph with these options.
    PairedBitVector<Slice> active_slices;
    // The number of glyphs per slice with these options.
    PairedVector<uint16_t, Slice> glyph_count;
  };

  using CanvasOptionsMap =
      absl::flat_hash_map<CanvasOptionsKey, CanvasOptionsInfo>;

  // A single glyph may contain a separate atlas entry for the main part of the
  // text and the stroke.
  using GlyphInfo = sliced_glyph_atlas::GlyphInfo;

  // Maps a unique identifier for a glyph to information about that glyph in the
  // atlas.
  using GlyphMap = absl::flat_hash_map<CanvasOptionsGlyphKey, GlyphInfo>;

  using TextureStatus = sliced_glyph_atlas::TextureStatus;

  struct SlicedAtlasEntry {
    SliceId slice;
    AtlasPacker::ScopedAtlasEntry entry;
  };

  struct SlicedGlyphInfo {
    SliceId slice;
    const GlyphInfo& info;
  };

  // Gets the glyph info for the given glyph advance.
  // Note the GlyphAdvance is passed by value as the process of getting the
  // GlyphInfo can be destructive to the GlyphAdvance.
  std::optional<SlicedGlyphInfo> GetOrAddGlyphInfo(
      GlyphEmulator::Glyph& glyph, const CanvasOptionsGlyphKey& glyph_key,
      bool& out_added_new_glyph);

  void EndFrame();
  // Converts the internal structure of GlyphInfo used to track glyphs to the
  // external structure Glyph that is provided by SlicedGlyphAtlas to actually
  // render and layout a Glyph for a string.
  static Glyph GlyphInfoToGlyph(const GlyphInfo& glyph_info, SliceId slice,
                                float advance_width, float2 pixel_ratio_scale,
                                float2 glyph_atlas_size, bool is_emoji);

  // Helper that draws a glyph or string in the canvas source.
  void DrawGlyphKeyOrGlyphString(
      ScopedCanvas& canvas, const GlyphEmulator::GlyphKeyOrGlyphString& glyph,
      float2 position, const ScopedCanvas::TextOptions& canvas_options);

  AsyncScopedCanvas* GetOrStartDrawing(ScopedCanvas::DrawMode draw_mode,
                                       SliceId slice);

  std::optional<SlicedAtlasEntry> TryAddAtlasEntry(
      uint2 atlas_entry_size, CanvasOptionsInfo& info,
      const CanvasOptionsGlyphKey& glyph_key);

  // Gets a cached GlyphInfo by key or nullopt if the glyph info is not yet
  // added to this atlas.
  std::optional<SlicedGlyphInfo> GetGlyphInfo(const CanvasOptionsGlyphKey& key,
                                              const CanvasOptionsInfo& info);

  using PendingCanvasGlyphs = sliced_glyph_atlas::PendingCanvasGlyphs;

  // Stage 2b of the process of adding glyphs to the atlas described in the
  // SlicedGlyphAtlas class documentation described above.
  // Adds the given glyphs to the atlas. Note: The GlyphAdvances are passed by
  // value as the process of adding glyphs is destructive to the GlyphAdvances.
  void AddGlyphs(std::vector<GlyphEmulator::Glyph> glyphs,
                 const ScopedCanvas::TextOptions& canvas_options,
                 std::vector<Glyph>& result,
                 PendingCanvasGlyphs& pending_canvas_glyphs,
                 float2 subpixel_render_ratio);

  // Called at PreRender time to blit any newly rendered slices into the
  // composite texture. The blits are performed by rendering a quad to an
  // offscreen render target.
  void RenderBlits(filament::Renderer& renderer);

  CanvasOptionsInfo& GetCanvasOptionsInfo(
      const CanvasOptionsKey& canvas_options_key);

  void ClearUnusedGlyphs();

  BaseView& view_;

  // The size of the texture used for the glyph atlas in pixels.
  uint2 atlas_texture_size_ = {2048, 2048};
  uint2 atlas_grid_size_ = {1, 1};

  std::unique_ptr<Texture> composite_texture_;
  // The manager of the composite texture when there is >1 slice.
  std::unique_ptr<TextureManager> texture_manager_;
  // The slices that need to be blitted by the texture manager. Separate so they
  // can accumulate prior to the texture manager being assigned.
  PairedBitVector<Slice> pending_renders_;

  GlyphEmulator glyph_emulator_;

  // Slices are constructed in-place.
  Slice* slice_storage_ = nullptr;
  TypedSpan<Slice> slices_;

  // Operations that precede knowing which slice a glyph will be added to are
  // performed on the shared canvas source. It never actually draws anything,
  // draws are handled by the per-slice canvas sources.
  std::unique_ptr<AsyncCanvasSource> shared_canvas_source_;

  // Future that is marked as ready when the physical pixel ratio has been set.
  Future<absl::Status> physical_pixel_ratio_available_;

  mutable absl::Mutex canvas_options_map_mutex_;
  CanvasOptionsMap canvas_options_map_
      ABSL_GUARDED_BY(canvas_options_map_mutex_);
  // The cursor for the next slice to add a glyph to.
  SliceId addition_cursor_;

  const SlicedGlyphAtlas::GlyphInfo kEmptyGlyphInfo{
      .atlas_entry = AtlasPacker::ScopedAtlasEntry::Empty(),
      .measurements = TextMetrics::default_instance(),
      .fallback_font = nullptr};
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_SLICED_GLYPH_ATLAS_H_
