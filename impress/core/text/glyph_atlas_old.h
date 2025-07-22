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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_OLD_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_OLD_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/base/thread_annotations.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/variant.h"
#include "core/async/future.h"
#include "core/atlas/atlas_packer.h"
#include "core/atlas/shelf_atlas_packer.h"
#include "core/canvas/async_canvas_source.h"
#include "core/canvas/async_scoped_canvas.h"
#include "core/canvas/fonts/font_holder.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/canvas/fonts/font_params_helper.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/hash.h"
#include "core/common/ref_counter.h"
#include "core/common/rememberer.h"
#include "core/config.h"
#include "core/math/almost_equal.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/text/glyph_atlas.h"
#include "core/text/text_helpers.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

#if IMP_RUNTIME(DEV)
#include "core/text/editor/glyph_atlas_visualizer.h"
#endif

namespace imp {

// Represents an atlas of glyphs from fonts. Old implementation.
//
// TODO: Delete this when we're sure New doesn't cause problems.
//
// This is used to cache the glyph images from fonts in a texture so each glyph
// can be laid out and rendered on an individual quad instead of drawing an
// entire string onto a texture.
//
// Text is added to the atlas through a 3 stage process on call to GetGlyphs:
//
// Stage 1: Break up text into glyphs and determine their advance widths.
//    For canvas sources that support glyphs this is simply delegated to the
//    canvas source. For other canvas sources this is a 3 step approximation
//    process.
//    1a. (non-glyph support canvas sources only): Break up text into separable
//        and non-separable chunks.
//    1b. (non-glyph support canvas sources only): Measure glyph advances widths
//        in each chunk, treating non-separable chunks as one glyph and breaking
//        other chunks apart along codepoint boundaries.
//    1c. (non-glyph support canvas sources only): Flatten advance widths from
//        (1b) into GlyphAdvances.
// Stage 2: Allocate entries in the atlas for each new glyph.
//    2a. Use the canvas source to measure the glyphs.
//    2b. Allocate the space in the atlas.
// Stage 3: Use the canvas source to draw any new glyphs to the atlas texture
//    in the space allocated for them in stage (2).
//
// Note on thread-safety: Most of GlyphAtlas functionality is thread-safe, but
// certain functions, namely GetTexture(), may only be called from the
// foreground thread since they interact with Filament. See per-function
// comments.
class GlyphAtlasOld : public GlyphAtlas, public Rememberer {
 public:
  GlyphAtlasOld(BaseView& view, Config config = kDefaultConfig);
  GlyphAtlasOld(BaseView& view,
                std::unique_ptr<AsyncCanvasSource> canvas_source,
                Config config = kDefaultConfig);
  ~GlyphAtlasOld();

  // For the given text, returns a list of glyphs used to render the text.
  //
  // The text must be in UTF8 encoding.
  //
  // Returns an error status if unable to get the glyphs. This could happen
  // if the options specify an invalid font name, or if GlyphAtlas is being
  // destructed.
  //
  // Calling this method will add glyphs to the atlas on-demand.
  // If the atlas runs out of space to add a new glyph, it will attempt to clear
  // unused glyphs (tracked using reference counting) to make room.
  //
  // This function is thread-safe.
  Future<std::vector<Glyph>> GetGlyphs(absl::string_view text,
                                       const TextOptions& options)
      ABSL_LOCKS_EXCLUDED(canvas_mutex_) override;

  // Registers a font with the GlyphAtlas so that it can be used for glyphs.
  //
  // If no font is registered, then the default system font can still be used.
  //
  // This function is thread-safe.
  void AddFont(absl::string_view font_name,
               std::unique_ptr<FontHolder> font_holder)
      ABSL_LOCKS_EXCLUDED(fonts_mutex_) override;

  // For the given text options, prepare the font for the respective system.
  //
  // In some cases such as on wasm we may need to download a font asynchronously
  // before it may be used.
  Future<absl::Status> PrepareFont(absl::string_view text,
                                   const TextOptions& options) override;

  // Returns a vector of the group numbers for each glyph based on their
  // combining character information, referenced by the index in the vector.
  // Every set of combined characters will have the same group number. Each
  // group number is unique and the numbers in the vector may not be sequential.
  //
  // Note that every entry in the vector refers to the glyph of the same index
  // in the text string. The phrase "combining character" is used to refer to
  // characters that are intended to modify other characters, which in this
  // implementation are glyphs that are meant to be rendered together with their
  // associated (grouped) glyphs. See combining character information at:
  // https://en.wikipedia.org/wiki/Combining_character
  //
  // The vector is guaranteed to be the same length as the text.
  //
  // For example, if a string contains four glyphs, and the first two glyphs are
  // to be combined and hence the same group while the latter two are separate,
  // then the first two entries in the vector will be the same group while the
  // latter two have their own groups. A valid vector returned for that could be
  // {0, 0, 2, 3};
  //
  // If the text is not separable, then all entries in the vector will be of the
  // same value.
  // If there's no combining character information, then an empty vector is
  // returned.
  Future<std::vector<ScopedCanvas::GlyphGroup>> GetCombinedCharacterGroups(
      absl::string_view text, const TextOptions& options) override;

  // Returns the metrics in pixels that a given text string will take up based
  // on the options passed in, and text tracking, which adds additional space
  // between each glyph.
  //
  // The final drawn size will also be impacted by the transform of the
  // TextRenderer used to draw the glyphs.
  //
  // An error is returned if the options specify an invalid font name, or if
  // GlyphAtlas is being destructed.
  //
  // This function is thread-safe.
  Future<ScopedCanvas::TextMetrics> GetTextMetrics(
      absl::string_view text, const TextOptions& options) override;

  // Returns information about the font based on the given text options.
  //
  // If no font is specified, then the default font is used.
  // If the specified font wasn't added via AddFont, or if GlyphAtlas is being
  // destructed, then an error is returned.
  //
  // The resulting info respects the size of the font.
  //
  // This function is thread-safe.
  Future<ScopedCanvas::FontInfo> GetFontInfo(
      const TextOptions& options) override;

  // Returns a raw pointer to the Texture that will be used to draw the glyphs.
  // Warning: the Texture* may become invalid if GlyphAtlas recreates the
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

  // LINT.IfChange(font_size_pixels)
  static constexpr const int kDefaultFontSizePixels = 150.0f;
  // LINT.ThenChange(//depot/google3/third_party/impress/core/text/text_renderer_state.proto:font_size_pixels)

  // On WASM we want to render text at 2x the size and super sample it if the
  // window.devicePixelRatio property is less than 2.0. Note that we only use
  // the subpixel render ratio for supersampling in the x direction.
  Future<GlyphEmulator::SuperSampleInfo> GetSuperSampleInfo() const override;

  // Provides the percentage of the atlas that is currently occupied
  //
  // This function is thread-safe.
  float GetAtlasUtilization() const override;
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
  bool ShouldSuperSample() const;
  float2 GetSubpixelRenderRatio() const;

  // Represents a font. Every font corresponds to a unique FontId and is used
  // for differentiating between them.
  using FontId = HashValue;
  // A globally unique identifier for a glyph, composed by a GlyphId and FontId.
  struct GlyphKey {
    ScopedCanvas::GlyphId glyph_id;
    FontId font_id;

    bool operator==(const GlyphKey& other) const {
      return glyph_id == other.glyph_id && font_id == other.font_id;
    }

    template <typename H>
    friend H AbslHashValue(H hash, const GlyphKey& glyph_key) {
      return H::combine(std::move(hash), glyph_key.glyph_id, glyph_key.font_id);
    }
  };
  // Depending on if CanvasSource supports the kGlyph feature, the glyph is
  // either represented by an Id or a string representing the character.
  using GlyphKeyOrGlyphString = absl::variant<GlyphKey, std::string>;

  // Type that can be used as the key for a map and contains the
  // CanvasSource options for drawing glyphs derived from a particular
  // TextOption and the glyph itself. It's guaranteed that no two glyphs fetched
  // with different CanvasOptionsGlyphKey will look the same when drawn.
  struct CanvasOptionsGlyphKey {
    // Options that control how the text is drawn.
    ScopedCanvas::TextOptions canvas_options;
    GlyphKeyOrGlyphString glyph;
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

  // A glyph and its advance width. This is intended to represent the same
  // thing as the struct of the same name in
  // (broken link)
  // except that it represents the Glyph as a GlyphKeyOrGlyphString instead
  // of a GlyphId. This allows it to be used in cases where glyphs are not
  // supported by the canvas source.
  struct GlyphAdvance {
    // The glyph to draw.
    GlyphKeyOrGlyphString glyph;

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

    // If this glyph contains non-separable script. This is to distinguish when
    // coloring is required as a comparison for the canvas key.
    bool contains_non_separable_script;
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
  const GlyphInfo& GetOrAddGlyphInfo(
      GlyphAdvance glyph_advance, const CanvasOptionsGlyphKey& glyph_key,
      const absl::flat_hash_map<GlyphKeyOrGlyphString,
                                ScopedCanvas::TextMetrics>& measurement_map)
      ABSL_LOCKS_EXCLUDED(glyph_map_mutex_);

  // Updates the actual texture atlas to include all the glyphs.
  void DrawAllGlyphsToCanvas()
      ABSL_LOCKS_EXCLUDED(canvas_mutex_, glyph_map_mutex_);

  Future<ScopedCanvas::TextOptions> CanvasOptionsFromGlyphAtlasOptions(
      const TextOptions& options)
      ABSL_LOCKS_EXCLUDED(fonts_mutex_, system_fonts_mutex_);

  void EndFrame() ABSL_LOCKS_EXCLUDED(canvas_mutex_);

  // Updates the glyphs that are in the glyph atlas to the canvas asynchronously
  Future<absl::Status> DrawGlyphsToCanvasAsync(
      std::unique_ptr<std::vector<CanvasOptionsGlyphKey>> glyphs)
      ABSL_LOCKS_EXCLUDED(canvas_mutex_);

  // Updates the glyphs that are in the glyph atlas to the canvas
  absl::Status DrawGlyphsToCanvas(std::vector<CanvasOptionsGlyphKey>& glyphs)
      ABSL_LOCKS_EXCLUDED(glyph_map_mutex_, canvas_mutex_);

  // Converts the internal structure of GlyphInfo used to track glyphs to the
  // external structure Glyph that is provided by GlyphAtlas to actually render
  // and layout a Glyph for a string.
  static Glyph GlyphInfoToGlyph(const GlyphInfo& glyph_info,
                                float advance_width, float2 pixel_ratio_scale,
                                float2 glyph_atlas_size, bool is_emoji);

  void DrawGlyphToCanvas(ScopedCanvas& canvas, CanvasOptionsGlyphKey glyph,
                         const GlyphInfo& glyph_info)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  // Helper that draws a glyph or string in the canvas source.
  void DrawGlyphKeyOrGlyphString(
      ScopedCanvas& canvas, const GlyphKeyOrGlyphString& glyph, float2 position,
      const ScopedCanvas::TextOptions& canvas_options)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  static std::string ToString(const GlyphKeyOrGlyphString& glyph);

  AsyncScopedCanvas* GetOrStartDrawing(ScopedCanvas::DrawMode draw_mode)
      ABSL_EXCLUSIVE_LOCKS_REQUIRED(canvas_mutex_);

  std::optional<AtlasPacker::ScopedAtlasEntry> TryAddAtlasEntry(
      uint2 atlas_entry_size) ABSL_LOCKS_EXCLUDED(atlas_packer_mutex_);

  // Stage 1 of the process of adding glyphs to the atlas described in the
  // GlyphAtlas class documentation described above.
  // This breaks up a string into a series of glyphs and their advance widths.
  //
  // note that this returns a std::unique_ptr<std::vector> instead of an
  // std::vector because Future.Then copies the vector otherwise. A copy of the
  // vector is illegal since GlyphAdvance contains a std::unique_ptr.
  Future<std::unique_ptr<std::vector<GlyphAdvance>>> BreakIntoGlyphs(
      absl::string_view text, const ScopedCanvas::TextOptions& canvas_options);

  // For canvas sources that do not natively support glyphs, glyph_atlas will
  // approximate the computation of glyphs from a string by doing the following:
  // A. Breaking the string into chunks of separable and non-separable text
  //    chunks.
  // B. Measure the advance width of the glyphs in each chunk. Separable chunks
  //    are subdivided into glyphs along codepoint boundaries. Non-separable
  //    chunks  are treated as one glyph.
  // C. Flatten the computed advance widths from (B) into a vector of
  //    GlyphAdvances.
  //
  // GetChunks in text_helpers.h is the step (A) above.
  // This is step (C).
  // Note: there may be fewer GlyphAdvances added than there are widths if any
  // widths are 0. In that case the character is merged with the previous
  // character to create one GlyphAdvance.
  static void GetGlyphsForChunk(const Chunk& chunk,
                                const std::vector<float>& advance_widths,
                                const ScopedCanvas::TextOptions& canvas_options,
                                std::vector<GlyphAdvance>& out_glyph_advances);

  // Gets a cached GlyphInfo by key or a nullptr if the glyph info is not yet
  // added to this atlas.
  const GlyphInfo* /*absl_nullable*/ GetGlyphInfo(const CanvasOptionsGlyphKey& key);

  // Stage 2a of the process of adding glyphs to the atlas described in the
  // GlyphAtlas class documentation described above.
  // Measures the given glyphs, or extracts cached measurements for glyphs
  // that are already stored in this atlas.
  Future<absl::flat_hash_map<GlyphKeyOrGlyphString, ScopedCanvas::TextMetrics>>
  MeasureGlyphs(const std::vector<GlyphAdvance>& glyphs,
                ScopedCanvas::TextOptions canvas_options);

  // Stage 2b of the process of adding glyphs to the atlas described in the
  // GlyphAtlas class documentation described above.
  // Adds the given glyphs to the atlas. Note: The GlyphAdvances are passed by
  // value as the process of adding glyphs is destructive to the GlyphAdvances.
  void AddGlyphs(
      std::vector<GlyphAdvance> glyphs,
      const ScopedCanvas::TextOptions& canvas_options,
      const absl::flat_hash_map<GlyphKeyOrGlyphString,
                                ScopedCanvas::TextMetrics>& measurement_map,
      std::vector<Glyph>& result,
      std::vector<CanvasOptionsGlyphKey>& pending_canvas_glyphs);

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

  // A unique ID for the latest prepare request. If a prepare request is
  // returned with a version less than this is should be ignored.
  uint32_t prepare_request_id_ ABSL_GUARDED_BY(canvas_mutex_) = 0;

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

  mutable absl::Mutex fonts_mutex_;
  StringMap<std::unique_ptr<FontHolder>> fonts_ ABSL_GUARDED_BY(fonts_mutex_);

  mutable absl::Mutex system_fonts_mutex_;
  absl::flat_hash_map<SystemFontParams, std::unique_ptr<FontHolder>,
                      SystemFontParamsKeyHash, SystemFontParamsKeyEquals>
      system_fonts_ ABSL_GUARDED_BY(system_fonts_mutex_);

  const ScopedCanvas::TextMetrics kEmptyMeasurements{.origin = {0, 0},
                                                     .size = {0, 0}};
  const GlyphInfo kEmptyGlyphInfo{
      .atlas_entry = AtlasPacker::ScopedAtlasEntry::Empty(),
      .measurements = kEmptyMeasurements,
      .fallback_font = nullptr};

  // Creates a thread-safe flag that is set to true when the GlyphAtlas is
  // being destroyed.
  struct CompletionGate {
    absl::Mutex mutex_;
    bool complete_ ABSL_GUARDED_BY(mutex_) = false;
  };

  std::shared_ptr<CompletionGate> completion_gate_ =
      std::make_shared<CompletionGate>();
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_GLYPH_ATLAS_OLD_H_
