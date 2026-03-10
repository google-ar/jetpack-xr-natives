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

#ifndef THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_RENDERER_H_
#define THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_RENDERER_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "core/async/future.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/allocator_pool.h"
#include "core/common/invocable.h"
#include "core/geometry/shapes/box.h"
#include "core/geometry/shapes/rect.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_data.h"
#include "core/ncsb/component.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/component_system.h"
#include "core/ncsb/isf_info.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/update_phase.h"
#include "core/text/glyph_atlas.h"
#include "core/text/sliced_glyph_atlas.h"
#include "core/text/text_glyphs.h"
#include "core/text/text_metrics.proto.h"
#include "core/text/text_renderer_state.proto.imp.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Renders a text string in 3D space.
//
// This component is an alternative to using CanvasSource directly to render
// text. In some ways, it's more constrained than CanvasSource. However, it's
// built to be more optimal in cases where a large number of rapidly changing
// labels need to be rendered.
//
// Internally it uses a glyph atlas, so each glyph in the text is
// rendered on a separate quad. This means that all labels share instances of
// the same glyph instead of duplicating them in texture memory and repeatedly
// re-drawing them.
// TODO: Add methods to dynamically modify the text renderer after
// creation.
class TextRenderer : public Component {
 public:
  struct SharedMesh {
    OwnedMeshPtr mesh;
    MeshData mesh_data;
    bool dirty;
  };

  class System : public ComponentSystem<TextRenderer> {
   public:
    explicit System(BaseView* view);

    void AfterLastComponentRemoved() override;
    void PostComponentsUpdated(const FrameTime& frame_time) override;

    AllocatorPool<SharedMesh>::Handle AllocSubMesh(int size);

   private:
    std::unique_ptr<AllocatorPool<SharedMesh>> mesh_pool_;
  };

  // A configuration for customizing how the text renderer lays the glyphs out,
  // according to the glyph path function.
  // The glyph path function is used to determine the final transform of the
  // glyphs, while the glyph pivot is for determining the point on the glyph on
  // which to pivot.
  struct TextLayoutProvider {
    // The glyph path function consumes the position as a fraction of each
    // glyph's pivot point along the entire text, and the result is used to
    // calculate the position of the glyph in relation to the origin of the
    // text. In short, the result is a function of the position of the glyph
    // along the text. For example, a monospaced 4 letter word with a pivot
    // point of 0, 0 will result in the function consuming 0, 0.25, 0.5, and
    // 0.75 for the glyph positions. This function is also used to calculate the
    // gradient of the function at the position of every glyph, and the glyphs
    // are rotated accordingly. By default, there's no glyph path function, and
    // the glyphs are laid out across the +x axis. Only the x and y axes are
    // used for positioning the glyphs; the z axis is ignored.
    Invocable<float3(float t, float text_length)> glyph_path_func;
    // A function to indicate that the underlying path data has changed since
    // the last call to this method, and the glyph geometry needs to be
    // regenerated.
    Invocable<bool()> has_changed;
    // A function to indicate if text should be rendered. Provider might skip
    // text rendering if path line becomes too distorted or too short to render
    // the text. If not defined, default function always returns true.
    Invocable<bool()> should_render = []() { return true; };
  };

  Future<absl::Status> Setup();
  Future<absl::Status> Setup(TextLayoutProvider layout_provider);

  void Cleanup();

  Future<absl::Status> OnIsfStateChanged();

  TextRendererState GetState() const;

  float4 GetTextColor() const;
  float2 GetOffset() const;
  void SetOffset(float2 offset);
  float2 GetPivot() const;
  void SetPivot(float2 pivot);
  TextRendererState::VerticalPivot GetTopPivot() const;
  TextRendererState::VerticalPivot GetBottomPivot() const;
  TextRendererState::HorizontalPivot GetLeftPivot() const;
  TextRendererState::HorizontalPivot GetRightPivot() const;
  bool HasStroke() const;
  float GetStrokeWidthPixels() const;
  float4 GetStrokeColor() const;

  // Returns the sum of all the glyph advances, which is what would be the width
  // of the entire text if the glyphs were to be laid out on a horizontal line,
  // including the tracking between each glyph.
  float GetTextAdvance() const;

  // Return the height of the glyphs in the text.
  float GetTextHeight() const;

  // Returns the material used to render the text.
  //
  // When a custom material asset is used, this provides access to the material
  // so that custom parameters can be set on it.
  // Note that the material is only available after the component is done
  // initializing.
  BorrowedMaterialPtr BorrowMaterial() const;

  filament::VertexDomain GetVertexDomain() const { return vertex_domain_; }

  ComponentHandle<MeshRenderer> GetRenderer() const { return renderer_; }

  float GetMetersPerPixel() const;

  // Get the bounds surrounding the rendered text in the space of the renderer.
  Box GetLocalBounds() const;

  // Get the path of the rendered text in the space of the renderer.
  absl::StatusOr<std::vector<float3>> GetLocalPath() const;

  // Get the bounds surrounding the rendered text in world space.
  Box GetWorldBounds() const;

  // Get the bounds surrounding the rendered text in screen space. Returns an
  // error if the bounds cannot be calculated in screen space.
  absl::StatusOr<Rect> GetScreenBounds() const;

  // Get the local bounds surrounding the rendered text in screen space (screen
  // bounds without considering screen position). Returns an error if the bounds
  // cannot be calculated in screen space.
  absl::StatusOr<Rect> GetScreenLocalBounds() const;

  // Get the path through the rendered text in screen space. Returns an error if
  // the path cannot be calculated in screen space.
  absl::StatusOr<std::vector<float3>> GetScreenPath() const;

  std::optional<const TextLayoutProvider*> GetTextLayoutProvider() const;

  void Update(const FrameTime& frame_time);

  // Calculates the bounds of the entire text. This is called automatically on
  // Setup, but needs to be called if the text mesh is set manually.
  void CalculateBounds();

  // Calculates the path of the entire text. This is called automatically on
  // Setup, but needs to be called if the text mesh is set manually.
  //
  // The path follows roughly the midpoint of the text.
  void CalculatePath();

  // Finds out whether path text collider is outdated.
  bool IsPathChanged() const { return is_path_changed_; }

  // Notifies that path text collider is updated.
  void ResetPathChanged() { is_path_changed_ = false; }

  // Get node world position projected on screen. If node world position is not
  // in front of the camera near plane it returns a failed status.
  absl::StatusOr<float2> GetNodeScreenPosition() const;

  float GetOpacityMultiplier() const;
  void SetOpacityMultiplier(float opacity);
  std::optional<TextAndFontMetrics> GetPrecomputedMetrics() const;

  // Gets the font params to use to render the text.
  const SystemFontParams* GetSystemFontParams() const;

  // Returns true if force_non_separable is actually set. Even if the user
  // specifies force_non_separable in text options, we may not be able to
  // respect it based on workarounds, specifically text tracking and text on a
  // path.
  bool GetForceNonSeparable() const { return force_non_separable_; }

 private:
  struct GlyphVerticesAndVisualBounds {
    std::vector<float3> vertices;
    Rect bounds;
  };

  struct GlyphPlacement {
    float3 position;
    mat2f rotation;
    float center;
  };

  Future<FontInfo> GetFontInfo(const GlyphEmulator::TextOptions& options);
  Future<TextGlyphs> GetTextGlyphs(absl::string_view text,
                                   const GlyphEmulator::TextOptions& options);
  Future<GlyphEmulator::SuperSampleInfo> GetSuperSampleInfo(
      bool force_off) const;
  Future<std::vector<ScopedCanvas::GlyphGroup>> GetCombinedCharacterGroups(
      absl::string_view text, const GlyphEmulator::TextOptions& options);
  imp::Texture* GetTexture();

  Future<absl::Status> SetupImpl(
      std::optional<TextLayoutProvider> layout_provider);

  // Creates and updates the text geometry and mesh.
  Future<absl::Status> UpdateMeshesAndMaterials();

  void ApplyColorsToMaterial(imp::Material& material);

  // Glyph vertices store their screen space coordinates in the xy components
  // and an optional depth value in the z component.
  void CalculateTextDimensionsAndPivot();
  void OffsetText(std::vector<float3>* glyph_vertices, float2 offset);
  Box AddGlyphsToTextMesh(MeshData& mesh_data, int mesh_data_offset);
  GlyphVerticesAndVisualBounds CalculateGlyphVerticesAndVisualBounds() const;
  GlyphVerticesAndVisualBounds CalculateGlyphVerticesAndVisualBounds(
      const TextGlyphs& glyphs) const;
  void RenderTextGlyphPass(int vertex_offset, int index_offset,
                           const std::vector<float3>& glyph_vertices,
                           bool has_stroke, MeshData& mesh_data);

  void RecalculateMesh(bool force_regenerate_mesh = false);

  float GetVerticalPivot(TextRendererState::VerticalPivot pivot, float min,
                         float max) const;
  float GetHorizontalPivot(TextRendererState::HorizontalPivot pivot, float min,
                           float max, float typographic_width) const;

  std::vector<GlyphPlacement> CalculateGlyphPlacements(
      const TextGlyphs& glyphs) const;

  // Determines how far along the fill bounds of the text the given pos_x is.
  // pos_x is relative to the left edge of the typographic bounds of the text.
  // A return value of 0 means pos_x is at the left edge of the fill bounds of
  // the text and 1 means pos_x is at the right edge of the fill bounds of the
  // text. The return value can be less than 0 or greater than 1 if the position
  // is outside of the fill bounds of the text.
  float PercentAlongText(float pos_x) const;

  TextRendererState state_;
  filament::VertexDomain vertex_domain_;
  std::optional<TextLayoutProvider> text_layout_provider_;
  std::optional<AllocatorPool<SharedMesh>::Handle> shared_mesh_handle_;
  ComponentHandle<MeshRenderer> renderer_;
  // The dimensions of the fill bounds of the text. text_dimensions_.x is the
  // visual width of the rendered text not including the stroke width and
  // text_dimensions_.y is the visual height of the tallest glyph in the string
  // not including the stroke width.
  float2 fill_bounds_size_;
  // The left edge of the fill bounds relative to the typographic bounds left
  // edge.
  float fill_bounds_left_;
  // bounds_ are the actual visual bounds of the text, offset according to the
  // pivot and offset. If there is no path, these dimensions SHOULD be equal to
  // text_dimensions_; if not, they describe the visual extent of the glyphs as
  // they are bent about the path.
  Box bounds_;
  // Is empty when path function is void.
  std::vector<float3> path_;
  float2 text_pivot_;
  bool is_path_changed_;
  // If true, it means glyphs are not separated to be saved in the glyph atlas.
  bool force_non_separable_;

  GlyphAtlas* glyph_atlas_ = nullptr;
  SlicedGlyphAtlas* sliced_glyph_atlas_ = nullptr;
  FontInfo font_info_;

  std::optional<TextGlyphs> text_glyphs_;
  std::vector<ScopedCanvas::GlyphGroup> glyph_groups_;

  NodeHandle root_;

 public:
  using IsfInfo = IsfInfo<&TextRenderer::state_>;
  static constexpr UpdatePhase kUpdatePhase = UpdatePhase::kEnd;
  static constexpr bool kRunInEditMode = true;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_TEXT_TEXT_RENDERER_H_
