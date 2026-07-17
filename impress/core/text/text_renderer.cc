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

#include "core/text/text_renderer.h"

#include <math.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/async/future.h"
#include "core/canvas/fonts/font_params.proto.imp.h"
#include "core/canvas/scoped_canvas.h"
#include "core/common/allocator_pool.h"
#include "core/common/filament_helpers.h"
#include "core/common/invocable.h"
#include "core/common/registry.h"
#include "core/config.h"
#include "core/geometry/geometry_helper.h"
#include "core/geometry/shapes/rect.h"
#include "core/materials/material.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/vertex_format.h"
#include "core/ncsb/component_system.h"
#include "core/render/texture.h"
#include "core/text/glyph_atlas.h"
#include "core/text/glyph_emulator.h"
#include "core/text/sliced_glyph_atlas.h"
#include "core/text/text_glyphs.h"
#include "core/text/text_metrics.proto.h"
#include "core/text/text_renderer_assets.h"
#include "core/text/text_renderer_state.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/camera/camera_manager.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"
#include "core/view/utils/proto/view_config.proto.imp.h"

namespace imp {
namespace {
using VertexAttribute = VertexFormat::VertexAttribute;
using AttributeType = VertexFormat::AttributeType;

constexpr float kDefaultMetersPerPixel = 0.0025f;
constexpr float2 kDefaultPivot = {0.0f, 0.5f};
constexpr absl::string_view kDebugName = "tr";
// One size unit = one quad. With an initial max order of 10, we have 1024 quads
// max in the initial pool.
constexpr int kSharedMeshInitialMaxOrder = 10;

// Vertex format for the glyph mesh. Currently excludes tangents since text
// is always rendered unlit.
const VertexFormat kVertexFormat = {{.attribute = VertexAttribute::POSITION,
                                     .type = AttributeType::FLOAT3,
                                     .attribute_group_override = 0},
                                    {.attribute = VertexAttribute::UV0,
                                     .type = AttributeType::FLOAT2,
                                     .attribute_group_override = 0}};

struct TextMeshVertex {
  float3 position;
  float2 uv;
};

// Vertex format for deinterleaved glyph mesh.
const VertexFormat kDeinterleavedVertexFormat = {
    {.attribute = VertexAttribute::POSITION,
     .type = AttributeType::FLOAT3,
     .attribute_group_override = 0},
    {.attribute = VertexAttribute::UV0,
     .type = AttributeType::FLOAT2,
     .attribute_group_override = 1}};

// Helper used to organize basic vertices for a quad. See below for more
// details.
struct QuadVertex {
  float2 position;
  float2 uv;
};

// Basic Vertices for a Quad.
// When using MESH_PER_RENDERER or BATCHED_MESHES rendering modes,
// these vertices are used to generate a quad for each glyph.
// The positions and UV coordinates are transformed based on the size of the
// glyph, position of the glyph, and the uv coordinates of the glyph in the
// texture atlas.
constexpr std::array<QuadVertex, 4> kQuadVertices = {
    QuadVertex{.position = {0.0f, 0.0f}, .uv = {0.0f, 1.0f}},  // Bottom Left
    QuadVertex{.position = {1.0f, 0.0f}, .uv = {1.0f, 1.0f}},  // Bottom Right
    QuadVertex{.position = {1.0f, 1.0f}, .uv = {1.0f, 0.0f}},  // Top Right
    QuadVertex{.position = {0.0f, 1.0f}, .uv = {0.0f, 0.0f}}   // Top Left
};

// Basic indices for a quad.
// When using MESH_PER_RENDERER or BATCHED_MESHES rendering modes,
// these indices are offset to target the vertices for the correct glyph.
constexpr std::array<uint16_t, 6> kQuadIndices = {
    0, 1, 2,  // Triangle 1
    0, 2, 3,  // Triangle 2
};

constexpr absl::string_view kGlyphAtlasParam = "GlyphAtlas";
constexpr absl::string_view kTextColorFactorParam = "TextColorFactor";
constexpr absl::string_view kStrokeColorFactorParam = "StrokeColorFactor";
constexpr absl::string_view kShouldSuperSampleParam = "ShouldSuperSample";

// Obtains the rotation matrix for the gradient of function between v1 and v2
mat2f GetRotation(const Invocable<float3(float t, float l)>* graph_func,
                  float v1, float v2, float text_length) {
  float2 delta =
      (*graph_func)(v2, text_length).xy - (*graph_func)(v1, text_length).xy;
  if (AlmostEqual(delta, float2(0))) {
    return mat2f();
  }
  float2 gradient_vector = normalize(delta);
  int sign = gradient_vector.y > 0 ? 1 : -1;
  float cosine = dot(gradient_vector, float2{1, 0}) / length(gradient_vector);
  float angle = acos(std::clamp(cosine, -1.0f, 1.0f)) * sign;

  return mat2f::rotate(angle);
}

GlyphAtlas::TextureSize GetAtlasTextureSize(
    ViewConfig::GlyphAtlasTextureSize texture_size) {
  switch (texture_size) {
    default:
    case ViewConfig::GlyphAtlasTextureSize::SIZE2048:
      return GlyphAtlas::TextureSize::k2048;
    case ViewConfig::GlyphAtlasTextureSize::SIZE2048X4096:
      return GlyphAtlas::TextureSize::k2048_4096;
    case ViewConfig::GlyphAtlasTextureSize::SIZE4096:
      return GlyphAtlas::TextureSize::k4096;
    case ViewConfig::GlyphAtlasTextureSize::SIZE256X256X8X8:
      IMP_LOG(imp::FATAL) << "GlyphAtlas does not support compound atlas textures.";
      return GlyphAtlas::TextureSize::k2048;
  }
}

SlicedGlyphAtlas::TextureSize GetSlicedAtlasTextureSize(
    ViewConfig::GlyphAtlasTextureSize texture_size) {
  switch (texture_size) {
    default:
    case ViewConfig::GlyphAtlasTextureSize::SIZE2048:
      return SlicedGlyphAtlas::TextureSize::k2048;
    case ViewConfig::GlyphAtlasTextureSize::SIZE2048X4096:
      return SlicedGlyphAtlas::TextureSize::k2048_4096;
    case ViewConfig::GlyphAtlasTextureSize::SIZE4096:
      return SlicedGlyphAtlas::TextureSize::k4096;
    case ViewConfig::GlyphAtlasTextureSize::SIZE256X256X8X8:
      return SlicedGlyphAtlas::TextureSize::k256_256_8_8;
  }
}

void InitMeshDataIndices(MeshData& mesh_data) {
  absl::Span<uint16_t> indices = mesh_data.Indices<uint16_t>();
  for (int i = 0; i < indices.length(); i++) {
    indices[i] = (i / kQuadIndices.size()) * kQuadVertices.size() +
                 kQuadIndices[i % kQuadIndices.size()];
  }
}

}  // namespace

TextRenderer::System::System(BaseView* view)
    : ComponentSystem<TextRenderer>(view) {}

void TextRenderer::System::AfterLastComponentRemoved() { mesh_pool_.reset(); }

void TextRenderer::System::PostComponentsUpdated(const FrameTime& frame_time) {
  if (!mesh_pool_) {
    return;
  }
  // Update all dirty meshes.
  mesh_pool_->ForEachResource([](SharedMesh& shared_mesh) {
    if (shared_mesh.dirty) {
      shared_mesh.mesh->UpdateMeshData(&shared_mesh.mesh_data,
                                       Mesh::AabbSource::kAssignedOrUnchanged);
      shared_mesh.dirty = false;
    }
  });
}

AllocatorPool<TextRenderer::SharedMesh>::Handle
TextRenderer::System::AllocSubMesh(int size) {
  if (!mesh_pool_) {
    // TODO: is it okay to pass the view into this lambda?
    BaseView& view = GetView();
    mesh_pool_ = AllocatorPool<SharedMesh>::Create(
        kSharedMeshInitialMaxOrder, [&view](int size) {
          const MeshDescription kMeshDescription = {
              *view.GetConfig()
                      .experimental_feature_flags
                      ->enable_deinterleaved_text_renderer
                  ? kDeinterleavedVertexFormat
                  : kVertexFormat,
              MeshDescription::IndexType::USHORT, size * kQuadVertices.size(),
              size * kQuadIndices.size()};

          MeshData mesh_data = MeshData(kMeshDescription);
          InitMeshDataIndices(mesh_data);

          OwnedMeshPtr mesh = view.GetMeshFactory().CreateByCopyingMeshData(
              MeshFactory::PrimitiveType::TRIANGLES, mesh_data, Box(),
              kDebugName);

          return SharedMesh{std::move(mesh), std::move(mesh_data),
                            /*dirty=*/false};
        });
  }
  return mesh_pool_->Alloc(size);
}

Future<absl::Status> TextRenderer::Setup() { return SetupImpl(std::nullopt); }

Future<absl::Status> TextRenderer::Setup(TextLayoutProvider layout_provider) {
  return SetupImpl(std::move(layout_provider));
}

Future<absl::Status> TextRenderer::SetupImpl(
    std::optional<TextLayoutProvider> layout_provider) {
  if (state_.text.empty()) {
    return Future<absl::Status>(absl::FailedPreconditionError(
        "Unable to create TextRenderer with empty string."));
  }

  if (layout_provider.has_value()) {
    text_layout_provider_ = std::move(*layout_provider);
  }

  const ViewConfig& view_config = GetView().GetConfig();
  ExperimentalFeatureFlags experimental_feature_flags =
      *view_config.experimental_feature_flags;
  std::optional<ViewConfig::GlyphAtlasTextureSize> texture_size =
      *view_config.glyph_atlas_texture_size;
  // TODO: Address duplication when SlicedGlyphAtlas is ready.
  if (texture_size.has_value() &&
      (*texture_size == ViewConfig::GlyphAtlasTextureSize::SIZE256X256X8X8)) {
    sliced_glyph_atlas_ =
        &GetView().GetRegistry().GetOrRegister<SlicedGlyphAtlas>(
            [this, texture_size = texture_size,
             flags = experimental_feature_flags] {
              if (texture_size.has_value()) {
                return std::make_unique<SlicedGlyphAtlas>(
                    GetView(),
                    SlicedGlyphAtlas::Config{
                        .texture_size =
                            GetSlicedAtlasTextureSize(*texture_size),
                        .use_hardware_rendering =
                            *flags.glyph_atlas_use_hardware_rendering,
                        .force_reset_on_view_resumed =
                            *flags.glyph_atlas_force_reset_on_view_resumed,
                        .force_auto_method_rendering =
                            *flags.glyph_atlas_force_auto_method_rendering,
                        .force_individual_glyph_source_instances =
                            *flags.force_individual_glyph_source_instances,
                        .use_bitmap_surface_provider =
                            *flags.glyph_atlas_use_bitmap_surface_provider});

              } else {
                return std::make_unique<SlicedGlyphAtlas>(GetView());
              }
            });
    // Update the texture parameter on the material when the texture changes.
    GetView().GetDispatcher().Connect(
        [this](const SlicedGlyphAtlas::TextureChangedEvent& event) {
          if (renderer_) {
            Material* material = renderer_->GetMaterial();
            if (material) {
              material->SetParameter(kGlyphAtlasParam,
                                     sliced_glyph_atlas_->GetTexture());
            }
          }
        },
        this);
  } else {
    glyph_atlas_ = &GetView().GetRegistry().GetOrRegister<GlyphAtlas>(
        [this, texture_size = texture_size,
         flags = experimental_feature_flags] {
          if (texture_size.has_value()) {
            return std::make_unique<GlyphAtlas>(
                GetView(),
                GlyphAtlas::Config{
                    .texture_size = GetAtlasTextureSize(*texture_size),
                    .use_hardware_rendering =
                        *flags.glyph_atlas_use_hardware_rendering,
                    .force_reset_on_view_resumed =
                        *flags.glyph_atlas_force_reset_on_view_resumed,
                    .force_auto_method_rendering =
                        *flags.glyph_atlas_force_auto_method_rendering,
                    .force_individual_glyph_source_instances =
                        *flags.force_individual_glyph_source_instances,
                    .use_bitmap_surface_provider =
                        *flags.glyph_atlas_use_bitmap_surface_provider});

          } else {
            return std::make_unique<GlyphAtlas>(GetView());
          }
        });
    // Update the texture parameter on the material when the texture changes.
    GetView().GetDispatcher().Connect(
        [this](const GlyphAtlas::TextureChangedEvent& event) {
          if (renderer_) {
            Material* material = renderer_->GetMaterial();
            if (material) {
              material->SetParameter(kGlyphAtlasParam,
                                     glyph_atlas_->GetTexture());
            }
          }
        },
        this);
  }

  return UpdateMeshesAndMaterials();
}

void TextRenderer::Cleanup() { GetView().DestroyNode(root_); }

Future<FontInfo> TextRenderer::GetFontInfo(
    const GlyphEmulator::TextOptions& options) {
  return (glyph_atlas_ != nullptr) ? glyph_atlas_->GetFontInfo(options)
                                   : sliced_glyph_atlas_->GetFontInfo(options);
}

Future<TextGlyphs> TextRenderer::GetTextGlyphs(
    absl::string_view text, const GlyphEmulator::TextOptions& options) {
  return (glyph_atlas_ != nullptr)
             ? glyph_atlas_->GetTextGlyphs(text, options)
             : sliced_glyph_atlas_->GetTextGlyphs(text, options);
}

Future<GlyphEmulator::SuperSampleInfo> TextRenderer::GetSuperSampleInfo(
    bool force_off) const {
  return (glyph_atlas_ != nullptr)
             ? glyph_atlas_->GetSuperSampleInfo(force_off)
             : sliced_glyph_atlas_->GetSuperSampleInfo(force_off);
}

Future<std::vector<ScopedCanvas::GlyphGroup>>
TextRenderer::GetCombinedCharacterGroups(
    absl::string_view text, const GlyphEmulator::TextOptions& options) {
  return (glyph_atlas_ != nullptr)
             ? glyph_atlas_->GetCombinedCharacterGroups(text, options)
             : sliced_glyph_atlas_->GetCombinedCharacterGroups(text, options);
}

imp::Texture* TextRenderer::GetTexture() {
  return (glyph_atlas_ != nullptr) ? glyph_atlas_->GetTexture()
                                   : sliced_glyph_atlas_->GetTexture();
}

Future<absl::Status> TextRenderer::UpdateMeshesAndMaterials() {
  // TODO (broken link) Ensure that the color space here is correct
  GlyphEmulator::TextOptions options = {
      .font_params = state_.font_params,
      .font_size_pixels = state_.font_size_pixels,
      .stroke_width_pixels = state_.stroke_width_pixels,
      .color = GetTextColor(),
      .stroke_color = GetStrokeColor(),
      .force_non_separable = state_.force_non_separable &&
                             GlyphEmulator::CanForceNonSeparable(
                                 text_layout_provider_.has_value(),
                                 state_.text_tracking.value_or(0.0f)),
      .precomputed_metrics = GetPrecomputedMetrics(),
  };
  if (state_.text_tracking.has_value()) {
    options.text_tracking = *state_.text_tracking;
  }
  force_non_separable_ = options.force_non_separable;

  // Load the material.
  Future<MaterialPtr> text_material_future =
      GetView().GetMaterialFactory().LoadMaterial(
          state_.material.has_value()
              ? *state_.material
              : (GetView().GetConfig().glyph_atlas_texture_size ==
                         imp::ViewConfig::GlyphAtlasTextureSize::SIZE256X256X8X8
                     ? text_renderer_assets::kSlicedTextMaterialCmat.GetUrl()
                     : text_renderer_assets::kTextMaterialCmat.GetUrl()));

  Future<FontInfo> font_info_future;
  if (options.precomputed_metrics.has_value()) {
    font_info_future.Return(options.precomputed_metrics->font_info());
  } else {
    font_info_future = GetFontInfo(options);
  }

  Future<TextGlyphs> glyphs_future = GetTextGlyphs(state_.text, options);

  Future<std::vector<ScopedCanvas::GlyphGroup>>
      combined_character_indices_future =
          Future<std::vector<ScopedCanvas::GlyphGroup>>(
              std::vector<ScopedCanvas::GlyphGroup>());
  if (text_layout_provider_.has_value()) {
    combined_character_indices_future =
        GetCombinedCharacterGroups(state_.text, options);
  }

  Future<GlyphEmulator::SuperSampleInfo> super_sample_info_future =
      GetSuperSampleInfo(options.force_non_separable);

  return text_material_future
      .Merge(glyphs_future, font_info_future, combined_character_indices_future,
             super_sample_info_future)
      .Then([this](std::tuple<MaterialPtr, TextGlyphs, FontInfo,
                              std::vector<ScopedCanvas::GlyphGroup>,
                              GlyphEmulator::SuperSampleInfo>
                       result) -> absl::Status {
        font_info_ = std::get<2>(result);

        MaterialPtr& text_material = std::get<0>(result);
        text_glyphs_.emplace(std::move(std::get<1>(result)));
        if (text_glyphs_->IsEmpty()) {
          return absl::InvalidArgumentError(
              "Unable to update text geometry with empty glyphs for text: " +
              state_.text);
        }

        vertex_domain_ = text_material->GetFilamentMaterialInstance()
                             ->getMaterial()
                             ->getVertexDomain();

        // Create the root node if it doesn't exist.
        if (!root_) {
          root_ = GetView().CreateNode();
#if IMP_RUNTIME(DEV)
          root_->SetName("TextRenderer");
#endif  // IMP_RUNTIME(DEV)
          root_->SetParent(GetNode());
          MeshRenderer::FrustumCullingMode culling_mode =
              state_.disable_frustum_culling
                  ? MeshRenderer::FrustumCullingMode::kDisabled
                  : MeshRenderer::FrustumCullingMode::kEnabled;
          renderer_ = root_->AddComponent<MeshRenderer>(culling_mode);
        }

        if (text_layout_provider_.has_value()) {
          root_->SetEnabled(text_layout_provider_->should_render());
          glyph_groups_ = std::get<3>(result);
        }

        RecalculateMesh(/*force_regenerate_mesh=*/true);

        // Assign the texture to the material, and assign the material to
        // the render component.
        ApplyColorsToMaterial(*text_material);
        text_material->SetParameter(kGlyphAtlasParam, GetTexture());
        text_material->SetParameter(kShouldSuperSampleParam,
                                    std::get<4>(result).should_super_sample);
        renderer_->SetMaterial(std::move(text_material));

        return absl::OkStatus();
      });
}

Future<absl::Status> TextRenderer::OnIsfStateChanged() {
  return UpdateMeshesAndMaterials();
}

void TextRenderer::ApplyColorsToMaterial(imp::Material& material) {
  float opacity = GetOpacityMultiplier();
  float4 color = GetTextColor();
  color.a = std::clamp(color.a * opacity, 0.0f, 1.0f);
  material.SetParameter(kTextColorFactorParam, color);
  if (HasStroke()) {
    color = GetStrokeColor();
    color.a = std::clamp(color.a * opacity, 0.0f, 1.0f);
    material.SetParameter(kStrokeColorFactorParam, color);
  }
}

std::optional<const TextRenderer::TextLayoutProvider*>
TextRenderer::GetTextLayoutProvider() const {
  if (!text_layout_provider_) {
    return std::nullopt;
  }

  return &*text_layout_provider_;
}

float TextRenderer::GetTextAdvance() const {
  return (fill_bounds_size_.x + state_.stroke_width_pixels) *
         GetMetersPerPixel();
}

float TextRenderer::GetTextHeight() const {
  return (fill_bounds_size_.y + state_.stroke_width_pixels) *
         GetMetersPerPixel();
}

void TextRenderer::Update(const FrameTime& frame_time) {
  if (!text_layout_provider_.has_value()) {
    return;
  }
  if (text_layout_provider_->has_changed()) {
    bool should_render = text_layout_provider_->should_render();
    root_->SetEnabled(should_render);
    if (should_render) {
      RecalculateMesh();
    }
  }
}

void TextRenderer::RecalculateMesh(bool force_regenerate_mesh) {
  size_t glyph_count = text_glyphs_.has_value() ? text_glyphs_->Size() : 0ul;
  // Without stroke, there's only one pass where the regular fill glyph is
  // drawn, and with glyphs, all stroke glyphs are drawn first in one pass,
  // followed by the regular fill glyphs, hence needing double the quads.
  const int stroke_multiplier = HasStroke() ? 2 : 1;
  if (state_.batch) {
    // When force_regenerate_mesh is true, it means it is possible that the
    // text has been changed. Therefore the vertex
    // count and the mesh data will need to be recalculated.
    if (shared_mesh_handle_ && force_regenerate_mesh) {
      shared_mesh_handle_.reset();
    }

    if (!shared_mesh_handle_) {
      shared_mesh_handle_ = GetView()
                                .GetComponentManager()
                                .GetComponentSystem<TextRenderer>()
                                .AllocSubMesh(glyph_count * stroke_multiplier);
    }
    SharedMesh& shared_mesh = shared_mesh_handle_->GetResource();
    shared_mesh.dirty = true;

    Box box = AddGlyphsToTextMesh(shared_mesh.mesh_data,
                                  shared_mesh_handle_->GetOffset());

    Mesh* existing_mesh = renderer_->GetMesh();
    if (existing_mesh && existing_mesh->IsSubmesh() && !force_regenerate_mesh) {
      existing_mesh->AssignAabb(box);
    } else {
      renderer_->SetMesh(GetView().GetMeshFactory().CreateSubMesh(
          shared_mesh.mesh.Borrow(),
          shared_mesh_handle_->GetOffset() * kQuadIndices.size(),
          shared_mesh_handle_->GetSize() * kQuadIndices.size(), box));
    }
  } else {
    const MeshDescription kMeshDescription = {
        *GetView()
                .GetConfig()
                .experimental_feature_flags->enable_deinterleaved_text_renderer
            ? kDeinterleavedVertexFormat
            : kVertexFormat,
        MeshDescription::IndexType::USHORT,
        glyph_count * kQuadVertices.size() * stroke_multiplier,
        glyph_count * kQuadIndices.size() * stroke_multiplier};
    MeshDataPtr mesh_data = std::make_unique<MeshData>(kMeshDescription);
    InitMeshDataIndices(*mesh_data);

    Box box = AddGlyphsToTextMesh(*mesh_data, 0);

    Mesh* existing_mesh = renderer_->GetMesh();
    if (existing_mesh && !existing_mesh->IsSubmesh() &&
        !force_regenerate_mesh) {
      existing_mesh->AssignAabb(box);
      existing_mesh->UpdateMeshData(std::move(mesh_data));
    } else {
      // When `force_regenerate_mesh` is true, the text geometry has potentially
      // changed, requiring a full recalculation of vertex counts and mesh data.
      // Since updating mesh data with a different vertex count is not
      // supported, a new mesh must be created.
      renderer_->SetMesh(GetView().GetMeshFactory().CreateByMovingMeshData(
          MeshFactory::PrimitiveType::TRIANGLES, std::move(mesh_data), box,
          MeshFactory::MeshDataStorageMode::kDiscardMeshData, kDebugName));
    }
  }

  CalculateBounds();
  CalculatePath();
}

Box TextRenderer::AddGlyphsToTextMesh(MeshData& mesh_data,
                                      int mesh_data_offset) {
  float glyph_render_multiplier = GetMetersPerPixel();
  size_t glyph_count = text_glyphs_.has_value() ? text_glyphs_->Size() : 0ul;

  CalculateTextDimensionsAndPivot();
  GlyphVerticesAndVisualBounds glyph_vertices_and_visual_bounds =
      CalculateGlyphVerticesAndVisualBounds();

  // Get the bounds of the text for calculating the relative positions of each
  // glyph. Since we're only drawing 2D text for now, the AABB for the mesh is a
  // flat box.
  bounds_ = ToBox(glyph_vertices_and_visual_bounds.bounds);
  bounds_.center *= glyph_render_multiplier;
  bounds_.halfExtent *= glyph_render_multiplier;
  Box box = bounds_;
  // Box is rooted to origin by the bottom left corner as the renderer draws
  // the text based on the mask from origin.
  box.center = box.halfExtent;

  // Offset the text by a fixed number of pixels
  OffsetText(&glyph_vertices_and_visual_bounds.vertices, GetOffset());

  int vertex_offset = mesh_data_offset * kQuadVertices.size();
  int index_offset = mesh_data_offset * kQuadIndices.size();

  // All stroke glyphs, if any, are drawn first, followed by the
  // regular fill glyphs. This is to account for cases in some variable fonts
  // where the internal geometry may cause artifacting inside the empty space of
  // the stroke glyphs and the fill glyph is used to cover those artifacts.
  if (HasStroke()) {
    RenderTextGlyphPass(vertex_offset, index_offset,
                        glyph_vertices_and_visual_bounds.vertices, true,
                        mesh_data);

    // Offsets the vertices and indices for the second render pass for the fill
    // glyphs.
    vertex_offset += glyph_count * kQuadVertices.size();
    index_offset += glyph_count * kQuadIndices.size();
  }

  RenderTextGlyphPass(vertex_offset, index_offset,
                      glyph_vertices_and_visual_bounds.vertices, false,
                      mesh_data);

  return box;
}

std::vector<TextRenderer::GlyphPlacement>
TextRenderer::CalculateGlyphPlacements(const TextGlyphs& glyphs) const {
  std::vector<GlyphPlacement> glyph_placements;
  size_t glyph_count = glyphs.Size();
  glyph_placements.resize(glyph_count);
  const absl::Span<const float> advance_widths = glyphs.AdvanceWidths();

  //  By default the glyph is placed horizontally along the +x axis.
  if (!text_layout_provider_.has_value()) {
    float glyph_x_offset = 0.0f;
    for (int glyph_index = 0; glyph_index < glyph_count; ++glyph_index) {
      float glyph_center = advance_widths[glyph_index] * 0.5f;

      glyph_placements[glyph_index] = {
          .position = float3{glyph_x_offset + glyph_center, 0.0f, 0.0f},
          .rotation = {},
          .center = glyph_center};
      glyph_x_offset += advance_widths[glyph_index];
    }
    return glyph_placements;
  }
  // If we have a text layout provider, we need to calculate the glyph
  // placements based on the path function.

  // Create a temporary structure to hold the start and end positions of each
  // group of combining characters.
  // See https://en.wikipedia.org/wiki/Combining_character for more details.
  struct GroupBounds {
    float start;
    float end;
    float center;
  };

  std::vector<GroupBounds> group_bounds(glyph_count);
  // If there are no groups or the group information does not sufficiently cover
  // the number of glyphs, fallback to using a separate group for each glyph.
  if (glyph_groups_.size() < glyph_count) {
    float glyph_x_offset = 0.0f;
    for (int glyph_index = 0; glyph_index < glyph_count; ++glyph_index) {
      float advance_width = advance_widths[glyph_index];
      group_bounds[glyph_index].center = advance_width * 0.5f;
      group_bounds[glyph_index].start = glyph_x_offset;
      group_bounds[glyph_index].end = glyph_x_offset + advance_width;
      glyph_x_offset += advance_width;
    }
  } else {
    int current_group = 0;
    float group_start_x_offset = 0.0f;
    float group_current_x_offset = group_start_x_offset;
    for (int glyph_index = 0; glyph_index < glyph_count; ++glyph_index) {
      if (glyph_groups_[current_group] != glyph_groups_[glyph_index]) {
        float group_center =
            (group_current_x_offset - group_start_x_offset) * 0.5f;
        for (int i = current_group; i <= glyph_index; ++i) {
          group_bounds[i].center = group_center;
          group_bounds[i].start = group_start_x_offset;
          group_bounds[i].end = group_current_x_offset;
        }
        group_start_x_offset = group_current_x_offset;
        current_group = glyph_index;
      }
      group_current_x_offset += advance_widths[glyph_index];
    }

    float group_center = (group_current_x_offset - group_start_x_offset) * 0.5f;
    for (int i = current_group; i < glyph_count; ++i) {
      group_bounds[i].center = group_center;
      group_bounds[i].start = group_start_x_offset;
      group_bounds[i].end = group_current_x_offset;
    }
  }

  float glyph_x_offset = 0.0f;
  for (int glyph_index = 0; glyph_index < glyph_count; ++glyph_index) {
    float group_mid =
        group_bounds[glyph_index].start + group_bounds[glyph_index].center;
    // Calculate the position of the group on the path using the center of the
    // group as a percentage of the length of the entire text.
    float3 glyph_path_point = text_layout_provider_->glyph_path_func(
        PercentAlongText(group_mid), fill_bounds_size_.x);
    glyph_placements[glyph_index] = {
        // Scale the position of the glyph on the calculated path to the
        // dimensions of the text. The z component has no effect on the path
        // position, and is passed through as-is for users to make use of if
        // needed.
        .position = float3{glyph_path_point.xy * fill_bounds_size_.x,
                           glyph_path_point.z},
        // Rotate the glyph based on the start and end positions of the glyph on
        // the path
        .rotation =
            GetRotation(&text_layout_provider_->glyph_path_func,
                        PercentAlongText(group_bounds[glyph_index].start),
                        PercentAlongText(group_bounds[glyph_index].end),
                        fill_bounds_size_.x),

        // Offset the group center from the glyph x offset for rotating in
        // relation to the group
        .center = group_mid - glyph_x_offset};
    glyph_x_offset += advance_widths[glyph_index];
  }

  return glyph_placements;
}

float TextRenderer::PercentAlongText(float pos_x) const {
  return (pos_x - fill_bounds_left_) / fill_bounds_size_.x;
}

TextRenderer::GlyphVerticesAndVisualBounds
TextRenderer::CalculateGlyphVerticesAndVisualBounds() const {
  return CalculateGlyphVerticesAndVisualBounds(*text_glyphs_);
}

TextRenderer::GlyphVerticesAndVisualBounds
TextRenderer::CalculateGlyphVerticesAndVisualBounds(
    const TextGlyphs& glyphs) const {
  size_t glyph_count = glyphs.Size();
  std::vector<float3> glyph_vertices(glyph_count * kQuadVertices.size());

  float2 visual_bounds_min = float2(std::numeric_limits<float>::max());
  float2 visual_bounds_max = float2(std::numeric_limits<float>::min());

  std::vector<GlyphPlacement> glyph_placements =
      CalculateGlyphPlacements(glyphs);

  const absl::Span<const float4> atlas_origins_and_sizes =
      glyphs.AtlasOriginsAndSizes();
  const absl::Span<const float4> actual_origins_and_sizes =
      glyphs.ActualOriginsAndSizes();

  for (int glyph_index = 0; glyph_index < glyph_count; ++glyph_index) {
    float3 translation = glyph_placements[glyph_index].position;
    mat2f rotation = glyph_placements[glyph_index].rotation;
    float center = glyph_placements[glyph_index].center;

    for (int vertex_index = 0; vertex_index < kQuadVertices.size();
         ++vertex_index) {
      const QuadVertex& quad_vertex = kQuadVertices[vertex_index];
      // atlas_origin doesn't account for the stroke width, so offset the
      // vertex by half the stroke width.
      float4 atlas_origin_and_size_value = atlas_origins_and_sizes[glyph_index];
      float4 actual_origin_and_size_value =
          actual_origins_and_sizes[glyph_index];
      float2 mesh_vertex =
          quad_vertex.position * atlas_origin_and_size_value.zw +
          atlas_origin_and_size_value.xy -
          float2(state_.stroke_width_pixels / 2.0f) - text_pivot_;
      mesh_vertex.x -= center;
      // Rotate the glyph vertices, the translate them along the path
      mesh_vertex = rotation * mesh_vertex + translation.xy;
      int glyph_vert_index =
          vertex_index + (glyph_index * kQuadVertices.size());
      glyph_vertices[glyph_vert_index] = float3(mesh_vertex, translation.z);

      // Calculate the visual vertex and grow the bounds of the node to fit.
      // actual_origin doesn't account for the stroke width, so offset the
      // vertex by half the stroke width.
      float2 visual_vertex =
          quad_vertex.position * actual_origin_and_size_value.zw +
          actual_origin_and_size_value.xy -
          float2(state_.stroke_width_pixels / 2.0f) - text_pivot_;
      visual_vertex.x -= center;
      visual_vertex = rotation * visual_vertex + translation.xy;

      visual_bounds_min = min(visual_bounds_min, visual_vertex);
      visual_bounds_max = max(visual_bounds_max, visual_vertex);
    }
  }

  float2 extents = (visual_bounds_max - visual_bounds_min) / 2.0f;
  Rect bounds = {.center = visual_bounds_min + extents, .half_extent = extents};

  return {
      std::move(glyph_vertices),
      std::move(bounds),
  };
}

void TextRenderer::RenderTextGlyphPass(
    int vertex_offset, int index_offset,
    const std::vector<float3>& glyph_vertices, bool has_stroke,
    MeshData& mesh_data) {
  float glyph_render_multiplier = GetMetersPerPixel();
  size_t glyph_count = text_glyphs_->Size();
  absl::Span<const float4> uv_origins_and_sizes =
      text_glyphs_->UVOriginsAndSizes();

  bool enable_deinterleaved_text_renderer =
      *GetView()
           .GetConfig()
           .experimental_feature_flags->enable_deinterleaved_text_renderer;

  absl::Span<float3> positions;
  absl::Span<float2> uvs;
  TextMeshVertex* pass_vertices;

  if (enable_deinterleaved_text_renderer) {
    positions = mesh_data.Vertices<float3>(0);
    uvs = mesh_data.Vertices<float2>(1);
  } else {
    pass_vertices = &mesh_data.VertexAt<TextMeshVertex>(vertex_offset);
  }

  // Label space y is down, Filament's is up.
  for (int glyph_index = 0; glyph_index < glyph_count; ++glyph_index) {
    for (int vertex_index = 0; vertex_index < kQuadVertices.size();
         ++vertex_index) {
      const QuadVertex& quad_vertex = kQuadVertices[vertex_index];
      float4 uv_origin_and_size = uv_origins_and_sizes[glyph_index];
      int glyph_vert_index =
          vertex_index + (glyph_index * kQuadVertices.size());
      if (enable_deinterleaved_text_renderer) {
        positions[vertex_offset + glyph_vert_index] = float3(
            glyph_vertices[glyph_vert_index].xy * glyph_render_multiplier,
            glyph_vertices[glyph_vert_index].z);
      } else {
        pass_vertices[glyph_vert_index].position = float3(
            glyph_vertices[glyph_vert_index].xy * glyph_render_multiplier,
            glyph_vertices[glyph_vert_index].z);
      }
      // Assign the UV coordinates to point to the correct glyph in the texture
      // atlas.
      float2 vertex_uv_coords =
          uv_origin_and_size.xy + (uv_origin_and_size.zw * quad_vertex.uv);

      // In the text_material_common.glsl:getGlyphColor we check the signs to
      // return the stroke, text, or sample color.
      if (has_stroke) {
        vertex_uv_coords.x *= -1;
      }
      if (enable_deinterleaved_text_renderer) {
        uvs[vertex_offset + glyph_vert_index] = vertex_uv_coords;
      } else {
        pass_vertices[glyph_vert_index].uv = vertex_uv_coords;
      }
    }
  }
}

void TextRenderer::OffsetText(std::vector<float3>* glyph_vertices,
                              float2 offset) {
  // Label space y is down, Filament's is up.
  offset.y = -offset.y;
  for (int vertex_index = 0; vertex_index < glyph_vertices->size();
       ++vertex_index) {
    glyph_vertices->at(vertex_index) += float3(offset, 0.0f);
  }
}

TextRendererState TextRenderer::GetState() const { return state_; }

float4 TextRenderer::GetTextColor() const {
  if (state_.color_rgba()) {
    return *state_.color_rgba();
  } else if (state_.color_rgb()) {
    return float4(*state_.color_rgb(), 1.0f);
  }

  return kOne4;
}

void TextRenderer::CalculateTextDimensionsAndPivot() {
  size_t glyph_count = text_glyphs_->Size();
  const absl::Span<const float> advance_widths = text_glyphs_->AdvanceWidths();
  const absl::Span<const float4> actual_origins_and_sizes =
      text_glyphs_->ActualOriginsAndSizes();
  float glyph_x_offset = 0.0f;

  // box_min/max defines the rendered bounds of the text if each glyph is
  // rendered starting at 0,0 and moving right by each glyph's advance width.
  // The bounds include an additional half stroke width in each direction.
  float2 box_min = float2(std::numeric_limits<float>::max());
  float2 box_max = float2(-std::numeric_limits<float>::max());

  float typographic_width = 0.0f;
  for (int glyph_index = 0; glyph_index < glyph_count; ++glyph_index) {
    float4 actual_origin_and_size = actual_origins_and_sizes[glyph_index];
    typographic_width += advance_widths[glyph_index];
    for (int vertex_index = 0; vertex_index < kQuadVertices.size();
         ++vertex_index) {
      const QuadVertex& quad_vertex = kQuadVertices[vertex_index];
      // actual_origin does not account for the stoke width, so offset the
      // vertex by half the stroke width.
      float2 vertex = actual_origin_and_size.xy +
                      quad_vertex.position * actual_origin_and_size.zw -
                      float2(state_.stroke_width_pixels / 2.0f);

      vertex.x += glyph_x_offset;

      box_min = min(box_min, vertex);
      box_max = max(box_max, vertex);
    }

    glyph_x_offset += advance_widths[glyph_index];
  }

  fill_bounds_size_ = box_max - box_min - float2(state_.stroke_width_pixels);
  fill_bounds_left_ = box_min.x + state_.stroke_width_pixels / 2;

  float2 pivot_min = {
      GetHorizontalPivot(GetLeftPivot(), box_min.x, box_max.x,
                         typographic_width),
      GetVerticalPivot(GetBottomPivot(), box_min.y, box_max.y),
  };

  float2 pivot_max = {
      GetHorizontalPivot(GetRightPivot(), box_min.x, box_max.x,
                         typographic_width),
      GetVerticalPivot(GetTopPivot(), box_min.y, box_max.y),
  };

  text_pivot_ = GetPivot() * (pivot_max - pivot_min);
  text_pivot_ += pivot_min;

  if (text_layout_provider_.has_value()) {
    // For a custom layout set, pivot.y represents the translation perpendicular
    // to the path, while pivot.x is disabled.
    text_pivot_.x = 0;
  }
}

float TextRenderer::GetVerticalPivot(TextRendererState::VerticalPivot pivot,
                                     float min, float max) const {
  switch (pivot) {
    case TextRendererState::VERTICAL_PIVOT_BOTTOM_EXTENT:
      return min;
    case TextRendererState::VERTICAL_PIVOT_DESCENT:
      return -font_info_.descent();
    case TextRendererState::VERTICAL_PIVOT_BASELINE:
      return 0.0f;
    case TextRendererState::VERTICAL_PIVOT_ASCENT:
      return -font_info_.ascent();
    case TextRendererState::VERTICAL_PIVOT_TOP_EXTENT:
      return max;
    default:
      IMP_LOG(imp::FATAL) << "GetVerticalPivot called with invalid value";
  }
}

float TextRenderer::GetHorizontalPivot(TextRendererState::HorizontalPivot pivot,
                                       float min, float max,
                                       float typographic_width) const {
#if IMP_PLATFORM(DESKTOP)
  // TODO: On desktop, the typographic pivots are not supported.
  // Remove this once they are.
  if (pivot == TextRendererState::HORIZONTAL_PIVOT_TYPOGRAPHIC_LEFT) {
    pivot = TextRendererState::HORIZONTAL_PIVOT_LEFT;
  } else if (pivot == TextRendererState::HORIZONTAL_PIVOT_TYPOGRAPHIC_RIGHT) {
    pivot = TextRendererState::HORIZONTAL_PIVOT_RIGHT;
  }
#endif  // IMP_PLATFORM(DESKTOP)
  switch (pivot) {
    case TextRendererState::HORIZONTAL_PIVOT_TYPOGRAPHIC_LEFT:
      return 0;
    case TextRendererState::HORIZONTAL_PIVOT_LEFT_EXTENT:
      return min;
    case TextRendererState::HORIZONTAL_PIVOT_LEFT:
      return min + state_.stroke_width_pixels / 2.0f;
    case TextRendererState::HORIZONTAL_PIVOT_RIGHT:
      return max - state_.stroke_width_pixels / 2.0f;
    case TextRendererState::HORIZONTAL_PIVOT_RIGHT_EXTENT:
      return max;
    case TextRendererState::HORIZONTAL_PIVOT_TYPOGRAPHIC_RIGHT:
      return typographic_width;
    default:
      IMP_LOG(imp::FATAL) << "GetHorizontalPivot called with invalid value";
  }
}

void TextRenderer::CalculateBounds() {
  float2 bounds_offset =
      GetOffset() * float2{1.0f, -1.0f} * GetMetersPerPixel();

  bounds_.center = {bounds_.center.xy + bounds_offset, 0};
  bounds_.halfExtent.z = 0;
}

void TextRenderer::CalculatePath() {
  if (!text_layout_provider_.has_value()) {
    return;
  }

  path_.clear();

  float render_multiplier = GetMetersPerPixel();
  float2 bounds_offset = GetOffset() * float2{1.0f, -1.0f} * render_multiplier;

  std::vector<GlyphPlacement> glyph_placements =
      CalculateGlyphPlacements(*text_glyphs_);
  size_t glyph_count = text_glyphs_->Size();
  const absl::Span<const float4> actual_origins_and_sizes =
      text_glyphs_->ActualOriginsAndSizes();
  for (int glyph_index = 0; glyph_index < glyph_count; ++glyph_index) {
    float3 translation = glyph_placements[glyph_index].position;
    mat2f rotation = glyph_placements[glyph_index].rotation;

    // Place point roughly at center of text.
    // TODO: (broken link) - This is incorrect for EXTENT vertical pivots since
    // text_pivot_ will include the stroke but fill_bounds_size_ will not.
    float2 point = (float2{0.0, fill_bounds_size_.y * 0.5 - text_pivot_.y +
                                    actual_origins_and_sizes[glyph_index].y});

    point = rotation * point + translation.xy;
    point *= render_multiplier;
    point += bounds_offset;
    path_.push_back(float3(point, translation.z));
  }

  is_path_changed_ = true;
}

absl::StatusOr<float2> TextRenderer::GetNodeScreenPosition() const {
  const auto& camera = GetView().GetCameraManager().GetCamera();

  std::optional<float2> node_screen_position =
      GetView().IsPreciseTranslationEnabled()
          ? camera->PixelFromWorldPointPrecise(
                GetNode()->GetWorldPositionPrecise())
          : camera->PixelFromWorldPoint(GetNode()->GetWorldPosition());

  if (!node_screen_position.has_value()) {
    return absl::FailedPreconditionError(
        "TextRenderer is placed closer than the near plane of the camera.");
  }
  return absl::StatusOr<float2>(*node_screen_position);
}

Box TextRenderer::GetLocalBounds() const {
  if (!renderer_) {
    IMP_LOG(imp::FATAL) << "GetLocalBounds called when the TextRenderer isn't ready.";
  }
  return bounds_;
}

absl::StatusOr<std::vector<float3>> TextRenderer::GetLocalPath() const {
  if (!renderer_) {
    return absl::UnavailableError(
        "GetLocalPath called when the TextRenderer isn't ready.");
  }

  if (!text_layout_provider_.has_value()) {
    return absl::UnavailableError("No path function find for this text.");
  }

  return path_;
}

Box TextRenderer::GetWorldBounds() const {
  return TransformBounds(GetLocalBounds(), GetNode()->GetWorldTrs());
}

absl::StatusOr<Rect> TextRenderer::GetScreenLocalBounds() const {
  if (vertex_domain_ != filament::VertexDomain::DEVICE) {
    return absl::UnavailableError(
        "TextRenderer is in world space, so screen bounds are unavailable.");
  }

  Rect bounds = ToRect(GetLocalBounds());
  float2 subpixel_ratio = GetView().GetHost()->GetSubpixelRatio();
  bounds.center /= subpixel_ratio;
  bounds.half_extent /= subpixel_ratio;
  // In screen space the y-axis is down
  bounds.center.y = -bounds.center.y;
  return bounds;
}

absl::StatusOr<Rect> TextRenderer::GetScreenBounds() const {
  absl::StatusOr<Rect> bounds = GetScreenLocalBounds();
  if (!bounds.ok()) {
    return bounds.status();
  }
  absl::StatusOr<float2> position = GetNodeScreenPosition();
  if (!position.ok()) {
    return position.status();
  }

  bounds->center = *position + bounds->center;
  return bounds;
}

absl::StatusOr<std::vector<float3>> TextRenderer::GetScreenPath() const {
  if (vertex_domain_ != filament::VertexDomain::DEVICE) {
    return absl::UnavailableError(
        "TextRenderer is in world space, so screen bounds are unavailable.");
  }

  absl::StatusOr<float2> node_screen_position = GetNodeScreenPosition();
  if (!node_screen_position.ok()) {
    return node_screen_position.status();
  }

  absl::StatusOr<std::vector<float3>> path = GetLocalPath();
  if (!path.ok()) {
    return absl::UnavailableError("Path of text is not unavailable.");
  }

  float2 subpixel_ratio = GetView().GetHost()->GetSubpixelRatio();
  // In screen space the y-axis is down
  std::vector<float3> path_screen;
  path_screen.reserve(path->size());
  for (int i = 0; i < path->size(); ++i) {
    float3 point = path->at(i);
    point.xy /= subpixel_ratio;
    point.y = -point.y;
    path_screen.push_back(float3{*node_screen_position + point.xy, 0.0f});
  }
  return path_screen;
}

float2 TextRenderer::GetOffset() const { return state_.offset; }

void TextRenderer::SetOffset(float2 offset) {
  state_.offset = offset;
  RecalculateMesh();
}

float2 TextRenderer::GetPivot() const {
  return state_.pivot.value_or(kDefaultPivot);
}

void TextRenderer::SetPivot(float2 pivot) {
  state_.pivot = pivot;
  RecalculateMesh();
}

TextRendererState::VerticalPivot TextRenderer::GetTopPivot() const {
  return (state_.top_pivot == TextRendererState::VERTICAL_PIVOT_UNSPECIFIED)
             ? TextRendererState::VERTICAL_PIVOT_ASCENT
             : state_.top_pivot;
}

TextRendererState::VerticalPivot TextRenderer::GetBottomPivot() const {
  return (state_.bottom_pivot == TextRendererState::VERTICAL_PIVOT_UNSPECIFIED)
             ? TextRendererState::VERTICAL_PIVOT_DESCENT
             : state_.bottom_pivot;
}

TextRendererState::HorizontalPivot TextRenderer::GetLeftPivot() const {
  return (state_.left_pivot == TextRendererState::HORIZONTAL_PIVOT_UNSPECIFIED)
             ? TextRendererState::HORIZONTAL_PIVOT_LEFT_EXTENT
             : state_.left_pivot;
}

TextRendererState::HorizontalPivot TextRenderer::GetRightPivot() const {
  return (state_.right_pivot == TextRendererState::HORIZONTAL_PIVOT_UNSPECIFIED)
             ? TextRendererState::HORIZONTAL_PIVOT_RIGHT_EXTENT
             : state_.right_pivot;
}

BorrowedMaterialPtr TextRenderer::BorrowMaterial() const {
  if (!renderer_) {
    IMP_LOG(imp::FATAL) << "BorrowMaterial called when the TextRenderer isn't ready.";
  }
  return renderer_->BorrowMaterial();
}

bool TextRenderer::HasStroke() const { return state_.stroke_width_pixels > 0; }

float TextRenderer::GetStrokeWidthPixels() const {
  return state_.stroke_width_pixels;
}

float4 TextRenderer::GetStrokeColor() const {
  if (state_.stroke_color_rgba()) {
    return *state_.stroke_color_rgba();
  } else if (state_.stroke_color_rgb()) {
    return float4(*state_.stroke_color_rgb(), 1.0f);
  }

  return kOne4;
}

std::optional<TextAndFontMetrics> TextRenderer::GetPrecomputedMetrics() const {
  return !state_.precomputed_metrics.glyph_metrics().empty()
             ? std::make_optional(state_.precomputed_metrics)
             : std::nullopt;
}

const SystemFontParams* TextRenderer::GetSystemFontParams() const {
  return state_.system_font_params();
}

float TextRenderer::GetMetersPerPixel() const {
  return vertex_domain_ == filament::VertexDomain::DEVICE
             ? 1.0f
             : kDefaultMetersPerPixel;
}

float TextRenderer::GetOpacityMultiplier() const {
  return state_.opacity_multiplier.value_or(1.0f);
}

void TextRenderer::SetOpacityMultiplier(float opacity) {
  if (AlmostEqual(GetOpacityMultiplier(), opacity)) return;

  state_.opacity_multiplier = opacity;

  // Update material colors.
  Material* material = renderer_->GetMaterial();
  if (material) {
    ApplyColorsToMaterial(*material);
  }
}

}  // namespace imp
