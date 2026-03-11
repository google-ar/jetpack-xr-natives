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

#include "core/loader/provider/usdz/usdz_consumer.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/libs/geometry/include/geometry/SurfaceOrientation.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/robin_map.h"
#include "core/common/schemas/math_generated.h"
#include "core/common/schemas/render_generated.h"
#include "core/common/typed_id.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/loader/provider/usdz/examined_model.h"
#include "core/loader/provider/usdz/mesh_geometry.h"
#include "core/loader/provider/usdz/mesh_material.h"
#include "core/loader/provider/usdz/mesh_parts.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/math.h"
#include "third_party/tinyusdz/src/prim-types.hh"
#include "third_party/tinyusdz/src/usdGeom.hh"
#include "third_party/tinyusdz/src/usdShade.hh"
#include "third_party/tinyusdz/src/value-types.hh"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_usdz {

using ::filament::geometry::SurfaceOrientation;
using ::tinyusdz::Path;

// usdz defaults to an export in centimeters, not meters.
static constexpr float kPositionScale = 0.01f;

namespace {

mat4f GetLocalTransform(const tinyusdz::Prim &prim) {
  bool resetXformStak;
  tinyusdz::value::matrix4d mat = GetLocalTransform(
      prim, &resetXformStak, tinyusdz::value::TimeCode::Default(),
      tinyusdz::value::TimeSampleInterpolationType::Held);
  return mat4f(mat.m[0][0], mat.m[0][1], mat.m[0][2], mat.m[0][3],  //^
               mat.m[1][0], mat.m[1][1], mat.m[1][2], mat.m[1][3],  //^
               mat.m[2][0], mat.m[2][1], mat.m[2][2], mat.m[2][3],  //^
               mat.m[3][0], mat.m[3][1], mat.m[3][2], mat.m[3][3]);
}

template <typename Element>
absl::StatusOr<absl::Span<Element>> CreateBuffer(size_t element_count,
                                                 BufferAccess *buffer) {
  uint8_t *buffer_ptr =
      BufferAccess::Create(sizeof(Element) * element_count, buffer);
  if (!buffer_ptr)
    return absl::InternalError(absl::StrFormat(
        "Failed to allocate storage for %llu elements", element_count));
  return absl::Span<Element>(reinterpret_cast<Element *>(buffer_ptr),
                             element_count);
}

struct EntityGeometry {
  BufferAccess positions_buffer;
  BufferAccess texcoords_buffer;
  BufferAccess orientations_buffer;
  BufferAccess colors_buffer;
  BufferAccess secondary_texcoords_buffer;

  absl::Span<float3> positions;
  absl::Span<float2> texcoords;
  absl::Span<quatf> orientations;
  absl::Span<ubyte4> colors;
  absl::Span<ushort2> secondary_texcoords;

  size_t parts_triangle_count;
  BufferAccess parts_indices_buffer;
  absl::Span<uint3> parts_indices;

  std::vector<filament::Box> face_bounds;
  std::vector<filament::Box> part_bounds;
  filament::Box bounds;

  static absl::StatusOr<EntityGeometry> Construct(const tinyusdz::Prim &prim,
                                                  const MeshGeometry &geometry,
                                                  const MeshParts &parts);

  // Private constructed and move-only
  EntityGeometry(const EntityGeometry &) = delete;
  EntityGeometry &operator=(const EntityGeometry &rhs) = delete;
  EntityGeometry(EntityGeometry &&rhs) = default;
  EntityGeometry &operator=(EntityGeometry &&rhs) = default;

 private:
  EntityGeometry() = default;
};

}  //  namespace

struct GeometryTransformer {
  size_t &indices_cursor;
  const MeshGeometry &geometry;
  absl::Span<float3> &normals;
  absl::Span<uint3> &indices;
  EntityGeometry &result;

  using point3f = tinyusdz::value::point3f;
  using texcoord2f = tinyusdz::value::texcoord2f;
  using normal3f = tinyusdz::value::normal3f;

  uint32_t operator()(int32_t face_vertex_offset, int32_t face_vertex_count) {
    uint32_t first_indices_index = static_cast<uint32_t>(indices_cursor);

    size_t vi0 = face_vertex_offset;
    const point3f &p0 = geometry.points[geometry.face_vertex_indices[vi0]];
    const texcoord2f &t0 = geometry.texcoords[geometry.texcoord_indices[vi0]];
    const normal3f &n0 = geometry.normals[vi0];
    result.positions[vi0] = float3(p0.x, p0.y, p0.z) * kPositionScale;
    result.texcoords[vi0] = float2(t0.s, 1.0f - t0.t);
    normals[vi0] = float3(n0.x, n0.y, n0.z);

    size_t vi1 = face_vertex_offset + 1;
    const point3f &p1 = geometry.points[geometry.face_vertex_indices[vi1]];
    const texcoord2f &t1 = geometry.texcoords[geometry.texcoord_indices[vi1]];
    const normal3f &n1 = geometry.normals[vi1];

    result.positions[vi1] = float3(p1.x, p1.y, p1.z) * kPositionScale;
    result.texcoords[vi1] = float2(t1.s, 1.0f - t1.t);
    normals[vi1] = float3(n1.x, n1.y, n1.z);

    for (size_t corner = 2; corner < face_vertex_count; ++corner) {
      size_t vi2 = face_vertex_offset + corner;

      const point3f &p2 = geometry.points[geometry.face_vertex_indices[vi2]];
      const texcoord2f &t2 = geometry.texcoords[geometry.texcoord_indices[vi2]];
      const normal3f &n2 = geometry.normals[vi2];

      result.positions[vi2] = float3(p2.x, p2.y, p2.z) * kPositionScale;
      result.texcoords[vi2] = float2(t2.s, 1.0f - t2.t);
      normals[vi2] = float3(n2.x, n2.y, n2.z);

      indices[indices_cursor++] = uint3(vi0, vi1, vi2);
      vi1 = vi2;
    }
    return first_indices_index;
  }
};

// static
absl::StatusOr<EntityGeometry> EntityGeometry::Construct(
    const tinyusdz::Prim &prim, const MeshGeometry &geometry,
    const MeshParts &parts) {
  EntityGeometry result;

  std::vector<int32_t> face_vertex_offsets;
  int32_t counter = 0;
  size_t face_count = geometry.face_vertex_counts.size();
  size_t triangle_count = 0;
  face_vertex_offsets.reserve(geometry.face_vertex_counts.size());
  absl::c_transform(geometry.face_vertex_counts,
                    std::back_inserter(face_vertex_offsets),
                    [&counter, &triangle_count](int32_t face_vertex_count) {
                      int transform_result = counter;
                      counter += face_vertex_count;
                      triangle_count += face_vertex_count - 2;
                      return transform_result;
                    });

  size_t vertex_count = geometry.normals.size();

  // Normals get converted to orientations, so they're intermediate data.
  BufferAccess normals_buffer;
  absl::Span<float3> normals;
  // The indices for all triangles; used to spool together part indices.
  BufferAccess indices_buffer;
  absl::Span<uint3> indices;

  MP_ASSIGN_OR_RETURN(
      result.positions,
      CreateBuffer<float3>(vertex_count, &result.positions_buffer));
  MP_ASSIGN_OR_RETURN(
      result.texcoords,
      CreateBuffer<float2>(vertex_count, &result.texcoords_buffer));
  MP_ASSIGN_OR_RETURN(
      result.orientations,
      CreateBuffer<quatf>(vertex_count, &result.orientations_buffer));
  MP_ASSIGN_OR_RETURN(normals,
                   CreateBuffer<float3>(vertex_count, &normals_buffer));
  MP_ASSIGN_OR_RETURN(result.colors,
                   CreateBuffer<ubyte4>(vertex_count, &result.colors_buffer));
  MP_ASSIGN_OR_RETURN(
      result.secondary_texcoords,
      CreateBuffer<ushort2>(vertex_count, &result.secondary_texcoords_buffer));
  MP_ASSIGN_OR_RETURN(indices,
                   CreateBuffer<uint3>(triangle_count, &indices_buffer));

  // Translate the geometry into a format compatible with vertex/index buffers.
  size_t indices_cursor = 0;
  std::vector<uint32_t> face_first_output_triangle;
  face_first_output_triangle.reserve(face_count);
  absl::c_transform(
      face_vertex_offsets, geometry.face_vertex_counts,
      std::back_inserter(face_first_output_triangle),
      GeometryTransformer{indices_cursor, geometry, normals, indices, result});
  // Adds an extra entry to enable peeking at N+1 to determine indices-per-face.
  face_first_output_triangle.push_back(static_cast<uint32_t>(indices_cursor));

  result.face_bounds.reserve(face_count);
  absl::c_transform(
      face_vertex_offsets, geometry.face_vertex_counts,
      std::back_inserter(result.face_bounds),
      [&result](int32_t face_vertex_offset, int32_t face_vertex_count) {
        using float_limits = std::numeric_limits<float>;
        float3 pos_min{float_limits::max()};
        float3 pos_max{float_limits::lowest()};

        for (size_t corner = 0; corner < face_vertex_count; ++corner) {
          size_t vi = face_vertex_offset + corner;
          float3 pos = result.positions[vi];
          pos_min = min(pos_min, pos);
          pos_max = max(pos_max, pos);
        }

        return filament::Box{}.set(pos_min, pos_max);
      });

  if (auto orientation_builder = absl::WrapUnique<SurfaceOrientation>(
          SurfaceOrientation::Builder()
              .vertexCount(vertex_count)
              .positions(result.positions.data())
              .uvs(result.texcoords.data())
              .normals(normals.data())
              .triangleCount(triangle_count)
              .triangles(indices.data())
              .build())) {
    // Writes out orientation data.
    orientation_builder->getQuats(
        const_cast<quatf *>(result.orientations.data()), vertex_count);
  } else {
    return absl::InternalError("Failed to create orientations");
  }

  // emit default colors
  absl::c_fill(result.colors, ubyte4{255, 255, 255, 255});
  // emit default secondary UVs
  absl::c_fill(result.secondary_texcoords, ushort2(0, 0));

  result.parts_triangle_count = absl::c_accumulate(parts.triangle_counts, 0ul);
  MP_ASSIGN_OR_RETURN(result.parts_indices,
                   CreateBuffer<uint3>(result.parts_triangle_count,
                                       &result.parts_indices_buffer));

  size_t parts_indices_cursor = 0;
  using float_limits = std::numeric_limits<float>;
  float3 bound_min = float_limits::max();
  float3 bound_max = float_limits::lowest();

  result.part_bounds.reserve(parts.face_indices.size());
  for (const std::vector<int32_t> &face_indices : parts.face_indices) {
    float3 part_bound_min = float_limits::max();
    float3 part_bound_max = float_limits::lowest();

    for (size_t face_index = 0, face_count = face_indices.size();
         face_index < face_count; ++face_index) {
      int32_t face = face_indices[face_index];
      filament::Box &face_bound = result.face_bounds[face];
      part_bound_min = min(part_bound_min, face_bound.getMin());
      part_bound_max = max(part_bound_max, face_bound.getMax());

      int32_t face_indices_begin_index = face_first_output_triangle[face];
      int32_t face_indices_end_index = face_first_output_triangle[face + 1];
      std::copy(indices.data() + face_indices_begin_index,
                indices.data() + face_indices_end_index,
                result.parts_indices.data() + parts_indices_cursor);
      parts_indices_cursor +=
          (face_indices_end_index - face_indices_begin_index);
    }

    bound_min = min(bound_min, part_bound_min);
    bound_max = max(bound_max, part_bound_max);

    result.part_bounds.push_back(
        filament::Box{}.set(part_bound_min, part_bound_max));
  }
  result.bounds = filament::Box{}.set(bound_min, bound_max);

  return result;
}

absl::StatusOr<FlatBufferAccess<schemas::LoadedModel>>
UsdzConsumer::BuildLoadedModel() {
  MP_ASSIGN_OR_RETURN(ExaminedModel examined_model,
                   ExaminedModel::FromStage(stage_));

  // Build materials.
  for (auto material : examined_model.material_prims.Ids<MaterialId>()) {
    const tinyusdz::Prim *material_prim =
        examined_model.material_prims[material];
    MP_RETURN_IF_ERROR(
        ExtractMaterial(*material_prim, static_cast<uint16_t>(material)));
  }

  // Build bones.
  ReserveBones(examined_model.bone_prims.size());
  for (auto bone : examined_model.bone_prims.Ids<BoneId>()) {
    const tinyusdz::Prim *bone_prim = examined_model.bone_prims[bone];
    mat4f local_transform = GetLocalTransform(*bone_prim);
    // TODO : Preserve initial rotation values if provided.
    MP_RETURN_IF_ERROR(AddBone(examined_model.bone_child_counts[bone],
                            PreciseTransform(local_transform),
                            bone_prim->element_name(),
                            static_cast<uint16_t>(bone)));
  }
  MP_RETURN_IF_ERROR(FinishBones());

  // Build entities.
  ReserveEntities(examined_model.entity_prims.size());
  for (auto entity : examined_model.entity_prims.Ids<EntityId>()) {
    const tinyusdz::Prim *entity_prim = examined_model.entity_prims[entity];
    MP_RETURN_IF_ERROR(
        ExtractEntity(*entity_prim, examined_model.entity_bones[entity]));
  }
  MP_RETURN_IF_ERROR(FinishEntities());
  MP_RETURN_IF_ERROR(FinishSkins());

  return Finish();
}

absl::StatusOr<UsdzConsumer::TextureId> UsdzConsumer::EnsureTexture(
    const SurfaceTexture &surface_texture, bool expect_srgb) {
  if (!surface_texture.file.has_value()) return absl::InternalError("No file");

  auto itr =
      encoded_image_from_path_.find(surface_texture.file->GetAssetPath());
  if (itr == encoded_image_from_path_.end())
    return absl::InternalError("Missing texture");

  const BufferAccess &encoded_image = itr->second;
  using SourceColorSpace = tinyusdz::UsdUVTexture::SourceColorSpace;

  bool is_srgb = false;
  switch (surface_texture.source_color_space.value_or(SourceColorSpace::Auto)) {
    default:
    case SourceColorSpace::Auto:
      is_srgb = expect_srgb;
      break;
    case SourceColorSpace::Raw:
      is_srgb = false;
      break;
    case SourceColorSpace::SRGB:
      is_srgb = true;
      break;
  }

  // Duplicate the image to allow multiple materials to sample the same texture.
  ImageData pending_texture_data =
      BufferAccess::Clone(encoded_image.Data(), encoded_image.Size());

  schemas::TextureInfoFlags pending_texture_flags =
      is_srgb ? schemas::TextureInfoFlags::IsSrgb
              : schemas::TextureInfoFlags::NONE;
  TextureId texture_id =
      AddTexture(next_texture_id_, surface_texture.file->GetAssetPath(),
                 pending_texture_flags, std::move(pending_texture_data));
  next_texture_id_++;
  return texture_id;
}

absl::Status UsdzConsumer::ExtractMaterial(const tinyusdz::Prim &prim,
                                           uint16_t material_index) {
  std::string full_path = prim.absolute_path().full_path_name();

  filament::Box root_bounds;
  GenericMaterialSpec generic_material_spec(
      schemas::GenericMaterialLightingModel::Lit,
      schemas::GenericMaterialBlendMode::Opaque,
      schemas::GenericMaterialDoubleSidedMode::SingleSided,
      schemas::GenericMaterialDepthClearMaterial::Disabled);
  GenericMaterialParameters generic_material_parameters;

  MP_ASSIGN_OR_RETURN(MeshMaterial attributes, MeshMaterial::Extract(prim));
  using color3f = tinyusdz::value::color3f;

  // Base Color
  {
    generic_material_parameters.base_color.emplace();
    auto base_color_factor = float4(1.0f, 1.0f, 1.0f, 1.0f);
    TextureId base_color_texture;
    if (const color3f *scalar =
            std::get_if<color3f>(&attributes.diffuse_color)) {
      base_color_factor = float4(scalar->r, scalar->g, scalar->b, 1.0f);
    } else if (const Path *path =
                   std::get_if<Path>(&attributes.diffuse_color)) {
      SurfaceTextureId surface_texture_id =
          attributes.surface_textures_from_name[path->prim_part()];
      if (!surface_texture_id) {
        return absl::InternalError("Failed to parse base color texture");
      }
      const SurfaceTexture &surface_texture =
          attributes.surface_textures[surface_texture_id];
      MP_ASSIGN_OR_RETURN(base_color_texture,
                       EnsureTexture(surface_texture, true));
      generic_material_parameters.base_color->texture =
          GenericMaterialTextureParameter{
              static_cast<uint16_t>(base_color_texture),
              filament::TextureSampler(),
              /*uv_transform=*/mat3f(),
              /*uses_uv1=*/false};
    } else {
      return absl::InternalError("Failed to parse base color");
    }
    generic_material_parameters.base_color->factor = kOne4;
  }

  // Normal
  {
    generic_material_parameters.normal.emplace();
    if (const Path *path = std::get_if<Path>(&attributes.normal)) {
      SurfaceTextureId surface_texture_id =
          attributes.surface_textures_from_name[path->prim_part()];
      if (!surface_texture_id) {
        return absl::InternalError("Failed to parse normal texture");
      }
      const SurfaceTexture &surface_texture =
          attributes.surface_textures[surface_texture_id];
      MP_ASSIGN_OR_RETURN(TextureId normal_texture,
                       EnsureTexture(surface_texture, false));
      generic_material_parameters.normal->texture =
          GenericMaterialTextureParameter{static_cast<uint16_t>(normal_texture),
                                          filament::TextureSampler(),
                                          /*uv_transform=*/mat3f(),
                                          /*uses_uv1=*/false};
      generic_material_parameters.normal->factor = 1.0f;
    }
  }

  // Metallic/Roughness
  {
    generic_material_parameters.metallic_roughness.emplace();
    float metallic_factor = 0.0f;
    float roughness_factor = 1.0f;

    const auto *scalar_roughness = std::get_if<float>(&attributes.roughness);
    const auto *scalar_metallic = std::get_if<float>(&attributes.metallic);
    const auto *path_roughness = std::get_if<Path>(&attributes.roughness);
    const auto *path_metallic = std::get_if<Path>(&attributes.metallic);

    if (scalar_roughness && scalar_metallic) {
      roughness_factor = *scalar_roughness;
      metallic_factor = *scalar_metallic;
    } else if (path_roughness || path_metallic) {
      // USDZ may output metallic/roughness as separate textures, which isn't
      // supported by the gltf spec or the impress material system.
      IMP_LOG(imp::ERROR) << "Unsupported metal/roughness configuration";
    } else {
      return absl::InternalError("Failed to parse metallic/roughness");
    }

    generic_material_parameters.metallic_roughness->metallic_factor =
        metallic_factor;
    generic_material_parameters.metallic_roughness->roughness_factor =
        roughness_factor;
  }

  flatbuffers::FlatBufferBuilder &fbb = GetFlatBufferBuilder();
  flatbuffers::Offset<schemas::GenericMaterialParameters>
      generic_material_parameters_offset =
          generic_material_parameters.ToFlatbuffer(fbb);

  const auto center = root_bounds.center;
  const auto half_extent = root_bounds.halfExtent;
  schemas::Box box(flatbuffers::Pack(center), flatbuffers::Pack(half_extent));

  MaterialId material = AddMaterial(
      material_index,
      schemas::CreateMaterialInfo(
          fbb,
          schemas::CreateGenericMaterial(
              fbb, fbb.CreateString(full_path.data(), full_path.size()),
              generic_material_spec.ToFlatbuffer(fbb),
              schemas::CreateBoundsInfo(fbb, &box),
              generic_material_parameters_offset),
          material_index));

  material_from_path_[full_path] = material;

  return absl::OkStatus();
}

absl::Status UsdzConsumer::ExtractEntity(const tinyusdz::Prim &prim,
                                         BoneId bone) {
  const tinyusdz::GeomMesh &mesh = *prim.as<tinyusdz::GeomMesh>();

  MP_ASSIGN_OR_RETURN(MeshGeometry geometry, MeshGeometry::Collect(mesh));

  MP_ASSIGN_OR_RETURN(MeshParts parts,
                   MeshParts::FromMesh(prim, geometry.face_vertex_counts));

  MP_ASSIGN_OR_RETURN(EntityGeometry entity_geometry,
                   EntityGeometry::Construct(prim, geometry, parts));

  // Emit vertex buffer
  std::vector<VertexBlock> vertex_blocks;
  vertex_blocks.push_back(
      VertexBlock::Positions(std::move(entity_geometry.positions_buffer)));
  vertex_blocks.emplace_back(
      VertexBlock::Tangents(std::move(entity_geometry.orientations_buffer)));
  vertex_blocks.emplace_back(VertexBlock::PrimaryUvFloats(
      std::move(entity_geometry.texcoords_buffer)));
  vertex_blocks.emplace_back(
      VertexBlock::Colors(std::move(entity_geometry.colors_buffer)));
  vertex_blocks.emplace_back(VertexBlock::SecondaryUvShorts(
      std::move(entity_geometry.secondary_texcoords_buffer)));

  VertexBufferId vertex_buffer = AddVertexBuffer(
      std::move(vertex_blocks), entity_geometry.orientations.size(), false);

  // Emit index buffer
  // TODO: Currently, IndexBuffer in usdz_consumer cannot be sparse
  // data.
  size_t parts_indices_count = entity_geometry.parts_triangle_count * 3;
  absl::optional<DenseDataAccess> parts_index_data = DenseDataAccess(
      BufferAccess::Wrap(reinterpret_cast<const uint8_t *>(
                             entity_geometry.parts_indices.data()),
                         sizeof(uint32_t) * parts_indices_count),
      parts_indices_count, sizeof(uint32_t));
  IndexBufferId parts_index_buffer =
      AddIndexBuffer(parts_index_data, parts_indices_count);
  if (!parts_index_buffer)
    return absl::InternalError("Failed to create index buffer for part.");

  // Emit parts
  std::vector<PartData> entity_parts;
  size_t parts_indices_cursor = 0;
  for (auto part : parts.face_indices.Ids<PartId>()) {
    const Path &material_path = parts.material_paths[part];
    auto part_triangle_count = parts.triangle_counts[part];
    MP_ASSIGN_OR_RETURN(MaterialId material, GetMaterial(material_path));

    MaterialsVariantsMappingLookup materials_variants_mappings;
    SkinningBufferId skinning_buffer;
    uint32_t morph_target_buffer_offset = 0;
    uint32_t morph_target_count = 0;
    entity_parts.push_back(PartData(
        "", parts_indices_cursor * 3, part_triangle_count * 3, vertex_buffer,
        parts_index_buffer, material, -1, PrimitiveType::TRIANGLES,
        std::move(materials_variants_mappings), skinning_buffer,
        morph_target_buffer_offset, morph_target_count));
    parts_indices_cursor += part_triangle_count;
  }

  // Emit entity
  SkinId skin;
  MorphTargetBufferId morph_target_buffer;
  std::vector<float> node_morph_target_weights;
  std::vector<float> mesh_morph_target_weights;
  LightPunctualId light_punctual;
  AudioEmitterId audio_emitter;
  absl::optional<filament::Box> bounds;
  absl::optional<RuntimeData> runtime;
  int child_count = 0;
  if (!entity_parts.empty()) {
    bounds.emplace(entity_geometry.bounds);
    runtime.emplace(RuntimeData::Default());
  }
  MP_RETURN_IF_ERROR(AddEntity(bone, skin, morph_target_buffer,
                            std::move(node_morph_target_weights),
                            std::move(mesh_morph_target_weights),
                            light_punctual, audio_emitter,
                            std::move(entity_parts), bounds, runtime,
                            child_count, prim.element_name())
                      .status());

  return absl::OkStatus();
}

absl::StatusOr<UsdzConsumer::MaterialId> UsdzConsumer::GetMaterial(
    const tinyusdz::Path &material_path) {
  auto it = material_from_path_.find(material_path.full_path_name());
  if (it == material_from_path_.end())
    return Error("Failed to find material %s", material_path.full_path_name());
  return it->second;
}

}  // namespace imp::loader::details::provider_usdz
