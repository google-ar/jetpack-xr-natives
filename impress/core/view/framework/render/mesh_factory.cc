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

#include "core/view/framework/render/mesh_factory.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/common/robin_map.h"
#include "core/common/typed_vector.h"
#include "core/geometry/shapes/box.h"
#include "core/math/aabb_helpers.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_builder.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_gpu_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/vertex_format.h"
#include "core/view/base_view.h"

namespace imp {
namespace {

using VertexAttribute = VertexFormat::VertexAttribute;
using AttributeType = VertexFormat::AttributeType;

static constexpr uint8_t kMinimumSphereResolution = 8;
static constexpr uint8_t kMinimumCylinderResolution = 8;
static constexpr uint8_t kMinimumCapsuleResolution = 8;
static constexpr uint8_t kMinimumConeResolution = 3;

const VertexFormat kVertexFormat = {
    {VertexAttribute::POSITION, AttributeType::FLOAT3},
    {VertexAttribute::UV0, AttributeType::FLOAT2},
    {VertexAttribute::TANGENTS, AttributeType::FLOAT4}};

const VertexFormat kVertexFormatWithColor = {
    {VertexAttribute::POSITION, AttributeType::FLOAT3},
    {VertexAttribute::UV0, AttributeType::FLOAT2},
    {VertexAttribute::TANGENTS, AttributeType::FLOAT4},
    {VertexAttribute::COLOR, AttributeType::FLOAT4}};

static constexpr float kQuadHalfExtent = 0.5f;

static constexpr float3 kQuadPositions[] = {
    {-kQuadHalfExtent, -kQuadHalfExtent, 0},
    {kQuadHalfExtent, -kQuadHalfExtent, 0},
    {kQuadHalfExtent, kQuadHalfExtent, 0},
    {-kQuadHalfExtent, kQuadHalfExtent, 0}};

static constexpr uint16_t kQuadIndices[] = {
    0, 1, 2,  //
    0, 2, 3,
};

static constexpr Box kRadius1Aabb = {
    /*center=*/{0, 0, 0},
    /*halfExtent=*/{kQuadHalfExtent, kQuadHalfExtent, 0}};

template <typename T, size_t N>
constexpr size_t SizeOfArray(const T (&array)[N]) {
  return N;
}

// Calculator for scaling uvs from positions.
class UVScaler {
 public:
  explicit UVScaler(bool flip_uv = false, float2 min = {-1, -1},
                    float2 max = {1, 1})
      : flip_uv_(flip_uv), min_(min), size_(max - min) {}

  // z is ignored.
  float2 UVFromPos(float3 pos) {
    auto uv = (pos.xy - min_) / size_;
    if (flip_uv_) {
      uv.y = 1.0f - uv.y;
    }
    return uv;
  }

 private:
  bool flip_uv_;
  float2 min_ = {};
  float2 size_ = {};
};

// Helps the mesh factory add vertex indices to a mesh data index buffer.
class ProceduralMeshBuilder {
  // A fixed bitangent to be used with all shapes for the time being.
  static constexpr float3 kBitangent = {0, 1, 0};

  // TODO:((broken link)): Refactor uv config and auto-calculation later.
  // TODO:((broken link)): Refactor handling of both vertices and indices.
 public:
  // Completes mesh construction and relinquishes ownership of the mesh data.
  std::unique_ptr<imp::MeshData> Build() {
    FinalizeTangents();
    return std::move(mesh_data_);
  }

  ProceduralMeshBuilder(size_t num_vertices, size_t num_faces,
                        VertexFormat vertex_format, bool is_smooth = true,
                        std::optional<float4> color = std::nullopt) {
    MeshDescription mesh_description =
        MeshDescription{vertex_format, MeshDescription::IndexType::USHORT,
                        num_vertices, num_faces * 3};
    mesh_data_ = std::make_unique<MeshData>(mesh_description);

    max_faces_ = num_faces;
    number_of_faces_ = 0;

    uv_coordinates_.reserve(num_vertices);
    smooth_normals_ = is_smooth;
    color_ = color;
  }

  int AddVertex(float3 position, int num_uv0s = 1) {
    int vertex_entry_index = vertex_metadata_by_strkey_.size();
    int uv_vertex_index = uv_coordinates_.size();
    int id = smooth_normals_ ? 0 : vertex_entry_index;
    std::string strkey = StrKeyFromFloat3(id, position);
    std::unique_ptr<VertexMetadata>& vertex_entry =
        vertex_metadata_by_strkey_[strkey];
    if (vertex_entry == nullptr) {
      vertex_entry = std::make_unique<VertexMetadata>();
      vertex_entry->position = position;
      vertex_entry->normal = {0, 0, 1};
      vertex_entry->tangent = {1, 0, 0};
      vertex_entry->bitangent = kBitangent;
      vertex_entry->index = vertex_entry_index;
    } else {
      vertex_entry_index = vertex_entry->index;
    }

    for (int i = uv_vertex_index; i < uv_vertex_index + num_uv0s; i++) {
      uv_coordinates_.push_back(UvCoordinate());
      uv_coordinates_[i].space_data = vertex_metadata_by_strkey_[strkey].get();
      uv_coordinates_[i].uv0 = {0, 1};
    }

    return vertex_entry_index;
  }

  void SetUV0(int uv_vertex_index, float2 uv0) {
    uv_coordinates_[uv_vertex_index].uv0 = uv0;
  }

  // Adds a triangle to the index buffer. Assumes counter-clockwise order.
  void AddTriangle(uint16_t point_a_index, uint16_t point_b_index,
                   uint16_t point_c_index, bool flip_normal = false) {
    //
    //          B
    //         / \
    //        /   \
    //       /     \
    //      C ----- A
    VertexMetadata* a = uv_coordinates_[point_a_index].space_data;
    VertexMetadata* b = uv_coordinates_[point_b_index].space_data;
    VertexMetadata* c = uv_coordinates_[point_c_index].space_data;

    float3 ab = normalize(b->position - a->position);
    float3 ac = normalize(c->position - a->position);
    float3 shared_normal = flip_normal ? cross(ac, ab) : cross(ab, ac);

    int starting_index = number_of_faces_ * 3;
    AddNormalToVertex(a, shared_normal);
    AddNormalToVertex(b, shared_normal);
    AddNormalToVertex(c, shared_normal);

    if (!flip_normal) {
      mesh_data_->IndexAt<uint16_t>(starting_index) = point_a_index;
      mesh_data_->IndexAt<uint16_t>(starting_index + 1) = point_b_index;
      mesh_data_->IndexAt<uint16_t>(starting_index + 2) = point_c_index;
    } else {  // Insert clockwise to flip the face
      mesh_data_->IndexAt<uint16_t>(starting_index) = point_c_index;
      mesh_data_->IndexAt<uint16_t>(starting_index + 1) = point_b_index;
      mesh_data_->IndexAt<uint16_t>(starting_index + 2) = point_a_index;
    }

    number_of_faces_++;
  }

  // Adds a quad between the given vertex indices.
  void AddQuad(uint16_t top_left_index, uint16_t top_right_index,
               uint16_t bottom_left_index, uint16_t bottom_right_index,
               bool flip_normal = false) {
    AddTriangle(bottom_left_index, top_right_index, top_left_index,
                flip_normal);
    AddTriangle(top_right_index, bottom_left_index, bottom_right_index,
                flip_normal);
  }

  // Adds a strip of faces along the given vertex indices.
  void AddQuadStrip(uint16_t top_start_index, uint16_t top_end_index,
                    uint16_t bottom_start_index, uint16_t bottom_end_index,
                    bool is_loop = true, bool flip_normal = false) {
    int num_pairs = top_end_index - top_start_index;
    for (int i = 1; i <= num_pairs; i++) {
      uint16_t bottom_left = bottom_start_index + i - 1;
      uint16_t bottom_right = bottom_start_index + i;
      uint16_t top_left = top_start_index + i - 1;
      uint16_t top_right = top_start_index + i;
      AddQuad(top_left, top_right, bottom_left, bottom_right, flip_normal);
    }
    if (is_loop) {
      AddQuad(bottom_end_index, bottom_start_index, top_start_index,
              top_end_index, flip_normal);
    }
  }

  // Creates a fan around a given center index along a ring of vertex indices.
  // Assumes the center index is a single shared vertex.
  void AddTriangleFan(uint16_t center_index, uint16_t fan_index_start,
                      uint16_t fan_index_end, bool is_loop = true,
                      bool flip_normal = false) {
    int num_faces = fan_index_end - fan_index_start;
    if (is_loop) num_faces++;
    for (int i = 0; i < num_faces; i++) {
      uint16_t left = fan_index_start + i;
      uint16_t increment = (is_loop) ? ((i + 1) % num_faces) : i + 1;
      uint16_t right = fan_index_start + increment;
      AddTriangle(right, left, center_index, flip_normal);
    }
  }

  // Creates a fan around a given center index along a ring of vertex indices.
  // Assumes the center index is duplicated per face in the fan.
  void AddTriangleFanWithDuplicateCenterVertices(uint16_t center_index_start,
                                                 uint16_t fan_index_start,
                                                 uint16_t fan_index_end,
                                                 bool is_loop = true,
                                                 bool flip_normal = false) {
    int num_faces = fan_index_end - fan_index_start;
    if (is_loop) num_faces++;
    for (int i = 0; i < num_faces; i++) {
      uint16_t center_index = center_index_start + i;
      uint16_t left = fan_index_start + i;
      uint16_t increment = (is_loop) ? ((i + 1) % num_faces) : i + 1;
      uint16_t right = fan_index_start + increment;
      AddTriangle(right, center_index, left, flip_normal);
    }
  }

 private:
  struct VertexMetadata {
    float3 position;
    float3 normal;
    float3 tangent;
    float3 bitangent;
    quatf tangents;
    int index;
  };

  struct UvCoordinate {
    VertexMetadata* space_data;
    float2 uv0;
  };

  // An internal reference to mesh data.
  std::unique_ptr<MeshData> mesh_data_;

  // The current total number of faces that have been added to index buffer.
  int number_of_faces_;

  // The max number of faces we've allocated space for.
  int max_faces_;

  // Vertices differentiated by the position.
  RobinMap<std::string, std::unique_ptr<VertexMetadata>>
      vertex_metadata_by_strkey_;

  std::vector<UvCoordinate> uv_coordinates_;

  bool smooth_normals_;

  std::optional<float4> color_;

  // The sum of the face normals at each vertex metadata.
  RobinMap<int, float3> sum_normals_by_vertex_metadata_index_;

  // Generates a key based on an id and a given float3.
  std::string StrKeyFromFloat3(int id, float3 data) {
    static constexpr absl::string_view kKeyFormat = "%d:%d-%d-%d";
    static const int kScale = 1e4;
    return absl::StrFormat(kKeyFormat, id,
                           static_cast<int>(floor(data.x * kScale)),
                           static_cast<int>(floor(data.y * kScale)),
                           static_cast<int>(floor(data.z * kScale)));
  }

  // Adds the normal to a sum of normals currently at the given vertex.
  // This sum is later normalized at the end.
  void AddNormalToVertex(VertexMetadata* vertex, float3 new_normal) {
    int index = vertex->index;
    auto sum_of_normals = sum_normals_by_vertex_metadata_index_.find(index);
    if (sum_of_normals == sum_normals_by_vertex_metadata_index_.end()) {
      sum_normals_by_vertex_metadata_index_.insert({index, new_normal});
      return;
    }
    sum_normals_by_vertex_metadata_index_[index] += new_normal;
  }

  // Calculates the tangents quaternion for a given vertex.
  // The tangents quaternion has the tangent, bitangent and normal
  // data stored within.
  void CalculateTangents(VertexMetadata* space_data) {
    float3 tangent;
    float3 bitangent = kBitangent;
    float3 normal;

    // Calculate the normal.
    float3 sum_of_normals =
        sum_normals_by_vertex_metadata_index_[space_data->index];
    normal = normalize(sum_of_normals);

    // Calculate the tangent using the bitangent and normal.
    tangent = cross(bitangent, normal);
    float tangent_norm = norm(tangent);
    if (tangent_norm > std::numeric_limits<float>::epsilon()) {
      tangent /= tangent_norm;
    } else {
      bitangent = kBack;
      tangent = normalize(cross(bitangent, normal));
    }

    // Calculate the tangents from the three direction vectors.
    space_data->tangents =
        mat3f::packTangentFrame({tangent, bitangent, normal});
  }

  // Calculates the tangents quaternion for all current vertices.
  void FinalizeTangents() {
    for (auto const& [hash_key, vertex] : vertex_metadata_by_strkey_) {
      CalculateTangents(vertex.get());
    }
    for (int i = 0; i < uv_coordinates_.size(); i++) {
      mesh_data_->VertexAttributeAt<float3>(i, VertexAttribute::POSITION) =
          uv_coordinates_[i].space_data->position;
      mesh_data_->VertexAttributeAt<float2>(i, VertexAttribute::UV0) =
          uv_coordinates_[i].uv0;
      mesh_data_->VertexAttributeAt<quatf>(i, VertexAttribute::TANGENTS) =
          uv_coordinates_[i].space_data->tangents;
      if (color_) {
        mesh_data_->VertexAttributeAt<float4>(i, VertexAttribute::COLOR) =
            *color_;
      }
    }
  }
};

// Generates a Half-ring of vertices around a center point, then adds them
// to a given vertex buffer. X goes from -1 to 1, Z goes from 0 to -1 to 0.
// (This is designed so the user is looking into the inside of the hemisphere
// from the base, so a video or photo is immersive in front of them)
// Will duplicate the first vertex for UV purposes.
void AddXZVertexHalfRing(ProceduralMeshBuilder& builder, float3 center_point,
                         float radius, int sides) {
  const float angle_incr = M_PI / sides;
  for (int i = 0; i <= sides; ++i) {
    float angle = M_PI + i * angle_incr;
    float3 vertex_offset = {std::cos(angle) * radius, 0.0f,
                            std::sin(angle) * radius};
    float3 ring_vertex = center_point + vertex_offset;
    builder.AddVertex(ring_vertex);
  }
}

// Generates a ring of vertices around a center point, then adds them
// to a given vertices buffer. The ring is generated along the XZ plane.
// Will duplicate the first vertex for UV purposes.
// If flip_face_direction is true, the vertices will be wound facing inside the
// sphere.
void AddXZVertexRing(ProceduralMeshBuilder& builder, float3 center_point,
                     float radius, int sides, bool flip_face_direction) {
  float direction = flip_face_direction ? -1.0f : 1.0f;
  // If the user is meant to be facing inside the sphere, we would like to
  // position the UV "seam" (transition between 0 and 1) to be behind the user
  // at +Z. This positions UV 0.5 at -Z directly in "front" of the center.
  const float angle_offset = flip_face_direction ? 0.5f * M_PI : 0.0f;
  const float angle_incr = -2.0f * M_PI / sides * direction;
  for (int i = 0; i <= sides; ++i) {
    float angle = angle_offset + i * angle_incr;
    float3 vertex_offset = {std::cos(angle) * radius, 0.0f,
                            std::sin(angle) * radius};
    float3 ring_vertex = center_point + vertex_offset;
    builder.AddVertex(ring_vertex);
  }
}

// Sets the UVs for a strip of indices along the x axis of a UV map.
void SetUvsAlongXAxis(ProceduralMeshBuilder& builder, int start_index,
                      int end_index, float uv_y, float start_uv_x = 0.0f,
                      float end_uv_x = 1.0f) {
  int index_count = end_index - start_index;
  float uv_unit_x = end_uv_x - start_uv_x;
  for (int i = 0; i <= index_count; i++) {
    float uv_x = (uv_unit_x * i) / index_count + start_uv_x;
    builder.SetUV0(start_index + i, {uv_x, uv_y});
  }
}

// Produce new instances, which will be wrapped by a Mesh.
void FillVertexBuffer(BaseView& view, filament::Engine* engine,
                      const MeshDescription& mesh_description,
                      BaseVertexBufferBuilder& builder,
                      filament::VertexBuffer::BufferDescriptor&& buffer,
                      std::optional<absl::string_view> name) {
  auto& vertex_builder =
      builder.BufferCount(1).VertexCount(mesh_description.vertex_count);

  if (name.has_value()) {
    vertex_builder.Name(absl::StrFormat("%s_vb", *name));
  }

  const auto& vertex_format = mesh_description.vertex_format;
  for (size_t i = 0; i < vertex_format.GetNumAttributes(); ++i) {
    const auto& attribute = vertex_format.GetAttributeAt(i);
    vertex_builder.Attribute(attribute.attribute, 0, attribute.type,
                             vertex_format.GetAttributeOffsetAt(i),
                             vertex_format.GetVertexSize(),
                             attribute.normalized);
  }
  vertex_builder.BufferAt(*engine, 0, std::move(buffer));
}

void FillIndexBuffer(BaseView& view, filament::Engine* engine,
                     const MeshDescription& mesh_description,
                     BaseIndexBufferBuilder& builder,
                     filament::IndexBuffer::BufferDescriptor&& buffer,
                     std::optional<absl::string_view> name) {
  auto& index_builder = builder.IndexCount(mesh_description.index_count)
                            .BufferType(mesh_description.index_type);

  if (name.has_value()) {
    index_builder.Name(absl::StrFormat("%s_ib", *name));
  }

  index_builder.Buffer(*engine, std::move(buffer));
}

}  // namespace

MeshFactory::MeshFactory(BaseView& view) : view_(view) {}

MeshPtr MeshFactory::CreateBox(CreateBoxSettings settings,
                               MeshDataStorageMode data_mode) {
  float3 half_extent = settings.size * .5f;
  float3 center = settings.center;

  constexpr int number_of_quads = 6;
  constexpr int number_of_vertices = 4 * number_of_quads;
  constexpr int number_of_faces = 2 * number_of_quads;

  // Used to help set the four points along a plane's axes for the face.
  int2 point_transforms[4] = {{-1, 1}, {1, 1}, {-1, -1}, {1, -1}};

  struct BoxFace {
    // The uv coordinate by cell units.
    int2 uv_cell;
    // The two axes of the plane the face sits on.
    // For example, the top face sits on the xz plane.
    int3 plane_axes;
    // Whether the face plane is offset from the center in the positive or
    // negative direction.
    int offset_axis_valence;
    // The order of the point transforms to apply to this face.
    int4 point_transform_order;
  };

  BoxFace face_data[] = {
      {.uv_cell = {1, 0},
       .plane_axes = {1, 0, 1},
       .offset_axis_valence = 1,
       .point_transform_order = {2, 3, 0, 1}},
      {.uv_cell = {0, 1},
       .plane_axes = {0, 1, 1},
       .offset_axis_valence = -1,
       .point_transform_order = {3, 1, 2, 0}},
      {.uv_cell = {1, 1},
       .plane_axes = {1, 1, 0},
       .offset_axis_valence = 1,
       .point_transform_order = {0, 1, 2, 3}},
      {.uv_cell = {2, 1},
       .plane_axes = {0, 1, 1},
       .offset_axis_valence = 1,
       .point_transform_order = {1, 3, 0, 2}},
      {.uv_cell = {3, 1},
       .plane_axes = {1, 1, 0},
       .offset_axis_valence = -1,
       .point_transform_order = {1, 0, 3, 2}},
      {.uv_cell = {1, 2},
       .plane_axes = {1, 0, 1},
       .offset_axis_valence = -1,
       .point_transform_order = {0, 1, 2, 3}},
  };

  // We assume a cubemap with this layout will be used:
  //   . . . +-----+ . . . . . .  - 0
  //   .     |     |           .
  //   .     |     |           .
  //   +-----+-----+-----+-----+  - 1
  //   |     |     |     |     |
  //   |     |     |     |     |
  //   +-----+-----+-----+-----+  - 2
  //   .     |     |           .
  //   .     |     |           .
  //   . . . +-----+ . . . . . .  - 3   uv_unit.y = 1/3
  //   |     |     |     |     |
  //   0     1     2     3     4
  //                           uv_unit.x = 1/4

  // The percentage a cell in the uv_map takes up along both plane axes.
  float2 uv_unit = {1.0f / 4, 1.0f / 3};

  // Used to help set the uvs for each vertex.
  int2 uv_transforms[4] = {{0, 0}, {1, 0}, {0, 1}, {1, 1}};

  // Set up the indices.
  auto mesh_builder = std::make_unique<ProceduralMeshBuilder>(
      number_of_vertices, number_of_faces,
      settings.color.has_value() ? kVertexFormatWithColor : kVertexFormat,
      /*is_smooth*/ false, settings.color);

  // Draw each face.
  int vertex_count = 0;
  for (int i = 0; i < number_of_quads; i++) {
    int3 plane_axes = face_data[i].plane_axes;

    // Convert the offset value into a directional vector.
    int3 offset_axis =
        int3{plane_axes.x ^ 1, plane_axes.y ^ 1, plane_axes.z ^ 1} *
        face_data[i].offset_axis_valence;

    // Replace later with existing matrix diagonal call.
    int3 diagonal[3] = {
        {plane_axes.x, 0, 0}, {0, plane_axes.y, 0}, {0, 0, plane_axes.z}};

    // Save the vectors for the two axes
    // that make up the plane the face sits on.
    int3 plane_vectors[2];
    int saved_plane_vectors = 0;
    for (int j = 0; j < 3; j++) {
      if (plane_axes[j] != 0) {
        plane_vectors[saved_plane_vectors] = diagonal[j];
        saved_plane_vectors++;
      }
    }

    // Take the point transforms to start generating the four points
    // along the face plane.
    float3 plane_points[4];
    for (int j = 0; j < 4; j++) {
      int point_index = face_data[i].point_transform_order[j];
      plane_points[j].x =
          point_transforms[point_index][0] * plane_vectors[0].x +
          point_transforms[point_index][1] * plane_vectors[1].x;
      plane_points[j].y =
          point_transforms[point_index][0] * plane_vectors[0].y +
          point_transforms[point_index][1] * plane_vectors[1].y;
      plane_points[j].z =
          point_transforms[point_index][0] * plane_vectors[0].z +
          point_transforms[point_index][1] * plane_vectors[1].z;
    }

    // Calculate the final vertex position, and set the UV0.
    for (int j = 0; j < 4; j++) {
      mesh_builder->AddVertex(center +
                              half_extent * (offset_axis + plane_points[j]));
      float2 uv = (face_data[i].uv_cell + uv_transforms[j]) * uv_unit;
      mesh_builder->SetUV0(vertex_count + j, uv);
    }

    // Add a quad to draw between the set vertices.
    mesh_builder->AddQuad(vertex_count, vertex_count + 1, vertex_count + 2,
                          vertex_count + 3);
    vertex_count += 4;
  }

  Box bounds{.center = center, .halfExtent = half_extent};

  MeshPtr mesh =
      CreateByMovingMeshData(PrimitiveType::TRIANGLES, mesh_builder->Build(),
                             bounds, data_mode, settings.name);
  return mesh;
}

// Creates a hemisphere with poles at +/-Y. The flat plane is at Z=0.
// X goes from -1 to 1. Z goes from 0 to -1 to 0, and UV's are set so that a
// photo or video can be texture mapped to the inside of the hemisphere to
// surround the facing direction of a User.
// TODO: (broken link) - Consider merging this with CreateSphere.
MeshPtr MeshFactory::CreateXYHemisphere(CreateXYHemisphereSettings settings,
                                        MeshDataStorageMode data_mode) {
  float resolution = settings.resolution;
  if (resolution < kMinimumSphereResolution) {
    resolution = kMinimumSphereResolution;
  }
  float3 center = settings.center;
  float radius = settings.radius;

  // Calculate the number of vertices.
  int number_of_rings = resolution - 1;
  int vertices_per_ring = resolution + 1;
  int vertices_at_top_pole = resolution;
  int vertices_at_bottom_pole = resolution;
  int number_of_vertices = vertices_per_ring * number_of_rings +
                           vertices_at_top_pole + vertices_at_bottom_pole;

  float uv_unit = 1.0f / resolution;

  int number_of_latitudes = resolution;
  int number_of_longitudes = resolution;

  int number_of_faces = number_of_longitudes * (number_of_longitudes - 1) * 2;
  ProceduralMeshBuilder mesh_builder(number_of_vertices, number_of_faces,
                                     kVertexFormat);

  float3 top_pole_point = center + (kUp * radius);
  mesh_builder.AddVertex(top_pole_point, vertices_at_top_pole);

  // V should be 1 at the top pole.
  SetUvsAlongXAxis(mesh_builder, 0, vertices_at_top_pole - 1,
                   /*uv_y=*/1.0f, uv_unit / 2, 1.0f - uv_unit / 2);

  // Since the very top of the sphere is occupied by the top pole vertex,
  // we want to start from the slice right under it.
  int top_pole_index = 0;
  int topmost_ring_index = top_pole_index + 1;
  for (int i = topmost_ring_index; i < number_of_latitudes; i++) {
    float ring_slice_angle = M_PI * i / number_of_latitudes;
    float ring_y_pos = radius * std::cos(ring_slice_angle);
    float3 ring_center = center + (ring_y_pos * kUp);
    float ring_radius = radius * std::sin((ring_slice_angle));
    AddXZVertexHalfRing(mesh_builder, ring_center, ring_radius,
                        number_of_longitudes);
    int start_index = vertices_at_top_pole + (i - 1) * vertices_per_ring;
    int end_index = start_index + vertices_per_ring - 1;
    SetUvsAlongXAxis(mesh_builder, start_index, end_index,
                     1.0f - static_cast<float>(i) / number_of_latitudes);
  }
  float3 bottom_pt = center - (kUp * radius);
  mesh_builder.AddVertex(bottom_pt, vertices_at_bottom_pole);

  // V should be 0 at the bottom pole.
  SetUvsAlongXAxis(mesh_builder, number_of_vertices - vertices_at_bottom_pole,
                   number_of_vertices - 1, /*uv_y=*/0.0f, uv_unit / 2,
                   1.0f - uv_unit / 2);

  // We duplicate the center point of the top pole.
  // This way we can lay out the triangle fan flat on a UV map.
  // For example, the top cap of a sphere with a resolution of 4:
  // would be laid out like this:
  //
  //     0   1   2   3
  //     .   .   .   .
  //    / \ / \ / \ / \
  //   +---+---+---+---+
  //   4   5   6   7   8
  int top_cap_ring_start = number_of_longitudes;
  int top_cap_ring_end = top_cap_ring_start + number_of_longitudes;
  mesh_builder.AddTriangleFanWithDuplicateCenterVertices(
      0, top_cap_ring_start, top_cap_ring_end, false);

  // Make the body of the sphere out of strips.
  // For example, for a sphere with a resolution of 4:
  //   4---5---6---7---8
  //   |   |   |   |   |    Strip 1
  //   9---10--11--12--13
  //   |   |   |   |   |    Strip 2
  //   14--15--16--17--18
  int strip_offset = resolution;
  int number_of_strips = number_of_rings - 1;
  for (int i = 0; i < number_of_strips; i++) {
    int top_start_index = strip_offset + i * vertices_per_ring;
    int top_end_index = top_start_index + vertices_per_ring - 1;
    int bottom_start_index = top_end_index + 1;
    int bottom_end_index = bottom_start_index + vertices_per_ring - 1;
    mesh_builder.AddQuadStrip(top_start_index, top_end_index,
                              bottom_start_index, bottom_end_index, false);
  }

  // Same idea with the bottom cap of the sphere.
  // For a sphere with a resolution of 4, the UVs
  // would be laid out like this:
  //
  //   14  15  16  17  18
  //   +---+---+---+---+
  //    \ / \ / \ / \ /
  //     `   `   `   `
  //     19  20  21  22
  int bottom_cap_ring_start =
      number_of_vertices - vertices_at_bottom_pole - vertices_per_ring;
  int bottom_cap_ring_end = bottom_cap_ring_start + vertices_per_ring - 1;
  int bottom_cap_center_start = bottom_cap_ring_end + 1;
  mesh_builder.AddTriangleFanWithDuplicateCenterVertices(
      bottom_cap_center_start, bottom_cap_ring_start, bottom_cap_ring_end,
      false, true);

  // The actual center of the AABB of the hemipshere is a half-radius from the
  // center of the base towards the pole in -Z.
  float half_radius = radius * 0.5f;
  float3 aabb_center{center.x, center.y, center.z - half_radius};
  Box bounds{.center = aabb_center,
             .halfExtent = float3(radius, radius, half_radius)};

  MeshPtr mesh =
      CreateByMovingMeshData(PrimitiveType::TRIANGLES, mesh_builder.Build(),
                             bounds, data_mode, settings.name);
  return mesh;
}

MeshPtr MeshFactory::CreateSphere(CreateSphereSettings settings,
                                  MeshDataStorageMode data_mode) {
  float resolution = settings.resolution;
  if (resolution < kMinimumSphereResolution) {
    resolution = kMinimumSphereResolution;
  }
  float3 center = settings.center;
  float radius = settings.radius;

  // Calculate the number of vertices.
  int number_of_rings = resolution - 1;
  int vertices_per_ring = resolution + 1;
  int vertices_at_top_pole = resolution;
  int vertices_at_bottom_pole = resolution;
  int number_of_vertices = vertices_per_ring * number_of_rings +
                           vertices_at_top_pole + vertices_at_bottom_pole;

  float uv_unit = 1.0f / resolution;

  int number_of_latitudes = resolution;
  int number_of_longitudes = resolution;

  int number_of_faces = number_of_longitudes * (number_of_longitudes - 1) * 2;
  ProceduralMeshBuilder mesh_builder(
      number_of_vertices, number_of_faces,
      settings.color.has_value() ? kVertexFormatWithColor : kVertexFormat,
      /*is_smooth=*/true, settings.color);

  float3 top_pole_point = center + (kUp * radius);
  mesh_builder.AddVertex(top_pole_point, vertices_at_top_pole);

  float uv_y = settings.flip_uv ? 1.0f : 0.0f;
  SetUvsAlongXAxis(mesh_builder, 0, vertices_at_top_pole - 1, uv_y, uv_unit / 2,
                   1.0f - uv_unit / 2);

  // Since the very top of the sphere is occupied by the top pole vertex,
  // we want to start from the slice right under it.
  int top_pole_index = 0;
  int topmost_ring_index = top_pole_index + 1;
  for (int i = topmost_ring_index; i < number_of_latitudes; i++) {
    float ring_slice_angle = M_PI * i / number_of_latitudes;
    float ring_y_pos = radius * std::cos(ring_slice_angle);
    float3 ring_center = center + (ring_y_pos * kUp);
    float ring_radius = radius * std::sin((ring_slice_angle));
    AddXZVertexRing(mesh_builder, ring_center, ring_radius,
                    number_of_longitudes, settings.flip_face_direction);
    int start_index = vertices_at_top_pole + (i - 1) * vertices_per_ring;
    int end_index = start_index + vertices_per_ring - 1;

    float uv_y_displacement = (1.0f * i) / number_of_latitudes;
    uv_y = settings.flip_uv ? 1.0f - uv_y_displacement : uv_y_displacement;
    SetUvsAlongXAxis(mesh_builder, start_index, end_index, uv_y);
  }
  float3 bottom_pt = center - (kUp * radius);
  mesh_builder.AddVertex(bottom_pt, vertices_at_bottom_pole);
  uv_y = settings.flip_uv ? 0.0f : 1.0f;
  SetUvsAlongXAxis(mesh_builder, number_of_vertices - vertices_at_bottom_pole,
                   number_of_vertices - 1, uv_y, uv_unit / 2,
                   1.0f - uv_unit / 2);

  // We duplicate the center point of the top pole.
  // This way we can lay out the triangle fan flat on a UV map.
  // For example, the top cap of a sphere with a resolution of 4:
  // would be laid out like this:
  //
  //     0   1   2   3
  //     .   .   .   .
  //    / \ / \ / \ / \
  //   +---+---+---+---+
  //   4   5   6   7   8
  int top_cap_ring_start = number_of_longitudes;
  int top_cap_ring_end = top_cap_ring_start + number_of_longitudes;
  mesh_builder.AddTriangleFanWithDuplicateCenterVertices(
      0, top_cap_ring_start, top_cap_ring_end, false);

  // Make the body of the sphere out of strips.
  // For example, for a sphere with a resolution of 4:
  //   4---5---6---7---8
  //   |   |   |   |   |    Strip 1
  //   9---10--11--12--13
  //   |   |   |   |   |    Strip 2
  //   14--15--16--17--18
  int strip_offset = resolution;
  int number_of_strips = number_of_rings - 1;
  for (int i = 0; i < number_of_strips; i++) {
    int top_start_index = strip_offset + i * vertices_per_ring;
    int top_end_index = top_start_index + vertices_per_ring - 1;
    int bottom_start_index = top_end_index + 1;
    int bottom_end_index = bottom_start_index + vertices_per_ring - 1;
    mesh_builder.AddQuadStrip(top_start_index, top_end_index,
                              bottom_start_index, bottom_end_index, false);
  }

  // Same idea with the bottom cap of the sphere.
  // For a sphere with a resolution of 4, the UVs
  // would be laid out like this:
  //
  //   14  15  16  17  18
  //   +---+---+---+---+
  //    \ / \ / \ / \ /
  //     `   `   `   `
  //     19  20  21  22
  int bottom_cap_ring_start =
      number_of_vertices - vertices_at_bottom_pole - vertices_per_ring;
  int bottom_cap_ring_end = bottom_cap_ring_start + vertices_per_ring - 1;
  int bottom_cap_center_start = bottom_cap_ring_end + 1;
  mesh_builder.AddTriangleFanWithDuplicateCenterVertices(
      bottom_cap_center_start, bottom_cap_ring_start, bottom_cap_ring_end,
      false, true);

  Box bounds{.center = center, .halfExtent = float3(radius)};

  MeshPtr mesh =
      CreateByMovingMeshData(PrimitiveType::TRIANGLES, mesh_builder.Build(),
                             bounds, data_mode, settings.name);
  return mesh;
}

MeshPtr MeshFactory::CreateCylinder(CreateCylinderSettings settings,
                                    MeshDataStorageMode data_mode) {
  float resolution = settings.resolution;
  if (resolution < kMinimumCylinderResolution) {
    resolution = kMinimumCylinderResolution;
  }
  // Resolution faces of quads (2 faces each) on the sides, plus two caps.
  int number_of_faces = resolution * 4;

  float height = settings.height;

  // Create two rings for the top and bottom caps to separate the indices in UV
  // space.
  int number_of_rings = 4;
  int vertices_per_ring = resolution + 1;
  int vertices_at_top_pole = resolution;
  int vertices_at_bottom_pole = resolution;
  int number_of_vertices = vertices_per_ring * number_of_rings +
                           vertices_at_top_pole + vertices_at_bottom_pole;

  int top_pole_start_index = 0;
  int top_cap_ring_start_index = top_pole_start_index + vertices_at_top_pole;
  int top_side_ring_start_index = top_cap_ring_start_index + vertices_per_ring;
  int bottom_side_ring_start_index =
      top_side_ring_start_index + vertices_per_ring;
  int bottom_cap_ring_start_index =
      bottom_side_ring_start_index + vertices_per_ring;
  int bottom_pole_start_index = bottom_cap_ring_start_index + vertices_per_ring;

  ProceduralMeshBuilder mesh_builder(
      number_of_vertices, number_of_faces,
      settings.color.has_value() ? kVertexFormatWithColor : kVertexFormat,
      /*is_smooth*/ false, settings.color);

  float3 center = settings.center;
  float3 top_pole_point = center + (kUp * height);
  float3 bottom_pole_point = center;

  // --- Add Vertices ---

  // Top Cap Center Vertices
  mesh_builder.AddVertex(top_pole_point, vertices_at_top_pole);

  // Add ring Vertices. The first one is for the cap, and the second one for the
  // cylinder body.

  // Top Ring Vertices
  AddXZVertexRing(mesh_builder, top_pole_point, settings.radius, resolution,
                  false);
  AddXZVertexRing(mesh_builder, top_pole_point, settings.radius, resolution,
                  false);

  // Bottom Ring Vertices
  AddXZVertexRing(mesh_builder, bottom_pole_point, settings.radius, resolution,
                  false);
  AddXZVertexRing(mesh_builder, bottom_pole_point, settings.radius, resolution,
                  false);

  // Bottom Cap Center Vertices
  mesh_builder.AddVertex(bottom_pole_point, vertices_at_bottom_pole);

  // --- Set UVs ---
  // The UVs are set up so the top cap and bottom cap take up the left half of
  // the UV space, and the cylinder body takes up the right half.

  //   U=0.0           U=0.5           U=1.0
  // V=1.0 +-------------------+-------------------+
  //       |      .-------.    |    |    |    |    |
  //       |     /         \   |    |    |    |    |
  //       |    |           |  |    |    |    |    |
  //       |     \         /   |    |    |    |    |
  //       |      '-------'    |    |    |    |    |
  // V=0.5 +-------------------+-------------------+
  //       |      .-------.    |    |    |    |    |
  //       |     /         \   |    |    |    |    |
  //       |    |           |  |    |    |    |    |
  //       |     \         /   |    |    |    |    |
  //       |      '-------'    |    |    |    |    |
  // V=0.0 +-------------------+-------------------+
  //         <---- Caps ---->      <--- Sides --->

  float uv_unit_angle = 1.0f / resolution;
  float cap_u_center = 0.25f;
  float cap_v_top_center = 0.75f;
  float cap_v_bottom_center = 0.25f;
  if (settings.flip_uv) {
    cap_v_top_center = 0.25f;
    cap_v_bottom_center = 0.75f;
  }
  float cap_radius_uv = 0.25f;

  // Top Pole UVs
  for (int i = 0; i < vertices_at_top_pole; ++i) {
    mesh_builder.SetUV0(top_pole_start_index + i,
                        {cap_u_center, cap_v_top_center});
  }

  // Bottom Pole UVs
  for (int i = 0; i < vertices_at_bottom_pole; ++i) {
    mesh_builder.SetUV0(bottom_pole_start_index + i,
                        {cap_u_center, cap_v_bottom_center});
  }

  // Top Cap Ring UVs
  for (int i = 0; i < vertices_per_ring; ++i) {
    float angle = static_cast<float>(i) * uv_unit_angle * 2.0f * M_PI;
    float uv_x = cap_u_center + cap_radius_uv * std::cos(angle);
    float uv_y = cap_v_top_center + cap_radius_uv * std::sin(angle);
    mesh_builder.SetUV0(top_cap_ring_start_index + i, {uv_x, uv_y});
  }

  // Bottom Cap Ring UVs
  for (int i = 0; i < vertices_per_ring; ++i) {
    float angle = static_cast<float>(i) * uv_unit_angle * 2.0f * M_PI;
    float uv_x = cap_u_center + cap_radius_uv * std::cos(angle);
    float uv_y = cap_v_bottom_center + cap_radius_uv * std::sin(angle);
    mesh_builder.SetUV0(bottom_cap_ring_start_index + i, {uv_x, uv_y});
  }

  // Side Ring UVs
  float side_u_start = 0.5f;
  float side_u_end = 1.0f;
  float side_v_top = 1.0f;
  float side_v_bottom = 0.0f;
  if (settings.flip_uv) {
    side_v_top = 0.0f;
    side_v_bottom = 1.0f;
  }

  // Top Side Ring UVs
  for (int i = 0; i <= resolution; ++i) {
    float u = side_u_start + (side_u_end - side_u_start) * i / resolution;
    mesh_builder.SetUV0(top_side_ring_start_index + i, {u, side_v_top});
  }

  // Bottom Side Ring UVs
  for (int i = 0; i <= resolution; ++i) {
    float u = side_u_start + (side_u_end - side_u_start) * i / resolution;
    mesh_builder.SetUV0(bottom_side_ring_start_index + i, {u, side_v_bottom});
  }

  // --- Add Indices ---

  // Add the triangles for the top cap.
  mesh_builder.AddTriangleFanWithDuplicateCenterVertices(
      top_pole_start_index, top_cap_ring_start_index,
      top_cap_ring_start_index + resolution, false, false);

  // Add the quad strips to the sides of the cylinder.
  for (int i = 0; i < resolution; ++i) {
    uint16_t top_left_index = top_side_ring_start_index + i;
    uint16_t top_right_index = top_side_ring_start_index + i + 1;
    uint16_t bottom_left_index = bottom_side_ring_start_index + i;
    uint16_t bottom_right_index = bottom_side_ring_start_index + i + 1;

    mesh_builder.AddQuad(top_left_index, top_right_index, bottom_left_index,
                         bottom_right_index, false);
  }

  // Add the triangles for the bottom cap.
  mesh_builder.AddTriangleFanWithDuplicateCenterVertices(
      bottom_pole_start_index, bottom_cap_ring_start_index,
      bottom_cap_ring_start_index + resolution, false, true);

  float3 bounds_center = center + (kUp * height / 2.0f);
  Box bounds{.center = bounds_center,
             .halfExtent = {settings.radius, height / 2.0f, settings.radius}};

  MeshPtr mesh = CreateByMovingMeshData(
      PrimitiveType::TRIANGLES, mesh_builder.Build(), bounds, data_mode);

  return mesh;
}

MeshPtr MeshFactory::CreateCapsule(CreateCapsuleSettings settings,
                                   MeshDataStorageMode data_mode) {
  float resolution = settings.resolution;
  if (resolution < kMinimumCapsuleResolution) {
    resolution = kMinimumCapsuleResolution;
  }

  float diameter = 2.0f * settings.radius;
  int number_of_faces = resolution * (resolution - 1) * 2;
  float3 center = settings.center;
  float height = settings.height;

  // Cap the height so it can never be smaller than the diameter so that it
  // preserves the definition of a capsule.
  if (height < diameter) {
    height = diameter;
  }

  // Generate a sphere if the diameter equals the height.
  if (diameter == height) {
    CreateSphereSettings sphere_settings;
    sphere_settings.radius = settings.radius;
    sphere_settings.center = settings.center;
    sphere_settings.resolution = resolution;
    sphere_settings.flip_uv = settings.flip_uv;
    MeshPtr new_mesh = CreateSphere(sphere_settings, data_mode);
    return new_mesh;
  }

  // Calculate the number of vertices.
  int number_of_rings = resolution - 1;
  int vertices_per_ring = resolution + 1;
  int vertices_at_top_pole = resolution;
  int vertices_at_bottom_pole = resolution;
  int number_of_vertices = vertices_per_ring * number_of_rings +
                           vertices_at_top_pole + vertices_at_bottom_pole;

  // We make the capsule in three parts:
  // Upper hemisphere
  // Center cylinder
  // Bottom hemisphere
  //
  // We make the capsule from top to bottom, based on its 'resolution'.
  // Below is a capsule with a resolution of 6.
  // On the left is the ring number.
  // On the right is the part of the capsule the ring belongs in.
  //
  // 0 __
  //    T           , - ~ ~ ~ - ,    -------------+
  //    |       , '               ' ,             |
  // 1 _L     ,  _ _ ring_radius _ _  ,           |_ A. Upper Hemisphere
  //    T    ,                         ,          |
  //    |   ,                           ,         |
  // 2 _L   .___________________________.   ------+
  //    T   .---------------------------.   ------+
  //    |   |                           |         |
  // 3 _L   |  _ _ _ _ radius _ _ _ _ _ |         |_ B. Center Cylinder
  //    T   |                           |         |
  //    |   |                           |         |
  // 4 _L   .---------------------------.   ------+
  //    T   ,---------------------------,   ------+
  //    |   ,                           ,         |
  // 5 _L    , _ _ _ ring_radius _ _ _ ,          |_ C. Bottom Hemisphere
  //    T     ,                       ,           |
  //    |       ,                   ,'            |
  // 6 _L         ' - , _ _ _ , - '    -----------+
  //
  // 'ring_radius' is the radius of a ring for the Upper or Bottom Hemisphere.
  // 'radius' is the radius passed into the function.
  float segment_height = height / resolution;

  float hemisphere_resolution = std::floor(settings.radius / segment_height);
  float sphere_resolution = hemisphere_resolution * 2.0f;
  float cylinder_resolution = resolution - sphere_resolution;

  float top_ring_index__cylinder = hemisphere_resolution;
  float bottom_ring_index__cylinder = resolution - hemisphere_resolution;

  float half_cylinder_height =
      segment_height *
      (bottom_ring_index__cylinder - top_ring_index__cylinder) / 2.0f;

  // Since the very top of the cylinder is occupied by the top pole vertex,
  // we want to start from the slice right under it.
  int top_pole_index = 0;
  int top_ring_index = top_pole_index + 1;

  // Start writing to the index buffer.
  ProceduralMeshBuilder mesh_builder(
      number_of_vertices, number_of_faces,
      settings.color.has_value() ? kVertexFormatWithColor : kVertexFormat,
      /*is_smooth*/ true, settings.color);

  // Add the top point to the vertices.
  float3 top_pole_point = center + (kUp * height / 2.0f);
  mesh_builder.AddVertex(top_pole_point, vertices_at_top_pole);
  float uv_unit = 1.0f / resolution;
  SetUvsAlongXAxis(mesh_builder, 0, vertices_at_top_pole - 1, 0, uv_unit / 2,
                   1.0f - uv_unit / 2);

  // Calculate the vertex rings of the cylinder, based on Fig. 2 above
  for (int i = top_ring_index; i < resolution; i++) {
    // B. Center Cylinder:
    // Calculates vertex rings for the center cylinder.
    float cylinder_i = i - hemisphere_resolution;
    float lateral_angle = M_PI * cylinder_i / cylinder_resolution;
    float ring_position_y = half_cylinder_height * std::cos(lateral_angle);
    float3 ring_center = center + (ring_position_y * kUp);
    float ring_radius = settings.radius;

    // A. Upper Hemisphere:
    // Calculates vertex rings for the top cap of the capsule.
    if (i < top_ring_index__cylinder) {
      float cap_sphere_i = i;
      float cap_sphere_angle = M_PI * (cap_sphere_i / sphere_resolution);
      float cap_sphere_position_y =
          hemisphere_resolution * segment_height * std::cos(cap_sphere_angle);
      ring_center =
          center + ((cap_sphere_position_y + half_cylinder_height) * kUp);
      ring_radius = settings.radius * std::sin(cap_sphere_angle);

      // C. Bottom Hemisphere:
      // Calculates vertex rings for the bottom cap of the capsule.
    } else if (i > bottom_ring_index__cylinder) {
      float cap_sphere_i =
          i - bottom_ring_index__cylinder + top_ring_index__cylinder;
      float cap_sphere_angle = M_PI * (cap_sphere_i / sphere_resolution);
      float cap_sphere_position_y =
          hemisphere_resolution * segment_height * std::cos(cap_sphere_angle);
      ring_center =
          center - ((half_cylinder_height - cap_sphere_position_y) * kUp);
      ring_radius = settings.radius * std::sin(cap_sphere_angle);
    }
    // Push the vertex ring to our vertices buffer.
    AddXZVertexRing(mesh_builder, ring_center, ring_radius, resolution, false);
    int uv_start = vertices_at_top_pole + (i - 1) * vertices_per_ring;
    int uv_end = uv_start + vertices_per_ring - 1;
    float uv_y = ((height / 2.0f) - ring_center.y) / height;
    SetUvsAlongXAxis(mesh_builder, uv_start, uv_end, uv_y);
  }
  // Add the bottom point to our vertices buffer.
  float3 bottom_pole_point = center - (kUp * height / 2.0f);
  mesh_builder.AddVertex(bottom_pole_point, vertices_at_bottom_pole);
  SetUvsAlongXAxis(mesh_builder, number_of_vertices - vertices_at_bottom_pole,
                   number_of_vertices - 1, 1, uv_unit / 2, 1.0f - uv_unit / 2);

  // Add a triangle fan around the top pole of the capsule.
  int number_of_sides = resolution;
  int top_cap_ring_start = number_of_sides;
  int top_cap_ring_end = top_cap_ring_start + number_of_sides;
  mesh_builder.AddTriangleFanWithDuplicateCenterVertices(
      0, top_cap_ring_start, top_cap_ring_end, false);
  // Add a quad strip between every ring of vertices.
  int strip_offset = resolution;
  int number_of_strips = number_of_rings - 1;
  for (int i = 0; i < number_of_strips; i++) {
    int top_start_index = strip_offset + i * vertices_per_ring;
    int top_end_index = top_start_index + vertices_per_ring - 1;
    int bottom_start_index = top_end_index + 1;
    int bottom_end_index = bottom_start_index + vertices_per_ring - 1;
    mesh_builder.AddQuadStrip(top_start_index, top_end_index,
                              bottom_start_index, bottom_end_index, false);
  }
  // Add a triangle fan around the bottom pole of the capsule.
  int bottom_cap_ring_start =
      number_of_vertices - vertices_at_bottom_pole - vertices_per_ring;
  int bottom_cap_ring_end = bottom_cap_ring_start + vertices_per_ring - 1;
  int bottom_cap_center_start = bottom_cap_ring_end + 1;
  mesh_builder.AddTriangleFanWithDuplicateCenterVertices(
      bottom_cap_center_start, bottom_cap_ring_start, bottom_cap_ring_end,
      false, true);

  Box bounds{.center = center,
             .halfExtent = {settings.radius, height / 2.0f, settings.radius}};

  MeshPtr mesh =
      CreateByMovingMeshData(PrimitiveType::TRIANGLES, mesh_builder.Build(),
                             bounds, data_mode, settings.name);
  return mesh;
}

MeshPtr MeshFactory::CreateCone(CreateConeSettings settings,
                                MeshDataStorageMode data_mode) {
  float resolution = settings.resolution;
  if (resolution < kMinimumConeResolution) {
    resolution = kMinimumConeResolution;
  }

  int number_of_faces = resolution * 2;

  float height = settings.height;
  float radius = settings.radius;

  // Calculate the number of vertices.

  // Use two separate rings for the cone and the base.
  int number_of_rings = 2;
  int vertices_per_ring = resolution + 1;
  int vertices_at_apex = resolution;
  int vertices_at_bottom_pole = resolution;
  int number_of_vertices = vertices_per_ring * number_of_rings +
                           vertices_at_apex + vertices_at_bottom_pole;

  // Since the very top of the cone is occupied by the apex vertex,
  // we have the apex and then the base ring.
  int apex_start_index = 0;
  int bottom_cone_ring_index = apex_start_index + vertices_at_apex;
  int bottom_base_ring_index = bottom_cone_ring_index + vertices_per_ring;
  int bottom_pole_index = bottom_base_ring_index + vertices_per_ring;

  // Start writing to the index buffer.
  ProceduralMeshBuilder mesh_builder(
      number_of_vertices, number_of_faces,
      settings.color.has_value() ? kVertexFormatWithColor : kVertexFormat,
      /*is_smooth=*/false, settings.color);

  float3 center = settings.center;

  // Add the apex of the cone.
  float3 apex_point = center + (kUp * height);
  float3 bottom_center_point = center;

  // --- Add Vertices ---

  // Add the apex of the cone.
  mesh_builder.AddVertex(apex_point, vertices_at_apex);

  // Add two rings for the cone and the base.
  AddXZVertexRing(mesh_builder, bottom_center_point, radius, resolution, false);
  AddXZVertexRing(mesh_builder, bottom_center_point, radius, resolution, false);

  // Add the bottom pole
  mesh_builder.AddVertex(bottom_center_point, vertices_at_bottom_pole);

  // --- Set UVs ---
  // The UVs are set up so the base takes up the top left quadrant, and the fan
  // takes up the bottom half, with the apex in the middle, rotating clockwise.
  // The apex angle can change based on the radius and height of the cone.

  float uv_unit_angle = 1.0f / resolution;

  float base_u_center = 0.25f;
  float base_v_center = 0.25f;
  if (settings.flip_uv) {
    base_v_center = 1.0f - base_v_center;
  }
  float base_radius_uv = 0.25f;
  float cone_u_center = 0.5f;
  float cone_v_center = 0.5f;
  float cone_radius_uv = 0.5f;

  // Calculate the apex angle based on the radius and height of the cone.
  float apex_angle = 2 * std::atan2(radius, height);

  // Base Pole UVs
  for (int i = 0; i < vertices_at_bottom_pole; ++i) {
    mesh_builder.SetUV0(bottom_pole_index + i, {base_u_center, base_v_center});
  }

  // Base Ring UVs
  for (int i = 0; i < vertices_per_ring; ++i) {
    float angle = static_cast<float>(i) * uv_unit_angle * 2.0f * M_PI;
    float uv_x = base_u_center + base_radius_uv * std::cos(angle);
    float uv_y = base_v_center + base_radius_uv * std::sin(angle);
    mesh_builder.SetUV0(bottom_base_ring_index + i, {uv_x, uv_y});
  }

  // Apex UVs
  for (int i = 0; i < vertices_at_apex; ++i) {
    mesh_builder.SetUV0(apex_start_index + i, {cone_u_center, cone_v_center});
  }

  // Cone Ring UVs
  for (int i = 0; i < vertices_per_ring; ++i) {
    // Calculate the angle based on the apex angle instead of a full circle.
    float angle = static_cast<float>(i) * uv_unit_angle * apex_angle;
    float uv_x = cone_u_center + cone_radius_uv * std::cos(angle);
    float uv_y = cone_v_center - cone_radius_uv * std::sin(angle);
    if (settings.flip_uv) {
      uv_y = 1.0f - uv_y;
    }
    mesh_builder.SetUV0(bottom_cone_ring_index + i, {uv_x, uv_y});
  }

  // --- Set indices ---
  // Add the triangles for the cone side.
  mesh_builder.AddTriangleFanWithDuplicateCenterVertices(
      apex_start_index, bottom_cone_ring_index,
      bottom_cone_ring_index + resolution, false, false);

  // Add the triangles for the base.
  mesh_builder.AddTriangleFanWithDuplicateCenterVertices(
      bottom_pole_index, bottom_base_ring_index,
      bottom_base_ring_index + resolution, false, true);

  float3 bounds_center = center + (kUp * height / 2.0f);
  Box bounds{.center = bounds_center,
             .halfExtent = {radius, height / 2.0f, radius}};

  MeshPtr mesh =
      CreateByMovingMeshData(PrimitiveType::TRIANGLES, mesh_builder.Build(),
                             bounds, data_mode, settings.name);

  return mesh;
}

MeshPtr MeshFactory::CreateQuad(CreateQuadSettings settings,
                                MeshDataStorageMode data_mode) {
  const VertexFormat format =
      settings.color ? kVertexFormatWithColor : kVertexFormat;
  auto mesh_data = std::make_unique<MeshData>(
      MeshDescription{format, MeshDescription::IndexType::USHORT,
                      SizeOfArray(kQuadPositions), SizeOfArray(kQuadIndices)});

  float2 scalerExtent(kQuadHalfExtent, kQuadHalfExtent);
  UVScaler scaler{settings.flip_uv,
                  (-scalerExtent * settings.size) + settings.center,
                  (scalerExtent * settings.size) + settings.center};
  const quatf quad_packed_tangent_frame =
      mat3f::packTangentFrame({kRight, kUp, kBack});
  for (int i = 0; i < mesh_data->GetDescription().vertex_count; ++i) {
    float3 vertex = {(kQuadPositions[i].xy * settings.size) + settings.center,
                     settings.z};
    mesh_data->VertexAttributeAt<float3>(i, VertexAttribute::POSITION) = vertex;
    mesh_data->VertexAttributeAt<float2>(i, VertexAttribute::UV0) =
        scaler.UVFromPos(vertex);
    mesh_data->VertexAttributeAt<quatf>(i, VertexAttribute::TANGENTS) =
        quad_packed_tangent_frame;
    if (settings.color) {
      mesh_data->VertexAttributeAt<float4>(i, VertexAttribute::COLOR) =
          *settings.color;
    }
  }

  for (int i = 0; i < mesh_data->GetDescription().index_count; ++i) {
    mesh_data->IndexAt<uint16_t>(i) = kQuadIndices[i];
  }

  Box box = kRadius1Aabb;
  box.halfExtent.xy *= settings.size;
  box.center.z = settings.z;
  box.center.xy = settings.center;
  return CreateByMovingMeshData(PrimitiveType::TRIANGLES, std::move(mesh_data),
                                box, data_mode, settings.name);
}

// TODO: Move all this panel-generation code into separate files.

// Returns the vertex for a panel curved around a vertical cylinder of radius.
// Scale is used to stretch the panel without changing the radius.
float3 GetCylindricalPanelPointFromUV(float2 uv, float radius, float3 scale,
                                      float2 size) {
  float t = (uv.x - 0.5) * size.x * scale.x / radius + M_PI / 2.0;
  float2 v_x(-radius * cos(t), -radius * (sin(t) - 1.0f));
  float v_y = (uv.y - 0.5) * size.y * scale.y;
  return {v_x.x / scale.x, v_y / scale.y, v_x.y / scale.z};
}

// Returns the vertex position for a panel with the given settings at uv.
float3 CalculatePanelVertexPosition(CreateQuadSettings settings, float2 uv,
                                    float3 scale, bool use_size = true) {
  float3 vertex;
  if (settings.radius.has_value() && *settings.radius > 0) {
    float2 size = use_size ? settings.size : float2(1.0f, 1.0f);
    vertex = GetCylindricalPanelPointFromUV(uv, *settings.radius, scale, size);
  } else {
    vertex = {-kQuadHalfExtent + uv.x, -kQuadHalfExtent + uv.y, 0};
    if (use_size) {
      vertex.x *= settings.size.x;
      vertex.y *= settings.size.y;
    }
  }
  vertex += float3(settings.center.x, settings.center.y, settings.z);
  return vertex;
}

// Returns the default corner radius if none is specified in settings.
uint32_t GetCornerResolution(CreateQuadSettings settings) {
  if (!settings.corner_resolution.has_value() ||
      *settings.corner_resolution == 0) {
    return kDefaultRoundedCornerResolution;
  }
  return *settings.corner_resolution;
}

// Holds the data for a rounded corner for the CreatePanel function.
// The corner is generated by sweeping along a circle from arc_begin -> arc_end.
//
// There will be n + 1 triangles, where n is the number of vertices specified
// by settings.corner_resolution.
//
//   arc_end --> 2PI
//  .---.---.---. .   angle
//  |   |   |   |    /.
//  .---.---.---.   /   .
//  |   |   |   |crux    0 <-- arc_begin
//  .---.---.---.---.---.
//  |   |   |   |   |   |
//  .---.---.---.---.---.
//  |   |   |   |   |   |
//  .---.---.---.---.---.
//  |   |   |   |   |   |
//  .---.---.---.---.---.
//
struct CurvedCorner {
  enum class Corner { kBottomLeft, kBottomRight, kTopRight, kTopLeft };
  // The beginning and ends of the arc of the corner, in radians.
  float arc_begin;
  float arc_end;
  // The uv of the crux of the corner, i.e. the pointy part towards the center.
  float2 uv_crux;
  // The number of bands to render on the edges
  uint2 edge_resolution;

  // Sweeps a vertex circle of radius settings.corner_radius across the given
  // arc and add to the crux vertex to get a consistent corner radius in meters.
  // Adds a triangle strip where each triangle shares the crux vertex.
  int32_t GenerateCorner(const CreateQuadSettings& settings,
                         ProceduralMeshBuilder& builder, AabbCalculator& aabb,
                         float3 scale = kOne3) {
    float3 crux_vertex = CalculatePanelVertexPosition(settings, uv_crux, scale);
    uint32_t crux_index = builder.AddVertex(crux_vertex);
    builder.SetUV0(crux_index, uv_crux);
    aabb.AddVertex(crux_vertex);
    uint32_t last_index = crux_index;

    // TODO: need to render bands so the curve matches, where each
    // side has a number of verts appropriate in length to closely match
    // resolution but is each swept in a curve and then completed in triangle
    // strips. See diagrams 1a and 1b.
    // Move along the nearer strip and continue with the current "crux" vertex
    // until it is further away from the next vertex than the next
    // nearer-strip-vertex. When you get to the last one, just keep using it.
    // This should work since there will always be at least as many verts in the
    // next shell.

    uint32_t corner_start = last_index + 1;
    uint32_t sides = GetCornerResolution(settings);
    const float angle_incr = (arc_end - arc_begin) / static_cast<float>(sides);
    for (int i = 0; i <= sides; i++) {
      float angle = arc_begin + i * angle_incr;
      // Note: the arc needs settings.size divided out because it's multiplied
      // back in inside CalculatePanelVertexPosition and we *do* need the size
      // factor of the uv_crux component of the uv to get us to the right corner
      // location. We don't want to include the size in the radius part of the
      // calculation because it's important to keep the radius absolute in
      // meters.
      float2 uv(
          uv_crux.x + cos(angle) * *settings.corner_radius / settings.size.x,
          uv_crux.y + sin(angle) * *settings.corner_radius / settings.size.y);

      float3 vertex = CalculatePanelVertexPosition(settings, uv, scale);
      last_index = builder.AddVertex(vertex);
      builder.SetUV0(last_index, uv);
      aabb.AddVertex(vertex);
    }
    // Need to include the old vertices.
    builder.AddTriangleFan(crux_index, corner_start, last_index, false);
    return last_index;
  }
};

// Stores the index in the vertex buffer of all four corners of a quad.
// The quad is drawn with two triangles:
//   top_left    top_right
//     |-------------|
//     |  \          |
//     |    \        |
//     |      \      |
//     |        \    |
//     |          \  |
//     |_____________|
//   bottom_left   bottom_right]

struct QuadIndices {
  uint32_t bottom_left;
  uint32_t top_left;
  uint32_t bottom_right;
  uint32_t top_right;

  QuadIndices(uint32_t bl, uint32_t tl, uint32_t br, uint32_t tr)
      : bottom_left(bl), top_left(tl), bottom_right(br), top_right(tr) {}
};

// Holds the data for a vertex on a flat or curved panel.
// if quad_indices is nullopt, this vertex is on the right/bottom edge and is
// already included in other quads.
struct PanelVertex {
  float2 uv;
  uint32_t index;
  std::optional<QuadIndices> quad_indices;
};

// Interface for generating PanelVertex objects at each point in a panel mesh.
// Iterate over this provider to get the list of all vertices and quads.
class PanelVertexProvider {
 public:
  virtual ~PanelVertexProvider() = default;

  class iterator {
   public:
    iterator(PanelVertexProvider* panel_uv_provider)
        : self(panel_uv_provider) {}

    iterator operator++() {
      if (!++(*self)) self = nullptr;
      return *this;
    }

    bool operator!=(const iterator& other) const { return self != other.self; }

    PanelVertex operator*() const { return self->GetCurrent(); }

   private:
    PanelVertexProvider* self;
  };

  iterator begin() {
    Reset();
    return iterator(this);
  }
  iterator end() { return iterator(nullptr); }

  virtual uint32_t GetVertexCount() const = 0;
  virtual uint32_t GetTriangleCount() const = 0;

 protected:
  // Resets the provider to the beginning of the vertices to begin iterating.
  virtual void Reset() = 0;

  // Moves to the next vertex in the panel. Returns true if the current vertex
  // is valid, false if the increment puts us beyond the range of the panel.
  virtual bool operator++() = 0;

  // Gets the PanelVertex at the current iteration point.
  // This is returned by iterator::operator* dereference as the loop data.
  virtual PanelVertex GetCurrent() const = 0;
};

// Generates a simple, flat panel of the resolution x resolution vertices.
//
// IMPORTANT: resolution refers to the number of divisions, i.e. vertices. The
// number of intervals is naturally one fewer than that, which creates many
// tricky discrepancies at the boundary conditions.
//
// A panel of resolution 3x3 will look like this w/ 9 vertices, 8 triangles:
//
// [0, 2]      [2,2]
//   . --- . --- .
//   |  \  |  \  |
//   . --- . --- .
//   |  \  |  \  |
//   . --- . --- .
// [0,0]       [2,0]
//
class SimplePanelVertexProvider : public PanelVertexProvider {
 public:
  SimplePanelVertexProvider(uint2 resolution)
      : resolution_(resolution),
        cursor_(kZero2),
        increment_(1 / static_cast<float>(resolution.x - 1),
                   1 / static_cast<float>(resolution.y - 1)) {}

  uint32_t GetVertexCount() const override {
    return resolution_.x * resolution_.y;
  }
  uint32_t GetTriangleCount() const override {
    return (resolution_.x - 1) * (resolution_.y - 1) * 2;
  }

 protected:
  void Reset() override { cursor_ = kZero2; }

  bool operator++() override {
    if (cursor_.y < resolution_.y - 1) {
      cursor_.y++;
    } else {
      cursor_.y = 0;
      cursor_.x++;
    }
    return cursor_.x < resolution_.x;
  }

  PanelVertex GetCurrent() const override {
    uint32_t current_index = GetCurrentIndex();
    return {.uv = {cursor_.x * increment_.x, cursor_.y * increment_.y},
            .index = current_index,
            .quad_indices = GetCurrentQuadIndices()};
  }

  uint2 GetResolution() const { return resolution_; }

  uint2 GetCursor() const { return cursor_; }

  float2 GetIncrement() const { return increment_; }

  virtual uint32_t GetCurrentIndex() const {
    return cursor_.x * resolution_.y + cursor_.y;
  }

  virtual std::optional<QuadIndices> GetCurrentQuadIndices() const {
    if (cursor_.x < resolution_.x - 1 && cursor_.y < resolution_.y - 1) {
      uint32_t same_y_on_next_column = GetCurrentIndex() + resolution_.y;
      return QuadIndices(GetCurrentIndex(), GetCurrentIndex() + 1,
                         same_y_on_next_column, same_y_on_next_column + 1);
    }
    return std::nullopt;
  }

 private:
  uint2 resolution_;
  uint2 cursor_;
  float2 increment_;
};

// Generates a panel skipping corners so they may be filled in with curved arcs.
//
// This is similar to SimplePanelVertexProvider, but skips the quads at all
// four corners by skipping a certain number of vertices along each axis.
// The plan is that we can replace these corners with different styles, such as
// a bevel or a curve.
//
// A panel of resolution 5x5 with 1x1 skip vertices will look like this
// w/ 32 vertices, 42 triangles:
//
// [0,5]                         [5,5]
//   .    9. --15. --21. --27.     .
//         |  \  |  \  |  \  |
//  3. ---8. --14. --20. --26. --- .31
//   |  \  |  \  |  \  |  \  |  \  |
//  2. ---7. --13. --19. --25. --- .30
//   |  \  |  \  |  \  |  \  |  \  |
//  1. ---6. --12. --18. --24. --- .29
//   |  \  |  \  |  \  |  \  |  \  |
//  0. ---5. --11. --17. --23. --- .28
//         |  \  |  \  |  \  |
//   .    4. --10. --16. --22.     .
// [0,0]                         [5,0]
//
// Note: the above diagram is super helpful for debugging. All quads are of the
// form  BL---TL
//       |  /  |
//       BR---TR
//
// The first two quads for this 5x5 corner-skip with 1x1 are:
//
//  [0, 1, 5]
//  [5, 1, 6]
//
//  [1, 2, 6]
//  [6, 2, 7]
//  ...
//
// Not all vertices are in the same number of quads and there is a different
// pattern of quads skipped vs vertices skipped. Important areas are vertex
// 3, vertex 9, and vertex 27.
// Vertex 9, for example, is not the start of a quad group. The prior quad is
//
// [8,  9, 14]
// [14, 9, 15]
//
// The cursor is now [x,y] = [1,5], which is vertex 9. Internally, the method
// ShouldSkipVertex() will return false here but VertexHasQuad() is also false.
// Thus, the next quad has vertex 10, or [2, 0], as it's first corner:
//
// [10, 11, 16]
// [16, 11, 17]
// ...
//
// To illustrate the effect of corner_skip_vertices, an increase to 2x2 yields:
//
// [0,5]                         [5,5]
//   .     .   15. --21.     .     .
//               |  \  |
//   .     .   14. --20.     .     .31
//               |  \  |
//  2. ---7. --13. --19. --25. --- .30
//   |  \  |  \  |  \  |  \  |  \  |
//  1. ---6. --12. --18. --24. --- .29
//               |  \  |
//   .     .   11. --17.     .     .
//               |  \  |
//   .     .   10. --16.     .     .
// [0,0]                         [5,0]
//
//
// These corners will be filled in later by sweeping 90 degree quarter circles
// along each corner and then offsetting by the crux vertex.
//
class SkipCornersVertexProvider : public SimplePanelVertexProvider {
 public:
  SkipCornersVertexProvider(uint2 resolution, uint2 corner_skip_vertices)
      : SimplePanelVertexProvider(resolution),
        corner_skip_vertices_(corner_skip_vertices) {}

  uint32_t GetVertexCount() const override {
    return SimplePanelVertexProvider::GetVertexCount() -
           corner_skip_vertices_.x * corner_skip_vertices_.y * 4;
  }

  uint32_t GetTriangleCount() const override {
    return SimplePanelVertexProvider::GetTriangleCount() -
           corner_skip_vertices_.x * corner_skip_vertices_.y * 4 * 2;
  }

  // Returns a CurvedCorner spec for all 4 corners of the panel. These corners
  // are initialized with the arc, crux, and edge information such that their
  // geometry can be generated.
  std::vector<CurvedCorner> GetCorners() const {
    float2 increment = GetIncrement();
    uint2 resolution = GetResolution();

    CurvedCorner bottom_left = {
        .arc_begin = 3 * M_PI / 2.0f,
        .arc_end = M_PI,
        .uv_crux = {corner_skip_vertices_.x * increment.x,
                    corner_skip_vertices_.y * increment.y},
        .edge_resolution = corner_skip_vertices_};

    CurvedCorner bottom_right = {
        .arc_begin = 2 * M_PI,
        .arc_end = 3 * M_PI / 2.0f,
        .uv_crux = {(resolution.x - corner_skip_vertices_.x - 1) * increment.x,
                    corner_skip_vertices_.y * increment.y},
        .edge_resolution = corner_skip_vertices_};

    CurvedCorner top_right = {
        .arc_begin = M_PI / 2.0f,
        .arc_end = 0,
        .uv_crux = {(resolution.x - corner_skip_vertices_.x - 1) * increment.x,
                    (resolution.y - corner_skip_vertices_.y - 1) * increment.y},
        .edge_resolution = corner_skip_vertices_};

    CurvedCorner top_left = {
        .arc_begin = M_PI,
        .arc_end = M_PI / 2.0f,
        .uv_crux = {corner_skip_vertices_.x * increment.x,
                    (resolution.y - corner_skip_vertices_.y - 1) * increment.y},
        .edge_resolution = corner_skip_vertices_};

    return {bottom_left, bottom_right, top_right, top_left};
  }

 protected:
  void Reset() override {
    SimplePanelVertexProvider::Reset();
    index_ = 0;
    // Move to the first valid vertex, since the logic in the increment operator
    // doesn't run when begin() is called. Otherwise, the [0,0] vertex would
    // always be returned even if it should be skipped.
    while (ShouldSkipVertex(GetCursor())) {
      SimplePanelVertexProvider::operator++();
    }
  }

  bool operator++() override {
    do {
      if (!SimplePanelVertexProvider::operator++()) {
        return false;
      }
    } while (ShouldSkipVertex(GetCursor()));
    // SimplePanelVertexProvider calculates index simply by using x and y but
    // we have to track our own index incrementally as some vertices are
    // skipped. We only increment index_ here after we get past all skips.
    index_++;
    return true;
  }

  uint32_t GetCurrentIndex() const override { return index_; }

  std::optional<QuadIndices> GetCurrentQuadIndices() const override {
    if (!VertexHasQuad()) {
      return std::nullopt;
    }

    uint2 cursor = GetCursor();
    uint2 resolution = GetResolution();
    uint32_t column_start = index_ - cursor.y;
    uint32_t current_height = resolution.y;
    if (cursor.x < corner_skip_vertices_.x ||
        cursor.x > resolution.x - corner_skip_vertices_.x - 1) {
      column_start += corner_skip_vertices_.y;
      current_height -= corner_skip_vertices_.y * 2;
    }
    int32_t same_y_on_next_column = column_start + current_height + cursor.y;
    if (cursor.x + 1 < corner_skip_vertices_.x ||
        cursor.x + 1 > resolution.x - corner_skip_vertices_.x - 1) {
      same_y_on_next_column -= corner_skip_vertices_.y;
    }

    return QuadIndices(GetCurrentIndex(), GetCurrentIndex() + 1,
                       same_y_on_next_column, same_y_on_next_column + 1);
  }

 private:
  bool ShouldSkipVertex(const uint2& cursor, bool print = false) const {
    uint2 resolution = GetResolution();
    // Reject if it is out of bounds.
    if (cursor.x > resolution.x - 1) return true;
    if (cursor.y > resolution.y - 1) return true;
    // Reject if it is within the corner regions.
    return ((cursor.x < corner_skip_vertices_.x ||
             cursor.x >= resolution.x - corner_skip_vertices_.x) &&
            (cursor.y < corner_skip_vertices_.y ||
             cursor.y >= resolution.y - corner_skip_vertices_.y));
  }

  bool VertexHasQuad() const {
    uint2 cursor = GetCursor();
    // Ensure that all 4 vertices are not in an excluded corner region.
    if (ShouldSkipVertex({cursor.x + 1, cursor.y})) return false;
    if (ShouldSkipVertex({cursor.x, cursor.y + 1}, true)) return false;
    if (ShouldSkipVertex({cursor.x + 1, cursor.y + 1})) return false;
    return true;
  }

  uint2 corner_skip_vertices_;
  uint32_t index_ = 0;
};

// Calculates the number of vertices to skip at the corner to round by radius.
// Note: resolution will be modified to be a multiple of the number of skip
// corners because it is much, much simpler that way.
uint2 CalculateCornerRadiusSkipVertices(float corner_radius, float2 size,
                                        uint2& resolution) {
  float target_segments_x = size.x / corner_radius;
  float target_segments_y = size.y / corner_radius;
  // The resolution is one more than target segments because it includes 0.
  float target_resolution_x = target_segments_x + 1;
  float target_resolution_y = target_segments_y + 1;

  // Make the actual resolution a multiple of target_segments that is the
  // closest to the requested resolution without going over.
  resolution.x =
      fmax(target_resolution_x,
           floor(resolution.x / target_segments_x) * target_segments_x + 1);
  resolution.y =
      fmax(target_resolution_y,
           floor(resolution.y / target_segments_y) * target_segments_y + 1);

  // Figure out how many vertices to exclude from the main grid.
  // To simplify, force the resolution to line up with the corner radius.
  return {corner_radius / size.x * resolution.x,
          corner_radius / size.y * resolution.y};
}

MeshPtr MeshFactory::CreateHighPolyQuad(CreateQuadSettings settings,
                                        size_t resolution_width,
                                        size_t resolution_height,
                                        std::optional<float> radius,
                                        float3 scale2,
                                        MeshDataStorageMode data_mode) {
  settings.resolution = resolution_width;
  settings.radius = radius;
  return CreatePanel(settings, scale2, data_mode);
}

// Creates a flat or curved panel with support for corner radius.
// By default, panel is created in the x/y axis pointing along +z.
MeshPtr MeshFactory::CreatePanel(CreateQuadSettings settings, float3 scale,
                                 MeshDataStorageMode data_mode) {
  uint2 resolution(settings.resolution, settings.resolution);
  std::unique_ptr<PanelVertexProvider> vertex_provider;
  uint32_t corner_extra_vertices = 0;
  std::vector<CurvedCorner> curved_corners;
  uint32_t corner_resolution = GetCornerResolution(settings);
  if (settings.corner_radius.has_value() && *settings.corner_radius > 0) {
    uint2 corner_skip_vertices = CalculateCornerRadiusSkipVertices(
        *settings.corner_radius, settings.size, resolution);
    // TODO: deprecate corner resolution
    // Match w/ resolution so we can tesselate the corner smoothly.
    std::unique_ptr<SkipCornersVertexProvider> skip_corners_vertex_provider =
        std::make_unique<SkipCornersVertexProvider>(resolution,
                                                    corner_skip_vertices);
    curved_corners = skip_corners_vertex_provider->GetCorners();
    vertex_provider = std::move(skip_corners_vertex_provider);
    // TODO: +1 for the crux, not exactly sure how to explain the
    // other +1 vertex. This is required to contain the resulting mesh, though,
    // which looks correct.
    corner_extra_vertices = 4 * (corner_resolution + 2);
  } else {
    vertex_provider = std::make_unique<SimplePanelVertexProvider>(resolution);
  }
  int32_t vertex_count =
      vertex_provider->GetVertexCount() + corner_extra_vertices;
  int32_t triangle_count = vertex_provider->GetTriangleCount();
  if (corner_extra_vertices > 0) {
    // Each corner has triangles equal to the number of vertices - 2, since the
    // last vertex is part of the previous triangle and the crux vertex is part
    // of every triangle.
    triangle_count += corner_extra_vertices - 8;
  }

  ProceduralMeshBuilder mesh_builder(
      vertex_count, triangle_count,
      settings.color.has_value() ? kVertexFormatWithColor : kVertexFormat,
      /*is_smooth*/ false, settings.color);

  AabbCalculator aabb;
  int32_t last_vertex_index = 0;
  // Note: have to iterate twice because ProceduralMeshBuilder requires vertices
  // to exist prior to building triangles from them.
  for (PanelVertex panel_vertex : *vertex_provider) {
    float3 vertex =
        CalculatePanelVertexPosition(settings, panel_vertex.uv, scale);

    last_vertex_index = mesh_builder.AddVertex(vertex);
    if (last_vertex_index != panel_vertex.index) {
      IMP_LOG(imp::FATAL) << "Mismatched index: actual: " << panel_vertex.index
                 << ", expected: " << last_vertex_index;
    }
    mesh_builder.SetUV0(last_vertex_index, panel_vertex.uv);
    aabb.AddVertex(vertex);
  }

  for (PanelVertex panel_vertex : *vertex_provider) {
    if (panel_vertex.quad_indices.has_value()) {
      mesh_builder.AddQuad(panel_vertex.quad_indices->top_left,
                           panel_vertex.quad_indices->top_right,
                           panel_vertex.quad_indices->bottom_left,
                           panel_vertex.quad_indices->bottom_right);
    }
  }

  // TODO: Change VertexProvider to GeometryProvider and have
  // generating triangles be part of that interface so the corners can also
  // be generated in that pattern.
  for (CurvedCorner& curved_corner : curved_corners) {
    curved_corner.GenerateCorner(settings, mesh_builder, aabb, scale);
  }

  return CreateByMovingMeshData(PrimitiveType::TRIANGLES, mesh_builder.Build(),
                                aabb.GetAabb(), data_mode, std::nullopt);
}

MeshPtr MeshFactory::CreateRegularPolygon(size_t side_count, float radius,
                                          bool flip_uv, float z,
                                          std::optional<UVConfig> uv_config,
                                          MeshDataStorageMode data_mode) {
  if (side_count < 3) {
    IMP_LOG(imp::FATAL) << "Not enough sides for regular polygon: " << side_count;
    return {};
  }

  // 1 vertex for each regular polygon vertex and one in the center.
  const size_t vertex_count = side_count + 1;

  // The number of indices in the triangle fan is equal to the number of
  // triangles times 3, since we are using PrimitiveType::kTriangles mode.
  const size_t triangle_count = side_count;
  const size_t index_count = triangle_count * 3;

  auto mesh_data = std::make_unique<MeshData>(
      MeshDescription{kVertexFormat, MeshDescription::IndexType::USHORT,
                      vertex_count, index_count});

  // Use custom center UV if provided.
  float2 center_uv =
      uv_config.has_value() ? uv_config->center : float2{0.5f, 0.5f};

  // Make a vertex in the center.
  mesh_data->VertexAttributeAt<float3>(0, VertexAttribute::POSITION) = {0, 0,
                                                                        z};
  mesh_data->VertexAttributeAt<float2>(0, VertexAttribute::UV0) = center_uv;
  mesh_data->VertexAttributeAt<quatf>(0, VertexAttribute::TANGENTS) =
      kIdentityQuatf;

  // Make a vertex for each side.
  const float side_wedge_angle_radians = 2.0f * M_PI / side_count;
  UVScaler scaler{flip_uv};
  AabbCalculator aabb;
  for (size_t side = 0; side < side_count; ++side) {
    const float vertex_angle_radians = side_wedge_angle_radians * side;
    const float3 vertex = {std::cos(vertex_angle_radians) * radius,
                           std::sin(vertex_angle_radians) * radius, z};
    mesh_data->VertexAttributeAt<float3>(side + 1, VertexAttribute::POSITION) =
        vertex;
    // Use custom edges UV if provided.
    float2 uv_coord =
        uv_config.has_value() ? uv_config->edges : scaler.UVFromPos(vertex);
    mesh_data->VertexAttributeAt<float2>(side + 1, VertexAttribute::UV0) =
        uv_coord;
    mesh_data->VertexAttributeAt<quatf>(side + 1, VertexAttribute::TANGENTS) =
        kIdentityQuatf;
    aabb.AddVertex(vertex);
  }

  // Add triangles with the indices: (0,1,2), (0,2,3), ... (0,n-1,n)
  for (int side = 1; side < side_count; ++side) {
    mesh_data->IndexAt<uint16_t>(side * 3) = 0;
    mesh_data->IndexAt<uint16_t>(side * 3 + 1) = side;
    mesh_data->IndexAt<uint16_t>(side * 3 + 2) = side + 1;
  }

  // Add the last triangle (0,n,1) to complete the full circle (we skipped
  // side=0 earlier).
  mesh_data->IndexAt<uint16_t>(0) = 0;
  mesh_data->IndexAt<uint16_t>(1) = side_count;
  mesh_data->IndexAt<uint16_t>(2) = 1;

  return CreateByMovingMeshData(PrimitiveType::TRIANGLES, std::move(mesh_data),
                                aabb.GetAabb(), data_mode, std::nullopt);
}

MeshPtr MeshFactory::CreateByCopyingMeshData(
    PrimitiveType primitive_type, const MeshData& mesh_data,
    std::optional<Box> aabb, std::optional<absl::string_view> name) {
  filament::Engine* engine = GetEngine();

  MeshBuilder mesh_builder(view_);
  BaseVertexBufferBuilder& vertex_buffer_builder =
      mesh_builder.CreateVertexBufferBuilder();
  FillVertexBuffer(view_, engine, mesh_data.GetDescription(),
                   vertex_buffer_builder, mesh_data.CopyVertexData(), name);

  BaseIndexBufferBuilder& index_buffer_builder =
      mesh_builder.CreateIndexBufferBuilder();
  FillIndexBuffer(view_, engine, mesh_data.GetDescription(),
                  index_buffer_builder, mesh_data.CopyIndexData(), name);

  TypedVector<filament::VertexBuffer*> out_vertex_buffers;
  TypedVector<filament::IndexBuffer*> out_index_buffers;
  mesh_builder.Build(&out_vertex_buffers, &out_index_buffers);

  Box verified_aabb;
  if (aabb.has_value()) {
    verified_aabb = *aabb;
  } else {
    verified_aabb = Mesh::CalculateAabb(const_cast<MeshData*>(&mesh_data));
  }

  // Using `new` to access a non-public constructor, see (broken link).
  MeshGpuDataPtr mesh_data_gpu = absl::WrapUnique(new MeshGpuData(
      view_, mesh_data.GetDescription(), out_vertex_buffers.front(),
      out_index_buffers.front(), primitive_type));

  MeshPtr mesh = absl::WrapUnique(
      new Mesh(std::move(mesh_data_gpu), nullptr, verified_aabb));

  return mesh;
}

// TODO: Investigate cleaning this up to avoid code duplication.
MeshPtr MeshFactory::CreateByMovingMeshData(
    PrimitiveType primitive_type, MeshDataPtr mesh_data,
    std::optional<Box> aabb, MeshDataStorageMode data_mode,
    std::optional<absl::string_view> name) {
  filament::Engine* engine = GetEngine();

  Box verified_aabb;
  if (aabb.has_value()) {
    verified_aabb = *aabb;
  } else {
    verified_aabb = Mesh::CalculateAabb(mesh_data.get());
  }

  MeshBuilder mesh_builder(view_);
  MeshDescription description = mesh_data->GetDescription();
  BaseVertexBufferBuilder& vertex_buffer_builder =
      mesh_builder.CreateVertexBufferBuilder();
  FillVertexBuffer(view_, engine, description, vertex_buffer_builder,
                   data_mode == MeshDataStorageMode::kStoreMeshData
                       ? mesh_data->CopyVertexData()
                       : mesh_data->MoveVertexData(),
                   name);

  BaseIndexBufferBuilder& index_buffer_builder =
      mesh_builder.CreateIndexBufferBuilder();
  FillIndexBuffer(view_, engine, description, index_buffer_builder,
                  data_mode == MeshDataStorageMode::kStoreMeshData
                      ? mesh_data->CopyIndexData()
                      : mesh_data->MoveIndexData(),
                  name);

  TypedVector<filament::VertexBuffer*> out_vertex_buffers;
  TypedVector<filament::IndexBuffer*> out_index_buffers;
  mesh_builder.Build(&out_vertex_buffers, &out_index_buffers);

  // Using `new` to access a non-public constructor, see (broken link).
  MeshGpuDataPtr mesh_data_gpu = absl::WrapUnique(
      new MeshGpuData(view_, description, out_vertex_buffers.front(),
                      out_index_buffers.front(), primitive_type));

  MeshPtr mesh = absl::WrapUnique(new Mesh(
      std::move(mesh_data_gpu),
      data_mode == MeshDataStorageMode::kStoreMeshData ? std::move(mesh_data)
                                                       : nullptr,
      verified_aabb));

  return mesh;
}

OwnedMeshPtr MeshFactory::CreateSubMesh(BorrowedMeshPtr parent_mesh,
                                        int index_render_offset,
                                        int index_render_count,
                                        const Box& aabb) {
  MeshRange parent_mesh_data_range = parent_mesh->GetMeshDataRange();
  int32_t parent_mesh_index_max =
      parent_mesh_data_range.offset + parent_mesh_data_range.count;
  int32_t offset = fmin(fmax(index_render_offset, 0), parent_mesh_index_max);
  int32_t count = fmin(index_render_count, parent_mesh_index_max - offset);

  return OwnedMeshPtr(new Mesh(parent_mesh, offset, count, aabb));
}

filament::Engine* MeshFactory::GetEngine() {
  return BaseView::GetSharedEngine();
}

}  // namespace imp
