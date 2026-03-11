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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_RENDER_MESH_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_RENDER_MESH_FACTORY_H_

#include <sys/types.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/geometry/shapes/box.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_data.h"
#include "core/view/base_view.h"

namespace imp {

class View;

static constexpr uint8_t kDefaultSphereResolution = 12;
static constexpr uint8_t kDefaultCylinderResolution = 12;
static constexpr uint8_t kDefaultCapsuleResolution = 12;
static constexpr uint8_t kDefaultQuadResolution = 5;
static constexpr uint8_t kDefaultRoundedCornerResolution = 12;

struct CreateBoxSettings {
  // The dimensions of the box
  float3 size = {1, 1, 1};
  float3 center = {0, 0, 0};
  // Whether to flip the mesh inside-out.
  bool flip_uv = false;
  std::optional<std::string> name = std::nullopt;
  // The color applied to VertexAttribute::COLOR for all vertices of the box.
  std::optional<float4> color = std::nullopt;
};

struct CreateSphereSettings {
  float radius = 1;
  float3 center = {0, 0, 0};
  // Determines the number of edge rings that make up the sphere,
  // and the number of vertices in each edge ring.
  uint8_t resolution = kDefaultSphereResolution;
  // Causes V to be mapped from 0 to 1, with 0 being the bottom of the sphere
  // and 1 being the top. This is useful when rendering immersive video.
  bool flip_uv = false;
  // Causes the vertices to be wound facing inside the sphere, useful for if the
  // camera is within the sphere. This will also cause the UV seam to be rotated
  // to align with +Z, which would be behind the camera if it aligned with the
  // sphere's origin pose.
  bool flip_face_direction = false;
  std::optional<std::string> name = std::nullopt;
  // The color applied to VertexAttribute::COLOR for all vertices of the sphere.
  std::optional<float4> color = std::nullopt;
};

// TODO: (broken link) - Delete this if merging CreateXYHemisphere with
// CreateSphere.
// Note that Hemispheres don't currently support flipping the UVs, or changing
// the face direction. Its UV's are vertically flipped from the Sphere's by
// default, and the face direction is always inward facing.
struct CreateXYHemisphereSettings {
  float radius = 1;
  float3 center = {0, 0, 0};
  // Determines the number of edge rings that make up the sphere,
  // and the number of vertices in each edge ring.
  uint8_t resolution = kDefaultSphereResolution;
  std::optional<std::string> name = std::nullopt;
  // Use a vertex format with only positions. Otherwise meshes may have tangents
  // and texture coordinates.
  bool is_position_only = false;
};

struct CreateCylinderSettings {
  // The radius of the cylinder along the XZ plane.
  float radius = .5f;
  // The height of the entire cylinder along the Y axis.
  float height = 1;
  // The center of the base of the cylinder. The cylinder extends upwards from
  // this point, with the top pole at center + (kUp * height).
  float3 center = {0, 0, 0};
  // Determines the number of edge rings that
  // make up the capsule, and the number of vertices in each edge ring.
  uint8_t resolution = kDefaultCylinderResolution;
  // Whether to flip the mesh inside-out.
  bool flip_uv = false;
  std::optional<std::string> name = std::nullopt;
  // The color applied to VertexAttribute::COLOR for all vertices of the
  // cylinder.
  std::optional<float4> color = std::nullopt;
};

struct CreateCapsuleSettings {
  // The radius of the capsule at its widest point (its 'equator')
  // along the XZ plane.
  float radius = .5f;
  // The height of the entire capsule along the Y axis.
  float height = 2;
  float3 center = {0, 0, 0};
  // Determines the number of edge rings that
  // make up the capsule, and the number of vertices in each edge ring.
  uint8_t resolution = kDefaultCapsuleResolution;
  // Whether to flip the mesh inside-out.
  bool flip_uv = false;
  std::optional<std::string> name = std::nullopt;
  // The color applied to VertexAttribute::COLOR for all vertices of the
  // capsule.
  std::optional<float4> color = std::nullopt;
};

struct CreateConeSettings {
  // The radius of the base of the cone in the XY plane.
  float radius = .5f;
  // The height of the cone along the Y axis.
  float height = 1;
  // The center of the base of the cone. The cone extends upwards from this
  // point, with the apex at center + (kUp * height).
  float3 center = {0, 0, 0};
  // Determines the number of edge rings that base of the cone.
  uint8_t resolution = kDefaultCylinderResolution;
  // Whether to flip the mesh inside-out.
  bool flip_uv = false;
  std::optional<std::string> name = std::nullopt;
  // The color applied to VertexAttribute::COLOR for all vertices of the cone.
  std::optional<float4> color = std::nullopt;
};

// A helper struct for any shapes that have flat 2D geometry.
struct CreateQuadSettings {
  // The x and y size of the 2D geometry (defaults to 2, meaning the quad goes
  // from [-1, 1] on both x and y axes).
  float2 size = {2, 2};
  // TODO: make center a float3 and remove z.
  // Controls the x and y center of the quad geometry, which defaults to zero
  // meaning the quad extends from the origin on both x and y axes.
  float2 center = {0, 0};
  // The z position of the quad vertices (defaults to 0).
  float z = 0;
  // Whether to flip the uv of the geometry (if your texture looks backwards).
  bool flip_uv = false;

  // If non-zero, curves the quad around a cylinder with radius meters.
  // This will cause the quad to be "high-poly" by default. Set resolution
  // to customize the complexity of the curvature.
  std::optional<float> radius = std::nullopt;
  // Determines the radius of the corners. If non-zero, will automatically
  // create a high-poly quad at default resolution unless resolution is set.
  std::optional<float> corner_radius = std::nullopt;
  std::optional<uint32_t> corner_resolution = std::nullopt;
  // If > 2, creates a "high-poly quad" with resolution vertices on each axis.
  uint32_t resolution = 2;
  std::optional<std::string> name = std::nullopt;
  // The color applied to VertexAttribute::COLOR for all vertices of the quad.
  std::optional<float4> color = std::nullopt;
};

// A helper struct for creating a custom mesh.
struct CreateCustomMeshSettings {
  // Vertex positions.
  std::vector<float> positions;
  // Vertex texture coordinates.
  std::vector<float> texcoords;
  // Vertex indices.
  std::optional<std::vector<uint32_t>> indices;
  // Draw mode.
  filament::RenderableManager::PrimitiveType draw_mode;
  std::optional<std::string> name = std::nullopt;
  // The color applied to VertexAttribute::COLOR for all vertices of the mesh.
  std::optional<float4> color = std::nullopt;
};

// A helper struct for mapping UVs when calling
// MeshFactory::CreateRegularPolygon.
struct UVConfig {
  float2 center;
  float2 edges;
};

// A factory for creating various types of Mesh assets, accessed through
// MeshPtrs.
class MeshFactory {
 public:
  enum class MeshDataStorageMode {
    // When creating Mesh, preserve MeshData for future uses.
    kStoreMeshData,
    // When creating Mesh, discard MeshData.
    kDiscardMeshData
  };

  using PrimitiveType = filament::RenderableManager::PrimitiveType;

  // Number of sides in a regular polygon with this shape.
  static constexpr size_t kTriangleSides = 3;

  explicit MeshFactory(BaseView& view);

  MeshPtr CreateBox(
      CreateBoxSettings settings = {},
      // Specifies if the mesh information should be
      // stored in memory, which is required for collisions.
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  MeshPtr CreateSphere(
      CreateSphereSettings settings = {},
      // Specifies if the mesh information should be
      // stored in memory, which is required for collisions.
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  MeshPtr CreateXYHemisphere(
      CreateXYHemisphereSettings settings = {},
      // Specifies if the mesh information should be
      // stored in memory, which is required for collisions.
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  MeshPtr CreateCylinder(
      CreateCylinderSettings settings = {},
      // Specifies if the mesh information should be
      // stored in memory, which is required for collisions.
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  MeshPtr CreateCapsule(
      CreateCapsuleSettings settings = {},
      // Specifies if the mesh information should be
      // stored in memory, which is required for collisions.
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  MeshPtr CreateCone(
      CreateConeSettings settings = {},
      // Specifies if the mesh information should be
      // stored in memory, which is required for collisions.
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  // TODO: delete CreateQuad() in favor of CreatePanel().
  // Create a quad with dimensions {-size.x / 2, -size.y / 2, z} to {size.x / 2,
  // size.y / 2, z}, facing +Z.
  // Vertices have UVs and tangents.  If flip_uv is false, the UV origin is
  // at the bottom-left vertex.  If flip_uv is true, the UV origin is at the
  // top-left vertex.
  MeshPtr CreateQuad(
      CreateQuadSettings settings = {},
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  // TODO: delete CreateHighPolyQuad() in favor of CreatePanel().
  // Create a quad with dimensions and width/height vertex resolution.
  // Optional radius parameter allows creation of cylindrical/spherical panels.
  MeshPtr CreateHighPolyQuad(
      CreateQuadSettings settings, size_t resolution_width = 100,
      size_t resolution_height = 100,
      std::optional<float> radius = std::nullopt,
      float3 scale2 = {1.0f, 1.0f, 1.0f},
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  // Creates a "panel" mesh based on the given settings. The result can be
  // anything from a flat quad with 4 vertices and two triangles to a curved,
  // high-poly mesh with rounded corners depending on the data in settings.
  MeshPtr CreatePanel(
      CreateQuadSettings settings, float3 scale2 = {1.0f, 1.0f, 1.0f},
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  // Create a regular polygon with default radius 1 in the XY plane centered at
  // the {0, 0, z}, facing +Z. This can be used to create triangles, squares,
  // hexagons, and more, with higher number of sides approximating a circle.
  // The first vertex will always be at {radius, 0, z}.
  MeshPtr CreateRegularPolygon(
      size_t side_count, float radius = 1.0f, bool flip_uv = false,
      float z = 0.0f, std::optional<UVConfig> uv_config = std::nullopt,
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  // Create a custom mesh with the given settings.
  // AABB will be calculated according to mesh data.
  MeshPtr CreateCustomMesh(
      CreateCustomMeshSettings settings = {},
      // Specifies if the mesh information should be
      // stored in memory, which is required for collisions.
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData);

  // Create a mesh with a copy of |mesh_data|. |mesh_data| cannot be destroyed
  // until the copy is done.
  // This method temporary doesn't support storing |mesh_data| on CPU, so that
  // corresponding MeshCollider (if added) will not be precise.
  // If no AABB or invalid AABB is provided, AABB will be calculated according
  // to mesh data.
  MeshPtr CreateByCopyingMeshData(
      PrimitiveType primitive_type, const MeshData& mesh_data,
      std::optional<Box> aabb,
      std::optional<absl::string_view> name = std::nullopt);

  // Create a mesh with |mesh_data|.
  // If no AABB or invalid AABB is provided, AABB will be calculated according
  // to mesh data.
  MeshPtr CreateByMovingMeshData(
      PrimitiveType primitive_type, MeshDataPtr mesh_data,
      std::optional<Box> aabb,
      MeshDataStorageMode data_mode = MeshDataStorageMode::kDiscardMeshData,
      std::optional<absl::string_view> name = std::nullopt);

  // Create a submesh from |mesh| with the given index range.
  // NOTE: you can pre-calculate the aabb by
  //       Box aabb = Mesh::CalculateAabb(parent_mesh.GetMeshData(),
  //                                       index_render_offset,
  //                                       index_render_count)
  //       and pass it to this function, if the parent mesh contains CPU mesh
  //       data.
  OwnedMeshPtr CreateSubMesh(BorrowedMeshPtr parent_mesh,
                             int index_render_offset, int index_render_count,
                             const Box& aabb);

 private:
  BaseView& view_;
  filament::Engine* GetEngine();
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_RENDER_MESH_FACTORY_H_
