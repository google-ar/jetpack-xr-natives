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

#ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_BVH_MESH_COLLIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_COLLISION_BVH_MESH_COLLIDER_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "filament/filament/include/filament/Box.h"
#include "core/collision/ray.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"

namespace imp {

// Main internal helper Bvh class for the BvhMeshCollider.
class Bvh {
 public:
  struct Options {
    bool intersect_backfaces = false;

    // Split a node if it has more than this number of triangles.
    // This value was selected based on benchmarking (see
    // mesh_bvh_benchmark_test.cc) and trades off construction time for
    // intersection time: the higher this is, the faster a BVH will construct
    // but the slower intersections will become (and vice versa).
    int max_num_leaf_triangles = 32;
  };
  // Intersect the bvh with a ray.
  struct RayIntersection {
    float distance;
    imp::float3 normal;
    int triangle_id;
  };

  // Construct a Bounding Volume Hierarchy given a mesh.
  // NOTE: The user is responsible for providing a valid submesh range.
  Bvh(const imp::MeshVertexAndIndexData& mesh_data, const Options& options,
      std::optional<MeshRange> submesh_range = std::nullopt);

  std::optional<RayIntersection> IntersectRay(const imp::Ray& ray) const;

 private:
  struct TriangleIndices {
    uint32_t i0;
    uint32_t i1;
    uint32_t i2;

    // Also keep track of the original triangle id index in the mesh since the
    // triangles are reordered during the bvh construction to keep all the
    // triangles in a leaf node contiguous.
    int id;
  };

  struct LeafNode {
    int begin_triangle_index;
    int end_triangle_index;
    int num_triangles() const {
      return end_triangle_index - begin_triangle_index;
    }
  };

  struct InternalNode {
    // The split dimension is the dimension of the bounding box that is
    // largest. Use negative value as a flag to indicate it is a internal node.
    enum SplitDimension : int { X = 0, Y_SPLIT = -1, Z_SPLIT = -2 };
    int left_child_index;
    SplitDimension split_dimension;
    int split_index() const { return -split_dimension; }
  };

  struct Node {
    // Create a node that references a list of triangles.
    Node(int begin_triangle_index, int end_triangle_index)
        : leaf_node({begin_triangle_index, end_triangle_index}) {}

    // Bounding box around all the triangles in the node or its children.
    filament::Aabb aabb;

    union {
      LeafNode leaf_node;
      InternalNode internal_node;
    };

    bool is_leaf() const { return leaf_node.end_triangle_index > 0; }
  };

  // Get the position of the vertex at |index|.
  inline const float3& vertex_at(int index) const {
    constexpr int kPositionAttributeOffset = 0;
    return vertex_data_.VertexAttributeAt<float3>(index,
                                                  kPositionAttributeOffset);
  }

  filament::Aabb ComputeLeafNodeAabb(const LeafNode& node) const;
  int ProcessNode(Bvh::Node* node_ptr,
                  std::vector<imp::float3>* triangle_centers_ptr);

  std::vector<Node> nodes_;
  Options options_;

  // Reference to the triangles for faster and easier access.
  std::vector<TriangleIndices> indices_;
  const imp::MeshVertexData& vertex_data_;
};

}  // namespace imp

#endif  // #ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_BVH_MESH_COLLIDER_H_
