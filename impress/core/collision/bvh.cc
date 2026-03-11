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

#include "core/collision/bvh.h"

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <optional>
#include <stack>
#include <utility>
#include <vector>

#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Box.h"
#include "core/collision/ray.h"
#include "core/common/trace.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"

namespace imp {
namespace {
using filament::Aabb;
using std::numeric_limits;
using std::optional;

// Compute the union of two Aabbs.
Aabb operator+(Aabb& b1, const Aabb& b2) {
  Aabb b;
  for (int i = 0; i < 3; ++i) {
    b.min[i] = std::min(b1.min[i], b2.min[i]);
    b.max[i] = std::max(b1.max[i], b2.max[i]);
  }

  return b;
}

// Expand an Aabb to include a point.
void operator+=(Aabb& b, const float3& p) {
  for (int i = 0; i < 3; ++i) {
    b.min[i] = std::min(b.min[i], p[i]);
    b.max[i] = std::max(b.max[i], p[i]);
  }
}

// Return true if the ray intersects the Aabb and false otherwise.
// min_ray_distance is the closest intersection found so far.
// The algorithm uses the standard "slab test" as documented in many
// places, such as:
// (broken link)
bool RayIntersectsAabb(const Aabb& box, const float3& ray_origin,
                       const float3& ray_inv_direction,
                       const float min_ray_distance) {
  // Test against the x-axis slab.
  float t_min, t_max;
  if (ray_inv_direction[0] < 0) {
    t_min = ray_inv_direction[0] * (box.max[0] - ray_origin[0]);
    t_max = ray_inv_direction[0] * (box.min[0] - ray_origin[0]);
  } else {
    t_min = ray_inv_direction[0] * (box.min[0] - ray_origin[0]);
    t_max = ray_inv_direction[0] * (box.max[0] - ray_origin[0]);
  }
  if (t_max > min_ray_distance) {
    t_max = min_ray_distance;
  }
  if (t_max < 0 || t_min > t_max) {
    return false;
  }

  // Test against the y-axis slab.
  float ty_min, ty_max;
  if (ray_inv_direction[1] < 0) {
    ty_min = ray_inv_direction[1] * (box.max[1] - ray_origin[1]);
    ty_max = ray_inv_direction[1] * (box.min[1] - ray_origin[1]);
  } else {
    ty_min = ray_inv_direction[1] * (box.min[1] - ray_origin[1]);
    ty_max = ray_inv_direction[1] * (box.max[1] - ray_origin[1]);
  }
  if (ty_max > t_min) {
    t_min = ty_min;
  }
  if (ty_max < t_max) {
    t_max = ty_max;
  }
  if (t_min > t_max || t_max < 0) {
    return false;
  }

  // Test against the z-axis slab.
  float tz_min, tz_max;
  if (ray_inv_direction[2] < 0) {
    tz_min = ray_inv_direction[2] * (box.max[2] - ray_origin[2]);
    tz_max = ray_inv_direction[2] * (box.min[2] - ray_origin[2]);
  } else {
    tz_min = ray_inv_direction[2] * (box.min[2] - ray_origin[2]);
    tz_max = ray_inv_direction[2] * (box.max[2] - ray_origin[2]);
  }
  if (tz_min > t_min) {
    t_min = tz_min;
  }
  if (tz_max < t_max) {
    t_max = tz_max;
  }

  return (t_min <= t_max) && (t_max >= 0);
}

// Intersect a triangle with a ray and return the distance and the triangle
// normal. It returns numeric_limits<float>::max() if there is no intersection
// (instead of this function returning an optional) so that it can be used in an
// inner loop to keep track of the closest intersection.
// Uses
// https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
float RayTriangleIntersect(const float3& p0, const float3& p1, const float3& p2,
                           const imp::Ray& ray, bool intersect_backfaces,
                           imp::float3* normal) {
  const float3 e1 = p1 - p0;
  const float3 e2 = p2 - p0;
  const float3 ray_cross_e2 = cross(ray.direction, e2);
  const float det = dot(e1, ray_cross_e2);

  // Triangle is parallel to the ray.
  if (std::abs(det) < numeric_limits<float>::epsilon()) {
    return numeric_limits<float>::max();
  }

  // Skip this triangle if the normal is pointing away from the ray
  // with backface culling enabled.
  if (!intersect_backfaces && det < 0) {
    return numeric_limits<float>::max();
  }

  const float inv_det = 1.0 / det;
  const float3 s = ray.origin - p0;

  // Test first barycentric coordinate u.
  const float u = inv_det * dot(s, ray_cross_e2);
  if (u < 0 || u > 1) {
    return numeric_limits<float>::max();
  }

  // Test second barycentric coordinate v.
  const float3 s_cross_e1 = cross(s, e1);
  const float v = inv_det * dot(ray.direction, s_cross_e1);
  if (v < 0 || u + v > 1) {
    return numeric_limits<float>::max();
  }

  // Compute the actual distance given that intersection point is in the
  // interior of the triangle.
  const float t = inv_det * dot(e2, s_cross_e1);
  if (t < 0) {
    return numeric_limits<float>::max();
  }

  *normal = cross(e1, e2);
  return t;
}
}  // namespace

// Intersect the bvh with a ray.
optional<Bvh::RayIntersection> Bvh::IntersectRay(
    const imp::Ray& input_ray) const {
  if (nodes_.empty()) {
    return std::nullopt;
  }

  // Normalize the ray direction since input rays are not required to be
  // normalized. Also compute the inverse of the direction to avoid
  // recomputing the divisions in the ray-aabb intersection test.
  Ray ray(input_ray.origin, normalize(input_ray.direction));
  float3 ray_inv_direction;
  for (int i = 0; i < 3; ++i) {
    if (std::abs(ray.direction[i]) > numeric_limits<float>::epsilon()) {
      ray_inv_direction[i] = 1.0f / ray.direction[i];
    } else if (ray.direction[i] >= 0.0f) {
      ray_inv_direction[i] = numeric_limits<float>::max();
    } else {
      ray_inv_direction[i] = -numeric_limits<float>::max();
    }
  }

  RayIntersection min_intersection{.distance = numeric_limits<float>::max()};

  // Put the nodes that need to be traced on a stack, starting with the root.
  std::stack<int> nodes_to_intersect;
  nodes_to_intersect.push(0);
  while (!nodes_to_intersect.empty()) {
    const Bvh::Node& node = nodes_[nodes_to_intersect.top()];
    nodes_to_intersect.pop();
    if (!RayIntersectsAabb(node.aabb, ray.origin, ray_inv_direction,
                           min_intersection.distance)) {
      continue;
    }
    if (node.is_leaf()) {
      // Intersect ray with leaf triangles.
      for (int i = node.leaf_node.begin_triangle_index;
           i < node.leaf_node.end_triangle_index; ++i) {
        const TriangleIndices& tri_indices = indices_[i];
        float3 normal;
        float ray_dist = RayTriangleIntersect(
            vertex_at(tri_indices.i0), vertex_at(tri_indices.i1),
            vertex_at(tri_indices.i2), ray, options_.intersect_backfaces,
            &normal);
        if (ray_dist < min_intersection.distance) {
          min_intersection.distance = ray_dist;
          min_intersection.normal = normal;
          min_intersection.triangle_id = tri_indices.id;
        }
      }
    } else {  // Internal node that the ray intersects its aabb.
      // Push the far child and then the near child on the stack
      // so that they are traced as close to front-to-back as possible.
      const int right_child_index = node.internal_node.left_child_index + 1;
      if (ray.direction[node.internal_node.split_index()] > 0) {
        nodes_to_intersect.push(right_child_index);
        nodes_to_intersect.push(node.internal_node.left_child_index);
      } else {
        nodes_to_intersect.push(node.internal_node.left_child_index);
        nodes_to_intersect.push(right_child_index);
      }
    }
  }

  if (min_intersection.distance < numeric_limits<float>::max()) {
    min_intersection.normal = normalize(min_intersection.normal);
    return min_intersection;
  }

  return std::nullopt;
}

Bvh::Bvh(const imp::MeshVertexAndIndexData& mesh_data, const Options& options,
         std::optional<MeshRange> submesh_range)
    : options_(options), vertex_data_(*mesh_data.vertex_data) {
  IMP_TRACE();
  int32_t offset = submesh_range.has_value() ? submesh_range->offset : 0;

  // Copy the gltf indices to a vector of TriangleIndices.
  if (mesh_data.index_data->GetDescription().index_type ==
      MeshDescription::IndexType::USHORT) {
    auto indices_uint16 = mesh_data.index_data->Indices<uint16_t>();
    uint16_t count = submesh_range.has_value() ? submesh_range->count
                                               : indices_uint16.size();
    uint16_t triangle_count = count / 3;
    indices_.reserve(triangle_count);
    for (int i = offset, id = 0; id < triangle_count; id++) {
      indices_.push_back(
          {indices_uint16[i++], indices_uint16[i++], indices_uint16[i++], id});
    }
  } else {
    auto indices_uint32 = mesh_data.index_data->Indices<uint32_t>();
    uint32_t count = submesh_range.has_value() ? submesh_range->count
                                               : indices_uint32.size();
    uint32_t triangle_count = count / 3;
    indices_.reserve(triangle_count);
    for (int i = offset, id = 0; id < triangle_count; id++) {
      indices_.push_back(
          {indices_uint32[i++], indices_uint32[i++], indices_uint32[i++], id});
    }
  }

  // Compute the triangle centers.
  int num_mesh_triangles = indices_.size();
  std::vector<imp::float3> triangle_centers;
  triangle_centers.reserve(num_mesh_triangles);
  for (const TriangleIndices& tri_indices : indices_) {
    float3 center =
        (1.0f / 3.0f) * (vertex_at(tri_indices.i0) + vertex_at(tri_indices.i1) +
                         vertex_at(tri_indices.i2));
    triangle_centers.push_back(center);
  }

  // Put the nodes that need to be processed on a stack. For each node on the
  // stack, if it is a leaf node, then update the aabb from the triangle
  // vertices. Otherwise, split the node and put the children on the stack.
  std::stack<int> nodes_to_process;
  nodes_.emplace_back(0, num_mesh_triangles);
  nodes_to_process.push(0);
  while (!nodes_to_process.empty()) {
    Bvh::Node& node = nodes_[nodes_to_process.top()];
    nodes_to_process.pop();

    int new_node_index = ProcessNode(&node, &triangle_centers);
    if (new_node_index >= 0) {
      nodes_to_process.push(new_node_index);
      nodes_to_process.push(new_node_index + 1);
    }
  }

  // Update the boxes of the inner nodes.  Since by construction children
  // always occur after the parent in the array, by going through the nodes
  // in reverse in the array, a child's box is guaranteed to already be
  // updated when updating the parent's box.
  for (int i = nodes_.size() - 1; i >= 0; i--) {
    Bvh::Node& node = nodes_[i];
    if (!node.is_leaf()) {
      node.aabb = nodes_[node.internal_node.left_child_index].aabb +
                  nodes_[node.internal_node.left_child_index + 1].aabb;
    }
  }
}

int Bvh::ProcessNode(Bvh::Node* node_ptr,
                     std::vector<imp::float3>* triangle_centers_ptr) {
  Node& node = *node_ptr;
  std::vector<imp::float3>& triangle_centers = *triangle_centers_ptr;

  // If the node has fewer than kTargetNumLeafTris triangles, then leave the
  // triangle range as set the aabb from its triangle vertices.
  if (node.leaf_node.num_triangles() <= options_.max_num_leaf_triangles) {
    node.aabb = ComputeLeafNodeAabb(node.leaf_node);
    return -1;
  }

  // Compute the bounding box of the node from the triangle centers.
  Aabb triangle_centers_box;
  const int begin_triangle_index = node.leaf_node.begin_triangle_index;
  const int end_triangle_index = node.leaf_node.end_triangle_index;
  for (int i = begin_triangle_index; i < end_triangle_index; ++i) {
    triangle_centers_box += triangle_centers[i];
  }

  // Determine a split plane based on the largest dimension of the
  // bounding box of this node.
  const float3 diag = triangle_centers_box.max - triangle_centers_box.min;
  const int split_index = (diag[0] > diag[1] && diag[0] > diag[2]) ? 0
                          : (diag[1] > diag[2])                    ? 1
                                                                   : 2;

  // Loop over the triangle centers associated with this node and partition
  // based on the split plane, swapping to keep the triangles contiguous.
  // In the future, may be worthwhile to compute a more optimal split value
  // using something like the Surface Area Heuristic.
  const float splitValue = 0.5f * (triangle_centers_box.max[split_index] +
                                   triangle_centers_box.min[split_index]);
  int l = begin_triangle_index;
  int r = end_triangle_index;
  int mid_triangle_index = begin_triangle_index;
  while (true) {
    while (l < r) {
      if (triangle_centers[l][split_index] >= splitValue) {
        break;
      }
      l++;
    }

    while (l < r) {
      if (triangle_centers[r - 1][split_index] < splitValue) {
        break;
      }
      r--;
    }
    if (l == r) {
      mid_triangle_index = r;
      break;
    }
    std::swap(indices_[l], indices_[r - 1]);
    std::swap(triangle_centers[l], triangle_centers[r - 1]);
  }

  // If the a partition did not separating the triangles into two groups
  // (which can happen if there are a lot of duplicate triangles), the keep
  // the node as a leaf.
  if (mid_triangle_index == begin_triangle_index ||
      mid_triangle_index == end_triangle_index) {
    node.aabb = ComputeLeafNodeAabb(node.leaf_node);
    return -1;
  } else {
    // Otherwise split the node and put the children on the stack.
    // Mark this node as an internal node by setting the split dimension and
    // the left child index.
    int left_child_index = nodes_.size();
    node.internal_node.split_dimension =
        static_cast<Bvh::InternalNode::SplitDimension>(-split_index);
    node.internal_node.left_child_index = left_child_index;

    // Create the left and right child nodes.
    // Note that we could consider not creating the node until it is ready to
    // be processed which would create a depth first tree, which might improve
    // cache locality. See
    // (broken link)
    nodes_.emplace_back(begin_triangle_index, mid_triangle_index);
    nodes_.emplace_back(mid_triangle_index, end_triangle_index);
    return left_child_index;
  }
}

// Compute the union of the aabbs of the triangles of a leaf node.
filament::Aabb Bvh::ComputeLeafNodeAabb(const LeafNode& node) const {
  Aabb aabb;
  for (int i = node.begin_triangle_index; i < node.end_triangle_index; ++i) {
    const TriangleIndices& tri_indices = indices_[i];
    aabb += vertex_at(tri_indices.i0);
    aabb += vertex_at(tri_indices.i1);
    aabb += vertex_at(tri_indices.i2);
  }
  return aabb;
}

}  // namespace imp
