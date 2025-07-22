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

#ifndef THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLISION_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COLLISION_COLLISION_HELPERS_H_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <utility>

#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "core/collision/plane.h"
#include "core/collision/ray.h"
#include "core/geometry/shapes/box.h"
#include "core/geometry/shapes/cone.h"
#include "core/geometry/shapes/cylinder.h"
#include "core/geometry/shapes/line_segment.h"
#include "core/geometry/shapes/rect.h"
#include "core/geometry/shapes/sphere.h"
#include "core/geometry/shapes/triangle.h"
#include "core/math/almost_equal.h"
#include "core/math/almost_equal_helper.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"

// Impress low level collision methods provide a means for testing collision
// shapes and hierarchies against rays, points, and other shapes.
namespace imp {
namespace collision {

constexpr float kAngleThreshold = 0.003f;

enum class Result : uint32_t {
  kDoesIntersect = 0x1,
  kDoesNotIntersect = 0x2,
};

// The information that locates the collided triangle in a GltfMesh or a
// multi-primitive mesh.
struct CollidedTriangle {
  // The ID of the primitive in the GltfMesh (or multi-primitive mesh) that the
  // triangle belongs to.
  size_t primitive_id;
  // The ID of the triangle containing the collision point.
  size_t triangle_id;
};

// The result from testing a mesh with a ray.
template <typename T>
struct MeshIntersection {
  // The position of collision point.
  TVec3<T> collision_point;
  // The distance from ray origin to the collision point.
  T intersection_dist;
  // The ID of the triangle containing the collision point.
  size_t triangle_id;
  // The normal of the surface at the collision point.
  TVec3<T> collision_normal;
};

// The result from testing a GltfMesh (or multi-primitive mesh) with a ray.
template <typename T>
struct MultiPrimitiveMeshIntersection : public MeshIntersection<T> {
  // The ID of the primitive in the GltfMesh (or multi-primitive mesh) that the
  // triangle belongs to.
  size_t primitive_id;
};

// An intersection from testing a cylinder with a ray, recorded in both 3D and
// cylindrical space.
template <typename T>
struct CylinderIntersection {
  // The position of collision point in 3D space.
  TVec3<T> intersection;
  // The distance from collision point and its projection on the cylinder's
  // bottom plane.
  T height;
  // The angle between the forward vector and the collision point.
  // arc_length ~[-PI, PI)
  T arc_length;
};

// The result from testing a cylinder with a ray.
template <typename T>
struct CylinderIntersectionInfo {
  // The point where the ray enter the cylinder.
  std::optional<CylinderIntersection<T>> enter_cylinder;
  // The point where the ray exit the cylinder.
  std::optional<CylinderIntersection<T>> exit_cylinder;
  // Whether the ray is tangent to the cylinder.
  bool tangent = false;
};

// An intersection from testing a surface with a ray. This is a simple structure
// that holds geometrical computation result. Note the difference from
// GenericRayHit<T>, which contains more information of the hitted object.
template <typename T>
struct RayIntersection {
  // The position of collision point on surface.
  TVec3<T> collision_point;
  // The normal of the surface at the collision point.
  TVec3<T> normal;
  // The distance from the ray origin to the collision point.
  T distance;
};

// Test an object against a point (p) in space, record the closest point (q) on
// the object and the distance between that point (q) to the point (p) in space.
template <typename T>
struct DistanceFromShape {
  T distance;
  TVec3<T> closest_point;
};

// Finds the closest point on the ray (first argument) to the reference line
// (infinite line, second argument). Returns a value "t" that the closest point
// is ray.origin + t * ray.direction. Returns std::nullopt if the reference line
// is parallel to the ray.
template <typename T>
std::optional<T> ClosestPointOnRayToLine(const GenericRay<T>& ray,
                                         const GenericRay<T>& reference_line);

// ClosestPointOnRayToLine assuming the rays' directions are pre-normalized.
template <typename T>
std::optional<T> ClosestPointOnRayToLinePreNormalized(
    const GenericRay<T>& ray, const GenericRay<T>& reference_line);

// Test an infinite plane for ray intersection.
// out_col_point parameter provides an intersection point if it exists,
// otherwise it's unchanged.
template <typename T>
Result PlaneIntersectsRay(const GenericPlane<T>& plane,
                          const GenericRay<T>& ray, TVec3<T>* out_col_point);

// Tests a limit plane against a ray. The planes parameters should be provided
// in plane local space, the ray should be transformed into the planes local
// space. out_col_point parameter provides an intersection point if it exists,
// otherwise it's unchanged.
//
// Please note that this method only works in float precision.
Result PlaneIntersectsRay(const Plane& local_plane,
                          const float3& local_plane_min,
                          const float3& local_plane_max, const Ray& local_ray,
                          float3* out_col_point);

// Get the distance squared from the nearest point on an AABB to another point.
float SquaredDistFromPointToAABB(const float3& point, const imp::Box& box);

// Test an AABB bounding box against a ray.
template <typename T>
std::optional<RayIntersection<T>> AABBIntersectsRay(const imp::Box& box,
                                                    const GenericRay<T>& ray);

// Test an AABB bounding box against a line segment. A good broad phase
// collision test, very efficient but only indicates collision or no
// collision. Based on the separating axes theorem, an excellent overview at
// the link
// (broken link)
Result AABBIntersectsLineSegment(const imp::Box& box,
                                 const LineSegment& line_segment);

// Test if an AABB contains a point.
Result AABBContainsPoint(const imp::Box& box, const float3& test_point);

// Test if two AABB's intersect
Result AABBIntersectsAABB(const imp::Box& box_a, const imp::Box& box_b);

// Test if an AABB bounding box contains intersects a sphere.
Result AABBIntersectsSphere(const imp::Box& box, const Sphere& sphere);

// Test if an AABB intersects a triangle.
Result AABBIntersectsTriangle(const imp::Box& box, const Triangle& triangle);

// Tests if a rectangle contains a 2D point
Result RectContainsPoint(const imp::Rect& rect, const float2& test_point);

// Test if a 2d capsule contains a 2D point. A stadium can be considered as an
// iso-surface of distance "radius" to a line segment.
Result Capsule2DContainPoint(const LineSegment& line_segment, float radius,
                             const float2& test_point);

// Test if a sphere contains a point.
Result SphereContainsPoint(const Sphere& sphere, const float3& test_point);

// Test if a sphere intersects a ray.
// out_col_point parameter provides an intersection point if it exists,
// otherwise it's unchanged.
template <typename T>
Result SphereIntersectsRay(const Sphere& sphere, const GenericRay<T>& ray,
                           T* out_intersection_dist, TVec3<T>* out_col_point);

// Test if a triangle intersects a ray.
// out_col_point parameter provides an intersection point if it exists,
// otherwise it's unchanged.
template <typename T>
Result TriangleIntersectsRay(const Triangle& triangle, const GenericRay<T>& ray,
                             bool collide_with_back_faces,
                             T* out_intersection_dist, TVec3<T>* out_col_point,
                             TVec3<T>* out_normal);

// Test if a mesh intersects a ray.
template <typename T>
std::optional<MeshIntersection<T>> MeshIntersectsRay(
    MeshVertexAndIndexData mesh, const GenericRay<T>& ray,
    bool collide_with_back_faces,
    std::optional<MeshRange> sub_mesh_range = std::nullopt);

// Test if a mesh intersects a ray.
template <typename T>
std::optional<MeshIntersection<T>> MeshIntersectsRay(
    MeshData* mesh, const GenericRay<T>& ray, bool collide_with_back_faces,
    std::optional<MeshRange> sub_mesh_range = std::nullopt);

// A GltfMesh can have multiple meshes, which are called primitives in GltfMesh.
// Test if multiple meshes (usually of one node) intersect a ray.
template <typename T>
std::optional<MultiPrimitiveMeshIntersection<T>>
MultiPrimitiveMeshIntersectsRay(absl::Span<const MeshVertexAndIndexData> meshes,
                                const GenericRay<T>& ray,
                                bool collide_with_back_faces);

// Test if the lateral surface of a truncated infinite cylinder intersects a
// ray. Truncation happens at 0 and length.
template <typename T>
CylinderIntersectionInfo<T> CylinderLateralSurfaceIntersectsRay(
    GenericCylinder<T> cylinder, const GenericRay<T>& ray);

// Test if a truncated finite cylinder intersects a ray.
template <typename T>
std::optional<RayIntersection<T>> CylinderIntersectsRay(
    const Cylinder& cylinder, const GenericRay<T>& ray);

// CylinderIntersectsRay assuming the ray's direction is pre-normalized.
template <typename T>
std::optional<RayIntersection<T>> CylinderIntersectsRayPreNormalized(
    const Cylinder& cylinder, const GenericRay<T>& ray);

// Test if a cone intersects a ray.
template <typename T>
std::optional<RayIntersection<T>> ConeIntersectsRay(const Cone& cone,
                                                    const GenericRay<T>& ray);

// ConeIntersectsRay assuming the ray's direction is pre-normalized.
template <typename T>
std::optional<RayIntersection<T>> ConeIntersectsRayPreNormalized(
    const Cone& cone, const GenericRay<T>& ray);

// Calculates the distance from a point to a line (the line is represented by a
// Ray, but the sign of direction doesn't matter).
template <typename T>
DistanceFromShape<T> DistanceFromPointToLine(TVec3<T>& point,
                                             const GenericRay<T>&);

// Calculate the distance from a ray to a line segment, return the distance and
// the closest point on the line segment, if the line segment is not behind the
// ray.
template <typename T>
std::optional<DistanceFromShape<T>> RayDistanceFromLineSegment(
    const GenericRay<T>& ray, const LineSegment& line_segment);

template <typename T>
std::optional<T> ClosestPointOnRayToLine(const GenericRay<T>& ray,
                                         const GenericRay<T>& reference_line) {
  return ClosestPointOnRayToLinePreNormalized<T>(
      {ray.origin, normalize(ray.direction)},
      {reference_line.origin, normalize(reference_line.direction)});
}

template <typename T>
std::optional<T> ClosestPointOnRayToLinePreNormalized(
    const GenericRay<T>& ray, const GenericRay<T>& reference_line) {
  // The reference line.
  TVec3<T> a0 = reference_line.origin;
  TVec3<T> a = reference_line.direction;

  // The ray.
  TVec3<T> b0 = ray.origin;
  TVec3<T> b = ray.direction;

  // Calculate the "x" to minimize the distance of "b0+x*b" to Ray (a0,a) by:
  // Vector from b0+x*b to a0: v1 = (b0 + x * b - a0)
  // Rejection of v1 on (a0,a): v2 = v1 - dot(v1, a) * a
  // distance = ||v2||
  // find shortest distance by solving: d(distance^2)/dx = 0
  // therefore, "x = x_top / x_bottom" equals to the following:
  TVec3<T> A = b - dot(b, a) * a;
  T x_bottom = dot(A, A);
  if (x_bottom == 0) {
    // The line is parallel with the ray.
    return std::nullopt;
  }
  TVec3<T> c0 = b0 - a0;
  TVec3<T> B = c0 - dot(c0, a) * a;
  T x_top = -dot(A, B);
  return x_top / x_bottom;
}

template <typename T>
std::optional<RayIntersection<T>> AABBIntersectsRay(const imp::Box& box,
                                                    const GenericRay<T>& ray) {
  // Note: ray.direction must not be of almost zero length or nan.
  TVec3<T> min = TVec3<T>(box.center - box.halfExtent);
  TVec3<T> max = TVec3<T>(box.center + box.halfExtent);

  T t_min = -std::numeric_limits<T>::max();
  T t_max = std::numeric_limits<T>::max();
  RayIntersection<T> result;

  // In case of very thin box (but still not have non-zero volume), these arrays
  // will indicate which axis the normal is on.
  bool normal_axis_min[3] = {false, false, false};
  bool normal_axis_max[3] = {false, false, false};

  // Test each axis for intersection against a slab.
  for (int i = 0; i < 3; ++i) {
    T pos_i = ray.origin[i];
    T dir_i = ray.direction[i];

    // It is intentional to keep the check loose (using float epsilon even in
    // double precision). This is because rotation is loaded/stored/dealed with
    // in float precision, which sometimes will accidentally make rays having a
    // small value of float epsilon in the direction that is actually zero.
    if (std::abs(dir_i) < ::imp::kFltEpsilon) {
      // Indicates ray is parallel to the slab, so check if origin is inside
      // the slab.
      if (pos_i < min[i] || pos_i > max[i]) {
        return absl::nullopt;
      }
    } else {
      // Turn division into multiplication.
      T distance_divisor = T(1) / dir_i;

      // Find values for t of equation Ray(t) = O + t*d for the min and max
      // values of the slab.
      T t1 = (min[i] - pos_i) * distance_divisor;
      T t2 = (max[i] - pos_i) * distance_divisor;

      if (t1 > t2) {
        std::swap(t1, t2);
      }

      // Intersection 't' for the slab interval.
      if (t1 > t_min) {
        t_min = t1;
        normal_axis_min[0] = normal_axis_min[1] = normal_axis_min[2] = false;
        normal_axis_min[i] = true;
      }
      if (t2 < t_max) {
        t_max = t2;
        normal_axis_max[0] = normal_axis_max[1] = normal_axis_max[2] = false;
        normal_axis_max[i] = true;
      }

      if (t_min > t_max) {
        return std::nullopt;
      }
    }
  }

  bool hit_inside = false;
  if (t_max < T(0)) {
    // The ray starts outside the box and points away from the box.
    return std::nullopt;
  } else if (t_min < T(0)) {
    // The ray starts inside the box.
    hit_inside = true;
  }

  // Output intersection point and distance.
  result.distance = hit_inside ? t_max : t_min;
  result.collision_point = ray.GetPointAt(result.distance);
  for (int i = 0; i < 3; ++i) {
    // Degenerate case: the box is the box's half extent is smaller than the
    // epsilon.
    if (abs(box.halfExtent[i]) < std::numeric_limits<T>::epsilon()) {
      result.normal = TVec3<T>{0, 0, 0};
      result.normal[i] = ray.direction[i] > 0 ? -1 : 1;
      return result;
    }

    T delta = result.collision_point[i] - box.center[i];
    // Uses is_normal_axis to find the normal axis, in the case that the box's
    // half extent is too small compared to the ray's origin, which makes the
    // collision point less accurate.
    bool is_normal_axis = hit_inside ? normal_axis_max[i] : normal_axis_min[i];
    if (abs(abs(delta) - box.halfExtent[i]) <
            std::numeric_limits<T>::epsilon() ||
        is_normal_axis) {
      // The collision point is on a face perpendicular to the axis.
      result.normal[i] = (delta > T(0) ? 1 : -1) * (hit_inside ? -1 : 1);
    } else {
      result.normal[i] = 0;
    }
  }
  result.normal = normalize(result.normal);

  return result;
}

template <typename T>
Result PlaneIntersectsRay(const GenericPlane<T>& plane,
                          const GenericRay<T>& ray, TVec3<T>* out_col_point) {
  T divisor = dot(plane.normal, ray.direction);

  if (abs(divisor) < imp::kFltEpsilon) {
    return Result::kDoesNotIntersect;
  }

  T t = (plane.distance - dot(plane.normal, ray.origin)) / divisor;

  if (t < 0) {
    return Result::kDoesNotIntersect;
  }

  *out_col_point = ray.origin + t * ray.direction;
  return Result::kDoesIntersect;
}

template <typename T>
Result SphereIntersectsRay(const Sphere& sphere, const GenericRay<T>& ray,
                           T* out_intersection_dist, TVec3<T>* out_col_point) {
  // See 5.3.2 Intersecting Ray or Segment Against Sphere Ericson, Christer.
  // Real-Time Collision Detection (p. 177)
  // ----[sphere surface]-----(in sphere)-----[sphere surface]------------->
  // zone 1      |               zone 2              |             zone 3

  TVec3<T> d = normalize(ray.direction);
  TVec3<T> m = ray.origin - sphere.center;
  T dm = dot(d, m);
  T mm = dot(m, m);
  T dist2 = mm - sphere.radius * sphere.radius;
  if (dist2 > 0 && dm > 0) {
    return Result::kDoesNotIntersect;
  }

  T descriminant = dm * dm - dist2;
  if (descriminant < 0) {
    return Result::kDoesNotIntersect;
  }

  // Take the smallest 't' value.
  T t = -dm - std::sqrt(descriminant);

  // t < 0 means the ray starts in the sphere (in zone 2) or outside the sphere
  // (in zone 3) and moves away from the sphere.
  if (t < 0) {
    t = -dm + std::sqrt(descriminant);
  }

  // t < 0 means the ray starts outside the sphere (in zone 3) and moves away
  // from the sphere.
  if (t < 0) {
    return Result::kDoesNotIntersect;
  }

  *out_intersection_dist = t / length(ray.direction);
  *out_col_point = ray.origin + t * d;
  return Result::kDoesIntersect;
}

template <typename T>
Result TriangleIntersectsRay(const Triangle& triangle, const GenericRay<T>& ray,
                             bool collide_with_back_faces,
                             T* out_intersection_dist, TVec3<T>* out_col_point,
                             TVec3<T>* out_normal) {
  // Note: This function uses the Möller–Trumbore intersection algorithm.
  // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm

  // Extract the triangle normal (assumes open-gl default counter-clockwise
  // winding order).
  TVec3<T> edge1 = triangle.p1 - triangle.p0;
  TVec3<T> edge2 = triangle.p2 - triangle.p0;
  TVec3<T> ray_cross_e2 = cross(ray.direction, edge2);
  T det = dot(edge1, ray_cross_e2);

  // Triangle is parallel to the ray
  if (det > -imp::kFltEpsilon && det < imp::kFltEpsilon) {
    return Result::kDoesNotIntersect;
  }

  // Early out if a collision with the back of the triangle isn't wanted
  if (!collide_with_back_faces && det < imp::kFltEpsilon) {
    return Result::kDoesNotIntersect;
  }

  T inv_det = 1.0 / det;

  TVec3<T> s = ray.origin - triangle.p0;

  T u = inv_det * dot(s, ray_cross_e2);

  if (u < 0 || u > 1) {
    return Result::kDoesNotIntersect;
  }

  TVec3<T> s_cross_e1 = cross(s, edge1);
  float v = inv_det * dot(ray.direction, s_cross_e1);

  if (v < 0 || u + v > 1) {
    return Result::kDoesNotIntersect;
  }

  float t = inv_det * dot(edge2, s_cross_e1);

  // This means that there is a line intersection but not a ray intersection.
  if (t < imp::kFltEpsilon) {
    return Result::kDoesNotIntersect;
  }

  // Output the collision point.
  *out_col_point = TVec3<T>(ray.origin + ray.direction * t);

  // Output the distance between collision point and ray origin.
  *out_intersection_dist = t;

  // Output the collision triangle normal.
  *out_normal = normalize(cross(edge1, edge2));

  return Result::kDoesIntersect;
}

template <typename T>
std::optional<MeshIntersection<T>> MeshIntersectsRay(
    MeshVertexAndIndexData mesh, const GenericRay<T>& ray,
    bool collide_with_back_faces, std::optional<MeshRange> sub_mesh_range) {
  MeshVertexData* vertex_data = mesh.vertex_data;
  MeshIndexData* index_data = mesh.index_data;
  MeshIntersection<T> intersection;

  // Distance between nearest hit point and ray origin, if hit point exist.
  T t_dist = std::numeric_limits<T>::max();
  Result final_result = Result::kDoesNotIntersect;

  size_t triangle_count = index_data->GetDescription().index_count / 3;

  absl::Span<uint16_t> indices16;
  absl::Span<uint32_t> indices32;
  if (index_data->GetDescription().index_type ==
      MeshDescription::IndexType::USHORT) {
    indices16 = index_data->Indices<uint16_t>();
  } else {
    indices32 = index_data->Indices<uint32_t>();
  }

  size_t triangle_offset = 0;
  if (sub_mesh_range.has_value()) {
    triangle_offset = sub_mesh_range->offset / 3;
    triangle_count = sub_mesh_range->count / 3;
  }

  const VertexFormat& vertex_format =
      vertex_data->GetDescription().vertex_format;
  size_t position_attribute_offset = vertex_format.GetAttributeOffsetAt(
      vertex_format
          .GetIndexForAttribute(VertexFormat::VertexAttribute::POSITION)
          .value());
  for (size_t i = triangle_offset; i < triangle_count + triangle_offset; i++) {
    size_t index0, index1, index2;
    if (index_data->GetDescription().index_type ==
        MeshDescription::IndexType::USHORT) {
      index0 = static_cast<size_t>(indices16[3 * i]);
      index1 = static_cast<size_t>(indices16[3 * i + 1]);
      index2 = static_cast<size_t>(indices16[3 * i + 2]);
    } else {
      index0 = static_cast<size_t>(indices32[3 * i]);
      index1 = static_cast<size_t>(indices32[3 * i + 1]);
      index2 = static_cast<size_t>(indices32[3 * i + 2]);
    }

    const Triangle tri(vertex_data->VertexAttributeAt<float3>(
                           index0, position_attribute_offset),
                       vertex_data->VertexAttributeAt<float3>(
                           index1, position_attribute_offset),
                       vertex_data->VertexAttributeAt<float3>(
                           index2, position_attribute_offset));

    TVec3<T> col_point;
    T intersection_dist;
    TVec3<T> col_normal;
    Result result =
        TriangleIntersectsRay(tri, ray, collide_with_back_faces,
                              &intersection_dist, &col_point, &col_normal);

    if (result == Result::kDoesIntersect && intersection_dist < t_dist) {
      final_result = Result::kDoesIntersect;
      intersection.collision_point = col_point;
      intersection.intersection_dist = intersection_dist;
      t_dist = intersection_dist;
      intersection.triangle_id = i;
      intersection.collision_normal = col_normal;
    }
  }
  if (final_result == Result::kDoesIntersect) {
    return intersection;
  } else {
    return std::nullopt;
  }
}

template <typename T>
std::optional<MeshIntersection<T>> MeshIntersectsRay(
    MeshData* mesh, const GenericRay<T>& ray, bool collide_with_back_faces,
    std::optional<MeshRange> sub_mesh_range) {
  return MeshIntersectsRay({mesh->GetVertexData(), mesh->GetIndexData()}, ray,
                           collide_with_back_faces, sub_mesh_range);
}

template <typename T>
std::optional<MultiPrimitiveMeshIntersection<T>>
MultiPrimitiveMeshIntersectsRay(absl::Span<const MeshVertexAndIndexData> meshes,
                                const GenericRay<T>& ray,
                                bool collide_with_back_faces) {
  MultiPrimitiveMeshIntersection<T> intersection;
  T t_dist = std::numeric_limits<T>::max();
  Result final_result = Result::kDoesNotIntersect;

  for (size_t i = 0; i < meshes.size(); i++) {
    std::optional<MeshIntersection<T>> result =
        MeshIntersectsRay(meshes[i], ray, collide_with_back_faces);

    if (result.has_value() && result->intersection_dist < t_dist) {
      final_result = Result::kDoesIntersect;
      intersection.collision_point = result->collision_point;
      intersection.primitive_id = i;
      intersection.triangle_id = result->triangle_id;
      intersection.intersection_dist = result->intersection_dist;
      intersection.collision_normal = result->collision_normal;
      t_dist = result->intersection_dist;
    }
  }
  if (final_result == Result::kDoesIntersect) {
    return intersection;
  } else {
    return std::nullopt;
  }
}

template <typename T>
CylinderIntersectionInfo<T> CylinderLateralSurfaceIntersectsRay(
    GenericCylinder<T> cylinder, const GenericRay<T>& ray) {
  TVec3<T> up = normalize(cylinder.up);
  TVec3<T> forward = normalize(cylinder.forward);
  TVec3<T> center = cylinder.center;
  T length = cylinder.length;
  T radius = cylinder.radius;

  // The ray is parallel with the cylinders up direction. No intersection.
  if (AlmostEqual(up, ray.direction) || AlmostEqual(up, -ray.direction)) {
    return CylinderIntersectionInfo<T>{};
  }

  TVec3<T> c1c = dot((center - ray.origin), up) * up;
  TVec3<T> center1 = center - c1c;
  TVec3<T> oc1 = center1 - ray.origin;
  TVec3<T> ray_direction_projection =
      normalize(ray.direction - dot(ray.direction, up) * up);
  TVec3<T> c1c2 =
      dot(oc1, ray_direction_projection) * ray_direction_projection - oc1;
  TVec3<T> center2 = c1c2 + center1;

  // The ray misses the cylinder from side.
  if (norm(c1c2) > radius) {
    return CylinderIntersectionInfo<T>{};
  }

  T half_chrod = sqrt(radius * radius - dot(c1c2, c1c2));
  TVec3<T> a1 = center2 - half_chrod * ray_direction_projection;
  TVec3<T> a2 = center2 + half_chrod * ray_direction_projection;

  T cos_theta = dot(ray_direction_projection, ray.direction);
  T tan_theta = sqrt(1 - cos_theta * cos_theta) / cos_theta;

  auto get_intersection = [&](TVec3<T> a) {
    std::optional<CylinderIntersection<T>> result;

    // The intersection is on the opposite direction of the ray.
    if (dot(a - ray.origin, ray.direction) < 0) {
      result = std::nullopt;
      return result;
    }

    T height = norm(a - ray.origin) * tan_theta *
               (dot(ray.direction, up) > 0 ? 1 : -1);
    T height1 = height + norm(c1c);
    // The ray doesn't miss the cylinder from up or down.
    if (height1 >= 0 && height1 <= length) {
      TVec3<T> c1a = normalize(a - center1);
      T angle = dot(c1a, forward);
      result = CylinderIntersection<T>{
          .intersection = height * up + a,
          .height = height1,
          .arc_length =
              dot(cross(c1a, forward), up) < 0 ? acos(angle) : -acos(angle)};
    } else {
      result = std::nullopt;
    }
    return result;
  };

  auto result =
      CylinderIntersectionInfo<T>{.enter_cylinder = get_intersection(a1),
                                  .exit_cylinder = get_intersection(a2)};
  if (result.enter_cylinder.has_value() && result.exit_cylinder.has_value() &&
      AlmostEqual<T>(half_chrod, 0.0)) {
    result.tangent = true;
  }

  return result;
}

template <typename T>
std::optional<T> RayIntersectPlane(const GenericRay<T>& ray, const TVec3<T>& n,
                                   const TVec3<T>& p0) {
  // Assuming vectors are all normalized
  T denom = dot(n, ray.direction);
  if (abs(denom) > kFltEpsilon) {
    TVec3<T> p0r0 = p0 - ray.origin;
    T t = dot(p0r0, n) / denom;
    if (t >= 0) {
      return t;
    }
  }
  return std::nullopt;
}

template <typename T>
std::optional<TVec3<T>> RayIntersectDisk(const GenericRay<T>& ray,
                                         const TVec3<T>& n, const TVec3<T>& p0,
                                         T radius) {
  std::optional<T> t = RayIntersectPlane(ray, n, p0);
  if (t.has_value()) {
    // Calculate intersection point
    TVec3<T> p = ray.origin + ray.direction * t.value();
    // Vector from disk center to intersection point
    TVec3<T> v = p - p0;
    // Squared distance from disk center to intersection point
    T d2 = dot(v, v);
    T radius_squared = radius * radius;
    if (d2 < radius_squared || AlmostEqual<T>(d2, radius_squared)) {
      return p;
    }
  }
  return std::nullopt;
}

template <typename T>
std::optional<RayIntersection<T>> CylinderIntersectsRay(
    const Cylinder& cylinder, const GenericRay<T>& ray) {
  T length_ray_direction_squared = dot(ray.direction, ray.direction);
  if (AlmostEqual<T>(length_ray_direction_squared, 1)) {
    return CylinderIntersectsRayPreNormalized<T>(cylinder, ray);
  }
  std::optional<RayIntersection<T>> result = CylinderIntersectsRayPreNormalized(
      cylinder, GenericRay<T>{ray.origin, normalize(ray.direction)});
  if (result.has_value()) {
    result->distance /= sqrt(length_ray_direction_squared);
  }
  return result;
}

template <typename T>
std::optional<RayIntersection<T>> CylinderIntersectsRayPreNormalized(
    const Cylinder& cylinder, const GenericRay<T>& ray) {
  // Adjust the ray origin so that the cylinder base is on the zx-plane at y = 0
  TVec3<T> origin = ray.origin - cylinder.base;
  TVec3<T> direction = ray.direction;

  const T r_squared = cylinder.radius * cylinder.radius;
  const T origin_xz_squared = origin.x * origin.x + origin.z * origin.z;
  const bool ray_start_on_cylinder_side =
      AlmostEqual<T>(origin_xz_squared, r_squared);
  const bool ray_start_on_cylinder_base = AlmostEqual<T>(origin.y, 0);
  const bool ray_start_on_cylinder_top =
      AlmostEqual<T>(origin.y, cylinder.height);
  const bool ray_inside_infinite_cylinder = origin_xz_squared < r_squared;
  const bool ray_start_on_cylinder_base_or_top =
      ray_start_on_cylinder_base || ray_start_on_cylinder_top;
  const bool ray_start_between_cylinder_base_and_top =
      origin.y > 0 && origin.y < cylinder.height;
  const bool ray_start_in_cylinder =
      ray_start_between_cylinder_base_and_top && ray_inside_infinite_cylinder;

  // Check the edge cases first: when the ray starts on cylinder boundaries
  // the hit point is at the ray origin.
  if (ray_inside_infinite_cylinder) {
    if (ray_start_on_cylinder_base_or_top) {
      collision::RayIntersection<T> result;
      result.collision_point = ray.origin;
      result.distance = 0;
      result.normal = ray_start_on_cylinder_top ? kUp : kDown;
      return result;
    }  // else the ray starts strictly inside the cylinder so fall through.
  } else if (ray_start_on_cylinder_side &&
             (ray_start_between_cylinder_base_and_top ||
              ray_start_on_cylinder_base_or_top)) {
    collision::RayIntersection<T> result;
    result.collision_point = ray.origin;
    result.distance = 0;
    result.normal = normalize(TVec3<T>(origin.x, 0, origin.z));
    return result;
  }

  // Ray does not start on a cylinder boundary, but either inside or outside.

  // Assume the base cap is closer for the ray to intersect for now.
  float3 cylinder_top = cylinder.base + cylinder.height * kUp;
  const float3* closer_cap_center = &cylinder.base;

  RayIntersection<T> result;

  std::optional<T> t1 = ClosestPointOnRayToLinePreNormalized<T>(
      ray, GenericRay<T>(cylinder.base, kUp));

  if (t1.has_value()) {
    // p1 is the closest point on the ray to the cylinder's center line.
    TVec3<T> p1 = origin + t1.value() * direction;
    T p1_xz_squared = p1.x * p1.x + p1.z * p1.z;

    if (p1_xz_squared > r_squared) {
      // The ray doesn't intersect the infinite cylinder.
      return std::nullopt;
    }

    // t is the distance from the ray origin to the intersection between the
    // ray and the infinite cylinder.
    T t = t1.value() -
          sqrt((r_squared - p1_xz_squared) /
               (direction.x * direction.x + direction.z * direction.z)) *
              (ray_start_in_cylinder ? -1 : 1);
    // l is the distance from the adjusted ray-cylinder intersection to the
    // zx-plane at y = 0.
    T l = origin.y + t * direction.y;

    // Return if the intersection is on the capped cylinder.
    if (l >= 0.0 && l <= cylinder.height) {
      result.collision_point = ray.origin + t * direction;
      result.distance = t;
      result.normal =
          normalize(TVec3<T>(result.collision_point.x - cylinder.base.x, 0,
                             result.collision_point.z - cylinder.base.z) *
                    (ray_start_in_cylinder ? -1 : 1));
      return result;
    }

    // No intersection on the side. Check if the ray intersects the caps.

    if (AlmostEqual<T>(direction.y, 0)) {
      // The ray is parallel to the caps of the cylinder.
      return std::nullopt;
    }

    // Adjust which cap is closer for the ray to intersect and the distance t.
    if (l > 0) {
      t = (cylinder.height - origin.y) / direction.y;
      closer_cap_center = &cylinder_top;
    } else {
      t = -origin.y / direction.y;
    }
    result.collision_point = ray.origin + t * direction;
    result.distance = t;

    // Check if the ray actually intersects the closer cap.
    TVec3<T> p = result.collision_point - *closer_cap_center;
    if (p.x * p.x + p.z * p.z > r_squared) {
      return std::nullopt;
    }

  } else {
    // The ray is parallel to the cylinder center line.
    // Adjust which cap is closer for the ray to intersect.
    if (!ray_start_in_cylinder) {
      if (origin.y > 0) {
        closer_cap_center = &cylinder_top;
      }
    } else {
      if (direction.y > 0) {
        closer_cap_center = &cylinder_top;
      }
    }
    std::optional<TVec3<T>> cap_intersection =
        RayIntersectDisk<T>(ray, kUp, *closer_cap_center, cylinder.radius);
    if (!cap_intersection.has_value()) {
      return std::nullopt;
    }
    result.collision_point = cap_intersection.value();
    result.distance = length(result.collision_point - ray.origin);
  }

  const bool is_base_cap = closer_cap_center == &cylinder.base;
  if (ray_start_in_cylinder) {
    result.normal = is_base_cap ? kUp : kDown;
  } else {
    result.normal = is_base_cap ? kDown : kUp;
  }
  return result;
}

template <typename T>
std::optional<RayIntersection<T>> RayIntersectsConeSidePreNormalized(
    const GenericRay<T>& ray, const Cone& cone) {
  // we use vectors from the tip of the cone in the calculations below, see
  // (broken link)/

  const TVec3<T> tip = cone.base + cone.height * kUp;
  const TVec3<T> cone_normal = kDown;
  const TVec3<T> co = ray.origin - tip;  // vector from cone tip to ray origin
  const T height_squared = cone.height * cone.height;
  const T cos_theta_squared =
      height_squared / (height_squared + cone.radius * cone.radius);
  const T dot_d_n = dot(ray.direction, cone_normal);
  const T dot_co_n = dot(co, cone_normal);
  const T dot_co_n_squared = dot_co_n * dot_co_n;
  const T dot_co_co = dot(co, co);

  // use a quadratic equation ax^2 + bx + c = 0 and solve for x
  // where x is t, the distance from the ray origin to the intersection on the
  // cone side
  T a = dot_d_n * dot_d_n - cos_theta_squared;
  T b = 2 * (dot_d_n * dot_co_n - dot(ray.direction, co) * cos_theta_squared);
  T c = dot_co_n_squared - dot_co_co * cos_theta_squared;

  // adjust for float math precision errors
  if (RoughlyEqual<T>(a, 0)) {
    a = 0;
  }
  if (RoughlyEqual<T>(b, 0)) {
    b = 0;
  }
  if (RoughlyEqual<T>(c, 0)) {
    c = 0;
  }

  // determinant, aka discriminant of the quadratic equation
  T det = b * b - 4 * a * c;
  if (RoughlyEqual<T>(det, 0)) {
    det = 0;
  }

  if (det < 0) {
    return std::nullopt;
  }

  bool hit_found = false;
  TVec3<T> cp;  // vector from the cone tip to the intersection point
  T t;          // result distance
  T h;          // result vertical distance used for normal calculation

  T t1, t2;  // candidate solutions to evaluate when >= 0

  if (a != 0) {
    det = sqrt(det);
    t1 = (-b - det) / (2 * a);
    t2 = (-b + det) / (2 * a);
    if (t1 > t2) {
      std::swap(t1, t2);  // make t1 <= t2 so we can skip eval t2 if t1 is a hit
    }
  } else if (b != 0) {
    // one candidate solution
    t1 = -c / b;
    t2 = -1;
  } else if (c == 0) {
    // infinitely many solutions, clamp distance to tip or base if ray origin is
    // outside the cone
    hit_found = true;
    t = sqrt(dot_co_co);
    h = dot_co_n;
    bool recalc_h = true;
    if (h > cone.height) {
      // ray origin is below the cone, clamp distance to base
      t -= t * cone.height / h;
    } else if (h > 0) {
      // ray origin is on the side of the cone
      t = 0;
    } else {
      // ray origin is above the tip of the cone, no clamping
      recalc_h = false;
    }
    cp = co + t * ray.direction;
    if (recalc_h) {
      h = dot(cp, cone_normal);
    }
  } else {
    // no solution
    return std::nullopt;
  }

  if (!hit_found && t1 >= 0.0) {
    cp = co + t1 * ray.direction;
    h = dot(cp, cone_normal);
    if ((h > 0.0 && h <= cone.height) || RoughlyEqual<T>(h, 0)) {
      hit_found = true;
      t = t1;
    }
  }

  if (!hit_found && t2 >= 0.0) {
    cp = co + t2 * ray.direction;
    h = dot(cp, cone_normal);
    if ((h > 0.0 && h <= cone.height) || RoughlyEqual<T>(h, 0)) {
      hit_found = true;
      t = t2;
    }
  }

  if (!hit_found) {
    return std::nullopt;
  }

  bool ray_origin_inside_cone = false;
  if (dot_co_n > 0) {
    const T cos_co_n_squared = dot_co_n_squared / dot_co_co;
    if (cos_co_n_squared > cos_theta_squared &&
        !AlmostEqual(cos_co_n_squared, cos_theta_squared)) {
      ray_origin_inside_cone = true;
    }
  }

  RayIntersection<T> result;
  result.collision_point = cp + tip;
  result.distance = t;
  if (AlmostEqual(result.collision_point, tip)) {
    result.normal = ray_origin_inside_cone ? kDown : kUp;
  } else {
    if (ray_origin_inside_cone) {
      result.normal = normalize(cone_normal - cp * h / dot(cp, cp));
    } else {
      result.normal = normalize(cp * h / dot(cp, cp) - cone_normal);
    }
  }

  return result;
}

template <typename T>
std::optional<RayIntersection<T>> ConeIntersectsRay(const Cone& cone,
                                                    const GenericRay<T>& ray) {
  T length_ray_direction_squared = dot(ray.direction, ray.direction);
  if (AlmostEqual<T>(length_ray_direction_squared, 1)) {
    return ConeIntersectsRayPreNormalized<T>(cone, ray);
  }
  std::optional<RayIntersection<T>> result = ConeIntersectsRayPreNormalized<T>(
      cone, GenericRay<T>{ray.origin, normalize(ray.direction)});
  if (result.has_value()) {
    result->distance /= sqrt(length_ray_direction_squared);
  }
  return result;
}

template <typename T>
std::optional<RayIntersection<T>> ConeIntersectsRayPreNormalized(
    const Cone& cone, const GenericRay<T>& ray) {
  // Check if the ray intersects the cone side and base and return the closer
  // intersection.
  std::optional<RayIntersection<T>> side_intersection =
      RayIntersectsConeSidePreNormalized<T>(ray, cone);

  std::optional<TVec3<T>> base_intersection_point =
      RayIntersectDisk<T>(ray, kUp, cone.base, cone.radius);

  if (base_intersection_point.has_value()) {
    collision::RayIntersection<T> result;
    result.distance = norm(base_intersection_point.value() - ray.origin);
    if (!side_intersection.has_value() ||
        result.distance < side_intersection->distance) {
      result.collision_point = base_intersection_point.value();
      result.normal = ray.origin.y <= cone.base.y ? kDown : kUp;
      return result;
    }
  }

  return side_intersection;
}

template <typename T>
DistanceFromShape<T> DistanceFromPointToLine(TVec3<T>& point,
                                             const GenericRay<T>& line) {
  TVec3<T> op = point - line.origin;
  TVec3<T> direction = normalize(line.direction);
  T t = dot(op, direction);
  return DistanceFromShape<T>{
      .distance = norm(point - (line.origin + direction * t)),
      .closest_point = point};
}

/* Notation:
 * a is the end of line segment that is closer in the ray diresction.
 * b is the end of line segment that is further in the ray diresction.
 * p (if exists) is the intersection between the line segment and the normal
 * plane of the ray that passes the ray's origin.
 * q (if exists) is the point on pb that is closest to the ray.
 */
template <typename T>
std::optional<DistanceFromShape<T>> RayDistanceFromLineSegment(
    const GenericRay<T>& ray, const LineSegment& line_segment) {
  TVec3<T> ray_direction = normalize(ray.direction);
  TVec3<T> line_start = line_segment.line_start;
  TVec3<T> line_end = line_segment.line_end;

  // Distance from projection of line_start/end on the ray to the ray origin.
  T t1 = dot(line_start - ray.origin, ray_direction);
  T t2 = dot(line_end - ray.origin, ray_direction);

  // Ignore the part of line segment that is behind the ray's normal plane.
  std::optional<LineSegment> truncated_line_segment;
  if (t1 * t2 <= 0) {
    TVec3<T> a = line_start;
    TVec3<T> b = line_end;
    TVec3<T> p;
    if (t1 > 0 && t2 < 0) {
      a = line_end;
      b = line_start;
      p = a - t2 / (t1 - t2) * (b - a);
    } else {
      p = a - t1 / (t2 - t1) * (b - a);
    }
    truncated_line_segment = LineSegment(p, b);
  } else if (t1 >= 0 && t2 >= 0) {
    if (t1 > t2) {
      truncated_line_segment = LineSegment(line_end, line_start);
    } else {
      truncated_line_segment = line_segment;
    }
  }

  if (!truncated_line_segment.has_value()) {
    return std::nullopt;
  }

  DistanceFromShape<T> result = {.distance = std::numeric_limits<T>::max()};

  TVec3<T> o = ray.origin;
  TVec3<T> d = ray_direction;
  TVec3<T> p = truncated_line_segment->line_start;
  TVec3<T> b = truncated_line_segment->line_end;
  TVec3<T> pb =
      truncated_line_segment->line_end - truncated_line_segment->line_start;
  TVec3<T> pb_n = normalize(pb);

  // calculate the "x" to minimize the distance of "p+x*pb_n" to Ray (o,d) by
  // vector from p+x*pb_n to o: v1 = (p + x * pb_n - o)
  // rejection of v1 on (o,d): v2 = v1 - dot(v1, o) * o
  // distance = ||v2||
  // for shortest distance: d(distance^2)/dx = 0
  // therefore, "x = x_top / x_bottom" equals to the following:
  TVec3<T> op = p - o;
  TVec3<T> A = pb_n - dot(pb_n, d) * d;
  TVec3<T> B = op - dot(op, d) * d;
  T x_top = -dot(A, B);
  T x_bottom = dot(A, A);

  if (x_bottom < kAngleThreshold) {
    // Ray (o, d) and LineSegment(p, b) is almost parallel
    DistanceFromShape<T> local = DistanceFromPointToLine(p, ray);
    if (local.distance < result.distance) {
      result = local;
    }
  } else {
    T x = x_top / x_bottom;
    if (x < 0) {
      DistanceFromShape<T> local = DistanceFromPointToLine(p, ray);
      if (local.distance < result.distance) {
        result = local;
      }
    } else if (x > norm(pb)) {
      DistanceFromShape<T> local = DistanceFromPointToLine(b, ray);
      if (local.distance < result.distance) {
        result = local;
      }
    } else {
      TVec3<T> q = p + x * pb_n;
      DistanceFromShape<T> local = DistanceFromPointToLine(q, ray);
      if (local.distance < result.distance) {
        result = local;
      }
    }
  }

  return result;
}

}  // namespace collision
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_COLLISION_HELPERS_H_
