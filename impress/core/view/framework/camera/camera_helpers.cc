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

#include "core/view/framework/camera/camera_helpers.h"

#include <cmath>
#include <limits>
#include <string>
#include <type_traits>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/variant.h"
#include "filament/libs/math/include/math/TVecHelpers.h"
#include "core/camera/camera_component.h"
#include "core/collision/ray.h"
#include "core/common/bit_flag.h"
#include "core/common/filament_helpers.h"
#include "core/common/robin_set.h"
#include "core/geometry/shapes/sphere.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/ncsb/path_manager.h"
#include "core/view/framework/assets/gltf_mesh.h"
#include "core/view/framework/assets/gltf_renderer.h"
// TODO: Update this to use the new camera_state.proto.imp.h
// location, when framework_cc_proto is split.
#include "core/camera/camera_state.proto.imp.h"
#include "core/view/framework/render/mesh_renderer.h"

namespace imp {

namespace {

constexpr float kEpsilon = 1e-5f;
constexpr float kMin = -std::numeric_limits<float>::max();
constexpr float kMax = std::numeric_limits<float>::max();

// TODO: Replace with built-in function after refactor is done.
// Returns the intersection point of a ray with a given plane.
// Note: a proper intersection point should check if the ray
// intersects with the plane at all. Since this is an internal helper
// function and we know the ray will always intersect the plane,
// we are skipping this check.
// We are solving for 't' based on the formula listed here:
// (broken link)
float3 IntersectionOfRayAndPlane(const Ray& ray, float3 plane_normal,
                                 float3 plane_point) {
  float3 norm_plane_normal = normalize(plane_normal);
  float t = (dot(plane_point, norm_plane_normal) -
             dot(ray.origin, norm_plane_normal)) /
            dot(ray.direction, norm_plane_normal);
  return ray.origin + (ray.direction * t);
}

Sphere GetNodeBoundingSphere(NodeHandle node) {
  float4 sphere;
  if (node->GetComponent<MeshRenderer>()) {
    sphere = TransformBounds(
                 node->GetComponent<MeshRenderer>()->GetMesh()->GetAabb(),
                 node->GetWorldTrs())
                 .getBoundingSphere();

  } else if (node->GetComponent<GltfMesh>()) {
    sphere =
        node->GetComponent<GltfMesh>()->GetWorldBounds().getBoundingSphere();
  } else if (node->GetComponent<GltfRenderer>()) {
    sphere = node->GetComponent<GltfRenderer>()
                 ->GetWorldBounds()
                 .getBoundingSphere();
  } else {
    sphere = {node->GetWorldPosition(), 0.0f};
  }
  return Sphere(sphere.xyz, sphere.w);
}

float GetRadius(NodeHandle node) { return GetNodeBoundingSphere(node).radius; }

bool HasOverlappingGroups(const RobinSet<std::string>& reference_groups,
                          NodeHandle node) {
  for (const std::string& group : node->GetGroups()) {
    if (reference_groups.contains(group)) {
      return true;
    }
  }
  return false;
}

absl::StatusOr<float3> GetIntersectionBetweenTwoRays(const Ray& a,
                                                     const Ray& b) {
  float3 normalized_a = normalize(a.direction);
  float3 normalized_b = normalize(b.direction);

  // Are the rays parallel? If yes, an intersection cannot exist.
  float3 plane_normal = cross(normalized_a, normalized_b);
  if (length(plane_normal) == 0)
    return absl::InvalidArgumentError(
        "Rays are parallel; point of intersection does not exist.");

  // Are the rays on the same plane? If no, an intersection cannot exist.
  float3 connecting_vector = b.origin - a.origin;
  if (dot(connecting_vector, plane_normal) > kEpsilon &&
      dot(connecting_vector, plane_normal) < -kEpsilon)
    return absl::InvalidArgumentError(
        "Rays do not lie on the same plane; "
        "point of intersection does not exist.");

  // Check if the length along ray a and the length along ray b towards
  // the point of intersection are both positive.
  // If the length along any ray is negative, then the intersection point
  // happens behind the origin point of a ray, making the intersection invalid.
  float cross_area_squared = pow(norm(plane_normal), 2);
  volatile float a_length =
      dot(cross(connecting_vector, normalized_b), plane_normal) /
      cross_area_squared;
  volatile float b_length =
      dot(cross(connecting_vector, normalized_a), plane_normal) /
      cross_area_squared;
  if (a_length <= 0 || b_length <= 0)
    return absl::InvalidArgumentError(
        "Rays do not cross each other; "
        "point of intersection does not exist.");

  // Calculate and return the intersection point.
  return a.origin + a_length * normalized_a;
}

// Finds the plane that sits between lower_bound and upper_bound,
// and returns a point of the plane as its representation.
float3 GetBisectingPlanePoint(float3 bisecting_ray, const Ray& lower_bound,
                              const Ray& upper_bound) {
  float3 plane_point = (lower_bound.origin + upper_bound.origin) * 0.5f;
  bool is_upper_bound_closer =
      dot(plane_point - upper_bound.origin, bisecting_ray) < 0;
  Ray closer_bound = is_upper_bound_closer ? upper_bound : lower_bound;
  Ray further_bound = is_upper_bound_closer ? lower_bound : upper_bound;
  further_bound.origin = IntersectionOfRayAndPlane(further_bound, bisecting_ray,
                                                   closer_bound.origin);
  return (further_bound.origin + closer_bound.origin) * 0.5f;
}

// Shifts a ray along the normal of the plane that the ray rests on,
// and then returns its new origin point. Ray must be parallel to the plane.
absl::StatusOr<float3> SnapRayToParallelPlane(float3 plane_normal,
                                              float3 new_plane_point,
                                              Ray& ray) {
  if ((abs(dot(plane_normal, ray.direction)) > kEpsilon)) {
    return absl::InvalidArgumentError("Ray is not parallel to the plane.");
  }
  bool is_positive_shift = dot(new_plane_point - ray.origin, plane_normal);
  float3 shift = is_positive_shift ? plane_normal : -plane_normal;
  return IntersectionOfRayAndPlane({ray.origin, shift}, plane_normal,
                                   new_plane_point);
}

// Finds and returns a ray along the perimeter of a node's bounds,
// in the given direction from center to boundary.
// Uses is_horizontal to determine which fov of the camera to use,
// if the camera is a perspective camera.
Ray GetBoundingRay(ComponentHandle<CameraComponent> camera,
                   float3 node_position, float node_radius, float3 direction,
                   bool is_horizontal) {
  quatf camera_rotation = camera->GetNode()->GetWorldRotation();
  bool is_orthographic =
      camera->GetProjectionType() == CameraState::ProjectionType::ORTHOGRAPHIC;

  if (is_orthographic) {
    float3 local_bound_point =
        node_position + node_radius * (camera_rotation * direction);
    bool is_z_direction = length(cross(direction, kForward)) == 0;
    float3 ray_direction = is_z_direction ? kRight : kForward;
    return Ray(local_bound_point, camera_rotation * ray_direction);
  } else {
    // Find the vector that points from the center of the sphere
    // to the point at which the resulting ray intersects.
    float fov = is_horizontal ? camera->GetHorizontalFovInDegrees()
                              : camera->GetVerticalFovInDegrees();
    float half_fov = ToRadians(fov * 0.5f);
    float3 center_to_a =
        normalize((camera_rotation * direction) * cos(half_fov) +
                  (camera_rotation * kBack) * sin(half_fov));
    float3 ray_origin = node_position + center_to_a * node_radius;
    float3 z_direction = {-direction.y, direction.x, direction.z};
    float3 ray_direction =
        normalize(cross(center_to_a, camera_rotation * z_direction));
    return Ray(ray_origin, ray_direction);
  }
}

}  // namespace

bool HasValidMesh(NodeHandle node, CameraHelperOptions options) {
  if (!node) return false;

  bool ignore_visibility =
      CheckBit(options, CameraHelperOptions::kIncludeDisabled);
  if (auto renderer = node->GetComponent<MeshRenderer>()) {
    if ((ignore_visibility || renderer->IsActive()) &&
        renderer->GetMesh() != nullptr && GetRadius(node) > 0) {
      return true;
    }
  }

  if (CheckBit(options, CameraHelperOptions::kIncludeDescendants)) {
    if (auto gltf_mesh = node->GetComponent<GltfMesh>()) {
      if ((ignore_visibility || gltf_mesh->IsActive()) && GetRadius(node) > 0) {
        return true;
      }
    }
    for (NodeHandle child : node->GetChildren()) {
      if (HasValidMesh(child, options)) return true;
    }
  } else {
    // in the case where we are not including descendants, check for
    // gltf_renderer instead of gltf_mesh.
    if (auto gltf_renderer = node->GetComponent<GltfRenderer>()) {
      if ((ignore_visibility || gltf_renderer->IsActive()) &&
          GetRadius(node) > 0) {
        return true;
      }
    }
  }

  return false;
}

absl::Status MoveIntoView(ComponentHandle<CameraComponent> camera,
                          const ViewTarget& target,
                          CameraHelperOptions options) {
  // Check that there are valid nodes for the camera to look at.
  std::vector<NodeHandle> valid_nodes =
      FilterNodesViewableByCamera(camera, target, options);
  if (valid_nodes.empty()) {
    return absl::InvalidArgumentError("No nodes to work with.");
  }

  quatf camera_rotation = camera->GetNode()->GetWorldRotation();
  const float3 camera_forward = camera_rotation * kForward;
  const float3 camera_up = camera_rotation * kUp;
  const float3 camera_right = camera_rotation * kRight;
  bool is_orthographic =
      camera->GetProjectionType() == CameraState::ProjectionType::ORTHOGRAPHIC;

  // Find the boundary rays that encapsulate the target.
  std::vector<float> intersection_values = {kMax, kMax, kMin, kMin, kMax};
  std::vector<Ray> bounds(intersection_values.size(), Ray());
  for (NodeHandle node : valid_nodes) {
    //             <up>
    //           .- o -.
    //         /         \
    // <left> o     x     o <right>
    //         \         /
    //           `- o -`
    //            <down>
    // For each direction, calculate the ray that originates at each radial
    // point and determine if we should store it as a boundary ray.
    // For orthogonal cameras, we are also storing a fifth ray
    // for the point that is furthest back.
    Sphere node_bounds = GetNodeBoundingSphere(node);
    float node_radius = node_bounds.radius;
    float3 node_position = node_bounds.center;

    Ray left_ray =
        GetBoundingRay(camera, node_position, node_radius, kLeft, true);
    float3 left_intersection =
        IntersectionOfRayAndPlane(left_ray, camera_forward, kZero3);
    float left_intersection_value = dot(left_intersection, camera_right);
    if ((left_intersection_value < intersection_values[0])) {
      intersection_values[0] = left_intersection_value;
      // TODO:((broken link)) Debug the math, so we can just use the
      // newly calculated intersection point as the origin, regardless
      // of projection type.
      bounds[0] = left_ray;
      if (is_orthographic) {
        bounds[0].origin = left_intersection;
      }
    }

    Ray down_ray =
        GetBoundingRay(camera, node_position, node_radius, kDown, false);
    float3 down_intersection =
        IntersectionOfRayAndPlane(down_ray, camera_forward, kZero3);
    float down_intersection_value = dot(down_intersection, camera_up);
    if ((down_intersection_value < intersection_values[1])) {
      intersection_values[1] = down_intersection_value;
      bounds[1] = down_ray;
      if (is_orthographic) {
        bounds[1].origin = down_intersection;
      }
    }

    Ray right_ray =
        GetBoundingRay(camera, node_position, node_radius, kRight, true);
    float3 right_intersection =
        IntersectionOfRayAndPlane(right_ray, camera_forward, kZero3);
    float right_intersection_value = dot(right_intersection, camera_right);
    if ((right_intersection_value > intersection_values[2])) {
      intersection_values[2] = right_intersection_value;
      bounds[2] = right_ray;
      if (is_orthographic) {
        bounds[2].origin = right_intersection;
      }
    }

    Ray up_ray = GetBoundingRay(camera, node_position, node_radius, kUp, false);
    float3 up_intersection =
        IntersectionOfRayAndPlane(up_ray, camera_forward, kZero3);
    float up_intersection_value = dot(up_intersection, camera_up);
    if ((up_intersection_value > intersection_values[3])) {
      intersection_values[3] = up_intersection_value;
      bounds[3] = up_ray;
      if (is_orthographic) {
        bounds[3].origin = up_intersection;
      }
    }

    // If the camera is orthographic, we also want to find and store
    // the point that is furthest back.
    if (is_orthographic) {
      Ray back_ray =
          GetBoundingRay(camera, node_position, node_radius, kBack, false);
      float3 back_intersection =
          IntersectionOfRayAndPlane(back_ray, camera_right, kZero3);
      float back_intersection_value = dot(back_intersection, camera_forward);
      if ((back_intersection_value < intersection_values[4])) {
        intersection_values[4] = back_intersection_value;
        bounds[4] = back_ray;
      }
    }
  }

  // Calculate the midpoint between each set of rays. This will represent
  // the plane we wish to shift the boundary rays onto.
  float3 bisecting_ray = camera_forward;
  float3 vertical_center_plane_point =
      GetBisectingPlanePoint(bisecting_ray, bounds[0], bounds[2]);
  float3 horizontal_center_plane_point =
      GetBisectingPlanePoint(bisecting_ray, bounds[1], bounds[3]);

  // For each boundary ray, shift its origin so it lies on the plane
  // that is between it and its partner ray.
  // This will allow the ray to intersect with its partner in the next step.
  bounds[0].origin = SnapRayToParallelPlane(
                         camera_up, horizontal_center_plane_point, bounds[0])
                         .value();
  bounds[1].origin = SnapRayToParallelPlane(
                         camera_right, vertical_center_plane_point, bounds[1])
                         .value();
  bounds[2].origin = SnapRayToParallelPlane(
                         camera_up, horizontal_center_plane_point, bounds[2])
                         .value();
  bounds[3].origin = SnapRayToParallelPlane(
                         camera_right, vertical_center_plane_point, bounds[3])
                         .value();

  // Find new position (and orthographic scale if needed),
  // then assign to camera.
  if (is_orthographic) {
    // Calculate the aspect ratio of the target bounds.
    float x_scale = length(bounds[2].origin - bounds[0].origin);
    float y_scale = length(bounds[3].origin - bounds[1].origin);
    float aspect_ratio_target = x_scale / y_scale;

    // Calculate the aspect ratio of the camera.
    mat4f camera_matrix = camera->GetProjectionMatrix();
    float camera_matrix_width = 2.0f / camera_matrix[0][0];
    float camera_matrix_height = 2.0f / camera_matrix[1][1];
    float aspect_ratio_camera = camera_matrix_width / camera_matrix_height;

    // Determine which scale to use by comparing the two aspect ratios.
    float new_scale = (abs(aspect_ratio_target) > abs(aspect_ratio_camera))
                          ? x_scale
                          : y_scale;
    if (new_scale == 0) {
      IMP_LOG(imp::FATAL) << "Calculated orthographic scale is invalid.";
    }
    camera->SetOrthographicProjection(new_scale);

    // Calculate a new center that will ensure the camera
    // is both centered and behind all objects.
    float3 center = (bounds[2].origin + bounds[0].origin) / 2.0f;
    center =
        IntersectionOfRayAndPlane({center, camera_rotation * kBack},
                                  camera_rotation * kBack, bounds[4].origin);
    camera->GetNode()->SetWorldPosition(center);
  } else {
    // Find the horizontal and vertical intersection points.
    absl::StatusOr<float3> x_intersection =
        GetIntersectionBetweenTwoRays(bounds[0], bounds[2]);
    absl::StatusOr<float3> y_intersection =
        GetIntersectionBetweenTwoRays(bounds[1], bounds[3]);

    // Use the intersection point that is further away.
    // This will ensure that we get all objects within the view.
    if (x_intersection.ok() && y_intersection.ok()) {
      bool x_intersection_is_further =
          dot(x_intersection.value() - y_intersection.value(),
              camera_rotation * kBack) > 0;
      float3 new_position = x_intersection_is_further ? x_intersection.value()
                                                      : y_intersection.value();
      camera->GetNode()->SetWorldPosition(new_position);
    }
  }
  return absl::OkStatus();
}

std::vector<NodeHandle> FilterNodesViewableByCamera(
    ComponentHandle<CameraComponent> camera, const ViewTarget& target,
    CameraHelperOptions options) {
  std::vector<NodeHandle> nodes;
  std::vector<NodeHandle> valid_nodes;

  absl::visit(
      [&camera, &nodes](auto&& view_target) {
        using type = std::decay_t<decltype(view_target)>;
        if constexpr (std::is_same_v<type, NodeHandle>) {
          if (view_target) nodes = {view_target};
        } else if constexpr (std::is_same_v<type, std::vector<NodeHandle>>) {
          if (!view_target.empty()) {
            nodes = view_target;
          }
        } else if constexpr (std::is_same_v<type, absl::string_view>) {
          int num_nodes = camera->GetNode()
                              ->GetView()
                              .GetGroupsManager()
                              .GetNumNodesInGroup(view_target);
          nodes.reserve(num_nodes);
          camera->GetNode()
              ->GetView()
              .GetGroupsManager()
              .ForEachActiveNodeInGroup(view_target, [&nodes](NodeHandle node) {
                nodes.push_back(node);
              });
        }
      },
      target);

  RobinSet<std::string> camera_groups;
  for (const std::string& group : camera->GetNode()->GetGroups()) {
    camera_groups.insert(group);
  }

  // Retrieve the children of each node and add to 'nodes' vector.
  if (CheckBit(options, CameraHelperOptions::kIncludeDescendants)) {
    PathManager& path_manager = camera->GetView().GetPathManager();
    for (NodeHandle node : nodes) {
      std::vector<NodeHandle> subnodes = path_manager.GetDescendants(node);
      nodes.insert(nodes.end(), subnodes.begin(), subnodes.end());
    }
  }

  // Weed out nodes that are not valid.
  for (NodeHandle node : nodes) {
    // If we are ignoring inactive nodes, and the node is disabled, skip.
    if (!CheckBit(options, CameraHelperOptions::kIncludeDisabled) &&
        !node->IsActive()) {
      continue;
    }

    // Check for a valid renderer with mesh.
    // We want to make sure we only check the validity on the current node,
    // regardless of the original value of kIncludeDescendants.
    CameraHelperOptions single_node_check{
        ClearBit(options, CameraHelperOptions::kIncludeDescendants)};
    if (!HasValidMesh(node, single_node_check)) continue;

    // Check visibility group overlap. If the node's visibility groups
    // are not in the camera's visibility group, skip.
    if (!HasOverlappingGroups(camera_groups, node)) continue;

    // Node has passed all requirements. Add to valid nodes array.
    valid_nodes.push_back(node);
  }

  return valid_nodes;
}

}  // namespace imp
