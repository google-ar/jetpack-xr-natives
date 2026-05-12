// Copyright 2025 Google LLC
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

#include "core/physics/physics_debug_draw.h"

#include <cmath>
#include <utility>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Box.h"
#include "core/common/debug_draw.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"

namespace imp {
namespace physics_debug_draw {

constexpr float kDebugDrawSphereRadius = 0.05f;
constexpr float kDebugDrawAxisLength = 1.0f;
constexpr float kSegmentsPerDegree = 0.5f;
constexpr debug_draw::Color kDebugDrawDefaultColor =
    debug_draw::kDebugColors[static_cast<int>(debug_draw::DebugColor::kYellow)];

PhysicsDebugDraw::PhysicsDebugDraw() {
  SetColor(kDebugDrawDefaultColor);
  ResetTransform();
}

void PhysicsDebugDraw::SetColor(const debug_draw::Color& color) {
  debug_draw_color_ = color;
}

void PhysicsDebugDraw::ResetColor() {
  debug_draw_color_ = kDebugDrawDefaultColor;
}

debug_draw::Color PhysicsDebugDraw::GetDefaultColor() const {
  return kDebugDrawDefaultColor;
}

void PhysicsDebugDraw::SetTransform(const mat4f& transform) {
  debug_draw_transform_ = transform;
  debug_draw_transform_is_identity_ = false;
}

void PhysicsDebugDraw::ResetTransform() {
  debug_draw_transform_ = kIdentityMat4f;
  debug_draw_transform_is_identity_ = true;
}

void PhysicsDebugDraw::DrawPivotConnectedToOrigin(NodeHandle node,
                                                  const float3& pivot) {
  DrawPivotConnectionToOrigin(node, pivot);

  debug_draw::Local debug_local(node.GetEntity());
  if (!debug_draw_transform_is_identity_) {
    debug_local.Transform(debug_draw_transform_);
  }

  debug_local.SphereLines(pivot, kDebugDrawSphereRadius, debug_draw_color_);
}

void PhysicsDebugDraw::DrawPivotConnectionToOrigin(NodeHandle node,
                                                   const float3& pivot) {
  debug_draw::Local debug_local(node.GetEntity());
  if (!debug_draw_transform_is_identity_) {
    debug_local.Transform(debug_draw_transform_);
  }

  debug_local.BoxLines(filament::Box(kZero3, kOne3 * kDebugDrawSphereRadius),
                       debug_draw_color_);
  debug_local.Line(kZero3, pivot, debug_draw_color_);
}

void PhysicsDebugDraw::DrawAxis(NodeHandle node, const float3& axis,
                                const float3& origin) {
  const float3 end_point = origin + kDebugDrawAxisLength * axis;

  debug_draw::Local debug_local(node.GetEntity());
  if (!debug_draw_transform_is_identity_) {
    debug_local.Transform(debug_draw_transform_);
  }

  debug_local.Line(origin, end_point, debug_draw_color_);

  DrawDot(node, end_point);
}

void PhysicsDebugDraw::DrawRotationAxis(NodeHandle node, const float3& pivot,
                                        const float3& axis) {
  debug_draw::Local debug_local(node.GetEntity());
  if (!debug_draw_transform_is_identity_) {
    debug_local.Transform(debug_draw_transform_);
  }

  debug_local.Line(pivot - kDebugDrawAxisLength * axis,
                   pivot + kDebugDrawAxisLength * axis, debug_draw_color_);
}

void PhysicsDebugDraw::DrawLine(NodeHandle node, const float3& start,
                                const float3& end) {
  debug_draw::Local debug_local(node.GetEntity());
  if (!debug_draw_transform_is_identity_) {
    debug_local.Transform(debug_draw_transform_);
  }

  debug_local.Line(start, end, debug_draw_color_);
}

void PhysicsDebugDraw::DrawLinearLimits(NodeHandle node, const float3& pivot,
                                        const float3& dir,
                                        const float& lower_limit,
                                        const float& upper_limit,
                                        const bool& show_direction) {
  const float3 start = pivot + dir * lower_limit;
  const float3 end = pivot + dir * upper_limit;

  debug_draw::Local debug_local(node.GetEntity());
  if (!debug_draw_transform_is_identity_) {
    debug_local.Transform(debug_draw_transform_);
  }

  debug_local.Line(start, end, debug_draw_color_);

  float3 up;
  float3 right;
  if (dot(dir, kYAxis3f) < 0.9f) {
    right = normalize(cross(dir, kYAxis3f));
    up = normalize(cross(right, dir));
  } else {
    right = normalize(cross(dir, kXAxis3f));
    up = normalize(cross(right, dir));
  }

  const float end_lines_length = 0.1f * kDebugDrawAxisLength;

  if (show_direction) {
    // Start cross lines.
    debug_local.Line(start - end_lines_length * up,
                     start + end_lines_length * up, debug_draw_color_);
    debug_local.Line(start - end_lines_length * right,
                     start + end_lines_length * right, debug_draw_color_);
  } else {
    // Start arrow lines.
    debug_local.Line(start,
                     start - end_lines_length * up + dir * end_lines_length,
                     debug_draw_color_);
    debug_local.Line(start,
                     start + end_lines_length * up + dir * end_lines_length,
                     debug_draw_color_);
  }

  // End arrow lines.
  debug_local.Line(end, end - end_lines_length * up - dir * end_lines_length,
                   debug_draw_color_);
  debug_local.Line(end, end + end_lines_length * up - dir * end_lines_length,
                   debug_draw_color_);
}

void PhysicsDebugDraw::DrawArc(NodeHandle node, const float& start,
                               const float& end, const float& radius) {
  debug_draw::Geometry geometry;
  geometry.type = filament::backend::PrimitiveType::LINE_STRIP;

  const float num_segments = std::ceilf((end - start) * kSegmentsPerDegree);
  const float segment_size = ToRadians(end - start) / num_segments;
  const float start_angle = ToRadians(start);
  for (int i = 0; i <= num_segments; ++i) {
    float angle = start_angle + i * segment_size;
    const float x = radius * std::cos(angle);
    const float y = radius * std::sin(angle);
    geometry.positions.emplace_back(float3(x, y, 0.0f));
    geometry.colors.emplace_back(debug_draw_color_);
    geometry.indices.emplace_back(i);
  }

  debug_draw::Local debug_local(node.GetEntity());
  if (!debug_draw_transform_is_identity_) {
    debug_local.Transform(debug_draw_transform_);
  }

  debug_local.UserDefined(std::move(geometry));
}

float3 PhysicsDebugDraw::GetArcPoint(const float& angle, const float& radius) {
  const float angle_radians = ToRadians(angle);
  const float x = radius * std::cos(angle_radians);
  const float y = radius * std::sin(angle_radians);
  return float3(x, y, 0.0f);
}

float3 PhysicsDebugDraw::GetArcTangent(const float& angle,
                                       const float& radius) {
  const float3 p = GetArcPoint(angle, radius);
  return normalize(float3(-p.y, p.x, 0.0f));
}

void PhysicsDebugDraw::DrawDot(NodeHandle node, const float3& center) {
  debug_draw::Local debug_local(node.GetEntity());
  if (!debug_draw_transform_is_identity_) {
    debug_local.Transform(debug_draw_transform_);
  }
  debug_local.BoxFaces(
      filament::Box(center, kOne3 * kDebugDrawSphereRadius * 0.25f),
      debug_draw_color_);
}

void PhysicsDebugDraw::DrawArrowTip(NodeHandle node, const mat4f& transform) {
  const float cone_height = kDebugDrawSphereRadius * 2.0f;
  debug_draw::Local debug_local(node.GetEntity());
  debug_local.Transform(debug_draw_transform_ * transform *
                        mat4f::rotation(-M_PI / 2, kXAxis3f) *
                        mat4f::translation(float3(0.0f, -cone_height, 0.0f)));
  debug_local.ConeLines({0.0f, 0.0f, 0.0f}, kDebugDrawSphereRadius, cone_height,
                        debug_draw_color_);
}

void PhysicsDebugDraw::DrawArrowTip(NodeHandle node, const float3& origin,
                                    const float3& direction) {
  mat4f transform = mat4f::lookAt(origin, origin + direction, kUp);
  DrawArrowTip(node, transform);
}

}  // namespace physics_debug_draw
}  // namespace imp
