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

#include "filament/filament/include/filament/Box.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "core/common/debug_draw.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"

namespace imp {
namespace physics_debug_draw {

constexpr float kDebugDrawSphereRadius = 0.05f;
constexpr float kDebugDrawAxisLength = 1.0f;
constexpr debug_draw::Color kDebugDrawDefaultColor =
    debug_draw::kDebugColors[static_cast<int>(debug_draw::DebugColor::kYellow)];

PhysicsDebugDraw::PhysicsDebugDraw() { SetColor(kDebugDrawDefaultColor); }

void PhysicsDebugDraw::SetColor(const debug_draw::Color& color) {
  debug_draw_color_ = color;
}

void PhysicsDebugDraw::ResetColor() {
  debug_draw_color_ = kDebugDrawDefaultColor;
}

void PhysicsDebugDraw::DrawPivotConnectedToOrigin(NodeHandle node,
                                                  const float3& pivot) {
  DrawPivotConnectionToOrigin(node, pivot);
  const utils::Entity& entity = node.GetEntity();
  debug_draw::Local(entity).SphereLines(pivot, kDebugDrawSphereRadius,
                                        debug_draw_color_);
}

void PhysicsDebugDraw::DrawPivotConnectionToOrigin(NodeHandle node,
                                                   const float3& pivot) {
  const utils::Entity& entity = node.GetEntity();
  debug_draw::Local(entity).BoxLines(
      filament::Box(kZero3, kOne3 * kDebugDrawSphereRadius), debug_draw_color_);
  debug_draw::Local(entity).Line(kZero3, pivot, debug_draw_color_);
}

void PhysicsDebugDraw::DrawAxis(NodeHandle node, const float3& axis,
                                const float3& origin) {
  const utils::Entity& entity = node.GetEntity();
  const float3 end_point = origin + kDebugDrawAxisLength * axis;
  debug_draw::Local(entity).Line(origin, end_point, debug_draw_color_);
  debug_draw::Local(entity).BoxFaces(
      filament::Box(end_point, kOne3 * kDebugDrawSphereRadius * 0.25f),
      debug_draw_color_);
}

void PhysicsDebugDraw::DrawLinearLimits(NodeHandle node, const float3& pivot,
                                        const float3& dir,
                                        const float& lower_limit,
                                        const float& upper_limit,
                                        const bool& show_direction) {
  const float3 start = pivot + dir * lower_limit;
  const float3 end = pivot + dir * upper_limit;

  const utils::Entity& entity = node.GetEntity();
  debug_draw::Local(entity).Line(start, end, debug_draw_color_);

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
    debug_draw::Local(entity).Line(start - end_lines_length * up,
                                   start + end_lines_length * up,
                                   debug_draw_color_);
    debug_draw::Local(entity).Line(start - end_lines_length * right,
                                   start + end_lines_length * right,
                                   debug_draw_color_);
  } else {
    // Start arrow lines.
    debug_draw::Local(entity).Line(
        start, start - end_lines_length * up + dir * end_lines_length,
        debug_draw_color_);
    debug_draw::Local(entity).Line(
        start, start + end_lines_length * up + dir * end_lines_length,
        debug_draw_color_);
  }

  // End arrow lines.
  debug_draw::Local(entity).Line(
      end, end - end_lines_length * up - dir * end_lines_length,
      debug_draw_color_);
  debug_draw::Local(entity).Line(
      end, end + end_lines_length * up - dir * end_lines_length,
      debug_draw_color_);
}
}  // namespace physics_debug_draw
}  // namespace imp
