/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_PHYSICS_DEBUG_DRAW_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_PHYSICS_DEBUG_DRAW_H_

#include "core/common/debug_draw.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"
namespace imp {
namespace physics_debug_draw {

// A class for drawing physics related debug geometry.
class PhysicsDebugDraw {
 public:
  PhysicsDebugDraw();
  ~PhysicsDebugDraw() = default;

  // Sets the color for drawing the debug geometry
  void SetColor(const imp::debug_draw::Color& color);

  // Resets the color to the default color
  void ResetColor();

  // Returns the default color (yellow)
  imp::debug_draw::Color GetDefaultColor() const;

  // Sets the transform for drawing the debug geometry
  void SetTransform(const mat4f& transform);

  // Resets the local transform to the identity transform
  void ResetTransform();

  // Draws a pivot (a sphere) connected to the origin of the node (a cube) with
  // a line
  void DrawPivotConnectedToOrigin(imp::NodeHandle node, const float3& pivot);

  // Draws only the origin of the node (a cube) and the line to the pivot
  void DrawPivotConnectionToOrigin(imp::NodeHandle node, const float3& pivot);

  // Draws an axis with a dot (small cube) on the end
  void DrawAxis(imp::NodeHandle node, const float3& axis,
                const float3& origin = kZero3);

  // Draws an axis from -1 to 1
  void DrawRotationAxis(imp::NodeHandle node, const float3& pivot,
                        const float3& axis);

  // Draw a line with the current color and transform
  void DrawLine(imp::NodeHandle node, const float3& start, const float3& end);

  // Draws a segment with end markings (arrows or crosses)
  // If show_direction is true, it will draw a cross on start point and an arrow
  // on the end point. If show_direction is false, it will draw arrows on both
  // ends.
  void DrawLinearLimits(imp::NodeHandle node, const float3& pivot,
                        const float3& dir, const float& lower_limit,
                        const float& upper_limit,
                        const bool& show_direction = false);

  void DrawArc(imp::NodeHandle node, const float& start, const float& end,
               const float& radius);

  float3 GetArcPoint(const float& angle, const float& radius);

  float3 GetArcTangent(const float& angle, const float& radius);

  // Draws a small cube at specified center (used for pivots, origins, centers
  // etc)
  void DrawDot(imp::NodeHandle node, const float3& center);

  void DrawArrowTip(imp::NodeHandle node,
                    const mat4f& transform = kIdentityMat4f);

  void DrawArrowTip(imp::NodeHandle node, const float3& origin,
                    const float3& direction);

 private:
  debug_draw::Local& DebugSpace(imp::NodeHandle node);

  imp::debug_draw::Color debug_draw_color_;
  imp::mat4f debug_draw_transform_;
  bool debug_draw_transform_is_identity_;
};
}  // namespace physics_debug_draw
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_PHYSICS_DEBUG_DRAW_H_
