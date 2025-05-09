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

#include "core/line_renderer/stroke_polyline_util.h"

namespace imp::line_renderer {

namespace {

constexpr int kIndicesPerTriangle = 3;
constexpr int kIndicesPerQuad = 6;

// Returns the max number of vertices for a single instance of a joint given
// the joint shape.
int MaxNumberOfVerticesPerSegmentForJointShape(LineJointShape joint_shape,
                                               bool has_stamps) {
  switch (joint_shape) {
    case LineJointShape::LINE_JOINT_SHAPE_ROUND:
      return kMaxSegmentsPerRoundedJoint + 2;
    case LineJointShape::LINE_JOINT_SHAPE_BEVEL:
    case LineJointShape::LINE_JOINT_SHAPE_MITER:
      // In the worst case, stamped lines need 7 vertices for each joint.
      // Lines without stamp need 5 in the worst case (though typically only
      // 3).
      return has_stamps ? 7 : 5;
  }
}

// Returns the max number of indices for a single instance of a joint given the
// shape.
int MaxNumberIndicesPerSegmentForJointShape(LineJointShape joint_shape) {
  switch (joint_shape) {
    case LineJointShape::LINE_JOINT_SHAPE_ROUND:
      // A triangle per segment in the joint.
      return kMaxSegmentsPerRoundedJoint * kIndicesPerTriangle;
    case LineJointShape::LINE_JOINT_SHAPE_BEVEL:
    case LineJointShape::LINE_JOINT_SHAPE_MITER:
      // Only one triangle required for these joints.
      return kIndicesPerTriangle;
  }
}

}  // namespace

int MaxVerticesForExtrudedStroke(int number_of_points, bool has_stamp,
                                 LineCapShape start_cap, LineCapShape end_cap,
                                 LineJointShape joint_shape,
                                 bool connect_to_previous, bool loop) {
  // 2 vertices at each end to form ends and 2 vertices at each end for caps.
  int count = (2 + 2) + (2 + 2);

  // Don't include any start/end caps if looping.
  if (!loop) {
    // Don't include a start cap if connecting to the previous line.
    if (!connect_to_previous) {
      switch (end_cap) {
        case LineCapShape::LINE_CAP_SHAPE_ROUNDED:
          count += kMaxSegmentsPerRoundedJoint;
          break;
        case LineCapShape::LINE_CAP_SHAPE_CUSTOM:
          // 2 vertices for custom end cap. These separate the cap from the rest
          // of the line, and have different texture coordinates.
          count += 2;
          break;
        default:
          break;
      }
    }

    switch (end_cap) {
      case LineCapShape::LINE_CAP_SHAPE_ROUNDED:
        count += kMaxSegmentsPerRoundedJoint;
        break;
      case LineCapShape::LINE_CAP_SHAPE_CUSTOM:
        // 2 vertices for custom end cap. These separate the cap from the rest
        // of the line, and have different texture coordinates.
        count += 2;
        break;
      case LineCapShape::LINE_CAP_SHAPE_ARROW_OUTER:
        // 5 extra vertices for arrowhead outer end cap.
        // These use two triangles put together to form the arrow, with one
        // shared vertex.
        count += 5;
        break;
      case LineCapShape::LINE_CAP_SHAPE_ARROW_INNER:
        // 7 extra vertices for arrowhead inner end cap.
        // These use two triangles put together to form the arrow, with one
        // shared vertex (5 vertices). The inner version is offset from the end
        // of the line using a small quad (2 extra vertices).
        count += 7;
        break;
      default:
        break;
    }
  }

  // There's a segment between each point.
  int num_segments = number_of_points - 1;
  if (connect_to_previous) {
    // Connecting to a previous line adds another segment.
    ++num_segments;
  }
  if (loop) {
    // Looping adds another segment.
    ++num_segments;
  }
  if (num_segments > 1) {
    count +=
        MaxNumberOfVerticesPerSegmentForJointShape(joint_shape, has_stamp) *
        (num_segments - 1);
  }

  return count;
}

int MaxIndicesForExtrudedStroke(int number_of_points, LineCapShape start_cap,
                                LineCapShape end_cap,
                                LineJointShape joint_shape,
                                bool connect_to_previous, bool loop) {
  int num_segments = number_of_points - 1;
  if (connect_to_previous) {
    // Connecting to a previous line adds another segment.
    ++num_segments;
  }
  if (loop) {
    // Looping adds another segment.
    ++num_segments;
  }
  int count = 0;
  // Add 2 for start and end caps.
  count += (num_segments + 2) * kIndicesPerQuad;

  if (loop) {
    // Looping adds another segment.
    ++num_segments;
  } else {
    if (!connect_to_previous) {
      switch (start_cap) {
        case LineCapShape::LINE_CAP_SHAPE_ROUNDED:
          // Add one triangle per rounded cap segment.
          count += kMaxSegmentsPerRoundedJoint * 3;
          break;
        default:
          break;
      }
    }
    switch (end_cap) {
      case LineCapShape::LINE_CAP_SHAPE_ROUNDED:
        // Add one triangle per rounded cap segment.
        count += kMaxSegmentsPerRoundedJoint * 3;
        break;
      case LineCapShape::LINE_CAP_SHAPE_ARROW_OUTER:
        // 6 extra indices for arrowhead outer end cap (two triangles).
        count += 6;
        break;
      case LineCapShape::LINE_CAP_SHAPE_ARROW_INNER:
        // 12 extra indices for arrowhead inner end cap: two triangles (6
        // indices), and a connecting quad to cover the offset area (6 indices).
        count += 12;
        break;
      default:
        break;
    }
  }

  if (num_segments > 1) {
    count += (num_segments - 1) *
             MaxNumberIndicesPerSegmentForJointShape(joint_shape);
  }

  return count;
}

}  // namespace imp::line_renderer
