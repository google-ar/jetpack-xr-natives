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

#include "core/line_renderer/line_extruder_3d.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>

#include "core/line_renderer/line_buffer.h"
#include "core/line_renderer/point_util.h"
#include "core/line_renderer/polyline.h"
#include "core/line_renderer/stroke_polyline_util.h"
#include "imp.h"  // IWYU pragma: export

namespace imp::line_renderer {

namespace {

// First element is the position and the second element is the normal.
using Polyline3fPoint = Polyline3f::PointT;

// The smallest allowed projection between a joint's miter and the orthogonal of
// the incoming direction. Anything smaller is considered too straight to need
// an extruded joint.
constexpr float kMinMiterProjection = std::numeric_limits<float>::epsilon();

// The minimum dot product between the in and out directions for bevel joints.
// Anything smaller means the bend is too sharp, and a fallback joint will be
// used instead.
constexpr float kMinInOutDotForBevelJoints = -(M_PI * 0.16f);

// The minimum dot product between the in and out directions for miter joints.
// Anything smaller means the bend is too sharp, and a fallback joint will be
// used instead.
constexpr float kMinInOutDotForMiterJoints = -(M_PI * 0.23f);

// The y texture coordinate for use at the tip of a start or end cap.
constexpr float kCapTextureY = 0.0f;

// The y texture coordinate for use along the body of a line.
constexpr float kBodyTextureY = 1.0f;

//  The extra extrusion applied to the length of arrow caps.
constexpr float kArrowCapLengthExtrusion = 2.5f;

// The extra extrusion applied to the width of arrow caps.
constexpr float kArrowCapWidthExtrusion = 2.0f;

// The offset from the end of the line that is applied to an arrowhead end cap.
// (This is the percentage of line width that the cap is moved backward along
// the line.)
constexpr float kArrowCapOffset = 0.8f;

constexpr float k2Pi = M_PI * 2.f;

struct Point {
  imp::float3 position;
  imp::float3 normal;
  imp::float3 offset;
};

// A struct that contains details on the joints position and direction.
typedef struct JointPosition {
  // The current point on the line, which is the middle of the joint.
  Point point;

  // The direction of the line coming into the position point.
  imp::float3 in_direction;

  // The direction vector that is orthogonal to `in_direction`.
  imp::float3 in_orthogonal;

  // The direction of the line going out of the position point.
  imp::float3 out_direction;

  // The direction vector that is orthogonal to `out_direction`.
  imp::float3 out_orthogonal;

  // The vector pointing from the position to the corner of the joint.
  imp::float3 miter;
} JointPosition;

// Normal accessors into Polyline3fPoint.
const imp::float3& GetNormal(const Polyline3fPoint& point) {
  return std::get<1>(point);
}
imp::float3& GetNormal(Polyline3fPoint& point) { return std::get<1>(point); }

// Returns Vector3f(0, 0, 0) for the normal if vector_ is Vector3f(0, 0, 0).
imp::float3 SafeNormalized(const imp::float3& v) {
  if (v.x == 0.0 && v.y == 0.0 && v.z == 0.0) {
    return v;
  } else {
    return normalize(v);
  }
}

// Returns a vector orthogonal to `dir` that is `dir` rotated 90 degrees
// clockwise around `normal`.
imp::float3 Orthogonal(const imp::float3& dir, const imp::float3& normal) {
  return cross(dir, normal);
}

// Approximately reconstructs the normal from a vertex attribute. We have to do
// this because we don't store the normal in the vertex attribute.
imp::float3 ReconstructGetNormal(const LineVertexAttributes& vertex,
                                 const imp::float3& approx_normal_dir) {
  // The extrusion vector could be to the left or right, so we extract the
  // original normal direction by dotting it with the approx normal direction.
  imp::float3 normal =
      normalize(cross(vertex.direction, vertex.extrusion_vector));
  if (dot(normal, approx_normal_dir) < 0) {
    return -normal;
  }
  return normal;
}

// Cross product between `a` and `b`. What makes it "2D" is that the cross
// product result is projected onto the `normal`.
float Cross2D(const imp::float3& a, const imp::float3& b,
              const imp::float3& normal) {
  return dot(cross(a, b), normal);
}

// Returns the point's position, normal, and offset vector.
// `orthogonal_offset_scale` determines the base length/scale of the offset that
// will be multiplied with the actual desired offset amount.
// `orthogonal_offset_scale` may also be negative to offset in the opposite
// direction.
Point PointWithOffset(const Polyline3f::const_iterator& begin,
                      const int num_points, const int index,
                      const float orthogonal_offset_scale,
                      const imp::float3& position) {
  const Polyline3fPoint& curr = begin[index];
  Polyline3fPoint curr_v_relative = curr;
  Polyline3f::GetPosition(curr_v_relative) -= position;
  if (abs(orthogonal_offset_scale) < std::numeric_limits<float>::epsilon()) {
    return {.position = Polyline3f::GetPosition(curr_v_relative),
            .normal = GetNormal(curr_v_relative),
            .offset = imp::float3(0.0f)};
  } else {
    //                        prev
    //                       /
    //                      /
    //                     /
    //                    /
    //     curr' ________/ curr
    //                   \
    //           |<---->| \
    //              /      \
    // orthogonal_offset    \
    //                       \
    //                        next
    const Polyline3fPoint& prev = index == 0 ? begin[0] : begin[index - 1];
    const Polyline3fPoint& next =
        index >= num_points - 1 ? begin[index] : begin[index + 1];
    imp::float3 in_v = SafeNormalized(Polyline3f::GetPosition(curr) -
                                      Polyline3f::GetPosition(prev));
    imp::float3 out_v = SafeNormalized(Polyline3f::GetPosition(next) -
                                       Polyline3f::GetPosition(curr));
    float in_length = length2(in_v);
    float out_length = length2(out_v);
    if (in_length == 0 && out_length == 0) {
      // All three points overlap, no way to apply offset.
      return {.position = Polyline3f::GetPosition(curr_v_relative),
              .normal = GetNormal(curr_v_relative),
              .offset = imp::float3(0.0f)};
    } else if (in_length == 0) {
      in_v = out_v;
    } else if (out_length == 0) {
      out_v = in_v;
    }

    // IMPORTANT: There's gotta be something wrong with this code:
    // 1) The bisector isn't normalized, but it should definitely be normalized.
    // Averaging 2 normalized vectors does not produce a normalized vector.
    // 2) The comment on the cross product being cos(theta) is wrong, cross
    // products actually produce sin(theta). It's super confusing that this
    // would work at all because the cross product for a straight line here
    // would be 0, which should result in a divide-by-zero when computing
    // `shift_v`.
    // The reason we're leaving it this way is because we don't have any render
    // tests for this code and it was ported as-is from iGMM.

    // Note that in_v and out_v are already normalized. A negative is applied to
    // the bisector because `Orthogonal()` produces a right-facing vector, but
    // we actually want a positive orthogonal offset to be left-facing.
    const imp::float3 bisector = -(Orthogonal(in_v, GetNormal(curr)) +
                                   Orthogonal(out_v, GetNormal(curr))) *
                                 0.5f;
    // cross_product == |in_v| * |bisector| * cos(theta), where theta is the
    // angle from in_v to bisector counter-clockwise.
    float cross_product = Cross2D(in_v, bisector, GetNormal(curr));
    imp::float3 shift_v = bisector * (orthogonal_offset_scale / cross_product);
    float double_abs_offset = std::abs(orthogonal_offset_scale) * 2;
    // Cap the shift length at 2 * abs(orthogonal_offset_scale) just in case.
    if (length(shift_v) > double_abs_offset) {
      shift_v = SafeNormalized(shift_v) * double_abs_offset;
    }
    return {Polyline3f::GetPosition(curr_v_relative),
            GetNormal(curr_v_relative), shift_v};
  }
}

// Returns the vector that is `angle` radians counterclockwise rotated around
// `normal` from the given vector.
inline imp::float3 GetRoundedGetNormal(float angle, const imp::float3& v,
                                       const imp::float3& normal) {
  return imp::quatf::fromAxisAngle(normal, angle) * v;
}

// Adds indices for a triangle to a short index buffer. Order them CCW.
inline void AddTriangleIndices(uint16_t i1, uint16_t i2, uint16_t i3,
                               LineBuffer& line_buffer) {
  line_buffer.AddIndex(i1);
  line_buffer.AddIndex(i2);
  line_buffer.AddIndex(i3);
}

// Adds indices for a quad (two triangles) to a short index buffer.
// The quad is split into two triangles: (1, 3, 2) and (3, 4, 2).
//  2----------------4
//  |                |
// <A>              <B>
//  |                |
//  1----------------3
inline void AddQuadIndices(uint16_t i1, uint16_t i2, uint16_t i3, uint16_t i4,
                           LineBuffer& line_buffer) {
  AddTriangleIndices(i1, i3, i2, line_buffer);
  AddTriangleIndices(i3, i4, i2, line_buffer);
}

// Update indices for a quad (two triangles) to a short index buffer.
// The quad is split into two triangles: (1, 3, 2) and (3, 4, 2) with new
// indices given for 3 and 4.
//  2----------------4
//  |                |
// <A>              <B>
//  |                |
//  1----------------3
inline void UpdateEndIndices(uint16_t i3, uint16_t i4,
                             LineBuffer& line_buffer) {
  uint16_t last_index = line_buffer.num_indices() - 1;
  line_buffer.ReplaceVertexIndexAtIndex(last_index - 4, i3);
  line_buffer.ReplaceVertexIndexAtIndex(last_index - 2, i3);
  line_buffer.ReplaceVertexIndexAtIndex(last_index - 1, i4);
}

// Finds the end point for a joint.
// This skips past any duplicates of current until a distinct point is found, or
// the end of the `polyline` is reached.
// `begin` Iterator pointing to the beginning of the polyline points.
// `num_points` The number of points in the polyline.
// `orthogonal_offset` Offset (in world units) to shift the line orthogonally.
//  Positive is to the left and negative to the right.
// `position` The position of the output coordinate system.
// `current` The current point on the line, which is the middle of the joint.
// `start_index` Index of the vertex in the polyline to try first.
// `out_end_point_index` Pointer to hold the index of the found point, or
// `num_points` if none was found.
// Returns the found point, adjusted for `orthogonal_offset` and `position`.
// This will be equal to `current` if no distinct point is found.
Point EndPointForJoint(const Polyline3f::const_iterator& begin, int num_points,
                       float orthogonal_offset, const imp::float3& position,
                       const Point& current, int start_index,
                       int* out_end_point_index) {
  Point next = current;
  int next_index = start_index;

  // Skip points until a non-duplicate is found, or reach the end of the line.
  while (next_index < num_points) {
    next = PointWithOffset(begin, num_points, next_index, orthogonal_offset,
                           position);
    if (next.position != current.position) {
      break;
    }
    ++next_index;
  }

  *out_end_point_index = next_index;
  return next;
}

// Connects a new line to the previously extruded one with vertices already in
// the buffer. `line_buffer` Buffer containing line vertices and indices. This
// should include vertices for at least one line already.
// `current` The current point on the new line.
// `has_stamps` Flag indicating if the line will have a stamped repeating
// texture.
// `out_previous` Pointer that will hold the previous point for extrusion.
// `out_current` Pointer that will hold the current point for extrusion.
// `out_next` Pointer that will hold the next point for extrusion.
// `out_distance` Pointer that will hold the distance to the previous point for
// extrusion.
// `out_previous_style_index` Pointer that will hold style indices for the
// previous line.
// `out_first_joint_index` Pointer that will hold the index of the point in the
// new line for the next joint extrusion.
// Returns index of the first vertex in the next quad.
uint16_t ConnectLineToPrevious(LineBuffer& line_buffer, const Point& current,
                               const Polyline3f::const_iterator& begin,
                               int num_points, float orthogonal_offset,
                               const imp::float3& position, bool has_stamps,
                               Point* out_previous, Point* out_current,
                               Point* out_next, float* out_distance,
                               uint16_t* out_previous_style_index,
                               int* out_first_joint_index) {
  // Extract previously-generated line info from vertex buffer.
  int num_vertices = line_buffer.num_vertices();
  const LineVertexAttributes& previous_end_vertex =
      line_buffer.VertexAtIndex(num_vertices - 1);
  const LineVertexAttributes& previous_begin_vertex =
      line_buffer.VertexAtIndex(num_vertices - 3);
  float unit_size = line_buffer.unit_size();
  float units_per_dp = line_buffer.units_per_dp();
  imp::float3 previous_end = previous_end_vertex.pos * unit_size;
  imp::float3 previous_begin = previous_begin_vertex.pos * unit_size;
  imp::float3 previous_end_normal =
      ReconstructGetNormal(previous_end_vertex, current.normal);
  imp::float3 previous_begin_normal =
      ReconstructGetNormal(previous_begin_vertex, current.normal);

  imp::float3 connection_direction = (current.position - previous_end);
  float connection_length = length(connection_direction);
  if (connection_length > 0) {
    // The first point of this new polyline is different than the previous end.
    // Setup a joint from previous_begin -> previous_end -> current.
    *out_next = current;
    *out_previous = {previous_begin, previous_begin_normal,
                     previous_begin_vertex.offset_direction};
    *out_current = {previous_end, previous_end_normal,
                    previous_end_vertex.offset_direction};
    *out_first_joint_index = 0;
  } else {
    // The first point is the same as the end of the previous segment. Setup for
    // a joint from previous_begin -> previous_end/current -> next.
    *out_next = EndPointForJoint(begin, num_points, orthogonal_offset, position,
                                 current, 1, out_first_joint_index);
    *out_previous = {previous_begin, previous_begin_normal,
                     previous_begin_vertex.offset_direction};
    *out_current = {previous_end, previous_end_normal,
                    previous_end_vertex.offset_direction};  // Same as current.
  }
  *out_distance = previous_begin_vertex.distance * units_per_dp;
  *out_previous_style_index = previous_end_vertex.style_index();

  // Remove the last two vertices that were added, since joint construction will
  // add them again. *EXCEPT* when there's no valid next point (all points are
  // equal), in which case no joint will be added.
  if (out_next->position != out_current->position) {
    num_vertices -= 2;
    line_buffer.set_num_vertices(num_vertices);
  }

  return static_cast<uint16_t>(num_vertices);
}

// Extrudes an arrowhead end cap.
// `line_buffer` Buffer for line vertices and indices.
// `current` The current point on the line.
// `direction` The direction of the line at the current point.
// `orthogonal` The vector that is orthogonal to `direction`.
// `extrusion_offset` Offset added to all extrusion vectors. This can be used to
// adjust the positioning of the arrow by some multiple of the line width, e.g.
// for an "inner" arrowhead inside a larger outer one. The wider the extrusion
// the more the offset moves the cap position.
// `distance` The distance along the line to the current point.
// `style_index` Style index to apply to the vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `index` Index of the first vertex in the next
// quad. Returns index of the first vertex in the next quad.
uint16_t ExtrudeArrowEndCap(LineBuffer& line_buffer, const Point& current,
                            const imp::float3& direction,
                            const imp::float3& orthogonal,
                            const imp::float3& extrusion_offset, float distance,
                            const uint16_t style_index,
                            const uint16_t packed_zoom_range, uint16_t index) {
  // This uses two triangles put together to form the arrow.
  // (Two to get different texture coordinates at the tip, which makes texturing
  // look better.) End of line with arrow cap:
  //
  //             6
  //             | .
  //             |   .
  //  1----------3     .
  //  |          |       .
  //  |          |         .
  // <A>       <B,4>-------7/8
  //  |          |         .
  //  |          |       .
  //  0----------2     .
  //             |   .
  //             | .
  //             5
  //
  // In the diagrams above, A is previous and B is current.
  // The first four vertices V0/V1/V2/V3 and corresponding quad indices have
  // already been added. V4 is added at B so we can form two triangles to make
  // up the point. (This is done to get nicer texture coordinates along the
  // edges of the point.)
  LineVertexAttributes vertex{
      .pos = current.position,
      .uv0 = {0.5f, kBodyTextureY},
      .extrusion_vector = extrusion_offset,
      .style_indices_and_packed_zoom_range = {style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = direction,
      .distance = distance,
      .offset_direction = current.offset,
  };
  // V4, shared base mid-point.
  line_buffer.AddVertex(vertex);

  // V5, first outer base point.
  vertex.extrusion_vector =
      extrusion_offset + orthogonal * kArrowCapWidthExtrusion;
  vertex.uv0.x = 1.0f;
  line_buffer.AddVertex(vertex);

  // V6, outer base point on the other side.
  vertex.extrusion_vector =
      extrusion_offset - orthogonal * kArrowCapWidthExtrusion;
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);

  // V7, tip point for use with V5.
  vertex.extrusion_vector =
      extrusion_offset + direction * kArrowCapLengthExtrusion;
  vertex.uv0.x = 1.0f;
  line_buffer.AddVertex(vertex);

  // V8, tip point for use with V6. This is the same as V7, but with different
  // texture coordinates.
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);

  AddTriangleIndices(index, index + 3, index + 1, line_buffer);
  AddTriangleIndices(index, index + 4, index + 2, line_buffer);
  index += 5;

  return index;
}

// Extrudes a U-turn (line doubling-back on itself).
// `line_buffer` Buffer for line vertices and indices.
// `current` The current point on the line.
// `direction` The direction of the line at the current point.
// `orthogonal` The vector that is orthogonal to `direction`.
// `distance` The distance along the line to the current point.
// `in_style_index` Style index to apply to the incoming vertices.
// `out_style_index` Style index to apply to the outgoing
// vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `index` Index of the first vertex in the next quad. Returns index of
// the first vertex in the next quad.
uint16_t ExtrudeUTurn(LineBuffer& line_buffer, const Point& current,
                      const imp::float3& direction,
                      const imp::float3& orthogonal, float distance,
                      const uint16_t in_style_index,
                      const uint16_t out_style_index,
                      const uint16_t packed_zoom_range, uint16_t index) {
  // Line u-turn.
  //  1-----------------3/4
  //  |                  |
  //  |                  |
  // <A>                <B>
  //  |                  |
  //  |                  |
  //  0-----------------2/5
  //
  LineVertexAttributes vertex{
      .pos = current.position,
      .uv0 = {1.0f, kBodyTextureY},
      .extrusion_vector = orthogonal,
      .style_indices_and_packed_zoom_range = {in_style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = direction,
      .distance = distance,
      .offset_direction = current.offset,
  };
  // V2.
  line_buffer.AddVertex(vertex);
  // V3.
  vertex.extrusion_vector = -orthogonal;
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);

  index += 2;

  // Switch to outgoing style indices.
  vertex.style_index() = out_style_index;

  // V4.
  vertex.direction = -direction, vertex.extrusion_vector = -orthogonal;
  vertex.uv0.x = 1.0f;
  line_buffer.AddVertex(vertex);
  // V5.
  vertex.extrusion_vector = orthogonal;
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);

  AddQuadIndices(index, index + 1, index + 2, index + 3, line_buffer);
  index += 2;

  return index;
}

// Extrudes a mitered joint.
// `line_buffer` Buffer for line vertices and indices.
// `current` The current point on the line, which is the middle of the joint.
// `in_direction` The direction of the line coming into the current point.
// `out_direction` The direction of the line going out of the current point.
// `miter` The vector pointing from the current to the corner of the joint.
// `line_direction` The average line direction at the joint.
// `distance` The distance along the line to the current point.
// `in_style_index` Style index to apply to the incoming vertices.
// `out_style_index` Style index to apply to the outgoing vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `has_stamps` Flag indicating if the line will have a stamped repeating
// texture.
// `index` Index of the first vertex in the next quad.
// Returns index of the first vertex in the next quad.
uint16_t ExtrudeMiteredJoint(
    LineBuffer& line_buffer, const Point& current,
    const imp::float3& in_direction, const imp::float3& out_direction,
    const imp::float3& miter, const imp::float3& line_direction, float distance,
    const uint16_t in_style_index, const uint16_t out_style_index,
    const uint16_t packed_zoom_range, bool has_stamps, uint16_t index) {
  // Pointy Miter Joint
  //  1------ outer edge ------------3
  //  |                            . |
  //  |                         .    |
  // <A>                     <B>     |
  //  |                    .         |
  //  |                 .            |
  //  0-- inner edge --2             |
  //                   |             |
  //                   |             |
  //                   |             |
  //                   |             |
  //                   |             |
  //                   4.....<C>.....5
  // Vertex ordering is actually the same for the left-curving version:
  //                   5.....<C>.....4
  //                   |             |
  //                   |             |
  //                   |             |
  //                   |             |
  //                   |             |
  //                   |             |
  //  1-- inner edge --3             |
  //  |                  .           |
  //  |                     .        |
  // <A>                     <B>     |
  //  |                          .   |
  //  |                             .|
  //  0----- outer edge -------------2
  // B above is the position. A->B is in_direction and B->C is out_direction,
  // both normalized. The miter is the vector from B to vertex 2. This function
  // adds the vertices for 2 and 3, and the indices for the next quad (2,3,4,5
  // above). The vertices 4 and 5 for the next quad haven't been added yet, but
  // will be in the next loop iteration or by the code immediately following the
  // loop. V2
  LineVertexAttributes vertex{
      .pos = current.position,
      .uv0 = {1.0f, kBodyTextureY},
      .extrusion_vector = miter,
      .style_indices_and_packed_zoom_range = {in_style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = has_stamps ? in_direction : line_direction,
      .distance = distance,
      .offset_direction = current.offset,
  };
  line_buffer.AddVertex(vertex);
  // V3
  vertex.extrusion_vector = -miter;
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);

  // Switch to outgoing style indices.
  vertex.style_index() = out_style_index;

  // Stamps and style changes need another pair of vertices with out-going
  // direction and distance.
  if (has_stamps || in_style_index != out_style_index) {
    // V2
    vertex.extrusion_vector = miter;
    vertex.uv0.x = 1.0f;
    vertex.direction = out_direction;
    line_buffer.AddVertex(vertex);
    // V3
    vertex.extrusion_vector = -miter;
    vertex.uv0.x = 0.0f;
    line_buffer.AddVertex(vertex);

    index += 2;
  }

  // Quad V2, V3, and next two vertices.
  AddQuadIndices(index, index + 1, index + 2, index + 3, line_buffer);
  index += 2;

  return index;
}

// Extrudes a beveled joint that curves left.
// `line_buffer` Buffer for line vertices and indices.
// `current` The current point on the line, which is the middle of the joint.
// `in_direction` The direction of the line coming into the current point.
// `in_orthogonal` The direction vector that is orthogonal to `in_direction`.
// `out_direction` The direction of the line going out of the current point.
// `out_orthogonal` The direction vector that is orthogonal to `out_direction`.
// `miter` The vector pointing from the current to the corner of the joint.
// `line_direction` The average line direction at the joint.
// `distance` The distance along the line to the current point.
// `in_style_index` Style index to apply to the incoming vertices.
// `out_style_index` Style index to apply to the outgoing vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `has_stamps` Flag indicating if the line will have a stamped repeating
// texture.
// `index` Index of the first vertex in the next quad.
// Returns index of the first vertex in the next quad.
uint16_t ExtrudeBeveledLeftJoint(
    LineBuffer& line_buffer, const Point& current,
    const imp::float3& in_direction, const imp::float3& in_orthogonal,
    const imp::float3& out_direction, const imp::float3& out_orthogonal,
    const imp::float3& miter, const imp::float3& line_direction, float distance,
    const uint16_t in_style_index, const uint16_t out_style_index,
    const uint16_t packed_zoom_range, bool has_stamps, uint16_t index) {
  // Beveled Corner Left.
  //                   6.....<C>.....5
  //                   |             |
  //                   |             |
  //                   |             |
  //                   |             |
  //                   |             |
  //  1-- outer edge --3             |
  //  |                 .  .         |
  //  |                  .      .    |
  // <A>                  .  <B>    .4
  //  |                     .      /
  //  |                      .   /
  //  0------ inner edge -----2/
  // V2.
  LineVertexAttributes vertex{
      .pos = current.position,
      .uv0 = {1.0f, kBodyTextureY},
      .extrusion_vector = in_orthogonal,
      .style_indices_and_packed_zoom_range = {in_style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = in_direction,
      .distance = distance,
      .offset_direction = current.offset,
  };
  line_buffer.AddVertex(vertex);
  // V3 (crux), in direction.
  vertex.extrusion_vector = -miter;
  vertex.direction = has_stamps ? in_direction : line_direction;
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);

  // Switch to outgoing style indices.
  vertex.style_index() = out_style_index;

  // Stamps need more vertices to wrap better around the corner.
  if (has_stamps) {
    index += 2;

    // V2 line direction.
    vertex.extrusion_vector = in_orthogonal;
    vertex.direction = line_direction;
    vertex.uv0.x = 1.0f;
    line_buffer.AddVertex(vertex);
    // V3 (crux), line direction.
    vertex.extrusion_vector = -miter;
    vertex.uv0.x = 0.0f;
    line_buffer.AddVertex(vertex);
    // V4 line direction.
    vertex.extrusion_vector = out_orthogonal;
    vertex.uv0.x = 1.0f;
    line_buffer.AddVertex(vertex);

    // Triangle V2, V4, V3.
    AddTriangleIndices(index, index + 2, index + 1, line_buffer);
    index += 3;

    // V3 (crux), out direction.
    vertex.extrusion_vector = -miter;
    vertex.direction = out_direction;
    vertex.uv0.x = 0.0f;
    line_buffer.AddVertex(vertex);
    // V4 out direction.
    vertex.extrusion_vector = out_orthogonal;
    vertex.uv0.x = 1.0f;
    line_buffer.AddVertex(vertex);

    // Quad V4, V3, and next two vertices.
    AddQuadIndices(index + 1, index, index + 2, index + 3, line_buffer);
    index += 2;
  } else {
    // V4.
    vertex.extrusion_vector = out_orthogonal;
    vertex.direction = out_direction;
    vertex.uv0.x = 1.0f;
    line_buffer.AddVertex(vertex);

    // Triangle V2, V4, V3.
    AddTriangleIndices(index, index + 2, index + 1, line_buffer);
    // Quad V4, V3, and next two vertices.
    AddQuadIndices(index + 2, index + 1, index + 3, index + 4, line_buffer);
    index += 3;
  }
  return index;
}

// Extrudes a beveled joint that curves right.
// `line_buffer` Buffer for line vertices and indices.
// `current` The current point on the line, which is the middle of the joint.
// `in_direction` The direction of the line coming into the current point.
// `in_orthogonal` The direction vector that is orthogonal to `in_direction`.
// `out_direction` The direction of the line going out of the current point.
// `out_orthogonal` The direction vector that is orthogonal to `out_direction`.
// `miter` The vector pointing from the current to the corner of the joint.
// `line_direction` The average line direction at the joint.
// `distance` The distance along the line to the current point.
// `in_style_index` Style index to apply to the incoming vertices.
// `out_style_index` Style index to apply to the outgoing vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `has_stamps` Flag indicating if the line will have a stamped
// repeating texture. `index` Index of the first vertex in the next quad.
// Returns index of the first vertex in the next quad.
uint16_t ExtrudeBeveledRightJoint(
    LineBuffer& line_buffer, const Point& current,
    const imp::float3& in_direction, const imp::float3& in_orthogonal,
    const imp::float3& out_direction, const imp::float3& out_orthogonal,
    const imp::float3& miter, const imp::float3& line_direction, float distance,
    const uint16_t in_style_index, const uint16_t out_style_index,
    const uint16_t packed_zoom_range, bool has_stamps, uint16_t index) {
  // Beveled Corner Right.
  //  1------ outer edge -----3
  //  |                      .  \
  //  |                     .      \
  // <A>                  .  <B>    .4
  //  |                  .     .     |
  //  |                 . .          |
  //  0-- inner edge --2             |
  //                   |             |
  //                   |             |
  //                   |             |
  //                   |             |
  //                   |             |
  //                   5.....<C>.....6
  // V2 (crux).
  LineVertexAttributes vertex{
      .pos = current.position,
      .uv0 = {1.0f, kBodyTextureY},
      .extrusion_vector = miter,
      .style_indices_and_packed_zoom_range = {in_style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = has_stamps ? in_direction : line_direction,
      .distance = distance,
      .offset_direction = current.offset,
  };
  line_buffer.AddVertex(vertex);
  // V3.
  vertex.extrusion_vector = -in_orthogonal;
  vertex.direction = in_direction;
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);

  // Switch to outgoing style indices.
  vertex.style_index() = out_style_index;

  // Stamps need more vertices to wrap better around the corner.
  if (has_stamps) {
    index += 2;

    // V2 (crux), line direction.
    vertex.extrusion_vector = miter;
    vertex.direction = line_direction;
    vertex.uv0.x = 1.0f;
    line_buffer.AddVertex(vertex);
    // V3 line direction.
    vertex.extrusion_vector = -in_orthogonal;
    vertex.uv0.x = 0.0f;
    line_buffer.AddVertex(vertex);
    // V4 line direction.
    vertex.extrusion_vector = -out_orthogonal;
    line_buffer.AddVertex(vertex);

    // Triangle V2, V4, V3.
    AddTriangleIndices(index, index + 2, index + 1, line_buffer);
    index += 3;

    // V2 (crux).
    vertex.extrusion_vector = miter;
    vertex.direction = out_direction;
    vertex.uv0.x = 1.0f;
    line_buffer.AddVertex(vertex);
    // V4.
    vertex.extrusion_vector = -out_orthogonal;
    vertex.direction = out_direction;
    vertex.uv0.x = 0.0f;
    line_buffer.AddVertex(vertex);
    // Quad V2, V4, and next two vertices.
    AddQuadIndices(index, index + 1, index + 2, index + 3, line_buffer);
    index += 2;
  } else {
    // V4.
    vertex.extrusion_vector = -out_orthogonal;
    vertex.direction = out_direction;
    vertex.uv0.x = 0.0f;
    line_buffer.AddVertex(vertex);

    // Triangle V2, V4, V3.
    AddTriangleIndices(index, index + 2, index + 1, line_buffer);
    // Quad V2, V4, and next two vertices.
    AddQuadIndices(index, index + 2, index + 3, index + 4, line_buffer);
    index += 3;
  }
  return index;
}

// Extrudes a rounded joint that curves left.
// `line_buffer` Buffer for line vertices and indices.
// `distance` The distance along the line to the position point.
// `in_style_index` Style index to apply to the incoming vertices.
// vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `index` Index of the first vertex in the joint.
// Returns index of the first vertex in the next quad.
uint16_t ExtrudeRoundedJointLeft(LineBuffer& line_buffer,
                                 const JointPosition& joint_pos, float distance,
                                 const uint16_t in_style_index,
                                 const uint16_t packed_zoom_range,
                                 uint16_t index, bool close = true) {
  // Rounded Corner Left.
  //                   6.....<C>........5
  //                   |                |
  //                   |                |
  //                   |                |
  //                   |                |
  //                   |                |
  //  1-- outer edge -3.               |
  //  |                 .. .            |
  //  |                  . .   .        |
  //  |                   .  .     .    |
  // <A>                   .   .        .4
  //  |                     .    .     /
  //  |                      .     */
  //  |                       .   /
  //  0------ inner edge -----2/
  // Generates a variable number of vertices and triangles that fan between
  // 2 and 4. Triangle order 2, 3, 4 clockwise. The final quad of 4, 3, 5, 6
  // is added to close the joint. Vertices corresponding to 5 and 6 are added
  // at a later time. The given position is in the center of the joint.
  float joint_angle = PointUtil3f::AngleBetweenVectors(
      joint_pos.in_orthogonal, joint_pos.out_orthogonal);
  // Up to kMaxSegmentsPerRoundedJoint segments for 180 degree joint angle.
  int segment_count = (kMaxSegmentsPerRoundedJoint * joint_angle / M_PI);

  // TODO: Handle stamped lines.
  LineVertexAttributes vertex{
      .pos = joint_pos.point.position,
      .uv0 = {1.f, kBodyTextureY},
      .extrusion_vector = joint_pos.in_orthogonal,
      .style_indices_and_packed_zoom_range = {in_style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = joint_pos.in_direction,
      .distance = distance,
      .offset_direction = joint_pos.point.offset,
  };
  // First joint index (2).
  line_buffer.AddVertex(vertex);

  // Crux (3).
  vertex.extrusion_vector = -joint_pos.miter;
  vertex.uv0.x = 0.f;
  line_buffer.AddVertex(vertex);

  const uint16_t first_round_index = index;
  const uint16_t crux_index = first_round_index + 1;

  uint16_t last_round_index = crux_index + 1;
  float rads_per_segment = joint_angle / static_cast<float>(segment_count);
  vertex.uv0.x = 1.f;
  for (int i = 1; i < segment_count; i++) {
    float vertex_angle = i * rads_per_segment;
    vertex.extrusion_vector = GetRoundedGetNormal(
        vertex_angle, joint_pos.in_orthogonal, joint_pos.point.normal);
    line_buffer.AddVertex(vertex);
    last_round_index++;
  }

  // Last joint vertex (4).
  vertex.extrusion_vector = joint_pos.out_orthogonal;
  vertex.direction = joint_pos.out_direction;
  line_buffer.AddVertex(vertex);

  // Triangle formed by the first joint, second vertex, and crux of the joint.
  AddTriangleIndices(first_round_index, first_round_index + 2, crux_index,
                     line_buffer);

  for (int i = 1; i < segment_count; i++) {
    // A fan of triangles of joint vertices spanning the angle between in ortho
    // and out ortho.
    AddTriangleIndices(crux_index + i, crux_index + i + 1, crux_index,
                       line_buffer);
  }

  if (close) {
    // Connects the join the next two vertices generated later.
    AddQuadIndices(last_round_index, crux_index, last_round_index + 1,
                   last_round_index + 2, line_buffer);
  }

  return last_round_index + 1;
}

// Extrudes a rounded joint that curves right.
// `line_buffer` Buffer for line vertices and indices.
// `distance` The distance along the line to the position point.
// `in_style_index` Style index to apply to the incoming vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.

// `index` Index of the first vertex in the joint.
// Returns index of the first vertex in the next quad.
uint16_t ExtrudeRoundedJointRight(LineBuffer& line_buffer,
                                  const JointPosition& joint_pos,
                                  float distance, const uint16_t in_style_index,
                                  const uint16_t packed_zoom_range,
                                  uint16_t index, bool close = true) {
  // Rounded Corner Right.
  //  1------ outer edge ------3.\
  //  |                        .   \
  //  |                       .      *
  //  |                      .   .    \
  // <A>                    .  .        \.4
  //  |                    . .      .   |
  //  |                   ..    .       |
  //  |                  .  .           |
  //  0-- inner edge --2.               |
  //                   |                |
  //                   |                |
  //                   |                |
  //                   |                |
  //                   |                |
  //                   5.....<C>........6
  // Generates a variable number of vertices and triangles that fan between
  // 2 and 4. Triangle order 2, 3, 4 clockwise. The final quad of 2, 4, 5, 6
  // is added to close the joint. Vertices corresponding to 5 and 6 are added
  // at a later time. The given position is in the center of the joint.
  float joint_angle = PointUtil3f::AngleBetweenVectors(
      joint_pos.in_orthogonal, joint_pos.out_orthogonal);
  // Up to kMaxSegmentsPerRoundedJoint segments for 180 degree joint angle.
  int segment_count = (kMaxSegmentsPerRoundedJoint * joint_angle / M_PI);

  // TODO: Handle stamped lines.
  LineVertexAttributes vertex{
      .pos = joint_pos.point.position,
      .uv0 = {1.f, kBodyTextureY},
      .extrusion_vector = joint_pos.miter,
      .style_indices_and_packed_zoom_range = {in_style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = joint_pos.in_direction,
      .distance = distance,
      .offset_direction = joint_pos.point.offset,
  };

  // Crux (2)
  line_buffer.AddVertex(vertex);

  // First joint (3)
  vertex.extrusion_vector = -joint_pos.in_orthogonal;
  vertex.uv0.x = 0.f;
  line_buffer.AddVertex(vertex);

  const int16_t crux_index = index;
  const int16_t first_round_index = crux_index + 1;
  uint16_t last_round_index = first_round_index + 1;
  float rads_per_segment = joint_angle / static_cast<float>(segment_count);
  for (int i = 1; i < segment_count; i++) {
    // Subtract 2pi so the vertices progress along the arc clockwise.
    float vertex_angle = k2Pi - (i * rads_per_segment);
    vertex.extrusion_vector = GetRoundedGetNormal(
        vertex_angle, -joint_pos.in_orthogonal, joint_pos.point.normal);
    line_buffer.AddVertex(vertex);
    last_round_index++;
  }

  // Last joint vertex (4).
  vertex.extrusion_vector = -joint_pos.out_orthogonal;
  vertex.direction = joint_pos.out_direction;
  line_buffer.AddVertex(vertex);

  // First triangle connecting the crux, the next joint vertex, and the first
  // joint vertex.
  AddTriangleIndices(crux_index, first_round_index + 1, first_round_index,
                     line_buffer);

  for (int i = 1; i < segment_count; i++) {
    // A fan of triangles between in ortho and out ortho, if any.
    AddTriangleIndices(crux_index, first_round_index + i + 1,
                       first_round_index + i, line_buffer);
  }

  if (close) {
    // Connects the join the next two vertices generated later.
    AddQuadIndices(crux_index, last_round_index, last_round_index + 1,
                   last_round_index + 2, line_buffer);
  }

  return last_round_index + 1;
}

// Extrudes a fallback joint.
// `line_buffer` Buffer for line vertices and indices.
// `curves_left` True if the joint curves to the left, false if it curves to the
// right.
// `current` The current point on the line, which is the middle of the joint.
// `in_direction` The direction of the line coming into the position point.
// `in_orthogonal` The direction vector that is orthogonal to `in_direction`.
// `out_direction` The direction of the line going out of the position point.
// `out_orthogonal` The direction vector that is orthogonal to `out_direction`.
// `line_direction` The average line direction at the joint.
// `distance` The distance along the line to the position point.
// `in_style_index` Style index to apply to the incoming vertices.
// `out_style_index` Style index to apply to the outgoing vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `has_stamps` Flag indicating if the line will have a stamped repeating
// texture.
// `index` Index of the first vertex in the next quad.
// Returns index of the first vertex in the next quad.
uint16_t ExtrudeFallbackJoint(
    LineBuffer& line_buffer, bool curves_left, const Point& current,
    const imp::float3& in_direction, const imp::float3& in_orthogonal,
    const imp::float3& out_direction, const imp::float3& out_orthogonal,
    const imp::float3& line_direction, float distance,
    const uint16_t in_style_index, const uint16_t out_style_index,
    const uint16_t packed_zoom_range, bool has_stamps, uint16_t index) {
  // Fallback Joint Right:
  //  1------ outer edge -----3
  //  |                4-_    | \
  //  |               /    -  |   \
  // <A>            /        <B>    \
  //  |           /           .  --   \
  //  |         /             .     -- 5
  //  0--- inner edge . . . . 2       /
  //         /                      /
  //       /                      /
  //     /                      /
  //   /                      /
  // /                      /
  // 6---------------------7
  // Fallback Joint Left:
  // 7---------------------6
  // \                      \
  //   \                      \
  //     \                      \
  //       \                      \
  //         \                      \
  //  1--- inner edge . . . . 3       \
  //  |         \             .     _  4
  //  |           \           .  _    /
  // <A>            \       _<B>    /
  //  |               \   _   |   /
  //  |                5-     | /
  //  0------ outer edge -----2
  // B above is the position. A->B is in_direction and B->C is out_direction,
  // both normalized. This function adds the vertices for 2, 3, 4, and 5
  // pictured above, and indices for the next quad (4,5,6,7 above). It also adds
  // indices for a "capping" triangle: 2,3,5 for a right-curving joint, 2,3,4
  // for left. (This triangle is partially underneath the 4,5,6,7 quad.) The
  // vertices 6 and 7 for the next quad haven't been added yet, but will be in
  // the next loop iteration or by the code immediately following the loop.

  // V2.
  LineVertexAttributes vertex{
      .pos = current.position,
      .uv0 = {1.0f, kBodyTextureY},
      .extrusion_vector = in_orthogonal,
      .style_indices_and_packed_zoom_range = {in_style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = in_direction,
      .distance = distance,
      .offset_direction = current.offset,
  };
  line_buffer.AddVertex(vertex);

  // V3.
  vertex.extrusion_vector = -in_orthogonal;
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);

  // VB -- for the capping triangle.
  vertex.extrusion_vector = {0, 0, 0};
  vertex.uv0.x = 0.5f;
  vertex.direction = line_direction;
  line_buffer.AddVertex(vertex);

  if (has_stamps && in_style_index != out_style_index) {
    // Style change requires extra vertices to avoid interpolating stamp texture
    // coordinates.
    if (curves_left) {
      // V4(a) -- V4 with incoming style indices for the triangle cap.
      vertex.extrusion_vector = out_orthogonal;
      vertex.uv0.x = 1.0f;
      line_buffer.AddVertex(vertex);
      // Triangle V2, V4a, VB.
      AddTriangleIndices(index, index + 3, index + 2, line_buffer);
    } else {
      // V5(a) -- V5 with incoming style indices for the triangle cap.
      vertex.extrusion_vector = -out_orthogonal;
      vertex.uv0.x = 0.0f;
      line_buffer.AddVertex(vertex);
      // Triangle VB, V5a, V3.
      AddTriangleIndices(index + 2, index + 3, index + 1, line_buffer);
    }
    index += 4;

    // Switch to outgoing style indices.
    vertex.style_index() = out_style_index;
  } else {
    if (curves_left) {
      // Triangle V2, V4, VB.
      AddTriangleIndices(index, index + 3, index + 2, line_buffer);
    } else {
      // Triangle VB, V5, V3.
      AddTriangleIndices(index + 2, index + 4, index + 1, line_buffer);
    }
    index += 3;
  }

  // V4.
  vertex.extrusion_vector = out_orthogonal;
  vertex.uv0.x = 1.0f;
  vertex.direction = out_direction;
  line_buffer.AddVertex(vertex);

  // V5.
  vertex.extrusion_vector = -out_orthogonal;
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);

  // Quad V4, V5, and next two vertices V6, V7 (added later).
  AddQuadIndices(index, index + 1, index + 2, index + 3, line_buffer);
  index += 2;

  return index;
}

// Extrudes a joint.
// `line_buffer` Buffer for line vertices and indices.
// `previous` The previous point on the line.
// `current` The current point on the line.
// `next` The next point on the line.
// `distance` The distance along the line to the previous point.
// `in_style_index` Style index to apply to the incoming vertices.
// `out_style_index` Style index to apply to the outgoing vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `joint_shape` The shape of the joint to extrude.
// `has_stamps` Flag indicating if the line will have a stamped repeating
// texture.
// `index` Index of the first vertex in the next quad.
// `out_distance` Pointer that will hold the distance up to the current point
// (not next).
// Returns index of the first vertex in the next quad.
uint16_t ExtrudeJoint(LineBuffer& line_buffer, const Point& previous,
                      const Point& current, const Point& next, float distance,
                      const uint16_t in_style_index,
                      const uint16_t out_style_index,
                      const uint16_t packed_zoom_range,
                      LineJointShape joint_shape, bool has_stamps,
                      uint16_t index, float* out_distance) {
  imp::float3 in_direction_with_length = current.position - previous.position;
  float in_length = length(in_direction_with_length);
  if (in_length == 0) {
    // Current and previous points are identical. Don't extrude anything yet.
    return index;
  }

  imp::float3 out_direction_with_length = next.position - current.position;
  float out_length = length(out_direction_with_length);
  if (out_length == 0) {
    // Next and current points are identical. Don't extrude anything yet.
    return index;
  }

  // Add distance from previous to current.
  distance += in_length;

  imp::float3 in_direction = in_direction_with_length / in_length;
  imp::float3 in_orthogonal = Orthogonal(in_direction, current.normal);
  imp::float3 out_direction = out_direction_with_length / out_length;

  imp::float3 direction_not_normalized = in_direction + out_direction;
  float direction_length = length(direction_not_normalized);
  if (direction_length == 0) {
    // Line doubles back on itself.
    index = ExtrudeUTurn(line_buffer, current, in_direction, in_orthogonal,
                         distance, in_style_index, out_style_index,
                         packed_zoom_range, index);
    *out_distance = distance;
    return index;
  }

  imp::float3 direction = direction_not_normalized / direction_length;
  imp::float3 miter = Orthogonal(direction, current.normal);

  float miter_projection = dot(miter, in_orthogonal);
  if (miter_projection < kMinMiterProjection &&
      in_style_index == out_style_index) {
    // There's no need to extrude joints that are straight (or extremely close
    // to straight) when there's no style change. Don't update out_distance.
    return index;
  }

  // TODO: Determine the minimum dot product for rounded joints.
  bool use_fallback_joint = false;
  float in_dot_out = dot(in_direction, out_direction);
  float min_dot = joint_shape == LineJointShape::LINE_JOINT_SHAPE_BEVEL
                      ? kMinInOutDotForBevelJoints
                      : kMinInOutDotForMiterJoints;
  if (in_dot_out < min_dot) {
    use_fallback_joint = true;
  } else {
    float miter_length = 1.0f / miter_projection;
    miter *= miter_length;
  }

  imp::float3 out_orthogonal = Orthogonal(out_direction, current.normal);
  // Determine if the joint curves left or right.
  float cross = Cross2D(in_direction_with_length, out_direction_with_length,
                        current.normal);
  bool curves_left = cross > 0;
  if (use_fallback_joint) {
    index = ExtrudeFallbackJoint(
        line_buffer, curves_left, current, in_direction, in_orthogonal,
        out_direction, out_orthogonal, direction, distance, in_style_index,
        out_style_index, packed_zoom_range, has_stamps, index);
  } else {
    switch (joint_shape) {
      case LineJointShape::LINE_JOINT_SHAPE_BEVEL: {
        if (curves_left) {
          index = ExtrudeBeveledLeftJoint(
              line_buffer, current, in_direction, in_orthogonal, out_direction,
              out_orthogonal, miter, direction, distance, in_style_index,
              out_style_index, packed_zoom_range, has_stamps, index);
        } else {
          index = ExtrudeBeveledRightJoint(
              line_buffer, current, in_direction, in_orthogonal, out_direction,
              out_orthogonal, miter, direction, distance, in_style_index,
              out_style_index, packed_zoom_range, has_stamps, index);
        }
        break;
      }

      case LineJointShape::LINE_JOINT_SHAPE_ROUND: {
        JointPosition joint_pos = {
            .point = current,
            .in_direction = in_direction,
            .in_orthogonal = in_orthogonal,
            .out_direction = out_direction,
            .out_orthogonal = out_orthogonal,
            .miter = miter,
        };

        if (curves_left) {
          index =
              ExtrudeRoundedJointLeft(line_buffer, joint_pos, distance,
                                      in_style_index, packed_zoom_range, index);
        } else {
          index = ExtrudeRoundedJointRight(line_buffer, joint_pos, distance,
                                           in_style_index, packed_zoom_range,
                                           index);
        }
        break;
      }

      case LineJointShape::LINE_JOINT_SHAPE_MITER: {
        index = ExtrudeMiteredJoint(line_buffer, current, in_direction,
                                    out_direction, miter, direction, distance,
                                    in_style_index, out_style_index,
                                    packed_zoom_range, has_stamps, index);
        break;
      }
    }
  }

  *out_distance = distance;
  return index;
}

// `line_buffer` Buffer for line vertices and indices.
// `current` The current point on the line.
// `distance` The distance along the line to the previous point.
// `style_index` Style index to apply to the incoming vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `index` Index of the first vertex in the next quad.
// Returns index of the first vertex in the next quad.
uint16_t ExtrudeRoundedCap(LineBuffer& line_buffer, const Point& current,
                           const float3& direction, const float3& orthogonal,
                           float distance, const uint16_t style_index,
                           const uint16_t packed_zoom_range, uint16_t index) {
  // Rounded cap
  //                 1
  //             2   |
  //                 |
  //         3       |
  //                 |
  //        .        0
  //                 |
  //         .       |
  //                 |
  //             .   |
  //                 N
  //
  // In the diagrams above, A is current. 0 is the "crux", 1 is the first point
  // in the fan, and we generate 2 through N by sweeping the curve at even
  // degree increments from 0 -> PI. A triangle fan is formed always with 0 as
  // one point to sweep the curve like [0, 2, 1], [0, 3, 2], etc.

  // Up to kMaxSegmentsPerRoundedJoint segments for 180 degree joint angle.
  int segment_count = kMaxSegmentsPerRoundedJoint;

  LineVertexAttributes vertex{
      .pos = current.position,
      .uv0 = {0.5f, kBodyTextureY},
      .extrusion_vector = {0, 0, 0},
      .style_indices_and_packed_zoom_range = {style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = direction,
      .distance = distance,
      .offset_direction = current.offset,
  };

  // Crux (0)
  line_buffer.AddVertex(vertex);
  // Set all u values to 0 for the cap edges. We might want to have half the
  // cap be 1 and half 0, but then we have to add a duplicate vertex at the
  // midpoint and we don't have a use-case for this yet.
  vertex.uv0.x = 0.0f;

  const int16_t crux_index = index;
  const int16_t first_round_index = crux_index + 1;
  uint16_t last_round_index = first_round_index;
  float rads_per_segment = M_PI / static_cast<float>(segment_count);
  for (int i = 0; i <= segment_count; i++) {
    // Subtract 2pi so the vertices progress along the arc clockwise.
    float vertex_angle = k2Pi - (i * rads_per_segment);
    vertex.extrusion_vector =
        GetRoundedGetNormal(vertex_angle, orthogonal, current.normal);
    line_buffer.AddVertex(vertex);
    last_round_index++;
  }

  for (int i = 0; i < segment_count; i++) {
    // A fan of triangles between in ortho and out ortho, if any.
    AddTriangleIndices(crux_index, first_round_index + i + 1,
                       first_round_index + i, line_buffer);
  }

  return last_round_index;
}

// Extrudes the start of a line, including start cap if present.
// `line_buffer` Buffer for line vertices and indices.
// `current` The current point on the line.
// `next` The next point on the line.
// `distance` The distance along the line to the current point.
// `start_cap` The start cap to generate.
// `style_index` Style index to apply to the vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `index` Index of the first vertex in the next quad. Returns index of
// the first vertex in the next quad.
uint16_t ExtrudeLineStart(LineBuffer& line_buffer, const Point& current,
                          const Point& next, float distance,
                          LineCapShape start_cap, const uint16_t style_index,
                          const uint16_t packed_zoom_range, uint16_t index) {
  // Start of line with no cap.
  //  1----------------3
  //  |                |
  //  |                |
  // <A>              <B>
  //  |                |
  //  |                |
  //  0----------------2
  //
  // Start of line with cap.
  //  1-----3----------------5
  //  | .   |                |
  //  |   . |                |
  //  |    <A>              <B>
  //  |   . |                |
  //  | .   |                |
  //  0-----2----------------4
  //
  // In the diagrams above, A is current and B is next.
  // A->0 and A->1 are the extrusion vectors forming the cap, each with a length
  // of sqrt(2). The final pair of vertices are added later, in a separate
  // method (either V2/V3 or V4/V5 above, depending on whether there are caps or
  // not). The indices for them are added here.

  imp::float3 direction = normalize(next.position - current.position);
  imp::float3 orthogonal = Orthogonal(direction, current.normal);

  LineVertexAttributes vertex{
      .pos = current.position,
      .uv0 = {1.0f, kBodyTextureY},
      .extrusion_vector = orthogonal,
      .style_indices_and_packed_zoom_range = {style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = direction,
      .distance = distance,
      .offset_direction = current.offset,
  };

  if (start_cap == LineCapShape::LINE_CAP_SHAPE_ARROW_OUTER ||
      start_cap == LineCapShape::LINE_CAP_SHAPE_ARROW_INNER) {
    // Arrowhead is not supported for start caps.
    start_cap = LineCapShape::LINE_CAP_SHAPE_NONE;
  }

  imp::float3 extrusion_modifier = {0, 0, 0};
  if (start_cap != LineCapShape::LINE_CAP_SHAPE_NONE) {
    if (start_cap == LineCapShape::LINE_CAP_SHAPE_CUSTOM) {
      // Custom cap vertices need cap base style indices.
      vertex.cap_index() = style_index + 1;
      // Custom caps are square. Adjust extrusion for interior cap vertices by
      // moving them forward along the line, towards the end.
      extrusion_modifier = direction;
    }

    if (start_cap == LineCapShape::LINE_CAP_SHAPE_ROUNDED) {
      index =
          ExtrudeRoundedCap(line_buffer, current, direction, orthogonal,
                            distance, style_index, packed_zoom_range, index);
    } else {
      // V0.
      vertex.extrusion_vector = -direction + orthogonal;
      vertex.uv0 = {1.0f, kCapTextureY};
      line_buffer.AddVertex(vertex);
      // V1.
      vertex.extrusion_vector = -direction - orthogonal;
      vertex.uv0.x = 0.0f;
      line_buffer.AddVertex(vertex);

      AddQuadIndices(index, index + 1, index + 2, index + 3, line_buffer);
      index += 2;
    }
  }

  // V0 (if no cap), otherwise V2.
  vertex.extrusion_vector = orthogonal + extrusion_modifier;
  vertex.distance = distance;
  vertex.uv0 = {1.0f, kBodyTextureY};
  line_buffer.AddVertex(vertex);
  // V1 (if no cap), otherwise V3.
  vertex.extrusion_vector = -orthogonal + extrusion_modifier;
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);
  if (start_cap == LineCapShape::LINE_CAP_SHAPE_CUSTOM) {
    // Custom caps need extra vertices to separate them from the body.
    // These are body vertices so should not have the custom cap style indices.
    vertex.cap_index() = 0;
    // V2 again.
    vertex.extrusion_vector = orthogonal + extrusion_modifier;
    vertex.uv0.x = 1.0f;
    line_buffer.AddVertex(vertex);
    // V3 again.
    vertex.extrusion_vector = -orthogonal + extrusion_modifier;
    vertex.uv0.x = 0.0f;
    line_buffer.AddVertex(vertex);
    index += 2;
  }

  AddQuadIndices(index, index + 1, index + 2, index + 3, line_buffer);
  index += 2;

  return index;
}

// Extrudes the end of a line, including end cap if present.
// `line_buffer` Buffer for line vertices and indices.
// `previous` The previous point on the line.
// `current` The current point on the line.
// `distance` The distance along the line to the current point.
// `end_cap` The end cap to generate.
// `style_index` Style index to apply to the vertices.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `index` Index of the first vertex in the next quad.
// Returns index of the first vertex in the next quad.
uint16_t ExtrudeLineEnd(LineBuffer& line_buffer, const Point& previous,
                        const Point& current, float distance,
                        LineCapShape end_cap, const uint16_t style_index,
                        const uint16_t packed_zoom_range, uint16_t index) {
  // End of line with no cap:
  //  1----------------3
  //  |                |
  //  |                |
  // <A>              <B>
  //  |                |
  //  |                |
  //  0----------------2
  //
  // End of line with cap:
  //  1----------------3-----5
  //  |                |   . |
  //  |                | .   |
  // <A>              <B>    |
  //  |                | .   |
  //  |                |   . |
  //  0----------------2-----4
  //
  // In the diagrams above, A is previous and B is current.
  // B->4 and B->5 are the extrusion vectors forming the cap, each with a length
  // of sqrt(2). The first pair of vertices V0/V1 and corresponding indices were
  // already added in a previous method.

  imp::float3 direction = current.position - previous.position;
  float last_length = length(direction);
  if (last_length == 0) {
    // Final point is identical to previous. Use direction from previous vertex.
    const LineVertexAttributes& v =
        line_buffer.VertexAtIndex(line_buffer.num_vertices() - 1);
    direction = v.direction;
  } else {
    direction /= last_length;
    distance += last_length;
  }
  imp::float3 orthogonal = Orthogonal(direction, current.normal);

  // Custom caps are square. Adjust extrusion for interior cap vertices by
  // moving them backward along the line, towards the start.
  imp::float3 extrusion_modifier =
      (end_cap == LineCapShape::LINE_CAP_SHAPE_CUSTOM) ? -direction
                                                       : imp::float3(0.0f);

  LineVertexAttributes vertex{
      .pos = current.position,
      .uv0 = {1.0f, kBodyTextureY},
      .extrusion_vector = orthogonal + extrusion_modifier,
      .style_indices_and_packed_zoom_range = {style_index,
                                              /*cap_index*/ 0,
                                              packed_zoom_range, 0},
      .direction = direction,
      .distance = distance,
      .offset_direction = current.offset,
  };
  // V2.
  line_buffer.AddVertex(vertex);
  // V3.
  vertex.extrusion_vector = -orthogonal + extrusion_modifier;
  vertex.uv0.x = 0.0f;
  line_buffer.AddVertex(vertex);
  index += 2;

  if (end_cap != LineCapShape::LINE_CAP_SHAPE_NONE) {
    if (end_cap == LineCapShape::LINE_CAP_SHAPE_ARROW_INNER ||
        end_cap == LineCapShape::LINE_CAP_SHAPE_ARROW_OUTER) {
      // Inner and outer have identical geometry. The extrusion offset will
      // shift smaller-width strokes back less, so they end up looking like
      // "inner" caps without different geometry.
      imp::float3 offset = -direction * kArrowCapOffset;
      index = ExtrudeArrowEndCap(line_buffer, current, direction, orthogonal,
                                 offset, distance, style_index,
                                 packed_zoom_range, index);
    } else {
      if (end_cap == LineCapShape::LINE_CAP_SHAPE_CUSTOM) {
        // Custom caps need extra vertices to separate them from the body.
        // These are cap vertices so use the custom cap style.
        vertex.cap_index() = -(style_index + 1);
        // V2 again.
        vertex.extrusion_vector = orthogonal + extrusion_modifier;
        vertex.uv0.x = 1.0f;
        line_buffer.AddVertex(vertex);
        // V3 again.
        vertex.extrusion_vector = -orthogonal + extrusion_modifier;
        vertex.uv0.x = 0.0f;
        line_buffer.AddVertex(vertex);
        index += 2;
      }

      if (end_cap == LineCapShape::LINE_CAP_SHAPE_ROUNDED) {
        index =
            ExtrudeRoundedCap(line_buffer, current, direction, -orthogonal,
                              distance, style_index, packed_zoom_range, index);
      } else {
        // V4.
        vertex.extrusion_vector = direction + orthogonal;
        vertex.uv0 = {1.0f, kCapTextureY};
        line_buffer.AddVertex(vertex);
        // V5.
        vertex.extrusion_vector = direction - orthogonal;
        vertex.uv0.x = 0.0f;
        line_buffer.AddVertex(vertex);

        AddQuadIndices(index - 2, index - 1, index, index + 1, line_buffer);
        index += 2;
      }
    }
  }

  return index;
}

}  // namespace

void ExtrudeLine3D(LineBuffer& line_buffer, const Polyline3f& polyline,
                   LineJointShape joint_shape, LineCapShape start_cap,
                   LineCapShape end_cap, bool has_stamps,
                   const uint16_t style_index, const uint16_t packed_zoom_range,
                   float start_distance, float total_unclipped_distance,
                   bool connect_to_previous, bool loop_self, bool loop_first,
                   float orthogonal_offset_scale, const imp::float3& position) {
  int num_points = polyline.size();
  if (num_points < 1 || (num_points < 2 && !connect_to_previous)) {
    return;
  }

  bool loop = loop_self || loop_first;

  // Index of the first vertex we will add.
  int start_index = line_buffer.num_vertices();
  uint16_t index = static_cast<uint16_t>(start_index);

  const Polyline3f::const_iterator begin = polyline.begin();

  Point first =
      PointWithOffset(begin, num_points, 0, orthogonal_offset_scale, position);
  Point current = first;
  Point previous = current;
  Point next = current;

  // Style indices for any previously-extruded vertices. These are only
  // different from this line's styles when connect_to_previous is true and the
  // previous line had different styles. Used as the incoming style when
  // constructing the first connecting joint.
  uint16_t previous_style_index = style_index;

  // Span distances for previously-extruded vertices.
  float previous_span_start = line_buffer.span_start_distance();
  float previous_span_end = line_buffer.span_end_distance();

  // The index of the point at the end of the first joint.
  int first_joint_index = 2;
  // Distance to current point along the line.
  if (connect_to_previous && start_distance == 0) {
    start_distance = previous_span_end;
  }
  float distance = start_distance;

  // Set span distances for this polyline.
  float start_offset = start_distance - previous_span_end;
  line_buffer.set_span_distance(previous_span_end + start_offset,
                                total_unclipped_distance);

  if (connect_to_previous) {
    // Connect geometry of new line to that of the previous line.
    index = ConnectLineToPrevious(line_buffer, current, begin, num_points,
                                  orthogonal_offset_scale, position, has_stamps,
                                  &previous, &current, &next, &distance,
                                  &previous_style_index, &first_joint_index);
    if (next.position == current.position) {
      // Do nothing if no valid next point (all points are the same).
      return;
    }
  } else if (loop) {
    // Setup a joint from end -> first -> second.
    // TODO: support loop_first properly.
    next = PointWithOffset(begin, num_points, 1, orthogonal_offset_scale,
                           position);
    previous = PointWithOffset(begin, num_points, num_points - 1,
                               orthogonal_offset_scale, position);
    first_joint_index = 0;
  } else {
    next = EndPointForJoint(begin, num_points, orthogonal_offset_scale,
                            position, current, 1, &first_joint_index);
    if (next.position == current.position) {
      // Do nothing if no valid next point (all points are the same).
      return;
    }

    // Extrude the start of the line, including any start cap.
    index = ExtrudeLineStart(line_buffer, current, next, distance, start_cap,
                             style_index, packed_zoom_range, index);

    current = next;
    next =
        EndPointForJoint(begin, num_points, orthogonal_offset_scale, position,
                         current, first_joint_index + 1, &first_joint_index);
  }

  // Extrude all the joints.
  int last_joint_index = loop ? num_points : num_points - 1;
  if (first_joint_index <= last_joint_index) {
    int joint_index = first_joint_index;
    while (joint_index <= last_joint_index) {
      if (loop && joint_index == last_joint_index) {
        next = first;
      }
      uint16_t previous_index = index;
      index = ExtrudeJoint(line_buffer, previous, current, next, distance,
                           previous_style_index, style_index, packed_zoom_range,
                           joint_shape, has_stamps, index, &distance);
      if (index > previous_index) {
        // Only update previous if actually extruded a joint. This should always
        // be the case unless there are duplicate points in the polyline. (Not
        // updating previous here skips over them.)
        previous = current;
        if (connect_to_previous && previous_style_index != style_index) {
          for (int i = previous_index; i < index; ++i) {
            // Update any vertices using previous styles to also use previous
            // span distances.
            if (line_buffer.VertexAtIndex(i).style_index() ==
                previous_style_index) {
              line_buffer.UpdateSpanDistancesForElement(i, previous_span_start,
                                                        previous_span_end);
            }
          }
        }
        // Reset previous style indices to those of the current line, now that
        // they are connected.
        previous_style_index = style_index;
      }
      current = next;
      next = EndPointForJoint(begin, num_points, orthogonal_offset_scale,
                              position, current, joint_index + 1, &joint_index);
    }
  }

  if (!loop) {
    // Extrude the end of the line, including any end cap.
    ExtrudeLineEnd(line_buffer, previous, current, distance, end_cap,
                   style_index, packed_zoom_range, index);
  } else {
    UpdateEndIndices(0, 1, line_buffer);
  }
}

}  // namespace imp::line_renderer
