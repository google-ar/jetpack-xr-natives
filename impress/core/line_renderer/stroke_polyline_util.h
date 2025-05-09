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

#ifndef THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_STROKE_POLYLINE_UTIL_H_
#define THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_STROKE_POLYLINE_UTIL_H_

#include "core/line_renderer/line_renderer.proto.imp.h"

namespace imp::line_renderer {

// The maximum number of segments when the rounded joint is 180 degrees.
constexpr int kMaxSegmentsPerRoundedJoint = 10;

// Returns the maximum number of vertices for a stroke with the given
// properties.
// `number_of_points` Number of points in the polyline for the stroke.
// `has_stamp` True if the stroke will have a repeating stamp texture,
// false if not.
// `start_cap` Type of start cap, from enum.
// `end_cap` Type of end cap, from enum.
// `joint_shape` The method in which the segments of the line are joined.
// `connect_to_previous` True if this stroke will be connected to a
// previous one, false if not.
// `loop` True if the line will be connected to its own start, or the start
// of the first line when grouped.
// Returns the maximum number of vertices needed for the stroke geometry.
int MaxVerticesForExtrudedStroke(
    int number_of_points, bool has_stamp = false,
    LineCapShape start_cap = LineCapShape::LINE_CAP_SHAPE_NONE,
    LineCapShape end_cap = LineCapShape::LINE_CAP_SHAPE_NONE,
    LineJointShape joint_shape = LineJointShape::LINE_JOINT_SHAPE_MITER,
    bool connect_to_previous = false, bool loop = false);

// Returns the maximum number of indices for a stroke with the given properties.
// `number_of_points` Number of points in the polyline for the stroke.
// `start_cap` Type of start cap, from enum.
// `end_cap` Type of end cap, from enum.
// `joint_shape` The method in which the segments of the line are joined.
// `connect_to_previous` True if this stroke will be connected to a
// previous one, false if not.
// `loop` True if the line will be connected to its own start, or the start
// of the first line when grouped.
// Returns the maximum number of indices needed for the stroke geometry.
int MaxIndicesForExtrudedStroke(
    int number_of_points,
    LineCapShape start_cap = LineCapShape::LINE_CAP_SHAPE_NONE,
    LineCapShape end_cap = LineCapShape::LINE_CAP_SHAPE_NONE,
    LineJointShape joint_shape = LineJointShape::LINE_JOINT_SHAPE_MITER,
    bool connect_to_previous = false, bool loop = false);

}  //  namespace imp::line_renderer

#endif  // THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_STROKE_POLYLINE_UTIL_H_
