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

#ifndef THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_EXTRUDER_3D_H_
#define THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_EXTRUDER_3D_H_

#include <cstdint>

#include "core/line_renderer/line_buffer.h"
#include "core/line_renderer/line_renderer.proto.imp.h"
#include "core/line_renderer/polyline.h"
#include "imp.h"  // IWYU pragma: export

namespace imp::line_renderer {

// Extrudes a 3D line with both positions and normals.
// Generates all geometry and appends vertices and indices to the buffers.
// `line_buffer` Buffer that will hold line vertices and indices.
// `polyline` The polyline to extrude.
// `joint_shape` The shape of the joints to generate.
// `start_cap` Type of start cap.
// `end_cap` Type of end cap.
// `has_stamps` True if the stroke will have a repeating stamp texture.
// `style_index` Index of line style to save in each vertex.
// `packed_zoom_range` The min and max zooms to display the line at, as returned
//  by VertexBufferUtils::PackZoomRange.
// `start_distance` Initial distance, in world units.
// `total_unclipped_distance` Total distance before any clipping, in world
// units.
// `connect_to_previous` True if this stroke will be connected to a
// previous one.
// `loop_self` True to loop this line back on itself.
// `loop_first` True to loop this line back on the first line in the buffer.
// `orthogonal_offset_scale` Scale of the offset by which the line is shifted
// orthogonally. Positive is to the left and negative to the right.
// `position` The position of the output coordinate system.
void ExtrudeLine3D(LineBuffer& line_buffer, const Polyline3f& polyline,
                   LineJointShape joint_shape, LineCapShape start_cap,
                   LineCapShape end_cap, bool has_stamps, uint16_t style_index,
                   uint16_t packed_zoom_range, float start_distance,
                   float total_unclipped_distance, bool connect_to_previous,
                   bool loop_self, bool loop_first,
                   float orthogonal_offset_scale, const imp::float3& position);

}  // namespace imp::line_renderer

#endif  // THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_EXTRUDER_3D_H_
