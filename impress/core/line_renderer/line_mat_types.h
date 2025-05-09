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

#ifndef THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_MAT_TYPES_H_
#define THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_MAT_TYPES_H_

#include "absl/strings/string_view.h"
#include "imp.h"  // IWYU pragma: export

namespace imp::line_renderer {

constexpr float GetMinLineWidth(const imp::Device& device) {
  return device.PixelsToPhysicalPixels(1.f) >= 2.f ? 0.f : 1.f;
}

struct LineVertexAttributes {
  // Named accessors into packed fields.
  imp::short4::reference style_index() {
    return style_indices_and_packed_zoom_range.x;
  }
  imp::short4::const_reference style_index() const {
    return style_indices_and_packed_zoom_range.x;
  }
  imp::short4::reference cap_index() {
    return style_indices_and_packed_zoom_range.y;
  }
  imp::short4::const_reference cap_index() const {
    return style_indices_and_packed_zoom_range.y;
  }
  imp::short4::reference packed_zoom_range() {
    return style_indices_and_packed_zoom_range.z;
  }
  imp::short4::const_reference packed_zoom_range() const {
    return style_indices_and_packed_zoom_range.z;
  }
  imp::short4::reference consumed_style_index() {
    return style_indices_and_packed_zoom_range.w;
  }
  imp::short4::const_reference consumed_style_index() const {
    return style_indices_and_packed_zoom_range.w;
  }
  // The position of the vertex in model space.
  imp::float3 pos;
  // Anti-aliasing texture coordinates.
  imp::float2 uv0;
  // The direction vector to extrude `position`. Use to determine the
  // position in the vertex shader.
  imp::float3 extrusion_vector;
  // .x: Index into the `styleProperties` texture for this vertex.
  // .y: Cap style index. Start caps are positive, while end caps are negative.
  // .z: Packed zoom range as returned from VertexBufferUtils::PackZoomRange.
  // .w: Unused.
  imp::short4 style_indices_and_packed_zoom_range;
  // The direction vector of a line at the vertex.
  imp::float3 direction;
  // The distance from the start of the line at the vertex.
  float distance;
  // The direction vector for which to apply an offset to the entire line. The
  // actual offset is scaled by an offset value from the style table.
  imp::float3 offset_direction;
};

inline imp::VertexFormat LineVertexFormat() {
  return {/*pos=*/{
              .attribute = imp::VertexFormat::VertexAttribute::POSITION,
              .type = imp::VertexFormat::AttributeType::FLOAT3,
          },
          /*uv0=*/
          {
              .attribute = imp::VertexFormat::VertexAttribute::UV0,
              .type = imp::VertexFormat::AttributeType::FLOAT2,
          },
          /*extrusion_vector=*/
          {
              .attribute = imp::VertexFormat::VertexAttribute::CUSTOM0,
              .type = imp::VertexFormat::AttributeType::FLOAT3,
          },
          /*style_indices_and_packed_zoom_range=*/
          {
              .attribute = imp::VertexFormat::VertexAttribute::CUSTOM1,
              .type = imp::VertexFormat::AttributeType::SHORT4,
          },
          /*direction=*/
          {
              .attribute = imp::VertexFormat::VertexAttribute::CUSTOM2,
              .type = imp::VertexFormat::AttributeType::FLOAT3,
          },
          /*distance=*/
          {
              .attribute = imp::VertexFormat::VertexAttribute::CUSTOM3,
              .type = imp::VertexFormat::AttributeType::FLOAT,
          },
          /*offset_direction=*/
          {
              .attribute = imp::VertexFormat::VertexAttribute::CUSTOM4,
              .type = imp::VertexFormat::AttributeType::FLOAT3,
          }};
}

/** Base parameters. **/

// int: The current stroke of the line.
constexpr absl::string_view kLineStrokeParameterKey = "stroke";

// int: The number of texels in the style texture dedicated per stroke.
constexpr absl::string_view kLineStyleTexelsPerStrokeParameterKey =
    "styleTexelsPerStroke";

// float: The number of pixels per model coordinate. Values in pixels may be
// multiplied by this value to convert them to model space.
constexpr absl::string_view kLineModelUnitsPerPixelParameterKey =
    "modelUnitsPerPixel";

// sampler2d: A single component texture that represents a series of dash
// patterns. Each row is a pattern. Each texel in the pattern is the alpha
// component at a point along a line.
constexpr absl::string_view kLineDashTextureParameterKey = "dashTexture";

// int2: The size of the dash texture in pixels.
constexpr absl::string_view kLineDashTextureSizeParameterKey =
    "dashTextureSize";

// float: The scale to apply to calculating the u-coordinate of the dash
// texture.
constexpr absl::string_view kLineDashScaleParameterKey = "dashScale";

// sampler2d: A texture which the alpha component represents a mask for
// anti-aliasing.
constexpr absl::string_view kLineAntiAliasingTextureParameterKey =
    "antiAliasingTexture";

// float: LOD to apply when sampling the antialias texture.
constexpr absl::string_view kLineAntiAliasingLodBiasParameterKey =
    "antiAliasingLodBias";

// float: The camera angle in radians.
constexpr absl::string_view kLineCameraTiltRadiansParameterKey =
    "cameraTiltRadians";

// float3: The direction vector of the camera.
constexpr absl::string_view kLineCameraDirectionWorldParameterKey =
    "cameraDirectionWorld";

// float: A multiplier for additional width applied to lines. Especially useful
// for low-re
constexpr absl::string_view kLineWidenIntensityParameterKey = "widenIntensity";

/** Line cap and Stamp parameters. **/

// An atlas containing textures used by the shader.
constexpr absl::string_view kLineTextureAtlasParameterKey = "textureAtlas";

// The size of the `textureAtlas` in texels.
constexpr absl::string_view kLineTextureAtlasSize = "textureAtlasSize";

// The a style texture describing the location of the start and end cap masks in
// the `textureAtlas`. It also describes the location of the texture stamped
// along the line in the `textureAtlas`. This texture has a row for each zoom
// level. For each zoom it has three texels. One for the start cap, one for the
// end cap, and one for the stamped texture. For each texel: x, y is the offset
// to apply in each dimension and zw is the scale. Use the following formula to
// access the region of the texture in the atlas given a uv coordinate:
// textureAtlasUV = uv * scale + offset
constexpr absl::string_view kLineCapAndStampInfoTextureParameterKey =
    "capAndStampInfoTexture";

// The size of `capAndStampInfoTexture` in texels.
constexpr absl::string_view kLineCapAndStampInfoTextureSizeParameterKey =
    "capAndStampInfoTextureSize";

constexpr absl::string_view kLineCapAndStampInfoTexelsPerStrokeParameterKey =
    "capAndStampInfoTexelsPerStroke";

// This set the repeating point of the stamped texture. In normalized texture
// coordinates [0, 1].
constexpr absl::string_view kLineStampOffset = "stampOffset";

// If the line width is less than this value, this value is used instead.
// This should be in half widths. If desired min width is 1px, provide
// 0.5.
constexpr absl::string_view kLineMinWidthParameterKey = "minLineWidth";

constexpr absl::string_view kConsumedDistanceKey = "consumedDistance";

}  // namespace imp::line_renderer

#endif  // THIRD_PARTY_IMPRESS_CORE_LINE_RENDERER_LINE_MAT_TYPES_H_
