// Copyright 2025 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

precision highp float;

struct QuadParallaxParams {
    vec2 uv;
    vec3 original_pos;
    float window_pop_dz;
    vec3 moved_window_pos;
    vec3 camera_pos;
    float extra_zoom_after_parallax;
    float max_media_inset;
    float parallax_amount;
};

vec2 quadParallaxUv(QuadParallaxParams p) {
    // All coordinates are in model space unless otherwise specified.
    // Photo/video is only rendered on the media plane - the original quad
    // vertices act as a window into the media.
    //
    //                                                view direction
    //                                                       -
    //    < Model Space >                                   /
    //                                                     /
    //          -z                ------------------------/----- media plane
    //                             |                     /
    //           |                 |                    /
    //           |        media_dz |                   /
    //           |                 |                  /    Original
    //           |                 |                 /    quad plane   | window_
    //  --------------------       |               /      (window)     | move_dz
    //  xy=0     |                 |              /                    |
    //           |                 |   ------------------ moved
    //           |       camera_dz |            /         quad plane
    //           |                 |           /          (window)
    //           |                 |          /
    //                             |         /
    //          +z                 |    camera_pos

    float media_dz = p.max_media_inset * p.parallax_amount;

    // media_scale is the size of the media plane relative to the original plane.
    // It scales with the depth of the media plane, with a little bit of
    // extra zoom to emphasize the parallax effect.
    float camera_dz = abs(getPosition().z - p.camera_pos.z);
    float media_scale = (camera_dz + media_dz) / camera_dz;
    media_scale = media_scale * mix(1., p.extra_zoom_after_parallax, p.parallax_amount);

    // Compute the view direction in uv space. To go from model space to uv
    // space, just flip the y axis as the vertices are already set at 0 and
    // 1 of xy axis in model space.
    vec3 view_direction_in_uv_space = normalize(p.moved_window_pos - p.camera_pos);
    view_direction_in_uv_space.y = -view_direction_in_uv_space.y;

    // Compute how much to laterally scale/translate the backing media.
    vec2 media_lateral_offset = view_direction_in_uv_space.xy / abs(view_direction_in_uv_space.z) * abs(media_dz + p.window_pop_dz);
    vec2 uv_center_offset = vec2(0.5);
    vec2 quad_parallax_uv = (p.uv - uv_center_offset + media_lateral_offset ) / media_scale + uv_center_offset;

    return quad_parallax_uv;
}
