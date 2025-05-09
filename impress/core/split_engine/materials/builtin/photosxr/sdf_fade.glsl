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

float sdRoundedBox(vec2 p, vec2 size, float r) {
    vec2 q = abs(p)-size+r;
    return min(max(q.x,q.y),0.0) + length(max(q,0.0)) - r;
}

  // Using a Signed Distance Field (SDF), fade the edges of a rectangle with
  // rounded corners, following the diagram below.
  // Diagram assumes |corner_radius| = 0.0.
  //                                               rect_dim
  // +----------------------------------------------------+
  // |                                                    |
  // |                                   base_rect_dim    |
  // |       +-----------------------------------+        |
  // |       |         |                         |        |
  // |       |         |fade_edge_thickness      |        |
  // |       |         |                         |        |
  // |       |      +--------------------+       |        |
  // |       |      |                    |       |        |
  // |       |      |                    |       |        |
  // |       |      |                    |       |        |
  // |       |      |                    |       |        |
  // |       |      |                    |       |        |
  // |       |      |                    |       |        |
  // |       |      |                    |       |        |
  // |       |      |                    |       |        |
  // |       |      |                    |       |        |
  // |       |      |                    |       |        |
  // |       |      | opacity=0.0        |       |        |
  // |       |      +--------------------+       |        |
  // |       |                                   |        |
  // |       |    opacity=0.5                    |        |
  // |       |                                   |        |
  // |       +-----------------------------------+        |
  // |       opacity=1.0                                  |
  // |                                                    |
  // |                                                    |
  // +----------------------------------------------------+
float edgeFadeAlpha(vec2 uv, vec2 rect_dim, float edge_fade_thickness, float base_rect_factor, float corner_radius) {
  vec2 uv_centered_and_scaled = ((uv - vec2(0.5, 0.5)) * 2.0) * rect_dim;

  vec2 base_rect_dim = rect_dim * base_rect_factor - vec2(edge_fade_thickness);
  float sdf_to_base_rect = sdRoundedBox(uv_centered_and_scaled, base_rect_dim, corner_radius);

  if (edge_fade_thickness == 0.0) {
    return sdf_to_base_rect > 0.0 ? 1.0 : 0.0;
  }

  float faded_edge_opacity = clamp(clamp(sdf_to_base_rect, 0., 1.) / edge_fade_thickness, 0.0, 1.0);
  faded_edge_opacity = smoothstep(0.0, 1.0, faded_edge_opacity);
  return faded_edge_opacity;
}
