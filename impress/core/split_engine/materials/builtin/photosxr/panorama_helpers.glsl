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

float angleBetween(vec3 a, vec3 b) {
  return acos(dot(normalize(a), normalize(b)));
}

// For panorama media, the mesh will start off as a high-density quad and
// morph into a photosphere when transitioning to immersive mode.
vec3 morphToSpherePos(vec3 model_pos, vec3 camera_pos, float morph_amount, float pano_sphere_radius) {
  float start_radius = length(vec3(0.0, 0.0, 0.0) - camera_pos);
  float r = mix(start_radius, pano_sphere_radius, morph_amount);
  vec2 normalized_quad_xy = model_pos.xy / 0.5;

  float min_theta = abs(angleBetween(vec3(0.0) - camera_pos, vec3(1.0, 0.0, 0.0) - camera_pos));
  float min_phi = abs(angleBetween(vec3(0.0) - camera_pos, vec3(0.0, 1.0, 0.0) - camera_pos));

  float theta =  normalized_quad_xy.y * mix(min_theta, PI / 2., morph_amount);
  float phi = normalized_quad_xy.x * mix(min_phi, PI, morph_amount);

  vec3 sphere_pos = vec3(r * sin(phi) * cos(theta),
                      r * sin(theta),
                      -(r * cos(phi) * cos(theta))) + camera_pos;

  vec3 morphed_pos = mix(model_pos, sphere_pos, morph_amount);

  return morphed_pos;
}

// Compute the uv coordinates necessary to give the effect that the morphed
// quad is a window into the virtual photosphere.
//             \----  --
//                      \--- -
//           virtual sphere    --
//                               \-
//                                  --
//                                    \-
//                                      --
//                                       \-
//                camera_pos       |      -\-
//                & camera_ray     |        \-
//              +-+ -------------------------| sphere_pos
//              +|+      radius    |         \-
//               |                 |          |
//              /|\                |          /
//            /- | \         vertices         /
//           -   |  -        (morphed_pos)    |
//               |                           /-
//              /\                           /
//             /  \                         /
//  -------------------------------------------|
vec2 panoParallaxUv(vec3 morphed_pos, vec3 camera_pos) {
  // Calculate the position of where the camera ray hits a virtual sphere at
  // radius near infinity.
  // Radius just needs to be a large enough constant to simulate infinity plane.
  float radius = 100000.0;
  vec3 camera_ray = morphed_pos - camera_pos;
  vec3 sphere_pos = camera_pos + radius * normalize(camera_ray);

  // Calculate theta and phi of the cartasian sphere position.
  float theta, phi;
  theta = asin(sphere_pos.y / radius);
  // If too close to extremities, set phi to a constant.
  if (abs(sphere_pos.y - radius - camera_pos.y) < 1e-6
      || abs(sphere_pos.y + radius - camera_pos.y) < 1e-6) {
      phi = 0.0;
  }
  else {
      phi = atan(sphere_pos.x, -sphere_pos.z);
  }

  // Obtain the normalized quad coordinates as if this virtual sphere has been
  // de-morphed to a quad, and convert to uv coordinates.
  vec2 normalized_quad_xy;
  normalized_quad_xy.x = clamp(phi / PI, -1.0, 1.0);
  normalized_quad_xy.y = clamp(-2.0 * theta / PI, -1.0, 1.0);
  vec2 uv_on_sphere = (normalized_quad_xy + 1.0) / 2.0;


  return clamp(uv_on_sphere, 0.0, 1.0);
}

// Tile the uv coordinates according to the pano fov.
vec2 tileUvToPanoFov(vec2 uv, float horizontal_fov_degrees,
                         float vertical_top_edge_degrees,
                         float vertical_bottom_edge_degrees) {
  float vertical_fov_degrees = vertical_top_edge_degrees
                                    - vertical_bottom_edge_degrees;
  vec2 tiling = vec2(360.0 / horizontal_fov_degrees, 180.0 / vertical_fov_degrees);
  uv.xy *= tiling.xy;
  uv.x -= (tiling.x - 1.0) / 2.0;
  uv.y -= ((tiling.y / 2.0) * (1. - vertical_top_edge_degrees / 90.0));
  return clamp(uv, 0.0, 1.0);
}

struct FadePanoBlurParams {
  vec2 uv;
  vec3 blur_color;
  vec3 fallback_color;
  vec2 color_mix_multiplier;
  vec2 color_mix_subtractor;
  float pano_spherical_amount;
};

vec3 panoFadeBlurToColor(FadePanoBlurParams p) {
    vec2 dxdy = abs(p.uv- 0.5) / 0.5;
    float color_mix_x = dxdy.x * p.color_mix_multiplier.x - p.color_mix_subtractor.x;
    float color_mix_y = dxdy.y * p.color_mix_multiplier.y - p.color_mix_subtractor.y;
    float color_mix = clamp(max(color_mix_x, color_mix_y), 0.0, 1.0) * p.pano_spherical_amount;
    return mix(p.blur_color, p.fallback_color, color_mix);
}

