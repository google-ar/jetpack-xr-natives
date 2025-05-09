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

vec3 surface_curve_pos(vec3 model_pos, float curve_radius, float curve_angle, float model_x_scale) {
  if (model_x_scale == 0.0) {
    return model_pos;
  }
  // Calculate the angle based on the horizontal vertex coordinate.
  float angle = model_pos.x * curve_angle;

  // Calculate the new position on the cylinder.
  float x = sin(angle) * curve_radius / model_x_scale;
  float z = (1.0 - cos(angle)) * curve_radius;
  return vec3(x, model_pos.y, z);
}
