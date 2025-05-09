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

struct NoiseAnimParams {
    vec2 xy;
    float pulse_xy_scale;
    vec4 pulse_exponent;
    vec4 pulse_multiplier;
    float grain_xy_scale;
    float grain_mask_exponent;
    float grain_mask_multiplier;
    float grain_dim_amount;
    float grain_exponent;
    float grain_multiplier;
    float t;
    float time_scale;
};

// Helpers for noise effect.
float hash(float n) { return fract(sin(n) * 1e4); }
float hash(vec2 p) { return fract(1e4 * sin(17.0 * p.x + p.y * 0.1) * (0.1 + abs(sin(p.y * 13.0 + p.x)))); }

float noise(vec2 uv) {
  // Calculate integer and fractional parts of the input coordinates.
  vec2 i = floor(uv);
  vec2 f = fract(uv);

  // Apply smoothstep interpolation to the fractional part.
  vec2 smooth_f = f * f * (3.0 - 2.0 * f);

  // Calculate hash values at the four corners of the grid cell.
  float bottom_left = hash(i);
  float bottom_right = hash(i + vec2(1.0, 0.0));
  float top_left = hash(i + vec2(0.0, 1.0));
  float top_right = hash(i + vec2(1.0, 1.0));

  // Interpolate between the corner values using bilinear interpolation.
  float top = mix(top_left, top_right, smooth_f.x);
  float bottom = mix(bottom_left, bottom_right, smooth_f.x);
  return mix(bottom, top, smooth_f.y);
}

float undulatingNoise(vec2 uv, float t) {
  return noise(uv * 2.0 + t) * noise(uv - 0.3 - t) * noise(uv * 1.5 + 7. + sin(t));
}

// This effect adds an undulating wave of rgb colors with noise grain.
vec3 spatializeNoiseAnimation(NoiseAnimParams p) {
  // The constants below not captured as parameters are just arbitrary values to
  // offset each color channel.
  vec2 xy_pulse = p.xy * p.pulse_xy_scale;
  float pulse_red = pow(undulatingNoise(xy_pulse, p.t * p.time_scale), p.pulse_exponent.r);
  float pulse_green = pow(undulatingNoise(xy_pulse + 0.7, p.t * p.time_scale + 10.), p.pulse_exponent.g);
  float pulse_blue = pow(undulatingNoise(xy_pulse + 6.2, p.t * p.time_scale + 44.), p.pulse_exponent.b);
  float pulse_white = pow(undulatingNoise(xy_pulse + 10.5, p.t + 100.), p.pulse_exponent.a);

  vec3 colorful_wave =  vec3(pulse_red * p.pulse_multiplier.r,
                       pulse_green * p.pulse_multiplier.g,
                       pulse_blue * p.pulse_multiplier.b) +
                       pulse_white * p.pulse_multiplier.a;

  // Add shimmering grain on top of the colorful wave.
  vec2 xy_grain = p.xy * p.grain_xy_scale;
  float grain_mask = clamp(pow(undulatingNoise(p.xy + 3.3, p.t + 25.), p.grain_mask_exponent) * p.grain_mask_multiplier, 0.0, 1.0);
  float grain = pow((undulatingNoise(xy_grain, p.t) - p.grain_dim_amount) * p.grain_multiplier, p.grain_exponent)
                          * grain_mask;
  return colorful_wave + clamp(grain, 0.0, 1.0);
}
