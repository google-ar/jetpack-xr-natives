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

// Provides functionality for super sampling using 4 samples

// TODO: Macro needed because filament doesn't support samplerExternal types as a
// function parameter. We should move back to using a function once this is supported.
#define SUPER_SAMPLE_INTO(tex, uv, result)                      \
    {                                                           \
        vec2 _ss_x = dFdx(uv) * 0.4;                            \
        vec2 _ss_y = dFdy(uv) * 0.4;                            \
        vec4 _ss_a = texture(tex, uv);                          \
        vec4 _ss_b = texture(tex, uv + _ss_x);                  \
        vec4 _ss_c = texture(tex, uv - _ss_x);                  \
        vec4 _ss_d = texture(tex, uv + _ss_y);                  \
        vec4 _ss_e = texture(tex, uv - _ss_y);                  \
        result = (_ss_a + _ss_b + _ss_c + _ss_d + _ss_e) / 5.0; \
    }
