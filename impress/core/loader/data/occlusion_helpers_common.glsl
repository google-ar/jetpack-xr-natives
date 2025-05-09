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

// Define compatibility functions for ESSL 1.0.
#if __VERSION__ == 100
mat4 transposeCompat(in mat4 i) {
  vec4 i0 = i[0];
  vec4 i1 = i[1];
  vec4 i2 = i[2];
  vec4 i3 = i[3];

  return mat4(vec4(i0.x, i1.x, i2.x, i3.x),
              vec4(i0.y, i1.y, i2.y, i3.y),
              vec4(i0.z, i1.z, i2.z, i3.z),
              vec4(i0.w, i1.w, i2.w, i3.w));
}
#else
#define transposeCompat transpose
#endif
