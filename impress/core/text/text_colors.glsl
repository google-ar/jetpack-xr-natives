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

#ifndef THIRD_PARTY_ARCORE_AR_IMP_CORE_TEXT_TEXT_COLORS_H_
#define THIRD_PARTY_ARCORE_AR_IMP_CORE_TEXT_TEXT_COLORS_H_

vec3 srgbToLinear(vec3 srgb_color) {
  return pow(srgb_color, vec3(2.2));
}

#endif  // THIRD_PARTY_ARCORE_AR_IMP_CORE_TEXT_TEXT_COLORS_H_
