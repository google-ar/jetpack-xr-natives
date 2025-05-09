// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/math/math.h"

#include <cmath>
#include <sstream>
#include <string>

#include "filament/libs/mathio/include/mathio/ostream.h"

namespace imp {

std::string ToString(const Box& v) {
  std::ostringstream ss;
  ss << "Box(center: " << v.center << ", halfExtent: " << v.halfExtent << ")";
  return ss.str();
}

mat3f MatrixFromUvTransform(float2 offset, float rotation, float2 scale) {
  float tx = offset.x;
  float ty = offset.y;
  float sx = scale.x;
  float sy = scale.y;
  float c = cos(rotation);
  float s = sin(rotation);
  return mat3f(sx * c, sx * s, tx,   // ^
               -sy * s, sy * c, ty,  // ^
               0, 0, 1);
}

void UvTransformFromMatrix(const mat3f& matrix, float2& offset, float& rotation,
                           float2& scale) {
  const float* data = matrix.asArray();
  offset.x = data[2];
  offset.y = data[5];
  scale.x = sqrt(data[0] * data[0] + data[1] * data[1]);

  if (scale.x != 0) {
    float c = data[0] / scale.x;
    if (c != 0) {
      rotation = acos(c);
      scale.y = data[4] / c;
    } else {
      float s = data[1] / scale.x;
      rotation = asin(s);
      scale.y = -data[3] / s;
    }
  } else {
    scale.y = sqrt(data[3] * data[3] + data[4] * data[4]);
    if (scale.y != 0) {
      float c = data[4] / scale.y;
      if (c != 0) {
        rotation = acos(c);
      } else {
        rotation = asin(-data[3] / scale.y);
      }
    } else {
      // Since scale.x and scale.y both equal to 0, the rotation is not stored
      // in the matrix, therefore, set the rotation to default.
      rotation = 0;
    }
  }
}

}  // namespace imp
