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

#include "core/math/quat.h"

#include <cmath>

#include "core/math/math.h"

namespace imp {

constexpr float kEpsilon = 1e-10;
constexpr float kPiOver2 = M_PI * 0.5;

quatf QuatFromEuler(const float3& eulers) {
  const quatf X = quatf::fromAxisAngle(kXAxis3f, ToRadians(eulers.x));
  const quatf Y = quatf::fromAxisAngle(kYAxis3f, ToRadians(eulers.y));
  const quatf Z = quatf::fromAxisAngle(kZAxis3f, ToRadians(eulers.z));

  return Y * X * Z;
}

float3 EulerFromQuat(const quatf& q) {
  float3 euler;

  constexpr float kSingularity = 0.5 - kEpsilon;
  float test = q.x * q.y + q.z * q.w;

  if (test > kSingularity) {  // singularity at north pole
    euler.y = ToDegrees(2 * atan2(q.x, q.w));
    euler.z = ToDegrees(M_PI / 2);
    euler.x = 0;
    return euler;
  }

  if (test < -kSingularity) {  // singularity at south pole
    euler.y = ToDegrees(-2 * atan2(q.x, q.w));
    euler.z = ToDegrees(-M_PI / 2);
    euler.x = 0;
    return euler;
  }

  float sqx = q.x * q.x;
  float sqy = q.y * q.y;
  float sqz = q.z * q.z;
  euler.y =
      ToDegrees(atan2(2 * q.y * q.w - 2 * q.x * q.z, 1 - 2 * sqy - 2 * sqz));
  euler.z = ToDegrees(asin(2 * test));
  euler.x =
      ToDegrees(atan2(2 * q.x * q.w - 2 * q.y * q.z, 1 - 2 * sqx - 2 * sqz));
  return euler;
}

float3 EulerFromQuatClamped(const quatf& quat) {
  float3 euler = EulerFromQuat(quat);
  return FloatModulo(euler, 360.0f, Clamp::kNonNegative);
}

bool IsYUp(quatf v) {
  constexpr auto kDotThreshold = 1.0f - 1.0e-3f;
  return dot(imp::kUp, v * imp::kUp) >= kDotThreshold;
}

}  // namespace imp
