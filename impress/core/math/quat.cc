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

constexpr float kEpsilon = 1e-6;

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

float3 EulerFromQuatYXZ(const quatf& q) {
  float3 euler;

  constexpr float kSingularityThreshold = 1.0f - kEpsilon;
  float sin_pitch = -2.0f * (q.y * q.z - q.w * q.x);

  // For the singularity (gimbal lock) cases, is it not possible to recover
  // separate Yaw and Roll angles, and so the final Roll angle represents the
  // combined rotation of both the Yaw and Roll axes.
  //
  // Imagine using `QuatFromEuler` to construct a quaternion `q`. If the
  // original angles were pitch=90, yaw=20, roll=30, then the final Roll angle
  // of `EulerFromQuat(q)` will be 30 - 20 = 10 degrees, as a positive Yaw
  // rotation is the same as a negative Roll rotation when the orientation is
  // up. On the other hand, if the angles were pitch=-90, yaw=20, roll=30, then
  // the final Roll will be 30 + 20 = 50 degrees, as a positive Yaw rotation is
  // the same as a positive Roll rotation when the orientation is down.

  if (sin_pitch > kSingularityThreshold) {  // singularity at north pole
    euler.z = ToDegrees(-2 * atan2(q.y, q.w));
    euler.x = 90;
    euler.y = 0;
    return euler;
  }

  if (sin_pitch < -kSingularityThreshold) {  // singularity at south pole
    euler.z = ToDegrees(2 * atan2(q.y, q.w));
    euler.x = -90;
    euler.y = 0;
    return euler;
  }

  float sqx = q.x * q.x;
  float sqy = q.y * q.y;
  float sqz = q.z * q.z;

  euler.z =
      ToDegrees(atan2(2 * (q.x * q.y + q.w * q.z), 1 - 2 * sqx - 2 * sqz));
  euler.x = ToDegrees(asin(sin_pitch));
  euler.y =
      ToDegrees(atan2(2 * (q.y * q.w + q.x * q.z), 1 - 2 * sqx - 2 * sqy));
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
