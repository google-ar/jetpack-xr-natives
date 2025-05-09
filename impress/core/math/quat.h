/*
 * Copyright 2024 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_QUAT_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_QUAT_H_

#include <string>
#include <type_traits>

#include "core/common/string_helpers.h"
#include "core/common/type_helpers.h"
#include "core/math/vec.h"
// IWYU pragma: begin_exports
#include "filament/libs/math/include/math/TQuatHelpers.h"
#include "filament/libs/math/include/math/quat.h"
// IWYU pragma: end_exports

namespace imp {

using quat = ::filament::math::quat;
using quatf = ::filament::math::quatf;

constexpr quat kIdentityQuat(1, 0, 0, 0);
constexpr quatf kIdentityQuatf(1.f, 0.f, 0.f, 0.f);

template <typename T>
using TQuaternion = ::filament::math::details::TQuaternion<T>;

// Helper to convert from euler angles to a quaternion.  This uses (x,y,z) as
// (pitch, yaw, roll), and uses a YXZ rotation order.  YXZ can be thought of as
// Turn to face the target(y), then look up or down(x), then tilt your head(z).
// The euler angles are passed in degrees.
quatf QuatFromEuler(const float3& eulers);

// Helper to convert from a quaternion to euler angles. This uses (x,y,z) as
// (pitch, yaw, roll).
// The angles are returned in degrees.
//
// Please note that this method doesn't guarantee non-negative euler angles. Use
// `EulerFromQuatClamped` if you need non-negative euler angles.
float3 EulerFromQuat(const quatf& q);

// Helper to convert from a quaternion to euler angles with degrees between 0
// and 360. This uses (x,y,z) as (pitch, yaw, roll).
// The angles are returned in degrees.
float3 EulerFromQuatClamped(const quatf& quat);

bool IsYUp(quatf v);

template <typename T>
using EnableIfQuaternion = std::enable_if_t<kIsAnyOf<T, quatf, quat>, int>;

// Note: without EnableIfQuaternion, the compiler will try to instantiate this
// template for all numerical types, including non-quaternion types.
template <typename T, EnableIfQuaternion<T> = 0>
std::string ToString(T v) {
  return FormatString("<% .3f, % .3fi, % .3fj, % .3fk>", v[3], v[0], v[1],
                      v[2]);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_QUAT_H_
