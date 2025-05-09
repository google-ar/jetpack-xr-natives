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

#ifndef THIRD_PARTY_IMPRESS_CORE_PHYSICS_PHYSICS_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_PHYSICS_PHYSICS_HELPER_H_

#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "bullet/src/LinearMath/btQuaternion.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"

namespace imp {
// Functions in this file are used to convert math types between Impress types
// from/to Bullet types.

// Convert Bullet vector to Impress vector.
template <typename T>
TVec3<T> ToVec3(const btVector3& bt_vec) {
  return TVec3<T>(bt_vec.getX(), bt_vec.getY(), bt_vec.getZ());
}

// Convert a Impress vector to Bullet vector.
btVector3 ToBtVector3(const float3& vec);

// Convert a Bullet transformation to Impress transformation.
Transform<float> ToTransform(const btTransform& bt_trans);

// Convert a Impress transformation to Bullet transformation.
btTransform ToBtTransform(const float3& translation, const quatf& rotation);

// Convert a Bullet quaternion to Impress quaternion.
quatf ToQuaternion(const btQuaternion& bt_quat);

// Convert a Impress quaternion to Bullet quaternion.
btQuaternion ToBtQuaternion(const quatf& rotation);

// Return corresponding Impress transformation of a Bullet rigid body.
Transform<float> GetWorldTransform(const btRigidBody& rigid_body);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PHYSICS_PHYSICS_HELPER_H_
