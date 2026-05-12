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

#include "core/physics/physics_helper.h"

#include "bullet/src/BulletDynamics/Dynamics/btRigidBody.h"
#include "bullet/src/LinearMath/btMatrix3x3.h"
#include "bullet/src/LinearMath/btQuaternion.h"
#include "bullet/src/LinearMath/btTransform.h"
#include "bullet/src/LinearMath/btVector3.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"

namespace imp {

btVector3 ToBtVector3(const float3& vec) {
  return btVector3(vec.x, vec.y, vec.z);
}

float3 ToFloat3(const btVector3& vec) {
  return float3(vec.x(), vec.y(), vec.z());
}

Transform<float> ToTransform(const btTransform& bt_trans) {
  Transform<float> transform;
  btQuaternion bt_quat = bt_trans.getRotation();
  transform.rotation = ToQuaternion(bt_quat);
  transform.translation = ToVec3<float>(bt_trans.getOrigin());
  return transform;
}

btTransform ToBtTransform(const float3& translation, const quatf& rotation) {
  btTransform bt_trans;
  bt_trans.setIdentity();
  bt_trans.setRotation(ToBtQuaternion(rotation));
  bt_trans.setOrigin(ToBtVector3(translation));
  return bt_trans;
}

mat4f ToMatrix(const btTransform& bt_trans) {
  const btMatrix3x3& basis = bt_trans.getBasis();
  const btVector3& origin = bt_trans.getOrigin();

  mat4f mat;

  for (int i = 0; i < 3; i++) {
    const float3 col = ToFloat3(basis.getColumn(i));
    mat[i] = float4(col, 0.0f);
  }
  mat[3] = float4(ToFloat3(origin), 1.0f);
  return mat;
}

quatf ToQuaternion(const btQuaternion& bt_quat) {
  return quatf(bt_quat.getW(), bt_quat.getX(), bt_quat.getY(), bt_quat.getZ());
}

btQuaternion ToBtQuaternion(const quatf& rotation) {
  return btQuaternion(rotation.x, rotation.y, rotation.z, rotation.w);
}

Transform<float> GetWorldTransform(const btRigidBody& rigid_body) {
  btTransform bt_trans;
  rigid_body.getMotionState()->getWorldTransform(bt_trans);
  return ToTransform(bt_trans);
}

}  // namespace imp
