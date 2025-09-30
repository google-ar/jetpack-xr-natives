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

#include "core/math/flatbuffer_support.h"

#include "core/common/schemas/math_generated.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"

namespace flatbuffers {

imp::schemas::Bool Pack(const bool &obj) { return imp::schemas::Bool(obj); }

bool UnPack(const imp::schemas::Bool &obj) { return obj.value(); }

imp::schemas::Float Pack(const float &obj) { return imp::schemas::Float(obj); }

float UnPack(const imp::schemas::Float &obj) {
  return static_cast<float>(obj.value());
}

imp::schemas::Float2 Pack(const imp::float2 &obj) {
  return imp::schemas::Float2(obj.x, obj.y);
}

imp::float2 UnPack(const imp::schemas::Float2 &obj) {
  return imp::float2(obj.x(), obj.y());
}

imp::schemas::Float3 Pack(const imp::float3 &obj) {
  return imp::schemas::Float3(obj.x, obj.y, obj.z);
}

imp::float3 UnPack(const imp::schemas::Float3 &obj) {
  return imp::float3(obj.x(), obj.y(), obj.z());
}

imp::schemas::Float4 Pack(const imp::float4 &obj) {
  return imp::schemas::Float4(obj.x, obj.y, obj.z, obj.w);
}

imp::float4 UnPack(const imp::schemas::Float4 &obj) {
  return imp::float4(obj.x(), obj.y(), obj.z(), obj.w());
}

imp::schemas::Double3 Pack(const imp::double3 &obj) {
  return imp::schemas::Double3(obj.x, obj.y, obj.z);
}

imp::double3 UnPack(const imp::schemas::Double3 &obj) {
  return imp::double3(obj.x(), obj.y(), obj.z());
}

imp::schemas::Quatf Pack(const imp::quatf &obj) {
  return imp::schemas::Quatf(obj.x, obj.y, obj.z, obj.w);
}

imp::quatf UnPack(const imp::schemas::Quatf &obj) {
  return imp::quatf(obj.w(), obj.x(), obj.y(), obj.z());
}

imp::schemas::Mat2f Pack(const imp::mat2f& obj) {
  const imp::float2& c0 = obj[0];
  const imp::float2& c1 = obj[1];
  return imp::schemas::Mat2f(c0[0], c0[1],  //
                             c1[0], c1[1]);
}

imp::mat2f UnPack(const imp::schemas::Mat2f& obj) {
  return imp::mat2f(obj.m00(), obj.m01(),  //
                    obj.m10(), obj.m11());
}

imp::schemas::Mat3f Pack(const imp::mat3f &obj) {
  const imp::float3 &c0 = obj[0];
  const imp::float3 &c1 = obj[1];
  const imp::float3 &c2 = obj[2];
  return imp::schemas::Mat3f(c0[0], c0[1], c0[2],  //
                             c1[0], c1[1], c1[2],  //
                             c2[0], c2[1], c2[2]);
}

imp::mat3f UnPack(const imp::schemas::Mat3f &obj) {
  return imp::mat3f(obj.m00(), obj.m01(), obj.m02(),  //
                    obj.m10(), obj.m11(), obj.m12(),  //
                    obj.m20(), obj.m21(), obj.m22());
}

imp::schemas::Mat4f Pack(const imp::mat4f &obj) {
  const imp::float4 &c0 = obj[0];
  const imp::float4 &c1 = obj[1];
  const imp::float4 &c2 = obj[2];
  const imp::float4 &c3 = obj[3];
  return imp::schemas::Mat4f(c0[0], c0[1], c0[2], c0[3],  //
                             c1[0], c1[1], c1[2], c1[3],  //
                             c2[0], c2[1], c2[2], c2[3],  //
                             c3[0], c3[1], c3[2], c3[3]);
}

imp::mat4f UnPack(const imp::schemas::Mat4f &obj) {
  return imp::mat4f(obj.m00(), obj.m01(), obj.m02(), obj.m03(),  //
                    obj.m10(), obj.m11(), obj.m12(), obj.m13(),  //
                    obj.m20(), obj.m21(), obj.m22(), obj.m23(),  //
                    obj.m30(), obj.m31(), obj.m32(), obj.m33());
}

imp::schemas::Mat4 Pack(const imp::mat4 &obj) {
  const imp::double4 &c0 = obj[0];
  const imp::double4 &c1 = obj[1];
  const imp::double4 &c2 = obj[2];
  const imp::double4 &c3 = obj[3];
  return imp::schemas::Mat4(c0[0], c0[1], c0[2], c0[3],  //
                            c1[0], c1[1], c1[2], c1[3],  //
                            c2[0], c2[1], c2[2], c2[3],  //
                            c3[0], c3[1], c3[2], c3[3]);
}

imp::mat4 UnPack(const imp::schemas::Mat4 &obj) {
  return imp::mat4(obj.m00(), obj.m01(), obj.m02(), obj.m03(),  //
                   obj.m10(), obj.m11(), obj.m12(), obj.m13(),  //
                   obj.m20(), obj.m21(), obj.m22(), obj.m23(),  //
                   obj.m30(), obj.m31(), obj.m32(), obj.m33());
}

imp::schemas::Transformf Pack(const imp::Transform<float> &obj) {
  return imp::schemas::Transformf(Pack(obj.translation), Pack(obj.rotation),
                                  Pack(obj.scale));
}

imp::Transform<float> UnPack(const imp::schemas::Transformf &obj) {
  return imp::Transform<float>(UnPack(obj.position()), UnPack(obj.rotation()),
                               UnPack(obj.scale()));
}

imp::schemas::PreciseTransform Pack(const imp::PreciseTransform &obj) {
  return imp::schemas::PreciseTransform(Pack(obj.translation),
                                        Pack(obj.rotation), Pack(obj.scale));
}

imp::PreciseTransform UnPack(const imp::schemas::PreciseTransform &obj) {
  return imp::PreciseTransform(UnPack(obj.position()), UnPack(obj.rotation()),
                               UnPack(obj.scale()));
}

}  // namespace flatbuffers
