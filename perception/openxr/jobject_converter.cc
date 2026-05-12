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

#include "openxr/jobject_converter.h"

#include <jni.h>
#include <openxr/openxr.h>
#include <openxr/public/all_extensions.h>

#include <cstdint>

#include "absl/strings/str_format.h"
#include "common/namespace_util.h"

namespace {
using ::androidx::xr::common::GetJxrClass;
using ::androidx::xr::common::GetJxrFullClassName;
using ::androidx::xr::common::PACKAGE_MATH;

XrVector3f ConvertToXrVector3f(JNIEnv* env, const jobject& vector3) {
  jclass vector3_cls = GetJxrClass(env, PACKAGE_MATH, "Vector3");
  jmethodID x_mid = env->GetMethodID(vector3_cls, "getX", "()F");
  jmethodID y_mid = env->GetMethodID(vector3_cls, "getY", "()F");
  jmethodID z_mid = env->GetMethodID(vector3_cls, "getZ", "()F");
  float x = env->CallFloatMethod(vector3, x_mid);
  float y = env->CallFloatMethod(vector3, y_mid);
  float z = env->CallFloatMethod(vector3, z_mid);
  return XrVector3f{.x = x, .y = y, .z = z};
}
}  // namespace

namespace androidx::xr::openxr {
XrQuaternionf ConvertToXrQuaternionf(JNIEnv* env, const jobject& quaternion) {
  jclass quaternion_cls = GetJxrClass(env, PACKAGE_MATH, "Quaternion");
  jmethodID x_mid = env->GetMethodID(quaternion_cls, "getX", "()F");
  jmethodID y_mid = env->GetMethodID(quaternion_cls, "getY", "()F");
  jmethodID z_mid = env->GetMethodID(quaternion_cls, "getZ", "()F");
  jmethodID w_mid = env->GetMethodID(quaternion_cls, "getW", "()F");
  float x = env->CallFloatMethod(quaternion, x_mid);
  float y = env->CallFloatMethod(quaternion, y_mid);
  float z = env->CallFloatMethod(quaternion, z_mid);
  float w = env->CallFloatMethod(quaternion, w_mid);
  return XrQuaternionf{.x = x, .y = y, .z = z, .w = w};
}

XrPosef ConvertToXrPosef(JNIEnv* env, const jobject& pose) {
  jclass pose_cls = GetJxrClass(env, PACKAGE_MATH, "Pose");
  jmethodID translation_mid = env->GetMethodID(
      pose_cls, "getTranslation",
      absl::StrFormat("()L%s;",
                      GetJxrFullClassName(env, PACKAGE_MATH, "Vector3"))
                      .c_str());
  jmethodID rotation_mid = env->GetMethodID(
      pose_cls, "getRotation",
      absl::StrFormat("()L%s;",
                      GetJxrFullClassName(env, PACKAGE_MATH, "Quaternion"))
                      .c_str());
  jobject translation_obj = env->CallObjectMethod(pose, translation_mid);
  jobject rotation_obj = env->CallObjectMethod(pose, rotation_mid);
  XrVector3f xr_vector3 = ConvertToXrVector3f(env, translation_obj);
  XrQuaternionf xr_quaternion = ConvertToXrQuaternionf(env, rotation_obj);
  return XrPosef{.orientation = xr_quaternion, .position = xr_vector3};
}

XrGeospatialPoseANDROID ConvertToXrGeospatialPose(
    JNIEnv* env, const jobject& geospatial_pose) {
  jclass geospatial_pose_cls = GetJxrClass(env, PACKAGE_MATH, "GeospatialPose");
  jmethodID latitude_mid =
      env->GetMethodID(geospatial_pose_cls, "getLatitude", "()D");
  jmethodID longitude_mid =
      env->GetMethodID(geospatial_pose_cls, "getLongitude", "()D");
  jmethodID altitude_mid =
      env->GetMethodID(geospatial_pose_cls, "getAltitude", "()D");
  jmethodID quaternion_mid = env->GetMethodID(
      geospatial_pose_cls, "getEastUpSouthQuaternion",
      absl::StrFormat("()L%s;",
                      GetJxrFullClassName(env, PACKAGE_MATH, "Quaternion"))
          .c_str());

  double latitude = env->CallDoubleMethod(geospatial_pose, latitude_mid);
  double longitude = env->CallDoubleMethod(geospatial_pose, longitude_mid);
  double altitude = env->CallDoubleMethod(geospatial_pose, altitude_mid);
  jobject quaternion_obj =
      env->CallObjectMethod(geospatial_pose, quaternion_mid);
  XrQuaternionf xr_quaternion = ConvertToXrQuaternionf(env, quaternion_obj);

  return XrGeospatialPoseANDROID{
      .eastUpSouthOrientation = xr_quaternion,
      .latitude = latitude,
      .longitude = longitude,
      .altitude = altitude,
  };
}

XrUuidEXT ConvertToXrUuid(JNIEnv* env, const jobject& uuid) {
  jclass uuid_cls = env->FindClass("java/util/UUID");
  jmethodID most_significant_bits_mid =
      env->GetMethodID(uuid_cls, "getMostSignificantBits", "()J");
  jmethodID least_significant_bits_mid =
      env->GetMethodID(uuid_cls, "getLeastSignificantBits", "()J");
  jlong most_significant_bits =
      env->CallLongMethod(uuid, most_significant_bits_mid);
  jlong least_significant_bits =
      env->CallLongMethod(uuid, least_significant_bits_mid);
  XrUuidEXT xr_uuid;
  const int bytes_per_fid = XR_UUID_SIZE / 2;
  for (int i = bytes_per_fid - 1; i >= 0; --i) {
    xr_uuid.data[i] = static_cast<uint8_t>(most_significant_bits & 0xff);
    most_significant_bits = most_significant_bits >> 8;
    xr_uuid.data[i + bytes_per_fid] =
        static_cast<uint8_t>(least_significant_bits & 0xff);
    least_significant_bits = least_significant_bits >> 8;
  }
  return xr_uuid;
}

XrSpace ConvertToXrSpace(const jlong& space) {
  static_assert(sizeof(XrSpace) <= sizeof(uint64_t));
  return reinterpret_cast<XrSpace>(static_cast<uint64_t>(space));
}

}  // namespace androidx::xr::openxr
