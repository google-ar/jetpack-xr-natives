// Copyright 2026 Google LLC
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
#include <memory>
#include <utility>
#include <vector>

#include "absl/strings/str_format.h"
#include "common/namespace_util.h"

namespace {
using ::androidx::xr::common::GetJxrClass;
using ::androidx::xr::common::GetJxrFullClassName;
using ::androidx::xr::common::PACKAGE_ARCORE_OPENXR;
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

XrTrackableImageTrackingModeANDROID ConvertToXrTrackingMode(int mode) {
  switch (mode) {
    case 0:
      return XrTrackableImageTrackingModeANDROID::
          XR_TRACKABLE_IMAGE_TRACKING_MODE_DYNAMIC_ANDROID;
    case 1:
      return XrTrackableImageTrackingModeANDROID::
          XR_TRACKABLE_IMAGE_TRACKING_MODE_STATIC_ANDROID;
    default:
      return XrTrackableImageTrackingModeANDROID::
          XR_TRACKABLE_IMAGE_TRACKING_MODE_MAX_ENUM_ANDROID;
  }
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

// Converts a JVM 'OpenXrAugmentedImageDatabaseEntry' object into:
// - an XrTrackableImageDatabaseEntryANDROID struct
// - the corresponding image data buffer (owned via unique_ptr)
AugmentedImageDatabaseEntry ConvertToAugmentedImageDatabaseEntry(
    JNIEnv* env, const jobject& entry) {
  jclass entry_class = GetJxrClass(
      env, PACKAGE_ARCORE_OPENXR,
      "OpenXrAugmentedImageDatabase$OpenXrAugmentedImageDatabaseEntry");
  jfieldID mode_field = env->GetFieldID(entry_class, "mode", "I");
  jfieldID width_field = env->GetFieldID(entry_class, "width", "I");
  jfieldID height_field = env->GetFieldID(entry_class, "height", "I");
  jfieldID buffer_size_field = env->GetFieldID(entry_class, "bufferSize", "I");
  jfieldID buffer_field = env->GetFieldID(entry_class, "buffer", "[B");
  jfieldID physical_width_field =
      env->GetFieldID(entry_class, "widthInMeters", "F");

  jint mode = env->GetIntField(entry, mode_field);
  jint width = env->GetIntField(entry, width_field);
  jint height = env->GetIntField(entry, height_field);
  jsize buffer_size = env->GetIntField(entry, buffer_size_field);
  jbyteArray buffer_array =
      (jbyteArray)env->GetObjectField(entry, buffer_field);
  auto buffer = std::make_unique<uint8_t[]>(buffer_size);
  env->GetByteArrayRegion(buffer_array, 0, buffer_size,
                          reinterpret_cast<jbyte*>(buffer.get()));
  jfloat physical_width = env->GetFloatField(entry, physical_width_field);

  XrTrackableImageDatabaseEntryANDROID xr_entry = {
      .type = XR_TYPE_TRACKABLE_IMAGE_DATABASE_ENTRY_ANDROID,
      .next = nullptr,
      .trackingMode = ConvertToXrTrackingMode(mode),
      .physicalWidth = physical_width,
      .imageWidth = static_cast<uint32_t>(width),
      .imageHeight = static_cast<uint32_t>(height),
      .format = XR_TRACKABLE_IMAGE_FORMAT_R8G8B8A8_ANDROID,
      .bufferSize = static_cast<uint32_t>(buffer_size),
      .buffer = buffer.get()};

  env->DeleteLocalRef(buffer_array);
  env->DeleteLocalRef(entry_class);

  return {xr_entry, std::move(buffer)};
}

// Converts a JVM 'OpenXrAugmentedImageDatabase' object into:
// - a vector of XrTrackableImageDatabaseEntryANDROID structs
// - a vector of corresponding image data buffers (owned via unique_ptr)
AugmentedImageDatabaseBuffers ConvertToAugmentedImageDatabaseEntryBufferPair(
    JNIEnv* env, const jobject& image_database) {
  std::vector<XrTrackableImageDatabaseEntryANDROID> entries;
  std::vector<std::unique_ptr<uint8_t[]>> buffers;

  jclass database_class =
      GetJxrClass(env, PACKAGE_ARCORE_OPENXR, "OpenXrAugmentedImageDatabase");
  jfieldID entries_field_id =
      env->GetFieldID(database_class, "entries", "Ljava/util/List;");
  jobject entries_list = env->GetObjectField(image_database, entries_field_id);
  jclass list_class = env->FindClass("java/util/List");
  jmethodID list_size_method = env->GetMethodID(list_class, "size", "()I");
  jmethodID list_get_method =
      env->GetMethodID(list_class, "get", "(I)Ljava/lang/Object;");
  jint size = env->CallIntMethod(entries_list, list_size_method);
  entries.reserve(size);
  buffers.reserve(size);

  for (jint i = 0; i < size; ++i) {
    jobject image = env->CallObjectMethod(entries_list, list_get_method, i);
    auto entry = ConvertToAugmentedImageDatabaseEntry(env, image);
    entries.push_back(entry.xr_entry);
    buffers.push_back(std::move(entry.buffer));
    env->DeleteLocalRef(image);
  }

  env->DeleteLocalRef(entries_list);
  env->DeleteLocalRef(database_class);
  env->DeleteLocalRef(list_class);

  return {std::move(entries), std::move(buffers)};
}

}  // namespace androidx::xr::openxr
