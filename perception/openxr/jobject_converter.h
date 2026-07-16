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

#ifndef JETPACK_XR_NATIVES_OPENXR_JOBJECT_CONVERTER_H_
#define JETPACK_XR_NATIVES_OPENXR_JOBJECT_CONVERTER_H_

#include <jni.h>
#include <openxr/openxr.h>
#include <openxr/public/all_extensions.h>

#include <memory>
#include <utility>
#include <vector>

namespace androidx::xr::openxr {

// Returns an 'XrPosef' from an 'androidx/xr/math/Pose' JVM object.
XrPosef ConvertToXrPosef(JNIEnv* env, const jobject& pose);

// Returns an 'XrUuidEXT' from a 'java/util/UUID' JVM object.
XrUuidEXT ConvertToXrUuid(JNIEnv* env, const jobject& uuid);

// Returns an 'XrSpace' from a 'long' JVM object.
XrSpace ConvertToXrSpace(const jlong& space);

// Returns an 'XrQuaternionf' from an 'androidx/xr/math/Quaternion' JVM object.
XrQuaternionf ConvertToXrQuaternionf(JNIEnv* env, const jobject& quaternion);

// Returns an 'XrGeospatialPoseANDROID' from an
// 'androidx/xr/math/GeospatialPose' JVM object.
XrGeospatialPoseANDROID ConvertToXrGeospatialPose(
    JNIEnv* env, const jobject& geospatial_pose);

struct AugmentedImageDatabaseEntry {
  XrTrackableImageDatabaseEntryANDROID xr_entry;
  std::unique_ptr<uint8_t[]> buffer;
};

struct AugmentedImageDatabaseBuffers {
  std::vector<XrTrackableImageDatabaseEntryANDROID> entries;
  std::vector<std::unique_ptr<uint8_t[]>> buffers;
};

// Converts a JVM 'OpenXrAugmentedImageDatabaseEntry' object into:
// - an XrTrackableImageDatabaseEntryANDROID struct
// - the corresponding image data buffer (owned via unique_ptr)
AugmentedImageDatabaseEntry ConvertToAugmentedImageDatabaseEntry(
    JNIEnv* env, const jobject& entry);

// Converts a JVM 'OpenXrAugmentedImageDatabase' object into:
// - a vector of XrTrackableImageDatabaseEntryANDROID structs
// - a vector of corresponding image data buffers (owned via unique_ptr)
AugmentedImageDatabaseBuffers ConvertToAugmentedImageDatabaseEntryBufferPair(
    JNIEnv* env, const jobject& image_database);

}  // namespace androidx::xr::openxr

#endif  // JETPACK_XR_NATIVES_OPENXR_JOBJECT_CONVERTER_H_
