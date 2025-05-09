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

#ifndef JETPACK_XR_NATIVES_OPENXR_JOBJECT_CONVERTER_H_
#define JETPACK_XR_NATIVES_OPENXR_JOBJECT_CONVERTER_H_

#include <jni.h>

#include <openxr/openxr.h>

namespace androidx::xr::openxr {

// Returns an 'XrPosef' from an 'androidx/xr/math/Pose' JVM object.
XrPosef ConvertToXrPosef(JNIEnv* env, const jobject& pose);

// Returns an 'XrUuidEXT' from a 'java/util/UUID' JVM object.
XrUuidEXT ConvertToXrUuid(JNIEnv* env, const jobject& uuid);

// Returns an 'XrSpace' from a 'long' JVM object.
XrSpace ConvertToXrSpace(const jlong& space);

}  // namespace androidx::xr::openxr

#endif  // JETPACK_XR_NATIVES_OPENXR_JOBJECT_CONVERTER_H_
