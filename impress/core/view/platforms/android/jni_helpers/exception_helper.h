/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_JNI_HELPERS_EXCEPTION_HELPER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_JNI_HELPERS_EXCEPTION_HELPER_H_

#include <jni.h>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace imp::android {

// Checks the Status. If it contains an error it throws the corresponding Java
// exception and returns the original Status object.
absl::Status ThrowIfError(JNIEnv* env, const absl::Status& status);

// Checks the StatusOr. If it contains an error it throws the corresponding
// Java exception and returns the original StatusOr object.
template <typename T>
const absl::StatusOr<T>& ThrowIfError(JNIEnv* env,
                                      const absl::StatusOr<T>& status_or) {
  if (!status_or.ok()) {
    // absl::StatusOr.status() is [[nodiscard]] in the external version of absl.
    absl::Status status = ThrowIfError(env, status_or.status());
  }
  return status_or;
}

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_JNI_HELPERS_EXCEPTION_HELPER_H_
