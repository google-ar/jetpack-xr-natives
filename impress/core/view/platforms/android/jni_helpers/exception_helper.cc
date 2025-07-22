// Copyright 2025 Google LLC
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

#include "core/view/platforms/android/jni_helpers/exception_helper.h"

#include <jni.h>

#include <string>

#include "absl/container/flat_hash_map.h"
#include "core/common/log.h"
#include "absl/status/status.h"

namespace {

constexpr char kCancellationExceptionClassName[] =
    "java/util/concurrent/CancellationException";
constexpr char kIllegalArgumentExceptionClassName[] =
    "java/lang/IllegalArgumentException";
constexpr char kTimeoutExceptionClassName[] =
    "java/util/concurrent/TimeoutException";
constexpr char kIllegalStateExceptionClassName[] =
    "java/lang/IllegalStateException";
constexpr char kSecurityExceptionClassName[] = "java/lang/SecurityException";
constexpr char kIndexOutOfBoundsExceptionClassName[] =
    "java/lang/IndexOutOfBoundsException";
constexpr char kUnsupportedOperationExceptionClassName[] =
    "java/lang/UnsupportedOperationException";
constexpr char kIOExceptionClassName[] = "java/io/IOException";
constexpr char kRuntimeExceptionClassName[] = "java/lang/RuntimeException";

void ThrowJavaExceptionFromStatus(JNIEnv* env, const char* class_name,
                                  const absl::Status& status) {
  if (status.ok()) {
    return;
  }

  std::string message = status.ToString();
  jclass exception_class = env->FindClass(class_name);
  if (exception_class != nullptr) {
    env->ThrowNew(exception_class, message.c_str());
  } else {
    IMP_LOG(imp::ERROR) << "Java exception was not found and cannot be thrown.";
    env->ExceptionClear();
    env->FatalError(message.c_str());
  }
}

}  // namespace

namespace imp::android {

absl::Status ThrowIfError(JNIEnv* env, const absl::Status& status) {
  if (status.ok()) {
    return status;
  }

  if (env->ExceptionCheck()) {
    IMP_LOG(imp::WARNING)
        << "ThrowIfError was called while a previously requested JNI exception"
        << "is already pending to be thrown.";
    return status;
  }

  static const absl::flat_hash_map<absl::StatusCode, const char*>
      kStatusToJavaExceptionClass = {
          {absl::StatusCode::kCancelled, kCancellationExceptionClassName},
          {absl::StatusCode::kInvalidArgument,
           kIllegalArgumentExceptionClassName},
          {absl::StatusCode::kNotFound, kIllegalArgumentExceptionClassName},
          {absl::StatusCode::kDeadlineExceeded, kTimeoutExceptionClassName},
          {absl::StatusCode::kAlreadyExists, kIllegalStateExceptionClassName},
          {absl::StatusCode::kResourceExhausted,
           kIllegalStateExceptionClassName},
          {absl::StatusCode::kFailedPrecondition,
           kIllegalStateExceptionClassName},
          {absl::StatusCode::kAborted, kIllegalStateExceptionClassName},
          {absl::StatusCode::kPermissionDenied, kSecurityExceptionClassName},
          {absl::StatusCode::kUnauthenticated, kSecurityExceptionClassName},
          {absl::StatusCode::kOutOfRange, kIndexOutOfBoundsExceptionClassName},
          {absl::StatusCode::kUnimplemented,
           kUnsupportedOperationExceptionClassName},
          {absl::StatusCode::kUnavailable, kIOExceptionClassName},
          {absl::StatusCode::kInternal, kRuntimeExceptionClassName},
          {absl::StatusCode::kDataLoss, kRuntimeExceptionClassName},
          {absl::StatusCode::kUnknown, kRuntimeExceptionClassName},
      };

  const auto it = kStatusToJavaExceptionClass.find(status.code());
  const char* class_name = (it != kStatusToJavaExceptionClass.end())
                               ? it->second
                               : kRuntimeExceptionClassName;

  ThrowJavaExceptionFromStatus(env, class_name, status);

  return status;
}

}  // namespace imp::android
