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
#include "core/materials/compiler/material_compiler_client_jni.h"

#include <jni.h>

#include <optional>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/materials/compiler/material_compiler_client.h"

namespace imp {

template <class T>
using MaterialCompilerClientAllowlist =
    JniAllowlist<T, JavaMaterialCompilerClient>;

template <class T>
constexpr auto ToJava = &MaterialCompilerClientAllowlist<T>::ToJava;

template <class T>
constexpr auto FromJava = &MaterialCompilerClientAllowlist<T>::FromJava;

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_materialcompiler_MaterialCompilerClient_##method_name  // NOLINT

extern "C" {

// LINT.IfChange(native_api)

JNI_METHOD(void, nOnServiceConnected)
(JNIEnv* env, jclass /*clazz*/, jlong java_client_handle, jint fd,
 jstring error_message) {
  JavaMaterialCompilerClient* client =
      FromJava<JavaMaterialCompilerClient>(java_client_handle);
  if (client) {
    client->OnServiceConnected(fd, GetString(env, error_message));
  }
}

JNI_METHOD(void, nClose)
(JNIEnv* env, jclass /*clazz*/, jlong java_client_handle) {
  JavaMaterialCompilerClient* client =
      FromJava<JavaMaterialCompilerClient>(java_client_handle);
  if (client) {
    client->OnClosed();
  }
}

// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/materialcompiler/MaterialCompilerClient.java:native_api
// )

}  // extern "C"

JavaMaterialCompilerClient::JavaMaterialCompilerClient(const Context& context)
    : JavaWrapper(
          context, "com/google/ar/imp/materialcompiler/MaterialCompilerClient",
          "(Landroid/content/Context;J)V", context.GetActivityContext(), this),
      activity_context_(context.GetActivityContext()) {
  // Set up method handles.
  start_service_method_ = GetMethodHandle("startService", "()V");
  close_method_ = GetMethodHandle("close", "()V");
}

JavaMaterialCompilerClient::~JavaMaterialCompilerClient() {
  if (Self()) {
    CloseService();
  }
};

// Kicks off the shader compilation service. Returns a non zero file
// descriptor if the service was successfully connected.
Future<int> JavaMaterialCompilerClient::StartService(const Context& context) {
  CallVoidMethod(start_service_method_);
  Future<int> connected_future;

  service_connected_future_ = connected_future;

  return connected_future.Then(
      [this](absl::StatusOr<int> fd) -> absl::StatusOr<int> {
        if (!fd.ok()) {
          CloseService();
          return fd.status();
        }
        return *fd;
      });
}

void JavaMaterialCompilerClient::OnServiceConnected(
    int fd, absl::string_view error_message) {
  std::optional<Future<int>> future = service_connected_future_.Lock();

  if (!future.has_value()) return;

  if (fd != 0) {
    future->Return(fd);
  } else {
    future->Return(absl::InternalError(error_message));
  }
}

void JavaMaterialCompilerClient::SetNativeClient(
    MaterialCompilerClient& native_client) {
  native_client_ = &native_client;
}

// Disconnects from the service and stops the service process.
// This is from C++ side signaling Java to close service, which eventually
// calls OnClosed.
void JavaMaterialCompilerClient::CloseService() {
  CallVoidMethod(close_method_);
}
// TODO: Make close robust; Close can happen bi-directional,
// either from Java or C++.
void JavaMaterialCompilerClient::OnClosed() {
  if (native_client_ != nullptr) {
    native_client_->Close();
    native_client_ = nullptr;
  }
}

}  // namespace imp
