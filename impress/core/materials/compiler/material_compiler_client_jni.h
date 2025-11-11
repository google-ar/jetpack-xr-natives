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
#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_CLIENT_JNI_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_CLIENT_JNI_H_

#include <jni.h>

#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/async/future_common.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/materials/compiler/material_compiler_client.h"

namespace imp {

// Wraps the Java side MaterialCompilerClient class.
// Provides the MaterialCompilerService socket file descriptor that was created
// in Java.
class JavaMaterialCompilerClient : public JavaWrapper {
 public:
  JavaMaterialCompilerClient(const Context& context,
                             absl::string_view native_library_override);

  ~JavaMaterialCompilerClient() override;

  // Starts material compiler service in Android, by creating a new process.
  Future<int> StartService(const Context& context);

  // Triggered when the Android MaterialCompilerService is successfully created
  // and is ready for material compilation. fd will be zero and there will be an
  // error message when the connection fails.
  void OnServiceConnected(int fd, absl::string_view error_message);

  // Sets the native client that will be used to communicate with the service.
  void SetNativeClient(MaterialCompilerClient& native_client);

  // Disconnects from the service and stops the service process.
  // This is from C++ side signaling Java to close service, which eventually
  // calls OnClosed.
  void CloseService();

  // Triggered when the Java side service connection is closed.
  void OnClosed();

 private:
  jobject activity_context_;

  JniHandle start_service_method_;
  JniHandle close_method_;

  WeakFuture<int> service_connected_future_;
  MaterialCompilerClient* native_client_ = nullptr;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_COMPILER_MATERIAL_COMPILER_CLIENT_JNI_H_
