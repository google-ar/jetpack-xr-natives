/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_JAVA_LOADER_CLIENT_JNI_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_JAVA_LOADER_CLIENT_JNI_H_

#include <jni.h>

#include "absl/memory/memory.h"
#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/loader/ipc/loader_client_base.h"

namespace imp::loader::ipc {

template <class T>
using LoaderClientAllowlist = JniAllowlist<T, LoaderClientBase>;

template <class T>
constexpr auto ToJava = &LoaderClientAllowlist<T>::ToJava;

template <class T>
constexpr auto FromJava = &LoaderClientAllowlist<T>::FromJava;

// Wraps Javas Boolean class.
// TODO Consolidate android java wrappers
class Boolean : public JavaWrapper {
 public:
  // Wraps an existing object as a boolean.
  Boolean(JNIEnv* env, jobject value);

  // Returns the boolean value.
  bool GetValue();

  // Verifies an object is an instance of Boolean.
  static bool IsInstanceOfBoolean(JNIEnv* env, jobject object);

 private:
  // Provides an uninstantiated Boolean for static calls.
  explicit Boolean(JNIEnv* env);
  imp::JniHandle get_value_method_;
};  // namespace imp::loader::ipc

// LINT.IfChange(api)
// Wraps Javas CompletableFuture class.
// TODO Consolidate android java wrappers
class CompletableFuture : public JavaWrapper {
 public:
  CompletableFuture(JNIEnv* env, jobject future);

  // Calls the Get() method of the future and converts the return into a bool.
  jboolean GetBoolean();

 private:
  imp::JniHandle get_method_;
};

// Wraps the Java side LoaderClient class
// Provides the LoaderService socket file descriptor that was created in Java.
class JavaLoaderClient : public JavaWrapper {
 public:
  JavaLoaderClient();
  JavaLoaderClient(JNIEnv* env, jobject activity_context);
  ~JavaLoaderClient() override;
  // Creates a loader service, blocks until the client has connected to the
  // service. This should be called from a background thread.
  bool ConnectToLoaderService(const Context& context);

  // Returns the file descriptor created by constructing this object.
  int GetClientSocketFileDescriptor();

  void SetNativeHandler(LoaderClientBase* native_client);

  // Disconnects from the service and stops the service process.
  void Disconnect();

 private:
  void SetupMethodHandles();

  // Starts the loader service, returns a java Future that indicates when the
  // service connection has been established.
  jobject StartLoaderServiceHelper();

  std::atomic_bool service_connected_ = false;
  jobject activity_context_;
  imp::JniHandle start_loader_service_method_;
  imp::JniHandle get_client_socket_fd_;
  imp::JniHandle close_method_;
  imp::JniHandle disconnect_method_;
  imp::JniHandle set_native_handler_method_;
};
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/view/ipc/LoaderClient.java:api
// )
}  // namespace imp::loader::ipc
#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_IPC_JAVA_LOADER_CLIENT_JNI_H_
