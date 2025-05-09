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

#include "core/loader/ipc/java_loader_client_jni.h"

using ::imp::loader::ipc::FromJava;
using ::imp::loader::ipc::LoaderClientBase;

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_view_ipc_LoaderClient_##method_name

extern "C" {

// LINT.IfChange(api)

JNI_METHOD(void, nHandleClose)
(JNIEnv* env, jclass /*clazz*/, jlong native_client) {
  LoaderClientBase* loader_client = FromJava<LoaderClientBase>(native_client);
  if (loader_client) {
    loader_client->Close(true);
  }
}

JNI_METHOD(void, nHandleDisconnect)
(JNIEnv* env, jclass /*clazz*/, jlong native_client) {
  LoaderClientBase* loader_client = FromJava<LoaderClientBase>(native_client);
  if (loader_client) {
    loader_client->EnsureClosed();
  }
}

// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/view/ipc/LoaderClient.java:native_api
// )

}  // extern "C"

namespace imp::loader::ipc {

Boolean::Boolean(JNIEnv* env, jobject value) : imp::JavaWrapper(env, value) {
  get_value_method_ = GetMethodHandle("booleanValue", "()Z");
}

bool Boolean::GetValue() {
  return static_cast<bool>(CallBooleanMethod(get_value_method_));
}

bool Boolean::IsInstanceOfBoolean(JNIEnv* env, jobject object) {
  Boolean boolean(env);
  return env->IsInstanceOf(object, boolean.Clazz()) == JNI_TRUE;
}

Boolean::Boolean(JNIEnv* env) : imp::JavaWrapper(env, "java/lang/Boolean") {}

CompletableFuture::CompletableFuture(JNIEnv* env, jobject future)
    : imp::JavaWrapper(env, future) {
  // Provides the get method which will allow native to block while waiting
  // for the future to complete.
  get_method_ = GetMethodHandle("get", "()Ljava/lang/Object;");
}

// Calls the Get() method of the future and converts the return into a bool.
jboolean CompletableFuture::GetBoolean() {
  jobject object = CallObjectMethod(get_method_);
  if (!Boolean::IsInstanceOfBoolean(Env(), object)) {
    return false;
  }
  return Boolean(Env(), object).GetValue();
}
JavaLoaderClient::JavaLoaderClient() : JavaWrapper(nullptr, nullptr, nullptr) {}
JavaLoaderClient::JavaLoaderClient(JNIEnv* env, jobject activity_context)
    : imp::JavaWrapper(env, "com/google/ar/imp/view/ipc/LoaderClient",
                       "(Landroid/content/Context;)V", activity_context),
      activity_context_(activity_context) {
  SetupMethodHandles();
}

JavaLoaderClient::~JavaLoaderClient() {
  if (Self()) {
    Disconnect();
  }
}
// Creates a loader service, blocks until the client has connected to the
// service. This should be called from a background thread.
bool JavaLoaderClient::ConnectToLoaderService(const Context& context) {
  // Gets the handle to LoaderClient's completable future.
  jobject future = StartLoaderServiceHelper();

  // Creates a native representation of the future.
  CompletableFuture completable_future(context.GetJniEnv(), future);

  // Blocks until the connection is made, which means this method is probably
  // best called from a background thread.
  jboolean service_connected = completable_future.GetBoolean();
  return service_connected;
}

// Returns the file descriptor created by constructing this object.
int JavaLoaderClient::GetClientSocketFileDescriptor() {
  return CallIntMethod(get_client_socket_fd_);
}

void JavaLoaderClient::SetNativeHandler(LoaderClientBase* native_client) {
  CallVoidMethod(set_native_handler_method_,
                 ToJava<LoaderClientBase>(native_client));
}

// Disconnects from the service and stops the service process.
void JavaLoaderClient::Disconnect() {
  CallVoidMethod(close_method_);
  CallVoidMethod(disconnect_method_);
}

void JavaLoaderClient::SetupMethodHandles() {
  // Gets a handle to the file descriptor getter method.
  get_client_socket_fd_ =
      GetMethodHandle("getClientSocketFileDescriptor", "()I");

  // Gets the disconnect method handle.
  close_method_ = GetMethodHandle("close", "()V");

  // Gets the disconnect method handle.
  disconnect_method_ = GetMethodHandle("disconnect", "()V");

  // Gets the handle to service start method.
  start_loader_service_method_ = GetMethodHandle(
      "startLoaderService", "()Ljava/util/concurrent/CompletableFuture;");

  set_native_handler_method_ = GetMethodHandle("setNativeHandler", "(J)V");
}

// Starts the loader service, returns a java Future that indicates when the
// service connection has been established.
jobject JavaLoaderClient::StartLoaderServiceHelper() {
  return CallObjectMethod(start_loader_service_method_);
}

}  // namespace imp::loader::ipc
