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

#include <jni.h>

#include <cstdint>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "core/common/jni_helpers.h"
#include "core/split_engine/android/extensions/split_engine_bridge.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/split_engine_bridge_sender.h"

#if __ANDROID_API__ >= 34
#include <android/binder_auto_utils.h>
#include <android/binder_ibinder_jni.h>

#include <cstddef>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client_ndk.h"
#include "core/view/platforms/android/jni_helpers/exception_helper.h"

#define JNI_METHOD_PROTOTYPE_RENDERER_CONNECTION_NDK(return_type, method_name) \
  IMP_JNI return_type JNICALL                                                  \
      Java_com_google_imp_splitengine_extensions_PrototypeRendererConnectionNdk_##method_name  // NOLINT

#endif  // __ANDROID_API__ >= 34

#define JNI_METHOD_REQUEST_CALLBACK(return_type, method_name) \
  IMP_JNI return_type JNICALL                                 \
      Java_com_google_imp_splitengine_extensions_RequestCallback_##method_name  // NOLINT

#define JNI_METHOD_MESSAGE_GROUP_CALLBACK(return_type, method_name) \
  IMP_JNI return_type JNICALL                                       \
      Java_com_google_imp_splitengine_extensions_MessageGroupCallback_##method_name  // NOLINT

namespace imp::split_engine {
namespace {

template <class T>
using SplitEngineBridgeAllowlist = ::imp::JniAllowlist<
    T, imp::split_engine::SplitEngineRequestCallback,
    imp::split_engine::SplitEngineMessageGroupCallback,
    imp::split_engine::SplitEngineSharedMemoryBridgeClient,
#if __ANDROID_API__ >= 34
    imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk,
#endif  // __ANDROID_API__ >= 34
    SplitEngineSharedMemoryBridgeClient::BufferHandle>;

template <class T>
constexpr auto ToJava = &SplitEngineBridgeAllowlist<T>::ToJava;

template <class T>
constexpr auto FromJava = &SplitEngineBridgeAllowlist<T>::FromJava;

}  // namespace

extern "C" {
#if __ANDROID_API__ >= 34
JNI_METHOD_PROTOTYPE_RENDERER_CONNECTION_NDK(jlong, nCreateBridge)
(JNIEnv* env, jclass /*clazz*/, jobject service_binder) {
  ndk::SpAIBinder native_binder =
      ndk::SpAIBinder(AIBinder_fromJavaBinder(env, service_binder));
  imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk* bridge =
      new imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk(
          native_binder, env);

  return ToJava<imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk>(
      bridge);
}

JNI_METHOD_PROTOTYPE_RENDERER_CONNECTION_NDK(void, nDestroyBridge)
(JNIEnv* env, jclass /*clazz*/, jlong native_bridge_handle) {
  auto* bridge =
      FromJava<imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk>(
          native_bridge_handle);

  
  delete bridge;
}

JNI_METHOD_PROTOTYPE_RENDERER_CONNECTION_NDK(jlong, nRegisterBuffer)
(JNIEnv* env, jclass /*clazz*/, jlong native_bridge_handle, jint fd,
 jint buffer_size_bytes) {
  auto* bridge =
      FromJava<imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk>(
          native_bridge_handle);

  

  absl::StatusOr<std::unique_ptr<
      imp::split_engine::SplitEngineSharedMemoryBridgeClient::BufferHandle>>
      buffer_handle =
          bridge->RegisterBuffer(fd, static_cast<size_t>(buffer_size_bytes));

  

  return ToJava<SplitEngineSharedMemoryBridgeClient::BufferHandle>(
      buffer_handle->release());
}

JNI_METHOD_PROTOTYPE_RENDERER_CONNECTION_NDK(void, nProcessRegion)
(JNIEnv* env, jclass /*clazz*/, jlong native_bridge_handle,
 jlong nativeBufferHandle, jint offsetBytes, jint regionLengthBytes) {
  auto* bridge =
      FromJava<imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk>(
          native_bridge_handle);
  

  auto* buffer_handle =
      FromJava<SplitEngineSharedMemoryBridgeClient::BufferHandle>(
          nativeBufferHandle);
  
  absl::Status status =
      bridge->ProcessRegion(*buffer_handle, offsetBytes, regionLengthBytes);
  
}

JNI_METHOD_PROTOTYPE_RENDERER_CONNECTION_NDK(jobject,
                                             nCreateExternalTextureSurface)
(JNIEnv* env, jclass /*clazz*/, jlong native_bridge_handle,
 jlongArray textureIdsArray) {
  auto* bridge =
      FromJava<imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk>(
          native_bridge_handle);
  
  jsize texture_ids_size = env->GetArrayLength(textureIdsArray);
  std::vector<TextureId> in_texture_ids(static_cast<size_t>(texture_ids_size));
  env->GetLongArrayRegion(textureIdsArray, 0, texture_ids_size,
                          reinterpret_cast<jlong*>(in_texture_ids.data()));
  absl::StatusOr<jobject> surface =
      bridge->CreateExternalTextureSurface(in_texture_ids);
  
  return env->NewLocalRef(*surface);
}

JNI_METHOD_PROTOTYPE_RENDERER_CONNECTION_NDK(void,
                                             nSetExternalTextureSurfaceSize)
(JNIEnv* env, jclass /*clazz*/, jlong native_bridge_handle, jlong textureId,
 jint width, jint height) {
  auto* bridge =
      FromJava<imp::split_engine::SplitEngineSharedMemoryBridgeClient>(
          native_bridge_handle);
  

  absl::Status status = bridge->SetExternalTextureSurfaceSize(
      static_cast<imp::split_engine::TextureId>(textureId),
      static_cast<int32_t>(width), static_cast<int32_t>(height));
  
}

JNI_METHOD_PROTOTYPE_RENDERER_CONNECTION_NDK(void, nSendRequest)
(JNIEnv* env, jclass /*clazz*/, jlong native_bridge_handle,
 jbyteArray dataArray, jlong nativeRequestCallbackHandle) {
  auto* bridge =
      FromJava<imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk>(
          native_bridge_handle);
  

  SplitEngineRequestCallback* callback =
      FromJava<SplitEngineRequestCallback>(nativeRequestCallbackHandle);
  
  jsize data_size = env->GetArrayLength(dataArray);
  std::vector<uint8_t> data(static_cast<size_t>(data_size));
  env->GetByteArrayRegion(dataArray, 0, data_size,
                          reinterpret_cast<jbyte*>(data.data()));
  // We can invoke the callback directly, since the Java callback is
  // guaranteed to be invoked on the same thread as the JNI call.
  absl::Status status =
      bridge->SendRequest(data, callback->GetNativeCallback());
  
}

JNI_METHOD_PROTOTYPE_RENDERER_CONNECTION_NDK(void,
                                             nRegisterMessageGroupCallback)
(JNIEnv* env, jclass /*clazz*/, jlong native_bridge_handle, jobject callback) {
  auto* bridge =
      FromJava<imp::split_engine::SplitEngineSharedMemoryBridgeClientNdk>(
          native_bridge_handle);
  

  auto message_group_callback =
      std::make_unique<SplitEngineMessageGroupCallback>(env, callback);
  bridge->RegisterMessageGroupCallback(std::move(message_group_callback));
}
#endif  // __ANDROID_API__ >= 34

JNI_METHOD_REQUEST_CALLBACK(void, nOnResult)(JNIEnv* env, jclass /*clazz*/,
                                             jlong nativeRequestCallbackHandle,
                                             jbyteArray response) {
  SplitEngineRequestCallback* callback =
      FromJava<SplitEngineRequestCallback>(nativeRequestCallbackHandle);
  

  std::vector<uint8_t> response_bytes;
  if (response) {
    jsize response_bytes_size = env->GetArrayLength(response);
    response_bytes.resize(response_bytes_size);
    env->GetByteArrayRegion(response, 0, response_bytes_size,
                            reinterpret_cast<jbyte*>(response_bytes.data()));
  }
  callback->OnResult(response_bytes);
  delete callback;
}

// TODO: (broken link) - use long instead of int for messageGroupId.
JNI_METHOD_MESSAGE_GROUP_CALLBACK(void, nOnMessageGroupComplete)
(JNIEnv* env, jclass /*clazz*/, jlong bridgeId, jint messageGroupId) {
  if (absl::Status release_result =
          imp::split_engine::SplitEngineBridgeSender::ReleaseMessageGroup(
              bridgeId, messageGroupId);
      !release_result.ok()) {
    IMP_LOG(imp::ERROR) << "Failed to release message group: "
               << release_result.ToString();
  }
}

}  // namespace
}  // namespace imp::split_engine
