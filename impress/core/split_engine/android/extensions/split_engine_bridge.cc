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

#include "core/split_engine/android/extensions/split_engine_bridge.h"

#include <jni.h>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/async/executor.h"
#include "core/common/jni_helpers.h"
#include "core/split_engine/android/split_engine_shared_memory_bridge_client.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {
namespace {
using BufferHandle = SplitEngineSharedMemoryBridgeClient::BufferHandle;

JniUniquePtr<jlongArray> ToLongArray(
    JNIEnv* env, const std::vector<TextureId>& in_texture_ids) {
  JniUniquePtr<jlongArray> texture_ids_array =
      CreateJniLongArray(env, in_texture_ids.size());
  env->SetLongArrayRegion(
      texture_ids_array.get(), 0, in_texture_ids.size(),
      reinterpret_cast<const jlong*>(in_texture_ids.data()));
  return texture_ids_array;
};

JniUniquePtr<jbyteArray> ToByteArray(JNIEnv* env,
                                     const std::vector<uint8_t>& data) {
  JniUniquePtr<jbyteArray> data_array = CreateJniByteArray(env, data.size());
  env->SetByteArrayRegion(data_array.get(), 0, data.size(),
                          reinterpret_cast<const jbyte*>(data.data()));
  return data_array;
};

class SplitEngineBufferHandle : public BufferHandle {
 public:
  explicit SplitEngineBufferHandle(JNIEnv* env, jobject buffer_handle)
      : handle_(WrapJni(env, env->NewGlobalRef((buffer_handle)))) {}

  JniUniquePtr<jobject> handle_;
};

// FatalIfJavaExceptionOccurred should be called after a Java method call. If a
// Java exception occurred, it will log a fatal error message and crash the
// process.
void FatalIfJavaExceptionOccurred(JNIEnv& env) {
  if (env.ExceptionCheck()) {
    jthrowable e = env.ExceptionOccurred();
    env.ExceptionClear();
    jclass clazz = env.GetObjectClass(e);
    jmethodID getMessage =
        env.GetMethodID(clazz, "getMessage", "()Ljava/lang/String;");
    jstring message = (jstring)env.CallObjectMethod(e, getMessage);
    IMP_LOG(imp::FATAL) << "SplitEngineBridge operation failed, System exception: "
               << env.GetStringUTFChars(message, NULL);
  }
}

}  // namespace

absl::StatusOr<std::unique_ptr<BufferHandle>> SplitEngineBridge::RegisterBuffer(
    int fd, size_t buffer_size_bytes) {
  // TODO: Refactor aidl to take a int64 instead of int32 for this
  // method.
  jobject buffer_handle =
      JavaWrapper::CallObjectMethod(register_buffer_, static_cast<jint>(fd),
                                    static_cast<jint>(buffer_size_bytes));
  FatalIfJavaExceptionOccurred(*Env());
  return std::make_unique<SplitEngineBufferHandle>(Env(), buffer_handle);
}

absl::Status SplitEngineBridge::ProcessRegion(const BufferHandle& buffer_handle,
                                              int offset_bytes,
                                              int region_length_bytes) {
  jobject token = static_cast<const SplitEngineBufferHandle*>(&buffer_handle)
                      ->handle_.get();
  JavaWrapper::CallVoidMethod(process_region_, token,
                              static_cast<jint>(offset_bytes),
                              static_cast<jint>(region_length_bytes));
  FatalIfJavaExceptionOccurred(*Env());
  return absl::OkStatus();
};

absl::StatusOr<jobject> SplitEngineBridge::CreateExternalTextureSurface(
    const std::vector<TextureId>& in_texture_ids) {
  jobject surface =
      JavaWrapper::CallObjectMethod(create_external_texture_surface_,
                                    ToLongArray(Env(), in_texture_ids).get());
  FatalIfJavaExceptionOccurred(*Env());
  return surface;
};

absl::Status SplitEngineBridge::SetExternalTextureSurfaceSize(
    TextureId in_texture_id, int32_t width, int32_t height) {
  JavaWrapper::CallVoidMethod(
      set_external_texture_surface_size_, static_cast<jlong>(in_texture_id),
      static_cast<jint>(width), static_cast<jint>(height));
  FatalIfJavaExceptionOccurred(*Env());
  return absl::OkStatus();
};

absl::Status SplitEngineBridge::SendRequest(
    const std::vector<uint8_t>& data,
    std::function<void(const std::vector<uint8_t>&)> callback) {
  // Note: the callback object JavaWrapper is released to java and deleted
  // in the native callback when it resolves.
  SplitEngineRequestCallback* request_callback =
      new SplitEngineRequestCallback(Env(), callback);
  auto data_array = ToByteArray(Env(), data);
  JavaWrapper::CallVoidMethod(send_request_, data_array.release(),
                              request_callback->Release());
  FatalIfJavaExceptionOccurred(*Env());
  return absl::OkStatus();
};

}  // namespace imp::split_engine
