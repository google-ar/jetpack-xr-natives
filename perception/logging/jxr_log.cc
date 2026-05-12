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

#include "logging/jxr_log_sink.h"
#include "logging/log.h"

namespace androidx::xr {
namespace {
class JniThreadAttacher {
 public:
  explicit JniThreadAttacher(JavaVM* vm) : vm_(vm) {
    if (vm_ == nullptr) {
      return;
    }
    jint get_env_result =
        vm_->GetEnv(reinterpret_cast<void**>(&env_), JNI_VERSION_1_6);
    if (get_env_result == JNI_EDETACHED) {
      if (vm_->AttachCurrentThread(&env_, nullptr) ==
          JNI_OK) {
        attached_ = true;
      } else {
        env_ = nullptr;
      }
    } else if (get_env_result != JNI_OK) {
      env_ = nullptr;
    }
  }

  ~JniThreadAttacher() {
    if (attached_ && vm_) {
      vm_->DetachCurrentThread();
    }
  }
  JNIEnv* get() const { return env_; }
  explicit operator bool() const { return env_ != nullptr; }

 private:
  JavaVM* vm_ = nullptr;
  JNIEnv* env_ = nullptr;
  bool attached_ = false;
};
}  // namespace

Log::Log(LogLevel level)
    : log_sink_(JxrLogSink::GetSharedInstance()), level_(level) {}

void JxrLogSink::error(const string& message) const {
  log_jni("error", message.c_str());
}

void JxrLogSink::warn(const string& message) const {
  log_jni("warn", message.c_str());
}

void JxrLogSink::info(const string& message) const {
  log_jni("info", message.c_str());
}

void JxrLogSink::debug(const string& message) const {
  log_jni("debug", message.c_str());
}

void JxrLogSink::verbose(const string& message) const {
  log_jni("verbose", message.c_str());
}

void JxrLogSink::log_jni(const char* method_name, const char* message) const {
  JniThreadAttacher attacher(g_VM);
  if (!attacher) {
    return;
  }

  JNIEnv* env = attacher.get();
  jclass logClass = env->GetObjectClass(log_singleton);
  if (logClass) {
    jmethodID method =
        env->GetMethodID(logClass, method_name, "(Ljava/lang/String;)V");
    if (method) {
      env->CallVoidMethod(log_singleton, method, env->NewStringUTF(message));
    }
    env->DeleteLocalRef(logClass);
  }
}

extern "C" {
JavaVM* g_VM = nullptr;
jobject log_singleton = nullptr;

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM* vm, void* reserved) {
  g_VM = vm;

  JNIEnv* env = nullptr;
  jint result = g_VM->GetEnv((void**)&env, JNI_VERSION_1_6);
  if (result != JNI_OK) {
    return result;
  }

  jclass logClass = env->FindClass("androidx/xr/runtime/Log");
  if (logClass == nullptr) {
    return JNI_ERR;
  }

  jfieldID instanceFieldID =
      env->GetStaticFieldID(logClass, "INSTANCE", "Landroidx/xr/runtime/Log;");
  if (instanceFieldID == nullptr) {
    return JNI_ERR;
  }

  jobject local_ref = env->GetStaticObjectField(logClass, instanceFieldID);
  if (local_ref == nullptr) {
    return JNI_ERR;
  }

  log_singleton = env->NewGlobalRef(local_ref);

  env->DeleteLocalRef(local_ref);
  env->DeleteLocalRef(logClass);

  if (log_singleton == nullptr) {
    return JNI_ERR;
  }

  return JNI_VERSION_1_6;
}

JNIEXPORT void JNICALL JNI_OnUnload(JavaVM* vm, void* reserved) {
  JNIEnv* env;
  if (vm->GetEnv((void**)&env, JNI_VERSION_1_6) != JNI_OK) {
    return;
  }

  // Release global references
  if (log_singleton != nullptr) {
    env->DeleteGlobalRef(log_singleton);
    log_singleton = nullptr;
  }
}
}  // extern "C"

}  // namespace androidx::xr
