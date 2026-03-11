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
  JNIEnv* env = nullptr;
  jint result = g_VM->AttachCurrentThread(&env, nullptr);
  if (result != JNI_OK) {
    return;
  }

  jclass logClass = env->GetObjectClass(log_singleton);
  if (logClass) {
    jmethodID method =
        env->GetMethodID(logClass, method_name, "(Ljava/lang/String;)V");
    if (method) {
      env->CallVoidMethod(log_singleton, method, env->NewStringUTF(message));
    }
  }

  env->DeleteLocalRef(logClass);
  g_VM->DetachCurrentThread();
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
