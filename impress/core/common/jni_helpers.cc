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

#include "core/common/jni_helpers.h"

#include <jni.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "core/common/log.h"
#include "absl/strings/cord.h"
#include "absl/strings/cord_buffer.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"

namespace imp {

namespace {

std::string ExceptionToString(JNIEnv* env, JniUniquePtr<jthrowable> exception) {
  JniUniquePtr<jclass> log_class =
      WrapJni(env, env->FindClass("android/util/Log"));
  jmethodID get_stack_trace_string_mid =
      env->GetStaticMethodID(log_class.get(), "getStackTraceString",
                             "(Ljava/lang/Throwable;)Ljava/lang/String;");
  JniUniquePtr<jstring> trace_jstring = WrapJni(
      env, static_cast<jstring>(env->CallStaticObjectMethod(
               log_class.get(), get_stack_trace_string_mid, exception.get())));
  return GetString(env, trace_jstring.get());
}

}  // namespace

void AssertNoException(JNIEnv* env) {
  jthrowable exception = env->ExceptionOccurred();
  if (exception) {
    env->ExceptionClear();
    IMP_LOG(imp::FATAL) << ExceptionToString(env, WrapJni(env, exception));
  }
}

bool JavaExceptionPrintClear(JNIEnv* env) {
  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
    env->ExceptionClear();
    return true;
  }
  return false;
}

JniObjectArray::JniObjectArray(JNIEnv* env, size_t length, jclass element_class,
                               jobject initial_element)
    : env_(env),
      array_(env->NewObjectArray(length, element_class, initial_element)) {
  if (array_ == nullptr) {
    ThrowError(env, Error("Failed to allocate object array!"));
  }
}

JniObjectArray::~JniObjectArray() {
  if (array_) {
    env_->DeleteLocalRef(array_);
    array_ = nullptr;
  }
}

jobjectArray JniObjectArray::Release() {
  jobjectArray result = array_;
  array_ = nullptr;
  return result;
}

void JniObjectArray::Set(jsize index, jobject value) {
  if (array_) {
    env_->SetObjectArrayElement(array_, index, value);
  }
}

jstring ToString(JNIEnv* env, const std::string& str) {
  return env->NewStringUTF(str.c_str());
}

jstring ToString(JNIEnv* env, absl::string_view view) {
  std::string null_terminated_string(view.data(), view.size());
  return env->NewStringUTF(null_terminated_string.data());
}

JniUniquePtr<jstring> ToJniString(JNIEnv* env, const std::string& str) {
  return WrapJni(env, ToString(env, str));
}

JniUniquePtr<jstring> ToJniString(JNIEnv* env, absl::string_view view) {
  return WrapJni(env, ToString(env, view));
}

jobjectArray ToStringArray(JNIEnv* env, std::vector<absl::string_view> views) {
  JniObjectArray strings(env, views.size(),
                         WrapJni(env, env->FindClass("java/lang/String")).get(),
                         ToJniString(env, absl::string_view()).get());
  for (int view_index = 0; view_index < views.size(); view_index++) {
    JniUniquePtr<jstring> java_string = ToJniString(env, views[view_index]);
    if (java_string == nullptr) {
      ThrowError(env, Error("Failed to allocate string!"));
      return nullptr;
    }
    strings.Set(view_index, java_string.get());
  }

  return strings.Release();
}

jbyteArray ToByteArray(JNIEnv* env, const BufferAccess& access) {
  jbyteArray result = env->NewByteArray(access.Size());
  if (!access.Empty()) {
    env->SetByteArrayRegion(result, 0, access.Size(),
                            reinterpret_cast<const jbyte*>(access.Data()));
  }
  return result;
}

jbyteArray ToByteArray(JNIEnv* env, absl::string_view str) {
  jbyteArray array = env->NewByteArray(str.size());
  if (!str.empty()) {
    env->SetByteArrayRegion(array, 0, str.size(),
                            reinterpret_cast<const jbyte*>(&str[0]));
  }
  return array;
}

BufferAccess FromByteArray(JNIEnv* env, jbyteArray byte_array) {
  jsize size = env->GetArrayLength(byte_array);
  jbyte* data = env->GetByteArrayElements(byte_array, nullptr);

  BufferAccess result =
      BufferAccess::Clone(reinterpret_cast<uint8_t*>(data), size);
  env->ReleaseByteArrayElements(byte_array, data, JNI_ABORT);
  return result;
}

absl::Cord ByteArrayToCord(JNIEnv* env, jbyteArray byte_array) {
  jsize array_len = env->GetArrayLength(byte_array);
  if (!array_len) {
    return absl::Cord();
  }

  absl::Cord result;
  jsize offset = 0;
  const jsize chunk_size = 4096;
  std::unique_ptr<jbyte[]> buffer(new jbyte[chunk_size]);

  while (offset < array_len) {
    jsize to_read = std::min(chunk_size, array_len - offset);
    env->GetByteArrayRegion(byte_array, offset, to_read, buffer.get());

    if (JavaExceptionPrintClear(env)) {
      return absl::Cord();
    }

    result.Append(absl::string_view(reinterpret_cast<const char*>(buffer.get()),
                                    to_read));
    offset += to_read;
  }
  return result;
}

void ThrowError(JNIEnv* env, const OptionalError& error) {
  jclass exception_class = env->FindClass("java/lang/RuntimeException");
  env->ThrowNew(exception_class, std::string(error.message()).c_str());
}

void ThrowJsonError(JNIEnv* env, const OptionalError& error) {
  jclass exception_class = env->FindClass("org/json/JSONException");
  env->ThrowNew(exception_class, std::string(error.message()).c_str());
}

std::string GetString(JNIEnv* env, jstring java_string) {
  jboolean is_copy;
  const char* name_pointer =
      java_string ? env->GetStringUTFChars(java_string, &is_copy) : nullptr;
  if (name_pointer == nullptr) {
    return "";
  }
  std::string result(name_pointer);
  env->ReleaseStringUTFChars(java_string, name_pointer);
  return result;
}

jfieldID GetFieldID(JNIEnv* env, jclass clazz, const char* field_name,
                    const char* field_signature) {
  if (clazz == nullptr || field_name == nullptr || field_signature == nullptr) {
    return nullptr;
  }
  jfieldID result = env->GetFieldID(clazz, field_name, field_signature);
  if (env->ExceptionCheck()) {
    env->ExceptionDescribe();
    env->ExceptionClear();
  }
  return result;
}

JniUniquePtr<jclass> FindClass(JNIEnv* env, const char* class_path) {
  return WrapJni(env, env->FindClass(class_path));
}

JniUniquePtr<jclass> GetObjectClass(JNIEnv* env, jobject object) {
  return WrapJni(env, env->GetObjectClass(object));
}

JniUniquePtr<jbyteArray> CreateJniByteArray(JNIEnv* env, size_t length) {
  jbyteArray result = env->NewByteArray(length);
  return WrapJni(env, result);
}

JniUniquePtr<jintArray> CreateJniIntArray(JNIEnv* env, size_t length) {
  jintArray result = env->NewIntArray(length);
  return WrapJni(env, result);
}

JniUniquePtr<jlongArray> CreateJniLongArray(JNIEnv* env, size_t length) {
  jlongArray result = env->NewLongArray(length);
  return WrapJni(env, result);
}

JniUniquePtr<jfloatArray> CreateJniFloatArray(JNIEnv* env, size_t length) {
  jfloatArray result = env->NewFloatArray(length);
  return WrapJni(env, result);
}

JniUniquePtr<jbooleanArray> CreateJniBooleanArray(JNIEnv* env, size_t length) {
  jbooleanArray result = env->NewBooleanArray(length);
  return WrapJni(env, result);
}

JniUniquePtr<jstring> CreateJniString(JNIEnv* env, const std::string& str) {
  jstring java_string = env->NewStringUTF(str.c_str());
  return WrapJni(env, java_string);
}

JniUniquePtr<jobjectArray> CreateJniObjectArray(JNIEnv* env, size_t length,
                                                jclass clazz, jobject initial) {
  jobjectArray result = env->NewObjectArray(length, clazz, initial);
  return WrapJni(env, result);
}

JniUniquePtr<jintArray> CreateJniIntArray(JNIEnv* env,
                                          absl::Span<int> contents) {
  JniUniquePtr<jintArray> result =
      WrapJni(env, env->NewIntArray(contents.size()));
  jint* array_ptr = env->GetIntArrayElements(result.get(), /*isCopy=*/nullptr);
  static_assert(sizeof(jint) == sizeof(int));
  std::memcpy(array_ptr, contents.data(), contents.size() * sizeof(int));
  env->ReleaseIntArrayElements(result.get(), array_ptr, /*mode=*/0);
  return result;
}

void DeleteRef(JNIEnv* env, jobject object) {
  if (object == nullptr) {
    return;
  }

  switch (env->GetObjectRefType(object)) {
    default:
    case JNIInvalidRefType:
      ThrowError(env, Error("Invalid Reference type returned"
                            "from 'native_to_java_method'"));
      break;
    case JNILocalRefType:
      env->DeleteLocalRef(object);
      break;
    case JNIGlobalRefType:
      env->DeleteGlobalRef(object);
      break;
    case JNIWeakGlobalRefType:
      env->DeleteWeakGlobalRef(object);
      break;
  }
}

std::string JavaWrapper::GetObjectClassName(JNIEnv* env, jobject object) {
  // This is commonly called while an exception is thrown. Temporarily clear the
  // exception so that JNI calls don't cause a crash.
  jthrowable exception = env->ExceptionOccurred();
  env->ExceptionClear();
  auto exception_cleanup = absl::MakeCleanup([env, exception]() {
    if (exception) {
      env->Throw(exception);
    }
  });

  JavaWrapper clazz{env, env->GetObjectClass(object), "java/lang/Class"};
  JniHandle get_name = clazz.GetMethodHandle("getName", "()Ljava/lang/String;");
  return clazz.CallStringMethod(get_name);
}

void android::DumpLocalReferenceTable(JNIEnv* env) {
  JniUniquePtr<jclass> vm_class =
      WrapJni(env, env->FindClass("dalvik/system/VMDebug"));
  jmethodID dump_mid =
      env->GetStaticMethodID(vm_class.get(), "dumpReferenceTables", "()V");
  env->CallStaticVoidMethod(vm_class.get(), dump_mid);
}

}  // namespace imp
