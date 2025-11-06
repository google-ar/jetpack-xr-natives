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
#include <jni.h>

#include <memory>

#include "core/common/jni_helpers.h"
#include "core/materialcompiler/material_compiler_service.h"

namespace imp_material_compiler {

template <class T>
using MaterialCompilerServiceAllowlist =
    imp::JniAllowlist<T, MaterialCompilerService>;

template <class T>
constexpr auto ToJava = &MaterialCompilerServiceAllowlist<T>::ToJava;

template <class T>
constexpr auto FromJava = &MaterialCompilerServiceAllowlist<T>::FromJava;

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_materialcompiler_MaterialCompilerService_##method_name  // NOLINT

extern "C" {

// LINT.IfChange(native_api)

JNI_METHOD(jlong, nCreate)
(JNIEnv* env, jclass /*clazz*/, jint fd) {
  auto service = std::make_unique<MaterialCompilerService>(fd);

  jlong result = ToJava<MaterialCompilerService>(service.release());
  return result;
}

// Destroy the native service.
JNI_METHOD(void, nClose)
(JNIEnv* env, jclass /*clazz*/, jlong native_service) {
  // Get the native service and release.
  std::unique_ptr<MaterialCompilerService> service(
      FromJava<MaterialCompilerService>(native_service));
  service.reset();
}

}  // extern "C"
}  // namespace imp_material_compiler

// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/materialcompiler/MaterialCompilerService.java:native_api)
