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

#include <jni.h>

#include <memory>

#include "absl/memory/memory.h"
#include "core/common/jni_helpers.h"
#include "core/loader/ipc/loader_service.h"

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_view_ipc_LoaderService_##method_name

using ::imp::GetThreadId;
using ::imp::GetThreadNiceness;
using ::imp::JniAllowlist;
using ::imp::loader::ipc::LoaderService;

namespace {

template <class T>
using LoaderServiceAllowlist = JniAllowlist<T, LoaderService>;

template <class T>
constexpr auto ToJava = &LoaderServiceAllowlist<T>::ToJava;

template <class T>
constexpr auto FromJava = &LoaderServiceAllowlist<T>::FromJava;

}  // namespace

extern "C" {

// LINT.IfChange(api)

// Creates the native loader service that processes the downloaded data.
// takes a file_descriptor (the client socket of a client/server socket pair
// created in java).
JNI_METHOD(jlong, nCreate)
(JNIEnv* env, jclass /*clazz*/, jint fd) {
  // Make the isolated process slightly lower priority than the main thread.
  imp::SetThreadNiceness(GetThreadId(), GetThreadNiceness(GetThreadId()) + 5);
  auto service = std::make_unique<LoaderService>(fd);
  jlong result = ToJava<LoaderService>(service.release());
  return result;
}

// Destroy the native loader.
JNI_METHOD(void, nDestroy)
(JNIEnv* env, jclass /*clazz*/, jlong native_service) {
  std::unique_ptr<LoaderService> service(
      FromJava<LoaderService>(native_service));
  service.reset();
}
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/view/ipc/LoaderService.java:api
// )

}  // extern "C"
