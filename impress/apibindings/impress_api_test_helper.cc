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

#include <string>

#include "apibindings/impress_api_test_context.h"
#include "core/common/jni_helpers.h"

#define JNI_METHOD_AOSP(return_type, method_name) \
  IMP_JNI return_type JNICALL                     \
      Java_androidx_xr_scenecore_impl_impress_ImpressApiTestHelper_##method_name  // NOLINT

extern "C" {

JNI_METHOD_AOSP(void, nativeResetTestState)
(JNIEnv* env, jclass /*clazz*/) { imp::ImpressApiTestContext::Get().Reset(); }

JNI_METHOD_AOSP(void, nativeSetExpectedLoadGltfPath)
(JNIEnv* env, jclass /*clazz*/, jstring path) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.expected_gltf_path = imp::GetString(env, path);
}

JNI_METHOD_AOSP(void, nativeSetLoadGltfAssetSuccess)
(JNIEnv* env, jclass /*clazz*/, jlong token) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.gltf_asset_loader_success_token = token;
  context.gltf_asset_loader_failure_message.clear();
}

JNI_METHOD_AOSP(void, nativeSetLoadGltfAssetFailure)
(JNIEnv* env, jclass /*clazz*/, jstring message) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.gltf_asset_loader_failure_message = imp::GetString(env, message);
}

}  // extern "C"
