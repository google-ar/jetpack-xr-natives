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

#include "apibindings/testing/marshalling/impress_api_test_context.h"
#include "apibindings/testing/marshalling/test_impress_api_view.h"
#include "core/common/buffer_access.h"
#include "core/common/jni_helpers.h"

#define JNI_METHOD_AOSP(return_type, method_name) \
  IMP_JNI return_type JNICALL                     \
      Java_androidx_xr_scenecore_impl_impress_ImpressApiTestHelper_##method_name  // NOLINT

extern "C" {

JNI_METHOD_AOSP(void, nativeResetTestState)
(JNIEnv* env, jclass /*clazz*/) { imp::ImpressApiTestContext::Get().Reset(); }

JNI_METHOD_AOSP(jlong, nativeCreateTestView)
(JNIEnv* env, jclass /*clazz*/) {
  // Create an instance of the test view class. The constructor of
  // TestImpressApiView installs all the test managers.
  auto* test_view = new imp::TestImpressApiView();
  return reinterpret_cast<jlong>(test_view);
}

JNI_METHOD_AOSP(void, nativeDestroyTestView)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto* test_view = reinterpret_cast<imp::TestImpressApiView*>(view_handle);
  delete test_view;
}

// glTF Operations
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

JNI_METHOD_AOSP(void, nativeSetExpectedLoadGltfAssetByteArray)
(JNIEnv* env, jclass /*clazz*/, jbyteArray data, jstring key) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  imp::BufferAccess native_data = imp::FromByteArray(env, data);
  context.expected_gltf_data = std::string(native_data.StringView());
  context.expected_gltf_key = imp::GetString(env, key);
}

JNI_METHOD_AOSP(void, nativeSetExpectedReleaseGltfAsset)
(JNIEnv* env, jclass /*clazz*/, jlong token) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.expected_gltf_token_release = token;
}

JNI_METHOD_AOSP(void, nativeSetExpectedInstanceGltfModel)
(JNIEnv* env, jclass /*clazz*/, jlong token, jboolean enable_collider) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.expected_gltf_token_instance = token;
  context.expected_instance_collider = enable_collider;
}

JNI_METHOD_AOSP(void, nativeSetInstanceGltfModelSuccess)
(JNIEnv* env, jclass /*clazz*/, jint node_id) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.instance_gltf_model_success_id = node_id;
}

JNI_METHOD_AOSP(void, nativeSetExpectedSetGltfModelColliderEnabled)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jboolean enable_collider) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.expected_node_id_collider = node_id;
  context.expected_collider_enabled = enable_collider;
}

JNI_METHOD_AOSP(void, nativeSetExpectedAnimateGltfModel)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jstring animation_name,
 jboolean loop) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.expected_node_id_anim = node_id;
  context.expected_anim_name = imp::GetString(env, animation_name);
  context.expected_anim_loop = loop;
}

JNI_METHOD_AOSP(void, nativeSetAnimateGltfModelSuccess)
(JNIEnv* env, jclass /*clazz*/) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.animator_failure_message.clear();
}

JNI_METHOD_AOSP(void, nativeSetAnimateGltfModelFailure)
(JNIEnv* env, jclass /*clazz*/, jstring message) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.animator_failure_message = imp::GetString(env, message);
}

JNI_METHOD_AOSP(void, nativeSetExpectedStopGltfModelAnimation)
(JNIEnv* env, jclass /*clazz*/, jint node_id) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.expected_node_id_stop_anim = node_id;
}

JNI_METHOD_AOSP(void, nativeSetExpectedGetGltfModelLocalBounds)
(JNIEnv* env, jclass /*clazz*/, jint node_id) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.expected_node_id_bounds = node_id;
}

JNI_METHOD_AOSP(void, nativeSetGetGltfModelLocalBoundsSuccess)
(JNIEnv* env, jclass /*clazz*/, jfloatArray center, jfloatArray half_extents) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  env->GetFloatArrayRegion(center, 0, 3, context.bounds_success_center);
  env->GetFloatArrayRegion(half_extents, 0, 3,
                           context.bounds_success_half_extent);
}

JNI_METHOD_AOSP(void, nativeSetExpectedSetMaterialOverride)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jlong material_handle,
 jstring node_name, jint primitive_index) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.expected_node_id_set_override = node_id;
  context.expected_material_handle = material_handle;
  context.expected_node_name_set_override = imp::GetString(env, node_name);
  context.expected_primitive_index_set_override = primitive_index;
}

JNI_METHOD_AOSP(void, nativeSetExpectedClearMaterialOverride)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jstring node_name,
 jint primitive_index) {
  imp::ImpressApiTestContext& context = imp::ImpressApiTestContext::Get();
  context.expected_node_id_clear_override = node_id;
  context.expected_node_name_clear_override = imp::GetString(env, node_name);
  context.expected_primitive_index_clear_override = primitive_index;
}

}  // extern "C"
