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

#include <cstddef>
#include <string>

#include "apibindings/testing/marshalling/model_test_context.h"
#include "apibindings/testing/marshalling/skybox_test_context.h"
#include "apibindings/testing/marshalling/test_impress_api_view.h"
#include "apibindings/testing/marshalling/texture_test_context.h"
#include "core/common/jni_helpers.h"

#define JNI_METHOD_AOSP(return_type, method_name) \
  IMP_JNI return_type JNICALL                     \
      Java_androidx_xr_scenecore_impl_impress_ImpressApiTestHelper_##method_name  // NOLINT

extern "C" {

JNI_METHOD_AOSP(void, nativeResetTestState)
(JNIEnv* env, jclass /*clazz*/) {
  imp::ModelTestContext::Get().Reset();
  imp::SkyboxTestContext::Get().Reset();
}

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
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.load_gltf_asset_path.expected_path = imp::GetString(env, path);
}

JNI_METHOD_AOSP(void, nativeSetLoadGltfAssetSuccess)
(JNIEnv* env, jclass /*clazz*/, jlong token) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.load_gltf_asset_path.success_token = token;
  context.load_gltf_asset_path.failure_message.clear();

  context.load_gltf_asset_bytes.success_token = token;
  context.load_gltf_asset_bytes.failure_message.clear();
}

JNI_METHOD_AOSP(void, nativeSetLoadGltfAssetFailure)
(JNIEnv* env, jclass /*clazz*/, jstring message) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  std::string msg = imp::GetString(env, message);
  context.load_gltf_asset_path.failure_message = msg;
  context.load_gltf_asset_bytes.failure_message = msg;
}

JNI_METHOD_AOSP(void, nativeSetExpectedLoadGltfAssetTestPattern)
(JNIEnv* env, jclass /*clazz*/, jint expected_size, jstring key) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.load_gltf_asset_bytes.expect_test_pattern = true;
  context.load_gltf_asset_bytes.expected_size =
      static_cast<size_t>(expected_size);
  context.load_gltf_asset_bytes.expected_key = imp::GetString(env, key);
}

JNI_METHOD_AOSP(void, nativeSetExpectedReleaseGltfAsset)
(JNIEnv* env, jclass /*clazz*/, jlong token) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.release_gltf_asset.expected_token = token;
}

JNI_METHOD_AOSP(void, nativeSetExpectedInstanceGltfModel)
(JNIEnv* env, jclass /*clazz*/, jlong token) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.instance_gltf_model.expected_token = token;
}

JNI_METHOD_AOSP(void, nativeSetInstanceGltfModelSuccess)
(JNIEnv* env, jclass /*clazz*/, jint node_id) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.instance_gltf_model.success_id = node_id;
}

JNI_METHOD_AOSP(void, nativeSetExpectedSetGltfModelColliderEnabled)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jboolean enable_collider) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.set_gltf_model_collider_enabled.expected_node_id = node_id;
  context.set_gltf_model_collider_enabled.expected_enabled = enable_collider;
}

JNI_METHOD_AOSP(void, nativeSetExpectedSetGltfReformAffordanceEnabled)
(JNIEnv* env, jclass /*clazz*/, jint impress_node_id,
 jboolean enable_affordance) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.set_gltf_reform_affordance_enabled.expected_node_id = impress_node_id;
  context.set_gltf_reform_affordance_enabled.expected_enabled =
      enable_affordance;
}

JNI_METHOD_AOSP(void, nativeSetExpectedAnimateGltfModel)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jstring animation_name,
 jboolean loop, jfloat speed, jfloat start_time, jint channel_id) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.animate_gltf_model.expected_node_id = node_id;
  context.animate_gltf_model.expected_name =
      imp::GetString(env, animation_name);
  context.animate_gltf_model.expected_loop = loop;
  context.animate_gltf_model.expected_speed = speed;
  context.animate_gltf_model.expected_start_time = start_time;
  context.animate_gltf_model.expected_channel_id = channel_id;
}

JNI_METHOD_AOSP(void, nativeSetAnimateGltfModelSuccess)
(JNIEnv* env, jclass /*clazz*/) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.animate_gltf_model.failure_message.clear();
}

JNI_METHOD_AOSP(void, nativeSetAnimateGltfModelFailure)
(JNIEnv* env, jclass /*clazz*/, jstring message) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.animate_gltf_model.failure_message = imp::GetString(env, message);
}

JNI_METHOD_AOSP(void, nativeSetExpectedStopGltfModelAnimation)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jint channel_id) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.stop_gltf_model_animation.expected_node_id = node_id;
  context.stop_gltf_model_animation.expected_channel_id = channel_id;
}

JNI_METHOD_AOSP(void, nativeSetExpectedToggleGltfModelAnimation)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jboolean toggle,
 jint channel_id) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.toggle_gltf_model_animation.expected_node_id = node_id;
  context.toggle_gltf_model_animation.expected_toggle = toggle;
  context.toggle_gltf_model_animation.expected_channel_id = channel_id;
}

JNI_METHOD_AOSP(void, nativeSetExpectedSetGltfModelAnimationSpeed)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jfloat speed, jint channel_id) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.set_gltf_model_animation_speed.expected_node_id = node_id;
  context.set_gltf_model_animation_speed.expected_speed = speed;
  context.set_gltf_model_animation_speed.expected_channel_id = channel_id;
}

JNI_METHOD_AOSP(void, nativeSetExpectedSetGltfModelAnimationPlaybackTime)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jfloat playback_time,
 jint channel_id) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.set_gltf_model_animation_playback_time.expected_node_id = node_id;
  context.set_gltf_model_animation_playback_time.expected_playback_time =
      playback_time;
  context.set_gltf_model_animation_playback_time.expected_channel_id =
      channel_id;
}

JNI_METHOD_AOSP(void, nativeSetExpectedGetGltfModelAnimationCount)
(JNIEnv* env, jclass /*clazz*/, jint node_id) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.get_gltf_model_animation_count.expected_node_id = node_id;
}

JNI_METHOD_AOSP(void, nativeSetGetGltfModelAnimationCountSuccess)
(JNIEnv* env, jclass /*clazz*/, jint count) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.get_gltf_model_animation_count.success_count = count;
}

JNI_METHOD_AOSP(void, nativeSetExpectedGetGltfModelAnimationName)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jint index) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.get_gltf_model_animation_name.expected_node_id = node_id;
  context.get_gltf_model_animation_name.expected_index = index;
}

JNI_METHOD_AOSP(void, nativeSetGetGltfModelAnimationNameSuccess)
(JNIEnv* env, jclass /*clazz*/, jstring name) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.get_gltf_model_animation_name.success_name =
      imp::GetString(env, name);
}

JNI_METHOD_AOSP(void, nativeSetExpectedGetGltfModelAnimationDurationSeconds)
(JNIEnv* env, jclass /*clazz*/, jint node_id, jint index) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.get_gltf_model_animation_duration_seconds.expected_node_id = node_id;
  context.get_gltf_model_animation_duration_seconds.expected_index = index;
}

JNI_METHOD_AOSP(void, nativeSetGetGltfModelAnimationDurationSecondsSuccess)
(JNIEnv* env, jclass /*clazz*/, jfloat duration) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.get_gltf_model_animation_duration_seconds.success_duration = duration;
}

JNI_METHOD_AOSP(void, nativeSetExpectedGetGltfModelLocalBounds)
(JNIEnv* env, jclass /*clazz*/, jint node_id) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  context.get_gltf_model_local_bounds.expected_node_id = node_id;
}

JNI_METHOD_AOSP(void, nativeSetGetGltfModelLocalBoundsSuccess)
(JNIEnv* env, jclass /*clazz*/, jfloatArray center, jfloatArray half_extents) {
  imp::ModelTestContext& context = imp::ModelTestContext::Get();
  env->GetFloatArrayRegion(center, 0, 3,
                           context.get_gltf_model_local_bounds.success_center);
  env->GetFloatArrayRegion(
      half_extents, 0, 3,
      context.get_gltf_model_local_bounds.success_half_extent);
}

// Skybox Operations
JNI_METHOD_AOSP(void, nativeSetExpectedLoadIblPath)
(JNIEnv* env, jclass /*clazz*/, jstring path) {
  imp::SkyboxTestContext& context = imp::SkyboxTestContext::Get();
  context.load_image_based_lighting_asset_path.expected_path =
      imp::GetString(env, path);
}

JNI_METHOD_AOSP(void, nativeSetLoadIblAssetSuccess)
(JNIEnv* env, jclass /*clazz*/, jlong token) {
  imp::SkyboxTestContext& context = imp::SkyboxTestContext::Get();
  context.load_image_based_lighting_asset_path.success_token = token;
  context.load_image_based_lighting_asset_path.failure_message.clear();

  context.load_image_based_lighting_asset_bytes.success_token = token;
  context.load_image_based_lighting_asset_bytes.failure_message.clear();
}

JNI_METHOD_AOSP(void, nativeSetLoadIblAssetFailure)
(JNIEnv* env, jclass /*clazz*/, jstring message) {
  imp::SkyboxTestContext& context = imp::SkyboxTestContext::Get();
  std::string msg = imp::GetString(env, message);
  context.load_image_based_lighting_asset_path.failure_message = msg;
  context.load_image_based_lighting_asset_bytes.failure_message = msg;
}

JNI_METHOD_AOSP(void, nativeSetExpectedLoadIblAssetTestPattern)
(JNIEnv* env, jclass /*clazz*/, jint expected_size, jstring key) {
  imp::SkyboxTestContext& context = imp::SkyboxTestContext::Get();
  context.load_image_based_lighting_asset_bytes.expect_test_pattern = true;
  context.load_image_based_lighting_asset_bytes.expected_size =
      static_cast<size_t>(expected_size);
  context.load_image_based_lighting_asset_bytes.expected_key =
      imp::GetString(env, key);
}

JNI_METHOD_AOSP(void, nativeSetExpectedReleaseIblAsset)
(JNIEnv* env, jclass /*clazz*/, jlong token) {
  imp::SkyboxTestContext& context = imp::SkyboxTestContext::Get();
  context.release_image_based_lighting_asset.expected_token = token;
}

JNI_METHOD_AOSP(void, nativeSetExpectedSetEnvironmentLight)
(JNIEnv* env, jclass /*clazz*/, jlong token) {
  imp::SkyboxTestContext& context = imp::SkyboxTestContext::Get();
  context.set_environment_light.expected_token = token;
}

JNI_METHOD_AOSP(void, nativeSetExpectedClearEnvironmentLight)
(JNIEnv* env, jclass /*clazz*/) {
  imp::SkyboxTestContext& context = imp::SkyboxTestContext::Get();
  context.clear_environment_light.expected_clear = true;
}

// Texture Operations
JNI_METHOD_AOSP(void, nativeSetExpectedLoadTexturePath)
(JNIEnv* env, jclass /*clazz*/, jstring path) {
  imp::TextureTestContext& context = imp::TextureTestContext::Get();
  context.load_texture_asset_path.expected_path = imp::GetString(env, path);
}

JNI_METHOD_AOSP(void, nativeSetLoadTextureAssetSuccess)
(JNIEnv* env, jclass /*clazz*/, jlong token) {
  imp::TextureTestContext& context = imp::TextureTestContext::Get();
  context.load_texture_asset_path.success_token = token;
  context.load_texture_asset_path.failure_message.clear();
}

JNI_METHOD_AOSP(void, nativeSetLoadTextureAssetFailure)
(JNIEnv* env, jclass /*clazz*/, jstring message) {
  imp::TextureTestContext& context = imp::TextureTestContext::Get();
  std::string msg = imp::GetString(env, message);
  context.load_texture_asset_path.failure_message = msg;
}

JNI_METHOD_AOSP(void, nativeSetExpectedBorrowReflectionTexture)
(JNIEnv* env, jclass /*clazz*/) {
  imp::TextureTestContext& context = imp::TextureTestContext::Get();
  context.borrow_reflection_texture.expected_call = true;
}

JNI_METHOD_AOSP(void, nativeSetBorrowReflectionTextureSuccessToken)
(JNIEnv* env, jclass /*clazz*/, jlong token) {
  imp::TextureTestContext& context = imp::TextureTestContext::Get();
  context.borrow_reflection_texture.success_token = token;
}

JNI_METHOD_AOSP(void, nativeSetExpectedGetReflectionTextureFromIbl)
(JNIEnv* env, jclass /*clazz*/, jlong ibl_token) {
  imp::TextureTestContext& context = imp::TextureTestContext::Get();
  context.get_reflection_texture_from_ibl.expected_ibl_token = ibl_token;
}

JNI_METHOD_AOSP(void, nativeSetGetReflectionTextureFromIblSuccessToken)
(JNIEnv* env, jclass /*clazz*/, jlong texture_token) {
  imp::TextureTestContext& context = imp::TextureTestContext::Get();
  context.get_reflection_texture_from_ibl.success_token = texture_token;
}

JNI_METHOD_AOSP(void, nativeSetExpectedBorrowTexture)
(JNIEnv* env, jclass /*clazz*/, jlong texture_handle) {
  imp::TextureTestContext& context = imp::TextureTestContext::Get();
  context.borrow_texture.expected_handle = texture_handle;
}

}  // extern "C"
