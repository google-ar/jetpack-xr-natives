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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/asset_animator.h"
#include "apibindings/asset_loader.h"
#include "apibindings/bindings_custom_mesh.h"
#include "apibindings/bindings_mesh_buffer.h"
#include "apibindings/generic_material_manager.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/jni_conversion_utils.h"
#include "apibindings/jni_utils.h"
#include "apibindings/mesh_manager.h"
#include "apibindings/model_manager.h"
#include "apibindings/node_manager.h"
#include "apibindings/skybox_manager.h"
#include "apibindings/stereo_surface.h"
#include "apibindings/stereo_surface_manager.h"
#include "apibindings/texture_manager.h"
#include "apibindings/water_material_manager.h"
#include "core/common/buffer_access.h"
#include "core/common/jni_helpers.h"
#include "core/material_library/generic_material_spec.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/content_security_level.h"
#include "core/view/platforms/android/jni_helpers/exception_helper.h"
#include "core/view/platforms/android/wrappers/surface.h"

#define JNI_METHOD_AOSP(return_type, method_name) \
  IMP_JNI return_type JNICALL                     \
      Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_##method_name  // NOLINT

#define JNI_METHOD_AOSP_OLD(return_type, method_name) \
  IMP_JNI return_type JNICALL                         \
      Java_androidx_xr_scenecore_impl_impress_ImpressApiImpl_##method_name  // NOLINT

using ::imp::JniAllowlist;

namespace {

template <class T>
using ImpressApiImplAllowlist = JniAllowlist<T, imp::ImpressApiView>;

template <class T>
constexpr auto FromJava = &ImpressApiImplAllowlist<T>::FromJava;

// Helper to validate the Impress View and log an error if it is null.
inline bool IsValidView(imp::ImpressApiView* view) {
  if (view == nullptr) {
    IMP_LOG(imp::ERROR) << "ImpressApiView is null";
    return false;
  }
  return true;
}

}  // namespace

extern "C" {

JNI_METHOD_AOSP(void, nSetup)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  view->SetupImpressApiNative();
}

JNI_METHOD_AOSP_OLD(void, nSetup)
(JNIEnv* env, jclass clazz, jlong view_handle) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetup(
      env, clazz, view_handle);
}

JNI_METHOD_AOSP(void, nReleaseImageBasedLightingAsset)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong ibl_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetSkyboxManager().ReleaseImageBasedLightingAsset(ibl_token));
}

JNI_METHOD_AOSP_OLD(void, nReleaseImageBasedLightingAsset)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong ibl_token) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nReleaseImageBasedLightingAsset(  // NOLINT
      env, clazz, view_handle, ibl_token);
}

JNI_METHOD_AOSP(void, nLoadImageBasedLightingAssetFromPath)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->GetSkyboxManager().LoadImageBasedLightingAsset(
      imp::GetString(env, path), std::move(asset_loader));
}

JNI_METHOD_AOSP_OLD(void, nLoadImageBasedLightingAssetFromPath)
(JNIEnv* env, jclass clazz, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nLoadImageBasedLightingAssetFromPath(  // NOLINT
      env, clazz, view_handle, j_asset_loader, path);
}

JNI_METHOD_AOSP(void, nLoadImageBasedLightingAssetFromByteArray)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jbyteArray data, jstring key) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  // Move the binary data for the resource into the Releaser callback for the
  // Cord. The data is now managed by the Impress resource system.
  imp::BufferAccess native_data = imp::FromByteArray(env, data);
  absl::string_view data_view = native_data.StringView();
  // TODO: Don't make a copy of the data by using ByteArrayToCord
  // instead of MakeCordFromExternal when it is fixed.
  absl::Cord data_cord = absl::MakeCordFromExternal(
      data_view, [native_data = std::move(native_data)]() {});
  view->GetSkyboxManager().LoadImageBasedLightingAsset(
      data_cord, imp::GetString(env, key), std::move(asset_loader));
}

JNI_METHOD_AOSP_OLD(void, nLoadImageBasedLightingAssetFromByteArray)
(JNIEnv* env, jclass clazz, jlong view_handle, jobject j_asset_loader,
 jbyteArray data, jstring key) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nLoadImageBasedLightingAssetFromByteArray(  // NOLINT
      env, clazz, view_handle, j_asset_loader, data, key);
}

JNI_METHOD_AOSP(void, nLoadGltfAssetFromPath)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->GetModelManager().LoadGltfAsset(imp::GetString(env, path),
                                        std::move(asset_loader));
}

JNI_METHOD_AOSP_OLD(void, nLoadGltfAssetFromPath)
(JNIEnv* env, jclass clazz, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nLoadGltfAssetFromPath(  // NOLINT
      env, clazz, view_handle, j_asset_loader, path);
}

JNI_METHOD_AOSP(void, nLoadGltfAssetFromByteArray)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jbyteArray data, jstring key) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  // Move the binary data for the resource into the Releaser callback for the
  // Cord. The data is now managed by the Impress resource system.
  imp::BufferAccess native_data = imp::FromByteArray(env, data);
  absl::string_view data_view = native_data.StringView();
  // TODO: Avoid a copy of the data.
  absl::Cord data_cord = absl::MakeCordFromExternal(
      data_view, [native_data = std::move(native_data)]() {});
  view->GetModelManager().LoadGltfAsset(data_cord, imp::GetString(env, key),
                                        std::move(asset_loader));
}

JNI_METHOD_AOSP_OLD(void, nLoadGltfAssetFromByteArray)
(JNIEnv* env, jclass clazz, jlong view_handle, jobject j_asset_loader,
 jbyteArray data, jstring key) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nLoadGltfAssetFromByteArray(  // NOLINT
      env, clazz, view_handle, j_asset_loader, data, key);
}

JNI_METHOD_AOSP(void, nReleaseGltfAsset)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong gltf_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().ReleaseGltfAsset(gltf_token));
}

JNI_METHOD_AOSP_OLD(void, nReleaseGltfAsset)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong gltf_token) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nReleaseGltfAsset(  // NOLINT
      env, clazz, view_handle, gltf_token);
}

JNI_METHOD_AOSP(int32_t, nInstanceGltfModel__JJ)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong gltf_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  absl::StatusOr<int32_t> result =
      view->GetModelManager().InstanceGltfModel(gltf_token);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(int32_t, nInstanceGltfModel__JJ)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong gltf_token) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nInstanceGltfModel__JJ(  // NOLINT
      env, clazz, view_handle, gltf_token);
}

// TODO: Remove this method once the Java side is updated.
JNI_METHOD_AOSP(int32_t, nInstanceGltfModel__JJZ)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong gltf_token,
 jboolean enable_collider) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nInstanceGltfModel__JJ(  // NOLINT
      env, clazz, view_handle, gltf_token);
}

JNI_METHOD_AOSP_OLD(int32_t, nInstanceGltfModel__JJZ)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong gltf_token,
 jboolean enable_collider) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nInstanceGltfModel__JJZ(  // NOLINT
      env, clazz, view_handle, gltf_token, enable_collider);
}

// TODO: (broken link) - impress_node is a jint in the Java side.
JNI_METHOD_AOSP(void, nSetGltfModelColliderEnabled)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong impress_node,
 jboolean enable_collider) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().SetGltfModelColliderEnabled(
               impress_node, enable_collider));
}

JNI_METHOD_AOSP_OLD(void, nSetGltfModelColliderEnabled)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong impress_node,
 jboolean enable_collider) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetGltfModelColliderEnabled(  // NOLINT
      env, clazz, view_handle, impress_node, enable_collider);
}

JNI_METHOD_AOSP(void, nSetGltfReformAffordanceEnabled)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jboolean enable_affordance, jboolean system_movable) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().SetGltfReformAffordanceEnabled(
               impress_node, enable_affordance, system_movable));
}

JNI_METHOD_AOSP_OLD(void, nSetGltfReformAffordanceEnabled)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node,
 jboolean enable_affordance, jboolean system_movable) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetGltfReformAffordanceEnabled(  // NOLINT
      env, clazz, view_handle, impress_node, enable_affordance, system_movable);
}

JNI_METHOD_AOSP(void, nSetCustomMeshReformAffordanceEnabled)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jboolean enable_affordance, jboolean system_movable) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetMeshManager().SetCustomMeshReformAffordanceEnabled(
               impress_node, enable_affordance, system_movable));
}

JNI_METHOD_AOSP_OLD(void, nSetCustomMeshReformAffordanceEnabled)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node,
 jboolean enable_affordance, jboolean system_movable) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetCustomMeshReformAffordanceEnabled(  // NOLINT
      env, clazz, view_handle, impress_node, enable_affordance, system_movable);
}

JNI_METHOD_AOSP(void, nAnimateGltfModel)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jstring animation_name, jboolean loop, jfloat speed, jfloat start_time,
 jint channel_id, jobject j_asset_animator) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  auto asset_animator =
      std::make_unique<imp::AssetAnimator>(env, j_asset_animator);

  view->GetModelManager().AnimateGltfModel(
      impress_node, imp::GetString(env, animation_name), loop, speed,
      start_time, channel_id, std::move(asset_animator));
}

JNI_METHOD_AOSP_OLD(void, nAnimateGltfModel)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node,
 jstring animation_name, jboolean loop, jfloat speed, jfloat start_time,
 jint channel_id, jobject j_asset_animator) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nAnimateGltfModel(  // NOLINT
      env, clazz, view_handle, impress_node, animation_name, loop, speed,
      start_time, channel_id, j_asset_animator);
}

JNI_METHOD_AOSP(void, nStopGltfModelAnimation)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint channel_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetModelManager().StopGltfModelAnimation(impress_node, channel_id));
}

JNI_METHOD_AOSP_OLD(void, nStopGltfModelAnimation)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node,
 jint channel_id) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nStopGltfModelAnimation(  // NOLINT
      env, clazz, view_handle, impress_node, channel_id);
}

JNI_METHOD_AOSP(void, nToggleGltfModelAnimation)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jboolean toggle, jint channel_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().ToggleGltfModelAnimation(
               impress_node, toggle, channel_id));
}

JNI_METHOD_AOSP_OLD(void, nToggleGltfModelAnimation)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node,
 jboolean toggle, jint channel_id) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nToggleGltfModelAnimation(  // NOLINT
      env, clazz, view_handle, impress_node, toggle, channel_id);
}

JNI_METHOD_AOSP(void, nSetGltfModelAnimationSpeed)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jfloat speed, jint channel_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().SetGltfModelAnimationSpeed(
               impress_node, speed, channel_id));
}

JNI_METHOD_AOSP_OLD(void, nSetGltfModelAnimationSpeed)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node, jfloat speed,
 jint channel_id) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetGltfModelAnimationSpeed(  // NOLINT
      env, clazz, view_handle, impress_node, speed, channel_id);
}

JNI_METHOD_AOSP(void, nSetGltfModelAnimationPlaybackTime)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jfloat playback_time, jint channel_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().SetGltfModelAnimationPlaybackTime(
               impress_node, playback_time, channel_id));
}

JNI_METHOD_AOSP_OLD(void, nSetGltfModelAnimationPlaybackTime)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node,
 jfloat playback_time, jint channel_id) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetGltfModelAnimationPlaybackTime(  // NOLINT
      env, clazz, view_handle, impress_node, playback_time, channel_id);
}

JNI_METHOD_AOSP(jint, nGetGltfModelAnimationCount)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  absl::StatusOr<int32_t> result =
      view->GetModelManager().GetGltfModelAnimationCount(impress_node);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(jint, nGetGltfModelAnimationCount)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetGltfModelAnimationCount(  // NOLINT
      env, clazz, view_handle, impress_node);
}

JNI_METHOD_AOSP(jstring, nGetGltfModelAnimationName)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint index) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return nullptr;

  absl::StatusOr<std::string> result =
      view->GetModelManager().GetGltfModelAnimationName(impress_node, index);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return nullptr;
  }
  return env->NewStringUTF(result->c_str());
}

JNI_METHOD_AOSP_OLD(jstring, nGetGltfModelAnimationName)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node, jint index) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetGltfModelAnimationName(  // NOLINT
      env, clazz, view_handle, impress_node, index);
}

JNI_METHOD_AOSP(jfloat, nGetGltfModelAnimationDurationSeconds)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint index) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1.0f;

  absl::StatusOr<float> result =
      view->GetModelManager().GetGltfModelAnimationDurationSeconds(impress_node,
                                                                   index);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return -1.0f;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(jfloat, nGetGltfModelAnimationDurationSeconds)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node, jint index) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetGltfModelAnimationDurationSeconds(  // NOLINT
      env, clazz, view_handle, impress_node, index);
}

JNI_METHOD_AOSP(void, nGetGltfModelLocalBounds)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jfloatArray out_center, jfloatArray out_half_extent) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<imp::Box> result =
      view->GetModelManager().GetGltfModelLocalBounds(impress_node);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return;
  }

  if (out_center == nullptr || env->GetArrayLength(out_center) < 3) {
    (void)imp::android::ThrowIfError(
        env, absl::InvalidArgumentError(
                 "out_center must be a non-null float array with a length of "
                 "at least 3."));
    return;
  }
  if (out_half_extent == nullptr || env->GetArrayLength(out_half_extent) < 3) {
    (void)imp::android::ThrowIfError(
        env, absl::InvalidArgumentError(
                 "out_half_extent must be a non-null float array with a length "
                 "of at least 3."));
    return;
  }

  env->SetFloatArrayRegion(out_center, 0, 3, &result->center[0]);
  env->SetFloatArrayRegion(out_half_extent, 0, 3, &result->halfExtent[0]);
}

JNI_METHOD_AOSP_OLD(void, nGetGltfModelLocalBounds)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node,
 jfloatArray out_center, jfloatArray out_half_extent) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetGltfModelLocalBounds(  // NOLINT
      env, clazz, view_handle, impress_node, out_center, out_half_extent);
}

JNI_METHOD_AOSP(jint, nCreateImpressNode)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  return view->GetNodeManager().CreateImpressNode();
}

JNI_METHOD_AOSP_OLD(jint, nCreateImpressNode)
(JNIEnv* env, jclass clazz, jlong view_handle) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nCreateImpressNode(  // NOLINT
      env, clazz, view_handle);
}

JNI_METHOD_AOSP(void, nDestroyImpressNode)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetNodeManager().DestroyImpressNode(impress_node));
}

JNI_METHOD_AOSP_OLD(void, nDestroyImpressNode)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nDestroyImpressNode(  // NOLINT
      env, clazz, view_handle, impress_node);
}

JNI_METHOD_AOSP(void, nSetImpressNodeParent)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node_child,
 jint impress_node_parent) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetNodeManager().SetImpressNodeParent(impress_node_child,
                                                       impress_node_parent));
}

JNI_METHOD_AOSP_OLD(void, nSetImpressNodeParent)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node_child,
 jint impress_node_parent) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetImpressNodeParent(  // NOLINT
      env, clazz, view_handle, impress_node_child, impress_node_parent);
}

JNI_METHOD_AOSP(jint, nGetImpressNodeParent)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  absl::StatusOr<int32_t> result =
      view->GetNodeManager().GetImpressNodeParent(impress_node);
  if (!result.ok()) {
    (void)imp::android::ThrowIfError(env, result.status());
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(jint, nGetImpressNodeParent)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetImpressNodeParent(  // NOLINT
      env, clazz, view_handle, impress_node);
}

JNI_METHOD_AOSP(jint, nGetImpressNodeChildCount)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  absl::StatusOr<int32_t> result =
      view->GetNodeManager().GetImpressNodeChildCount(impress_node);
  if (!result.ok()) {
    (void)imp::android::ThrowIfError(env, result.status());
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(jint, nGetImpressNodeChildCount)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetImpressNodeChildCount(  // NOLINT
      env, clazz, view_handle, impress_node);
}

JNI_METHOD_AOSP(jint, nGetImpressNodeChildAt)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint index) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  absl::StatusOr<int32_t> result =
      view->GetNodeManager().GetImpressNodeChildAt(impress_node, index);
  if (!result.ok()) {
    (void)imp::android::ThrowIfError(env, result.status());
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(jint, nGetImpressNodeChildAt)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node, jint index) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetImpressNodeChildAt(  // NOLINT
      env, clazz, view_handle, impress_node, index);
}

JNI_METHOD_AOSP(jstring, nGetImpressNodeName)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return nullptr;

  absl::StatusOr<absl::string_view> result =
      view->GetNodeManager().GetImpressNodeName(impress_node);
  if (!result.ok()) {
    (void)imp::android::ThrowIfError(env, result.status());
    // Returned value does not matter since an exception was thrown.
    return nullptr;
  }
  return env->NewStringUTF(result->data());
}

JNI_METHOD_AOSP_OLD(jstring, nGetImpressNodeName)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetImpressNodeName(  // NOLINT
      env, clazz, view_handle, impress_node);
}

JNI_METHOD_AOSP(void, nSetImpressNodeLocalTransform)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node, jfloat tx,
 jfloat ty, jfloat tz, jfloat qx, jfloat qy, jfloat qz, jfloat qw, jfloat sx,
 jfloat sy, jfloat sz) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  imp::Transform<float> transform;
  transform.translation = {tx, ty, tz};
  transform.rotation = {qx, qy, qz, qw};
  transform.scale = {sx, sy, sz};

  (void)imp::android::ThrowIfError(
      env, view->GetNodeManager().SetImpressNodeLocalTransform(impress_node,
                                                               transform));
}

JNI_METHOD_AOSP_OLD(void, nSetImpressNodeLocalTransform)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node, jfloat tx,
 jfloat ty, jfloat tz, jfloat qx, jfloat qy, jfloat qz, jfloat qw, jfloat sx,
 jfloat sy, jfloat sz) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetImpressNodeLocalTransform(  // NOLINT
      env, clazz, view_handle, impress_node, tx, ty, tz, qx, qy, qz, qw, sx, sy,
      sz);
}

JNI_METHOD_AOSP(void, nGetImpressNodeLocalTransform)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jfloatArray out_transform) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  if (out_transform == nullptr || env->GetArrayLength(out_transform) < 10) {
    (void)imp::android::ThrowIfError(
        env, absl::InvalidArgumentError("Array not present or too small."));
    return;
  }

  absl::StatusOr<imp::Transform<float>> result =
      view->GetNodeManager().GetImpressNodeLocalTransform(impress_node);
  if (!imp::android::ThrowIfError(env, result.status()).ok()) return;

  float raw_data[10] = {result->translation.x, result->translation.y,
                        result->translation.z, result->rotation.x,
                        result->rotation.y,    result->rotation.z,
                        result->rotation.w,    result->scale.x,
                        result->scale.y,       result->scale.z};

  env->SetFloatArrayRegion(out_transform, 0, 10, raw_data);
}

JNI_METHOD_AOSP_OLD(void, nGetImpressNodeLocalTransform)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node,
 jfloatArray out_transform) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetImpressNodeLocalTransform(  // NOLINT
      env, clazz, view_handle, impress_node, out_transform);
}

JNI_METHOD_AOSP(void, nSetImpressNodeRelativeTransform)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint relative_impress_node, jfloat tx, jfloat ty, jfloat tz, jfloat qx,
 jfloat qy, jfloat qz, jfloat qw, jfloat sx, jfloat sy, jfloat sz) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  imp::Transform<float> transform;
  transform.translation = {tx, ty, tz};
  transform.rotation = {qx, qy, qz, qw};
  transform.scale = {sx, sy, sz};

  (void)imp::android::ThrowIfError(
      env, view->GetNodeManager().SetImpressNodeRelativeTransform(
               impress_node, relative_impress_node, transform));
}

JNI_METHOD_AOSP_OLD(void, nSetImpressNodeRelativeTransform)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node,
 jint relative_impress_node, jfloat tx, jfloat ty, jfloat tz, jfloat qx,
 jfloat qy, jfloat qz, jfloat qw, jfloat sx, jfloat sy, jfloat sz) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetImpressNodeRelativeTransform(  // NOLINT
      env, clazz, view_handle, impress_node, relative_impress_node, tx, ty, tz,
      qx, qy, qz, qw, sx, sy, sz);
}

JNI_METHOD_AOSP(void, nGetImpressNodeRelativeTransform)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint relative_impress_node, jfloatArray out_transform) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  if (out_transform == nullptr || env->GetArrayLength(out_transform) < 10) {
    (void)imp::android::ThrowIfError(
        env, absl::InvalidArgumentError("Array not present or too small."));
    return;
  }

  absl::StatusOr<imp::Transform<float>> result =
      view->GetNodeManager().GetImpressNodeRelativeTransform(
          impress_node, relative_impress_node);
  if (!imp::android::ThrowIfError(env, result.status()).ok()) return;

  float raw_data[10] = {result->translation.x, result->translation.y,
                        result->translation.z, result->rotation.x,
                        result->rotation.y,    result->rotation.z,
                        result->rotation.w,    result->scale.x,
                        result->scale.y,       result->scale.z};

  env->SetFloatArrayRegion(out_transform, 0, 10, raw_data);
}

JNI_METHOD_AOSP_OLD(void, nGetImpressNodeRelativeTransform)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node,
 jint relative_impress_node, jfloatArray out_transform) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetImpressNodeRelativeTransform(  // NOLINT
      env, clazz, view_handle, impress_node, relative_impress_node,
      out_transform);
}

// TODO: (broken link) - Update this to return Status
JNI_METHOD_AOSP(jint, nCreateStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, int stereo_mode,
 int blending_mode, int content_security_level, jboolean use_super_sampling) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  // Validate stereo mode.
  if (stereo_mode < static_cast<int>(imp::MediaStereoMode::kUnknown) ||
      stereo_mode >
          static_cast<int>(
              imp::MediaStereoMode::kInterleavedRightPrimaryWithDepth)) {
    IMP_LOG(imp::ERROR) << "Invalid stereo mode provided: " << stereo_mode
               << ". Using kUnknown instead.";
    stereo_mode = static_cast<int>(imp::MediaStereoMode::kUnknown);
  }

  // Validate content security level.
  if (content_security_level !=
      static_cast<int>(imp::ContentSecurityLevel::kProtected)) {
    IMP_LOG(imp::ERROR) << "Invalid content security level provided: "
               << content_security_level << ". Using kNone instead.";
    content_security_level = static_cast<int>(imp::ContentSecurityLevel::kNone);
  }

  // Validate blending mode.
  if (blending_mode < static_cast<int>(imp::MediaBlendingMode::kTransparent) ||
      blending_mode > static_cast<int>(imp::MediaBlendingMode::kOpaque)) {
    IMP_LOG(imp::ERROR) << "Invalid blending mode provided: " << blending_mode
               << ". Using kTransparent instead.";
    blending_mode = static_cast<int>(imp::MediaBlendingMode::kTransparent);
  }

  bool use_super_sampling_bool = use_super_sampling == JNI_TRUE;
  absl::StatusOr<int32_t> result =
      view->GetStereoSurfaceManager().CreateStereoSurfaceEntity(
          static_cast<imp::MediaStereoMode>(stereo_mode),
          static_cast<imp::MediaBlendingMode>(blending_mode),
          static_cast<imp::ContentSecurityLevel>(content_security_level),
          use_super_sampling_bool);
  if (!result.ok()) {
    (void)imp::android::ThrowIfError(env, result.status());
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(jint, nCreateStereoSurfaceEntity)
(JNIEnv* env, jclass clazz, jlong view_handle, int stereo_mode,
 int blending_mode, int content_security_level, jboolean use_super_sampling) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nCreateStereoSurfaceEntity(  // NOLINT
      env, clazz, view_handle, stereo_mode, blending_mode,
      content_security_level, use_super_sampling);
}

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityCanvasShapeQuad)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id, jfloat width,
 jfloat height, jfloat corner_radius) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetStereoSurfaceEntityCanvasShape(
          node_id, imp::StereoSurface::Quad({width, height, corner_radius})));
}

JNI_METHOD_AOSP_OLD(void, nSetStereoSurfaceEntityCanvasShapeQuad)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id, jfloat width,
 jfloat height, jfloat corner_radius) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetStereoSurfaceEntityCanvasShapeQuad(  // NOLINT
      env, clazz, view_handle, node_id, width, height, corner_radius);
}

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityCanvasShapeCurvedRect)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id, jfloat width,
 jfloat height, jfloat corner_radius, jfloat curve_radius) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  auto unused = imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager().SetStereoSurfaceEntityCanvasShape(
               node_id, imp::StereoSurface::CurvedRect(
                            {width, height, corner_radius, curve_radius})));
}

JNI_METHOD_AOSP_OLD(void, nSetStereoSurfaceEntityCanvasShapeCurvedRect)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id, jfloat width,
 jfloat height, jfloat corner_radius, jfloat curve_radius) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetStereoSurfaceEntityCanvasShapeCurvedRect(  // NOLINT
      env, clazz, view_handle, node_id, width, height, corner_radius,
      curve_radius);
}

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityCanvasShapeSphere)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jfloat radius) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager().SetStereoSurfaceEntityCanvasShape(
               node_id, imp::StereoSurface::Sphere({radius})));
}

JNI_METHOD_AOSP_OLD(void, nSetStereoSurfaceEntityCanvasShapeSphere)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id, jfloat radius) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetStereoSurfaceEntityCanvasShapeSphere(  // NOLINT
      env, clazz, view_handle, node_id, radius);
}

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityCanvasShapeHemisphere)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jfloat radius) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager().SetStereoSurfaceEntityCanvasShape(
               node_id, imp::StereoSurface::Hemisphere({radius})));
}

JNI_METHOD_AOSP_OLD(void, nSetStereoSurfaceEntityCanvasShapeHemisphere)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id, jfloat radius) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetStereoSurfaceEntityCanvasShapeHemisphere(  // NOLINT
      env, clazz, view_handle, node_id, radius);
}

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityCanvasShapeCustomMesh)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jobject left_positions, jobject left_texcoords, jobject left_indices,
 jobject right_positions, jobject right_texcoords, jobject right_indices,
 jint draw_mode) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<imp::StereoSurface::StereoMesh> mesh = imp::BuildStereoMesh(
      env, left_positions, left_texcoords, left_indices, right_positions,
      right_texcoords, right_indices, draw_mode);
  if (!imp::android::ThrowIfError(env, mesh.status()).ok()) {
    return;
  }

  (void)imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager().SetStereoSurfaceEntityCanvasShape(
               node_id, *mesh));
}

JNI_METHOD_AOSP_OLD(void, nSetStereoSurfaceEntityCanvasShapeCustomMesh)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id,
 jobject left_positions, jobject left_texcoords, jobject left_indices,
 jobject right_positions, jobject right_texcoords, jobject right_indices,
 jint draw_mode) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetStereoSurfaceEntityCanvasShapeCustomMesh(  // NOLINT
      env, clazz, view_handle, node_id, left_positions, left_texcoords,
      left_indices, right_positions, right_texcoords, right_indices, draw_mode);
}

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityColliderEnabled)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jboolean enable_collider) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetStereoSurfaceEntityColliderEnabled(
          node_id, enable_collider));
}

JNI_METHOD_AOSP_OLD(void, nSetStereoSurfaceEntityColliderEnabled)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id,
 jboolean enable_collider) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetStereoSurfaceEntityColliderEnabled(  // NOLINT
      env, clazz, view_handle, node_id, enable_collider);
}

JNI_METHOD_AOSP(jobject, nGetSurfaceFromStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return nullptr;

  absl::StatusOr<imp::android::Surface*> result =
      view->GetStereoSurfaceManager().GetSurfaceFromStereoSurfaceEntity(
          node_id);
  if (!result.ok()) {
    (void)imp::android::ThrowIfError(env, result.status());
    // Returned value does not matter since an exception was thrown.
    return nullptr;
  }
  // Note that Impress' Android::Surface is a JNI wrapper around the Android
  // Surface class. This returns a (Java-managed) reference as a jobject.
  return (*result)->WeakReference();
}

JNI_METHOD_AOSP_OLD(jobject, nGetSurfaceFromStereoSurfaceEntity)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetSurfaceFromStereoSurfaceEntity(  // NOLINT
      env, clazz, view_handle, node_id);
}

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntitySurfaceSize)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id, jint width,
 jint height) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager()
          .SetSurfaceDimensionsForStereoSurfaceEntity(node_id, width, height));
}

JNI_METHOD_AOSP_OLD(void, nSetStereoSurfaceEntitySurfaceSize)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id, jint width,
 jint height) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetStereoSurfaceEntitySurfaceSize(  // NOLINT
      env, clazz, view_handle, node_id, width, height);
}

JNI_METHOD_AOSP(void, nSetFeatherRadiusForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jfloat radius_x, jfloat radius_y) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetFeatherRadiusForStereoSurfaceEntity(
          node_id, {radius_x, radius_y}));
}

JNI_METHOD_AOSP_OLD(void, nSetFeatherRadiusForStereoSurfaceEntity)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id, jfloat radius_x,
 jfloat radius_y) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetFeatherRadiusForStereoSurfaceEntity(  // NOLINT
      env, clazz, view_handle, node_id, radius_x, radius_y);
}

JNI_METHOD_AOSP(void, nSetStereoModeForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jint stereo_mode) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  // Validate stereo mode.
  if (stereo_mode < static_cast<int>(imp::MediaStereoMode::kUnknown) ||
      stereo_mode >
          static_cast<int>(
              imp::MediaStereoMode::kInterleavedRightPrimaryWithDepth)) {
    IMP_LOG(imp::ERROR) << "Invalid stereo mode provided: " << stereo_mode
               << ". Using kUnknown instead.";
    stereo_mode = static_cast<int>(imp::MediaStereoMode::kUnknown);
  }

  (void)imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager().SetStereoModeForStereoSurfaceEntity(
               node_id, static_cast<imp::MediaStereoMode>(stereo_mode)));
}

JNI_METHOD_AOSP_OLD(void, nSetStereoModeForStereoSurfaceEntity)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id, jint stereo_mode) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetStereoModeForStereoSurfaceEntity(  // NOLINT
      env, clazz, view_handle, node_id, stereo_mode);
}

JNI_METHOD_AOSP(void, nSetBlendingModeForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jint blending_mode) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  // Validate blending mode.
  if (blending_mode < static_cast<int>(imp::MediaBlendingMode::kTransparent) ||
      blending_mode > static_cast<int>(imp::MediaBlendingMode::kOpaque)) {
    IMP_LOG(imp::ERROR) << "Invalid blending mode provided: " << blending_mode
               << ". Using kTransparent instead.";
    blending_mode = static_cast<int>(imp::MediaBlendingMode::kTransparent);
  }

  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetBlendingModeForStereoSurfaceEntity(
          node_id, static_cast<imp::MediaBlendingMode>(blending_mode)));
}

JNI_METHOD_AOSP_OLD(void, nSetBlendingModeForStereoSurfaceEntity)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id,
 jint blending_mode) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetBlendingModeForStereoSurfaceEntity(  // NOLINT
      env, clazz, view_handle, node_id, blending_mode);
}

JNI_METHOD_AOSP(void, nSetContentColorMetadataForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jint color_standard, jint color_transfer, jint color_range,
 jint max_luminance) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<imp::MediaColorSpace::Standard> verified_color_standard =
      imp::MediaColorSpace::ToColorStandard(color_standard);
  if (!imp::android::ThrowIfError(env, verified_color_standard).ok()) {
    return;
  }

  absl::StatusOr<imp::MediaColorSpace::Transfer> verified_color_transfer =
      imp::MediaColorSpace::ToColorTransfer(color_transfer);
  if (!imp::android::ThrowIfError(env, verified_color_transfer).ok()) {
    return;
  }

  absl::StatusOr<imp::MediaColorSpace::Range> verified_color_range =
      imp::MediaColorSpace::ToColorRange(color_range);
  if (!imp::android::ThrowIfError(env, verified_color_range).ok()) {
    return;
  }

  absl::StatusOr<uint16_t> verified_max_luminance =
      imp::MediaColorSpace::ToMaxContentLightLevel(max_luminance);
  if (!imp::android::ThrowIfError(env, verified_max_luminance).ok()) {
    return;
  }

  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager()
          .SetContentColorMetadataForStereoSurfaceEntity(
              node_id, imp::MediaColorSpace(
                           *verified_color_standard, *verified_color_transfer,
                           *verified_color_range, *verified_max_luminance)));
}

JNI_METHOD_AOSP_OLD(void, nSetContentColorMetadataForStereoSurfaceEntity)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id,
 jint color_standard, jint color_transfer, jint color_range,
 jint max_luminance) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetContentColorMetadataForStereoSurfaceEntity(  // NOLINT
      env, clazz, view_handle, node_id, color_standard, color_transfer,
      color_range, max_luminance);
}

JNI_METHOD_AOSP(void, nResetContentColorMetadataForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager()
               .SetContentColorMetadataForStereoSurfaceEntity(node_id));
}

JNI_METHOD_AOSP_OLD(void, nResetContentColorMetadataForStereoSurfaceEntity)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nResetContentColorMetadataForStereoSurfaceEntity(  // NOLINT
      env, clazz, view_handle, node_id);
}

JNI_METHOD_AOSP(void, nSetPrimaryAlphaMaskForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jlong alpha_mask_token, jobject j_asset_loader) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetPrimaryAlphaMaskForStereoSurfaceEntity(
          node_id, alpha_mask_token));
}

JNI_METHOD_AOSP_OLD(void, nSetPrimaryAlphaMaskForStereoSurfaceEntity)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id,
 jlong alpha_mask_token, jobject j_asset_loader) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetPrimaryAlphaMaskForStereoSurfaceEntity(  // NOLINT
      env, clazz, view_handle, node_id, alpha_mask_token, j_asset_loader);
}

JNI_METHOD_AOSP(void, nSetAuxiliaryAlphaMaskForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jlong alpha_mask_token, jobject j_asset_loader) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager()
               .SetAuxiliaryAlphaMaskForStereoSurfaceEntity(node_id,
                                                            alpha_mask_token));
}

JNI_METHOD_AOSP_OLD(void, nSetAuxiliaryAlphaMaskForStereoSurfaceEntity)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id,
 jlong alpha_mask_token, jobject j_asset_loader) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetAuxiliaryAlphaMaskForStereoSurfaceEntity(  // NOLINT
      env, clazz, view_handle, node_id, alpha_mask_token, j_asset_loader);
}

JNI_METHOD_AOSP(void, nSetSubViewConfigForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jfloat left_bottom, jfloat left_left, jfloat left_right, jfloat left_top,
 jfloat right_bottom, jfloat right_left, jfloat right_right, jfloat right_top) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  auto unused = imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetSubViewConfigForStereoSurfaceEntity(
          node_id, left_bottom, left_left, left_right, left_top, right_bottom,
          right_left, right_right, right_top));
}

JNI_METHOD_AOSP_OLD(void, nSetSubViewConfigForStereoSurfaceEntity)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id, jfloat left_bottom,
 jfloat left_left, jfloat left_right, jfloat left_top, jfloat right_bottom,
 jfloat right_left, jfloat right_right, jfloat right_top) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetSubViewConfigForStereoSurfaceEntity(  // NOLINT
      env, clazz, view_handle, node_id, left_bottom, left_left, left_right,
      left_top, right_bottom, right_left, right_right, right_top);
}

JNI_METHOD_AOSP(void, nLoadTexture)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->GetTextureManager().LoadTexture(imp::GetString(env, path),
                                        std::move(asset_loader));
}

JNI_METHOD_AOSP_OLD(void, nLoadTexture)
(JNIEnv* env, jclass clazz, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nLoadTexture(  // NOLINT
      env, clazz, view_handle, j_asset_loader, path);
}

JNI_METHOD_AOSP(std::intptr_t, nBorrowReflectionTexture)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  absl::StatusOr<intptr_t> result =
      view->GetTextureManager().BorrowReflectionTexture();
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(std::intptr_t, nBorrowReflectionTexture)
(JNIEnv* env, jclass clazz, jlong view_handle) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nBorrowReflectionTexture(  // NOLINT
      env, clazz, view_handle);
}

JNI_METHOD_AOSP(std::intptr_t, nGetReflectionTextureFromIbl)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong ibl_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  absl::StatusOr<intptr_t> result =
      view->GetTextureManager().GetReflectionTextureFromIbl(ibl_token);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(std::intptr_t, nGetReflectionTextureFromIbl)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong ibl_token) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetReflectionTextureFromIbl(  // NOLINT
      env, clazz, view_handle, ibl_token);
}

JNI_METHOD_AOSP(void, nCreateWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jboolean is_alpha_map_version) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->GetWaterMaterialManager().CreateWaterMaterial(std::move(asset_loader),
                                                      is_alpha_map_version);
}

JNI_METHOD_AOSP_OLD(void, nCreateWaterMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jobject j_asset_loader,
 jboolean is_alpha_map_version) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nCreateWaterMaterial(  // NOLINT
      env, clazz, view_handle, j_asset_loader, is_alpha_map_version);
}

JNI_METHOD_AOSP(void, nSetReflectionMapOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jlong reflection_map, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetReflectionMapOnWaterMaterial(
               water_material, reflection_map, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetReflectionMapOnWaterMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong water_material,
 jlong reflection_map, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetReflectionMapOnWaterMaterial(  // NOLINT
      env, clazz, view_handle, water_material, reflection_map, min_filter,
      mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r, compare_mode,
      compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetNormalMapOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jlong normal_map, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetNormalMapOnWaterMaterial(
               water_material, normal_map, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetNormalMapOnWaterMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong water_material,
 jlong normal_map, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetNormalMapOnWaterMaterial(  // NOLINT
      env, clazz, view_handle, water_material, normal_map, min_filter,
      mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r, compare_mode,
      compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetNormalTilingOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_tiling) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetNormalTilingOnWaterMaterial(
               water_material, normal_tiling));
}

JNI_METHOD_AOSP_OLD(void, nSetNormalTilingOnWaterMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong water_material,
 jfloat normal_tiling) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetNormalTilingOnWaterMaterial(  // NOLINT
      env, clazz, view_handle, water_material, normal_tiling);
}

JNI_METHOD_AOSP(void, nSetNormalSpeedOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_speed) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetNormalSpeedOnWaterMaterial(
               water_material, normal_speed));
}

JNI_METHOD_AOSP_OLD(void, nSetNormalSpeedOnWaterMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong water_material,
 jfloat normal_speed) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetNormalSpeedOnWaterMaterial(  // NOLINT
      env, clazz, view_handle, water_material, normal_speed);
}

JNI_METHOD_AOSP(void, nSetAlphaStepMultiplierOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat alpha_step_multiplier) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetWaterMaterialManager().SetAlphaStepMultiplierOnWaterMaterial(
          water_material, alpha_step_multiplier));
}

JNI_METHOD_AOSP_OLD(void, nSetAlphaStepMultiplierOnWaterMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong water_material,
 jfloat alpha_step_multiplier) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetAlphaStepMultiplierOnWaterMaterial(  // NOLINT
      env, clazz, view_handle, water_material, alpha_step_multiplier);
}

JNI_METHOD_AOSP(void, nSetAlphaMapOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jlong alpha_map, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetAlphaMapOnWaterMaterial(
               water_material, alpha_map, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetAlphaMapOnWaterMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong water_material,
 jlong alpha_map, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetAlphaMapOnWaterMaterial(  // NOLINT
      env, clazz, view_handle, water_material, alpha_map, min_filter,
      mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r, compare_mode,
      compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetNormalZOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_z) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetNormalZOnWaterMaterial(
               water_material, normal_z));
}

JNI_METHOD_AOSP_OLD(void, nSetNormalZOnWaterMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong water_material,
 jfloat normal_z) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetNormalZOnWaterMaterial(  // NOLINT
      env, clazz, view_handle, water_material, normal_z);
}

JNI_METHOD_AOSP(void, nSetNormalBoundaryOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_boundary) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetNormalBoundaryOnWaterMaterial(
               water_material, normal_boundary));
}

JNI_METHOD_AOSP_OLD(void, nSetNormalBoundaryOnWaterMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong water_material,
 jfloat normal_boundary) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetNormalBoundaryOnWaterMaterial(  // NOLINT
      env, clazz, view_handle, water_material, normal_boundary);
}

JNI_METHOD_AOSP(void, nCreateGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jint lighting_model, jint blend_mode, jint double_sided_mode) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  absl::StatusOr<imp::GenericMaterialSpec> generic_material_spec =
      imp::BuildGenericMaterialSpecFromValues(lighting_model, blend_mode,
                                              double_sided_mode);
  if (!imp::android::ThrowIfError(env, generic_material_spec.status()).ok()) {
    return;
  }
  view->GetGenericMaterialManager().CreateGenericMaterial(
      std::move(asset_loader), *generic_material_spec);
}

JNI_METHOD_AOSP_OLD(void, nCreateGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jobject j_asset_loader,
 jint lighting_model, jint blend_mode, jint double_sided_mode) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nCreateGenericMaterial(  // NOLINT
      env, clazz, view_handle, j_asset_loader, lighting_model, blend_mode,
      double_sided_mode);
}

JNI_METHOD_AOSP(void, nSetBaseColorTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong base_color_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetBaseColorTextureOnGenericMaterial(
          generic_material, base_color_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetBaseColorTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong base_color_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetBaseColorTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, base_color_texture, min_filter,
      mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r, compare_mode,
      compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetBaseColorUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetBaseColorUvTransformOnGenericMaterial(generic_material,
                                                         uv_transform));
}

JNI_METHOD_AOSP_OLD(void, nSetBaseColorUvTransformOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetBaseColorUvTransformOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, m00, m01, m02, m10, m11, m12,
      m20, m21, m22);
}

JNI_METHOD_AOSP(void, nSetBaseColorFactorsOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat x, jfloat y, jfloat z, jfloat w) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetBaseColorFactorsOnGenericMaterial(
          generic_material, {x, y, z, w}));
}

JNI_METHOD_AOSP_OLD(void, nSetBaseColorFactorsOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material, jfloat x,
 jfloat y, jfloat z, jfloat w) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetBaseColorFactorsOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, x, y, z, w);
}

JNI_METHOD_AOSP(void, nSetMetallicRoughnessTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong metallic_roughness_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager()
          .SetMetallicRoughnessTextureOnGenericMaterial(
              generic_material, metallic_roughness_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetMetallicRoughnessTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong metallic_roughness_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetMetallicRoughnessTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, metallic_roughness_texture,
      min_filter, mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r,
      compare_mode, compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetMetallicRoughnessUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetMetallicRoughnessUvTransformOnGenericMaterial(
                   generic_material, uv_transform));
}

JNI_METHOD_AOSP_OLD(void, nSetMetallicRoughnessUvTransformOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetMetallicRoughnessUvTransformOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, m00, m01, m02, m10, m11, m12,
      m20, m21, m22);
}

JNI_METHOD_AOSP(void, nSetMetallicFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager().SetMetallicFactorOnGenericMaterial(
               generic_material, factor));
}

JNI_METHOD_AOSP_OLD(void, nSetMetallicFactorOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat factor) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetMetallicFactorOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, factor);
}

JNI_METHOD_AOSP(void, nSetRoughnessFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetRoughnessFactorOnGenericMaterial(
          generic_material, factor));
}

JNI_METHOD_AOSP_OLD(void, nSetRoughnessFactorOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat factor) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetRoughnessFactorOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, factor);
}

JNI_METHOD_AOSP(void, nSetNormalTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong normal_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager().SetNormalTextureOnGenericMaterial(
               generic_material, normal_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetNormalTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong normal_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetNormalTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, normal_texture, min_filter,
      mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r, compare_mode,
      compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetNormalUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetNormalUvTransformOnGenericMaterial(
          generic_material, uv_transform));
}

JNI_METHOD_AOSP_OLD(void, nSetNormalUvTransformOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetNormalUvTransformOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, m00, m01, m02, m10, m11, m12,
      m20, m21, m22);
}

JNI_METHOD_AOSP(void, nSetNormalFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager().SetNormalFactorOnGenericMaterial(
               generic_material, factor));
}

JNI_METHOD_AOSP_OLD(void, nSetNormalFactorOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat factor) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetNormalFactorOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, factor);
}

JNI_METHOD_AOSP(void, nSetAmbientOcclusionTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong ambient_occlusion_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager()
          .SetAmbientOcclusionTextureOnGenericMaterial(
              generic_material, ambient_occlusion_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetAmbientOcclusionTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong ambient_occlusion_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetAmbientOcclusionTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, ambient_occlusion_texture,
      min_filter, mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r,
      compare_mode, compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetAmbientOcclusionUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetAmbientOcclusionUvTransformOnGenericMaterial(
                   generic_material, uv_transform));
}

JNI_METHOD_AOSP_OLD(void, nSetAmbientOcclusionUvTransformOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetAmbientOcclusionUvTransformOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, m00, m01, m02, m10, m11, m12,
      m20, m21, m22);
}

JNI_METHOD_AOSP(void, nSetAmbientOcclusionFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetAmbientOcclusionFactorOnGenericMaterial(generic_material,
                                                           factor));
}

JNI_METHOD_AOSP_OLD(void, nSetAmbientOcclusionFactorOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat factor) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetAmbientOcclusionFactorOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, factor);
}

JNI_METHOD_AOSP(void, nSetEmissiveTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong emissive_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetEmissiveTextureOnGenericMaterial(
          generic_material, emissive_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetEmissiveTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong emissive_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetEmissiveTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, emissive_texture, min_filter,
      mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r, compare_mode,
      compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetEmissiveUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetEmissiveUvTransformOnGenericMaterial(
          generic_material, uv_transform));
}

JNI_METHOD_AOSP_OLD(void, nSetEmissiveUvTransformOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetEmissiveUvTransformOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, m00, m01, m02, m10, m11, m12,
      m20, m21, m22);
}

JNI_METHOD_AOSP(void, nSetEmissiveFactorsOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat x, jfloat y, jfloat z) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetEmissiveFactorsOnGenericMaterial(
          generic_material, {x, y, z}));
}

JNI_METHOD_AOSP_OLD(void, nSetEmissiveFactorsOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material, jfloat x,
 jfloat y, jfloat z) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetEmissiveFactorsOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, x, y, z);
}

JNI_METHOD_AOSP(void, nSetClearcoatTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong clearcoat_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetClearcoatTextureOnGenericMaterial(
          generic_material, clearcoat_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetClearcoatTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong clearcoat_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetClearcoatTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, clearcoat_texture, min_filter,
      mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r, compare_mode,
      compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetClearcoatNormalTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong clearcoat_normal_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager()
          .SetClearcoatNormalTextureOnGenericMaterial(
              generic_material, clearcoat_normal_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetClearcoatNormalTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong clearcoat_normal_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetClearcoatNormalTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, clearcoat_normal_texture,
      min_filter, mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r,
      compare_mode, compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetClearcoatRoughnessTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong clearcoat_roughness_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager()
          .SetClearcoatRoughnessTextureOnGenericMaterial(
              generic_material, clearcoat_roughness_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetClearcoatRoughnessTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong clearcoat_roughness_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetClearcoatRoughnessTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, clearcoat_roughness_texture,
      min_filter, mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r,
      compare_mode, compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetClearcoatFactorsOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat intensity, jfloat roughness, jfloat normal) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetClearcoatFactorsOnGenericMaterial(
          generic_material, {intensity, roughness, normal}));
}

JNI_METHOD_AOSP_OLD(void, nSetClearcoatFactorsOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat intensity, jfloat roughness, jfloat normal) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetClearcoatFactorsOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, intensity, roughness, normal);
}

JNI_METHOD_AOSP(void, nSetSheenColorTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong sheen_color_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetSheenColorTextureOnGenericMaterial(
          generic_material, sheen_color_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetSheenColorTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong sheen_color_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetSheenColorTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, sheen_color_texture,
      min_filter, mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r,
      compare_mode, compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetSheenColorFactorsOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat x, jfloat y, jfloat z) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetSheenColorFactorsOnGenericMaterial(
          generic_material, {x, y, z}));
}

JNI_METHOD_AOSP_OLD(void, nSetSheenColorFactorsOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material, jfloat x,
 jfloat y, jfloat z) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetSheenColorFactorsOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, x, y, z);
}

JNI_METHOD_AOSP(void, nSetSheenRoughnessTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong sheen_roughness_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetSheenRoughnessTextureOnGenericMaterial(
                   generic_material, sheen_roughness_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetSheenRoughnessTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong sheen_roughness_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetSheenRoughnessTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, sheen_roughness_texture,
      min_filter, mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r,
      compare_mode, compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetSheenRoughnessFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager()
          .SetSheenRoughnessFactorOnGenericMaterial(generic_material, factor));
}

JNI_METHOD_AOSP_OLD(void, nSetSheenRoughnessFactorOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat factor) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetSheenRoughnessFactorOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, factor);
}

JNI_METHOD_AOSP(void, nSetTransmissionTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong transmission_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<filament::TextureSampler> native_sampler =
      imp::BuildTextureSamplerFromValues(min_filter, mag_filter, wrap_mode_s,
                                         wrap_mode_t, wrap_mode_r, compare_mode,
                                         compare_func, anisotropyLog2);
  if (!imp::android::ThrowIfError(env, native_sampler.status()).ok()) {
    return;
  }
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetTransmissionTextureOnGenericMaterial(
          generic_material, transmission_texture, *native_sampler));
}

JNI_METHOD_AOSP_OLD(void, nSetTransmissionTextureOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jlong transmission_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetTransmissionTextureOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, transmission_texture,
      min_filter, mag_filter, wrap_mode_s, wrap_mode_t, wrap_mode_r,
      compare_mode, compare_func, anisotropyLog2);
}

JNI_METHOD_AOSP(void, nSetTransmissionUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetTransmissionUvTransformOnGenericMaterial(generic_material,
                                                            uv_transform));
}

JNI_METHOD_AOSP_OLD(void, nSetTransmissionUvTransformOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetTransmissionUvTransformOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, m00, m01, m02, m10, m11, m12,
      m20, m21, m22);
}

JNI_METHOD_AOSP(void, nSetTransmissionFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetTransmissionFactorOnGenericMaterial(
          generic_material, factor));
}

JNI_METHOD_AOSP_OLD(void, nSetTransmissionFactorOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat factor) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetTransmissionFactorOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, factor);
}

JNI_METHOD_AOSP(void, nSetIndexOfRefractionOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat index_of_refraction) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetIndexOfRefractionOnGenericMaterial(
          generic_material, index_of_refraction));
}

JNI_METHOD_AOSP_OLD(void, nSetIndexOfRefractionOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat index_of_refraction) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetIndexOfRefractionOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, index_of_refraction);
}

JNI_METHOD_AOSP(void, nSetAlphaCutoffOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat alpha_cutoff) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager().SetAlphaCutoffOnGenericMaterial(
               generic_material, alpha_cutoff));
}

JNI_METHOD_AOSP_OLD(void, nSetAlphaCutoffOnGenericMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong generic_material,
 jfloat alpha_cutoff) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetAlphaCutoffOnGenericMaterial(  // NOLINT
      env, clazz, view_handle, generic_material, alpha_cutoff);
}

JNI_METHOD_AOSP(void, nDestroyNativeObject)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  view->DestroyNativeObject(handle);
}

JNI_METHOD_AOSP_OLD(void, nDestroyNativeObject)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong handle) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nDestroyNativeObject(  // NOLINT
      env, clazz, view_handle, handle);
}

JNI_METHOD_AOSP(void, nSetGltfModelNodeMaterialOverride)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id, jlong material,
 jint primitive_index) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().SetGltfModelNodeMaterialOverride(
               node_id, material, primitive_index));
}

JNI_METHOD_AOSP_OLD(void, nSetGltfModelNodeMaterialOverride)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id, jlong material,
 jint primitive_index) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetGltfModelNodeMaterialOverride(  // NOLINT
      env, clazz, view_handle, node_id, material, primitive_index);
}

JNI_METHOD_AOSP(void, nClearGltfModelNodeMaterialOverride)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jint primitive_index) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().ClearGltfModelNodeMaterialOverride(
               node_id, primitive_index));
}

JNI_METHOD_AOSP_OLD(void, nClearGltfModelNodeMaterialOverride)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id,
 jint primitive_index) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nClearGltfModelNodeMaterialOverride(  // NOLINT
      env, clazz, view_handle, node_id, primitive_index);
}

JNI_METHOD_AOSP(void, nScheduleGltfReskinning)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().ScheduleReskinning(impress_node));
}

JNI_METHOD_AOSP_OLD(void, nScheduleGltfReskinning)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nScheduleGltfReskinning(  // NOLINT
      env, clazz, view_handle, impress_node);
}

JNI_METHOD_AOSP(void, nSetEnvironmentLight)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong ibl_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetSkyboxManager().SetEnvironmentLight(ibl_token));
}

JNI_METHOD_AOSP_OLD(void, nSetEnvironmentLight)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong ibl_token) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetEnvironmentLight(  // NOLINT
      env, clazz, view_handle, ibl_token);
}

JNI_METHOD_AOSP(void, nClearEnvironmentLight)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  view->GetSkyboxManager().ClearEnvironmentLight();
}

JNI_METHOD_AOSP_OLD(void, nClearEnvironmentLight)
(JNIEnv* env, jclass clazz, jlong view_handle) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nClearEnvironmentLight(  // NOLINT
      env, clazz, view_handle);
}

JNI_METHOD_AOSP(void, nDisposeAllResources)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(env, view->DisposeAllResources());
}

JNI_METHOD_AOSP_OLD(void, nDisposeAllResources)
(JNIEnv* env, jclass clazz, jlong view_handle) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nDisposeAllResources(  // NOLINT
      env, clazz, view_handle);
}

// Creates a mesh buffer with the given options.
//
// Parameters:
//   * view_handle: the handle to the ImpressApiView
//   * attribute_ids: array of VertexAttribute mapping for each layout attribute
//     (e.g. kPosition, kNormal, etc.)
//   * attribute_types: array of VertexAttributeType for each layout attribute
//     (e.g. kFloat3, kFloat2, etc.)
//   * buffer_indices: array of buffer indices for each layout attribute
//   * byte_offsets: array of byte offsets for each layout attribute
//   * byte_strides: array of byte strides for each vertex buffer
//   * max_vertices: maximum number of vertices the buffer can hold. Zero means
//     that the BindingsMeshBuffer object will calculate max vertices
//     automatically based on the provided data size.
//   * max_indices: maximum number of indices the buffer can hold. Zero means
//     that the BindingsMeshBuffer object will calculate max indices
//     automatically based on the provided data size.
//   * vertex_data: array of direct byte buffers with initial vertex data, one
//     buffer per vertex buffer.
//   * vertex_data_offsets: offsets in bytes into each vertex_data buffer
//   * vertex_data_sizes: sizes in bytes for the initial vertex data in each
//     buffer
//   * index_data: direct byte buffer with initial index data
//   * index_data_offset: offset in bytes into the index_data buffer
//   * index_data_size: size in bytes for the initial index data
JNI_METHOD_AOSP(jlong, nCreateMeshBuffer)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jintArray attribute_ids,
 jintArray attribute_types, jbyteArray buffer_indices, jintArray byte_offsets,
 jintArray byte_strides, jint max_vertices, jint max_indices,
 jobjectArray vertex_data, jintArray vertex_data_offsets,
 jintArray vertex_data_sizes, jobject index_data, jint index_data_offset,
 jint index_data_size) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  if (attribute_ids == nullptr) {
    if (!imp::android::ThrowIfError(
             env, absl::InvalidArgumentError("attribute_ids must not be null."))
             .ok()) {
      return -1;
    }
  }
  jsize attributes_count = env->GetArrayLength(attribute_ids);
  if (attributes_count == 0) {
    if (!imp::android::ThrowIfError(
             env,
             absl::InvalidArgumentError("Attribute arrays must not be empty."))
             .ok()) {
      return -1;
    }
  }

  if (attribute_types == nullptr) {
    if (!imp::android::ThrowIfError(
             env,
             absl::InvalidArgumentError("attribute_types must not be null."))
             .ok()) {
      return -1;
    }
  }
  if (buffer_indices == nullptr) {
    if (!imp::android::ThrowIfError(
             env,
             absl::InvalidArgumentError("buffer_indices must not be null."))
             .ok()) {
      return -1;
    }
  }
  if (byte_offsets == nullptr) {
    if (!imp::android::ThrowIfError(
             env, absl::InvalidArgumentError("byte_offsets must not be null."))
             .ok()) {
      return -1;
    }
  }
  if (attributes_count != env->GetArrayLength(attribute_types) ||
      attributes_count != env->GetArrayLength(buffer_indices) ||
      attributes_count != env->GetArrayLength(byte_offsets)) {
    if (!imp::android::ThrowIfError(
             env, absl::InvalidArgumentError(
                      "Attribute arrays must have the same length."))
             .ok()) {
      return -1;
    }
  }

  if (max_vertices < 0) {
    if (!imp::android::ThrowIfError(
             env,
             absl::InvalidArgumentError("max_vertices must be non-negative."))
             .ok()) {
      return -1;
    }
  }
  if (max_indices < 0) {
    if (!imp::android::ThrowIfError(
             env,
             absl::InvalidArgumentError("max_indices must be non-negative."))
             .ok()) {
      return -1;
    }
  }

  std::vector<jint> ids(attributes_count);
  env->GetIntArrayRegion(attribute_ids, 0, attributes_count, ids.data());

  std::vector<jint> types(attributes_count);
  env->GetIntArrayRegion(attribute_types, 0, attributes_count, types.data());

  std::vector<jbyte> buffers(attributes_count);
  env->GetByteArrayRegion(buffer_indices, 0, attributes_count, buffers.data());

  std::vector<jint> offsets(attributes_count);
  env->GetIntArrayRegion(byte_offsets, 0, attributes_count, offsets.data());

  imp::BindingsMeshBuffer::VertexLayout layout;
  layout.attributes.reserve(attributes_count);
  for (size_t i = 0; i < attributes_count; ++i) {
    int16_t byte_offset =
        imp::BindingsMeshBuffer::VertexAttributeDescriptor::kAutoOffset;
    if (offsets[i] >= 0) {
      if (offsets[i] > INT16_MAX) {
        if (!imp::android::ThrowIfError(
                 env,
                 absl::InvalidArgumentError("Byte offset out of 16-bit range."))
                 .ok()) {
          return -1;
        }
      }
      byte_offset = static_cast<int16_t>(offsets[i]);
    }

    layout.attributes.push_back(
        {.attribute =
             static_cast<imp::BindingsMeshBuffer::VertexAttribute>(ids[i]),
         .type = static_cast<imp::BindingsMeshBuffer::VertexAttributeType>(
             types[i]),
         .buffer_index = static_cast<uint8_t>(buffers[i]),
         .byte_offset = byte_offset});
  }

  if (byte_strides != nullptr) {
    jsize strides_count = env->GetArrayLength(byte_strides);
    std::vector<jint> strides(strides_count);
    env->GetIntArrayRegion(byte_strides, 0, strides_count, strides.data());

    layout.strides.reserve(strides_count);
    for (size_t i = 0; i < strides_count; ++i) {
      if (strides[i] < 0 || strides[i] > INT16_MAX) {
        if (!imp::android::ThrowIfError(
                 env,
                 absl::InvalidArgumentError("Byte stride must be non-negative "
                                            "and within 16-bit range."))
                 .ok()) {
          return -1;
        }
      }
      layout.strides.push_back(static_cast<int16_t>(strides[i]));
    }
  }

  imp::BindingsMeshBuffer::CreateOptions options;
  options.layout = std::move(layout);
  options.max_vertices = max_vertices;
  options.max_indices = max_indices;

  if (vertex_data != nullptr) {
    jsize num_vertex_buffers = env->GetArrayLength(vertex_data);
    if (vertex_data_sizes == nullptr) {
      if (!imp::android::ThrowIfError(
               env, absl::InvalidArgumentError(
                        "Vertex data sizes must be provided if vertex "
                        "data is provided."))
               .ok()) {
        return -1;
      }
    }
    if (vertex_data_offsets == nullptr) {
      if (!imp::android::ThrowIfError(
               env, absl::InvalidArgumentError(
                        "Vertex data offsets must be provided if vertex "
                        "data is provided."))
               .ok()) {
        return -1;
      }
    }
    if (env->GetArrayLength(vertex_data_sizes) != num_vertex_buffers ||
        env->GetArrayLength(vertex_data_offsets) != num_vertex_buffers) {
      if (!imp::android::ThrowIfError(
               env, absl::InvalidArgumentError(
                        "Vertex data arrays must have the same length."))
               .ok()) {
        return -1;
      }
    }

    std::vector<jint> sizes(num_vertex_buffers);
    env->GetIntArrayRegion(vertex_data_sizes, 0, num_vertex_buffers,
                           sizes.data());

    std::vector<jint> offsets(num_vertex_buffers);
    env->GetIntArrayRegion(vertex_data_offsets, 0, num_vertex_buffers,
                           offsets.data());

    options.initial_vertex_data.resize(num_vertex_buffers);
    for (size_t i = 0; i < num_vertex_buffers; ++i) {
      jobject buffer = env->GetObjectArrayElement(vertex_data, i);
      if (buffer != nullptr) {
        uint8_t* bytes =
            static_cast<uint8_t*>(env->GetDirectBufferAddress(buffer));
        if (bytes != nullptr) {
          jlong capacity = env->GetDirectBufferCapacity(buffer);
          if (offsets[i] < 0 || sizes[i] < 0 ||
              static_cast<long>(offsets[i]) + sizes[i] > capacity) {
            if (!imp::android::ThrowIfError(
                     env,
                     absl::InvalidArgumentError("Vertex data buffer offset and "
                                                "size are out of bounds."))
                     .ok()) {
              env->DeleteLocalRef(buffer);
              return -1;
            }
          }
          options.initial_vertex_data[i] =
              absl::MakeSpan(bytes + offsets[i], sizes[i]);
        } else {
          if (!imp::android::ThrowIfError(
                   env, absl::InvalidArgumentError(
                            "Vertex data buffer is not a direct byte buffer."))
                   .ok()) {
            env->DeleteLocalRef(buffer);
            return -1;
          }
        }
        env->DeleteLocalRef(buffer);
      }
    }
  }

  if (index_data != nullptr) {
    uint8_t* bytes =
        static_cast<uint8_t*>(env->GetDirectBufferAddress(index_data));
    if (bytes != nullptr) {
      jlong capacity = env->GetDirectBufferCapacity(index_data);
      if (index_data_offset < 0 || index_data_size < 0 ||
          static_cast<long>(index_data_offset) + index_data_size > capacity) {
        if (!imp::android::ThrowIfError(
                 env,
                 absl::InvalidArgumentError("Index data buffer offset and size "
                                            "are out of bounds."))
                 .ok()) {
          return -1;
        }
      }
      options.initial_index_data =
          absl::MakeSpan(bytes + index_data_offset, index_data_size);
    } else {
      if (!imp::android::ThrowIfError(
               env, absl::InvalidArgumentError(
                        "Index data buffer is not a direct byte buffer."))
               .ok()) {
        return -1;
      }
    }
  }

  absl::StatusOr<std::intptr_t> result =
      view->GetMeshManager().CreateMeshBuffer(options);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(jlong, nCreateMeshBuffer)
(JNIEnv* env, jclass clazz, jlong view_handle, jintArray attribute_ids,
 jintArray attribute_types, jbyteArray buffer_indices, jintArray byte_offsets,
 jintArray byte_strides, jint max_vertices, jint max_indices,
 jobjectArray vertex_data, jintArray vertex_data_offsets,
 jintArray vertex_data_sizes, jobject index_data, jint index_data_offset,
 jint index_data_size) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nCreateMeshBuffer(  // NOLINT
      env, clazz, view_handle, attribute_ids, attribute_types, buffer_indices,
      byte_offsets, byte_strides, max_vertices, max_indices, vertex_data,
      vertex_data_offsets, vertex_data_sizes, index_data, index_data_offset,
      index_data_size);
}

JNI_METHOD_AOSP(void, nDestroyMeshBuffer)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong mesh_buffer_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetMeshManager().DestroyMeshBuffer(mesh_buffer_handle));
}

JNI_METHOD_AOSP_OLD(void, nDestroyMeshBuffer)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong mesh_buffer_handle) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nDestroyMeshBuffer(  // NOLINT
      env, clazz, view_handle, mesh_buffer_handle);
}

JNI_METHOD_AOSP(jlong, nCreateCustomMesh)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong mesh_buffer_handle,
 jintArray subset_offsets, jintArray subset_counts, jintArray subset_topologies,
 jfloat center_x, jfloat center_y, jfloat center_z, jfloat half_extent_x,
 jfloat half_extent_y, jfloat half_extent_z) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  if (subset_offsets == nullptr) {
    if (!imp::android::ThrowIfError(
             env,
             absl::InvalidArgumentError("subset_offsets must not be null."))
             .ok()) {
      return -1;
    }
  }
  if (subset_counts == nullptr) {
    if (!imp::android::ThrowIfError(
             env, absl::InvalidArgumentError("subset_counts must not be null."))
             .ok()) {
      return -1;
    }
  }
  if (subset_topologies == nullptr) {
    if (!imp::android::ThrowIfError(
             env,
             absl::InvalidArgumentError("subset_topologies must not be null."))
             .ok()) {
      return -1;
    }
  }

  jsize subsets_count = env->GetArrayLength(subset_offsets);
  if (subsets_count == 0) {
    if (!imp::android::ThrowIfError(
             env,
             absl::InvalidArgumentError("Subset arrays must not be empty."))
             .ok()) {
      return -1;
    }
  }

  if (subsets_count != env->GetArrayLength(subset_counts) ||
      subsets_count != env->GetArrayLength(subset_topologies)) {
    if (!imp::android::ThrowIfError(
             env, absl::InvalidArgumentError(
                      "Subset arrays must have the same length."))
             .ok()) {
      return -1;
    }
  }

  std::vector<jint> offsets(subsets_count);
  env->GetIntArrayRegion(subset_offsets, 0, subsets_count, offsets.data());

  std::vector<jint> counts(subsets_count);
  env->GetIntArrayRegion(subset_counts, 0, subsets_count, counts.data());

  std::vector<jint> topologies(subsets_count);
  env->GetIntArrayRegion(subset_topologies, 0, subsets_count,
                         topologies.data());

  std::vector<imp::BindingsCustomMesh::Subset> subsets;
  subsets.reserve(subsets_count);
  for (size_t i = 0; i < subsets_count; ++i) {
    if (offsets[i] < 0) {
      if (!imp::android::ThrowIfError(
               env, absl::InvalidArgumentError(
                        "Subset offsets must be non-negative."))
               .ok()) {
        return -1;
      }
    }
    if (counts[i] < 0) {
      if (!imp::android::ThrowIfError(
               env, absl::InvalidArgumentError(
                        "Subset counts must be non-negative."))
               .ok()) {
        return -1;
      }
    }
    subsets.push_back(
        {.index_offset = offsets[i],
         .index_count = counts[i],
         .topology = static_cast<imp::BindingsCustomMesh::SubsetTopology>(
             topologies[i])});
  }

  // A bounding box with any negative half-extent value signals that a custom
  // bounding box is not being provided. In this case, the automatically
  // calculated bounding box of the mesh buffer will be used.
  std::optional<imp::Box> box;
  if (half_extent_x >= 0.0f && half_extent_y >= 0.0f && half_extent_z >= 0.0f) {
    imp::Box b;
    b.center = {center_x, center_y, center_z};
    b.halfExtent = {half_extent_x, half_extent_y, half_extent_z};
    box = b;
  }

  absl::StatusOr<std::intptr_t> result =
      view->GetMeshManager().CreateCustomMesh(
          static_cast<std::intptr_t>(mesh_buffer_handle), subsets, box);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(jlong, nCreateCustomMesh)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong mesh_buffer_handle,
 jintArray subset_offsets, jintArray subset_counts, jintArray subset_topologies,
 jfloat center_x, jfloat center_y, jfloat center_z, jfloat half_extent_x,
 jfloat half_extent_y, jfloat half_extent_z) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nCreateCustomMesh(  // NOLINT
      env, clazz, view_handle, mesh_buffer_handle, subset_offsets,
      subset_counts, subset_topologies, center_x, center_y, center_z,
      half_extent_x, half_extent_y, half_extent_z);
}

JNI_METHOD_AOSP(void, nGetCustomMeshAabb)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong custom_mesh_handle,
 jfloatArray out_aabb) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  absl::StatusOr<imp::Box> result = view->GetMeshManager().GetCustomMeshAabb(
      static_cast<std::intptr_t>(custom_mesh_handle));
  if (!imp::android::ThrowIfError(env, result).ok()) {
    return;
  }

  if (out_aabb == nullptr || env->GetArrayLength(out_aabb) != 6) {
    if (!imp::android::ThrowIfError(
             env,
             absl::InvalidArgumentError(
                 "out_aabb must be a non-null float array with a length of 6."))
             .ok()) {
      return;
    }
  }

  float raw_data[6] = {result->center[0],     result->center[1],
                       result->center[2],     result->halfExtent[0],
                       result->halfExtent[1], result->halfExtent[2]};
  env->SetFloatArrayRegion(out_aabb, 0, 6, raw_data);
}

JNI_METHOD_AOSP_OLD(void, nGetCustomMeshAabb)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong custom_mesh_handle,
 jfloatArray out_aabb) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nGetCustomMeshAabb(  // NOLINT
      env, clazz, view_handle, custom_mesh_handle, out_aabb);
}

JNI_METHOD_AOSP(void, nDestroyCustomMesh)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong custom_mesh_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetMeshManager().DestroyCustomMesh(custom_mesh_handle));
}

JNI_METHOD_AOSP_OLD(void, nDestroyCustomMesh)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong custom_mesh_handle) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nDestroyCustomMesh(  // NOLINT
      env, clazz, view_handle, custom_mesh_handle);
}

JNI_METHOD_AOSP(void, nSetCustomMeshNodeMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jint submesh_index, jlong material_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env,
      view->GetMeshManager().SetCustomMeshNodeMaterial(
          node_id, submesh_index, static_cast<std::intptr_t>(material_handle)));
}

JNI_METHOD_AOSP_OLD(void, nSetCustomMeshNodeMaterial)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id, jint submesh_index,
 jlong material_handle) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetCustomMeshNodeMaterial(  // NOLINT
      env, clazz, view_handle, node_id, submesh_index, material_handle);
}

JNI_METHOD_AOSP(void, nSetCustomMeshNodeColliderEnabled)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jboolean enable_collider) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  (void)imp::android::ThrowIfError(
      env, view->GetMeshManager().SetCustomMeshNodeColliderEnabled(
               node_id, enable_collider));
}

JNI_METHOD_AOSP_OLD(void, nSetCustomMeshNodeColliderEnabled)
(JNIEnv* env, jclass clazz, jlong view_handle, jint node_id,
 jboolean enable_collider) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nSetCustomMeshNodeColliderEnabled(  // NOLINT
      env, clazz, view_handle, node_id, enable_collider);
}

JNI_METHOD_AOSP(jint, nCreateCustomMeshNode)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong custom_mesh_handle,
 jlongArray material_handles, jint bone_count, jboolean enable_collider) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return -1;

  if (material_handles == nullptr) {
    if (!imp::android::ThrowIfError(
             env,
             absl::InvalidArgumentError("material_handles must not be null."))
             .ok()) {
      return -1;
    }
  }

  jsize material_count = env->GetArrayLength(material_handles);
  std::vector<jlong> handles(material_count);
  env->GetLongArrayRegion(material_handles, 0, material_count, handles.data());

  std::vector<std::intptr_t> native_handles;
  native_handles.reserve(material_count);
  for (jlong h : handles) {
    native_handles.push_back(static_cast<std::intptr_t>(h));
  }

  absl::StatusOr<int32_t> result = view->GetMeshManager().CreateCustomMeshNode(
      static_cast<std::intptr_t>(custom_mesh_handle), native_handles,
      bone_count, enable_collider);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP_OLD(jint, nCreateCustomMeshNode)
(JNIEnv* env, jclass clazz, jlong view_handle, jlong custom_mesh_handle,
 jlongArray material_handles, jint bone_count, jboolean enable_collider) {
  return Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nCreateCustomMeshNode(  // NOLINT
      env, clazz, view_handle, custom_mesh_handle, material_handles, bone_count,
      enable_collider);
}

JNI_METHOD_AOSP(void, nUpdateCustomMeshNodeBoneTransforms)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint offset, jfloatArray transforms) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  if (!IsValidView(view)) return;

  if (transforms == nullptr) {
    (void)imp::android::ThrowIfError(
        env, absl::InvalidArgumentError("transforms must not be null."));
    return;
  }

  jsize transforms_length = env->GetArrayLength(transforms);
  jfloat* transforms_data = env->GetFloatArrayElements(transforms, nullptr);
  if (transforms_data == nullptr) {
    return;
  }

  (void)imp::android::ThrowIfError(
      env, view->GetMeshManager().UpdateCustomMeshNodeBoneTransforms(
               impress_node, offset,
               absl::MakeSpan(transforms_data, transforms_length)));

  env->ReleaseFloatArrayElements(transforms, transforms_data, JNI_ABORT);
}

JNI_METHOD_AOSP_OLD(void, nUpdateCustomMeshNodeBoneTransforms)
(JNIEnv* env, jclass clazz, jlong view_handle, jint impress_node, jint offset,
 jfloatArray transforms) {
  Java_androidx_xr_scenecore_spatial_rendering_impress_ImpressApiImpl_nUpdateCustomMeshNodeBoneTransforms(  // NOLINT
      env, clazz, view_handle, impress_node, offset, transforms);
}

}  // extern "C"
