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

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/asset_animator.h"
#include "apibindings/asset_loader.h"
#include "apibindings/generic_material_manager.h"
#include "apibindings/impress_api_view.h"
#include "apibindings/jni_conversion_utils.h"
#include "apibindings/jni_utils.h"
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
      Java_androidx_xr_scenecore_impl_impress_ImpressApiImpl_##method_name  // NOLINT

using ::imp::JniAllowlist;

namespace {

template <class T>
using ImpressApiImplAllowlist = JniAllowlist<T, imp::ImpressApiView>;

template <class T>
constexpr auto FromJava = &ImpressApiImplAllowlist<T>::FromJava;

}  // namespace

extern "C" {

JNI_METHOD_AOSP(void, nSetup)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  view->SetupImpressApiNative();
}

JNI_METHOD_AOSP(void, nReleaseImageBasedLightingAsset)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong ibl_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetSkyboxManager().ReleaseImageBasedLightingAsset(ibl_token));
}

JNI_METHOD_AOSP(void, nLoadImageBasedLightingAssetFromPath)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->GetSkyboxManager().LoadImageBasedLightingAsset(
      imp::GetString(env, path), std::move(asset_loader));
}

JNI_METHOD_AOSP(void, nLoadImageBasedLightingAssetFromByteArray)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jbyteArray data, jstring key) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nLoadGltfAssetFromPath)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->GetModelManager().LoadGltfAsset(imp::GetString(env, path),
                                        std::move(asset_loader));
}

JNI_METHOD_AOSP(void, nLoadGltfAssetFromByteArray)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jbyteArray data, jstring key) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nReleaseGltfAsset)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong gltf_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().ReleaseGltfAsset(gltf_token));
}

JNI_METHOD_AOSP(int32_t, nInstanceGltfModel)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong gltf_token,
 jboolean enable_collider) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  absl::StatusOr<int32_t> result =
      view->GetModelManager().InstanceGltfModel(gltf_token, enable_collider);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

// TODO: (broken link) - impress_node is a jint in the Java side.
JNI_METHOD_AOSP(void, nSetGltfModelColliderEnabled)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong impress_node,
 jboolean enable_collider) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().SetGltfModelColliderEnabled(
               impress_node, enable_collider));
}

JNI_METHOD_AOSP(void, nSetGltfReformAffordanceEnabled)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jboolean enable_affordance, jboolean system_movable) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().SetGltfReformAffordanceEnabled(
               impress_node, enable_affordance, system_movable));
}

JNI_METHOD_AOSP(void, nAnimateGltfModelNew)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jstring animation_name, jboolean loop, jfloat speed, jfloat start_time,
 jint channel_id, jobject j_asset_animator) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_animator =
      std::make_unique<imp::AssetAnimator>(env, j_asset_animator);

  view->GetModelManager().AnimateGltfModelNew(
      impress_node, imp::GetString(env, animation_name), loop, speed,
      start_time, channel_id, std::move(asset_animator));
}

// TODO: (broken link) - Remove old animation APIs once all clients are migrated
// to new animation system.
JNI_METHOD_AOSP(void, nAnimateGltfModel)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jstring animation_name, jboolean loop, jobject j_asset_animator) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_animator =
      std::make_unique<imp::AssetAnimator>(env, j_asset_animator);
  view->GetModelManager().AnimateGltfModel(impress_node,
                                           imp::GetString(env, animation_name),
                                           loop, std::move(asset_animator));
}

JNI_METHOD_AOSP(void, nStopGltfModelAnimationNew)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint channel_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().StopGltfModelAnimationNew(impress_node,
                                                             channel_id));
}

// TODO: (broken link) - Remove old animation APIs once all clients are migrated
// to new animation system.
JNI_METHOD_AOSP(void, nStopGltfModelAnimation)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().StopGltfModelAnimation(impress_node));
}

JNI_METHOD_AOSP(void, nToggleGltfModelAnimationNew)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jboolean toggle, jint channel_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().ToggleGltfModelAnimationNew(
               impress_node, toggle, channel_id));
}

// TODO: (broken link) - Remove old animation APIs once all clients are migrated
// to new animation system.
JNI_METHOD_AOSP(void, nToggleGltfModelAnimation)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jboolean toggle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetModelManager().ToggleGltfModelAnimation(impress_node, toggle));
}

JNI_METHOD_AOSP(void, nSetGltfModelAnimationSpeed)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jfloat speed, jint channel_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().SetGltfModelAnimationSpeed(
               impress_node, speed, channel_id));
}

JNI_METHOD_AOSP(void, nSetGltfModelAnimationPlaybackTime)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jfloat playback_time, jint channel_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().SetGltfModelAnimationPlaybackTime(
               impress_node, playback_time, channel_id));
}

JNI_METHOD_AOSP(jint, nGetGltfModelAnimationCount)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  absl::StatusOr<int32_t> result =
      view->GetModelManager().GetGltfModelAnimationCount(impress_node);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP(jstring, nGetGltfModelAnimationName)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint index) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  absl::StatusOr<std::string> result =
      view->GetModelManager().GetGltfModelAnimationName(impress_node, index);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return nullptr;
  }
  return env->NewStringUTF(result->c_str());
}

JNI_METHOD_AOSP(jfloat, nGetGltfModelAnimationDurationSeconds)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint index) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  absl::StatusOr<float> result =
      view->GetModelManager().GetGltfModelAnimationDurationSeconds(impress_node,
                                                                   index);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return -1.0f;
  }
  return *result;
}

JNI_METHOD_AOSP(void, nGetGltfModelLocalBounds)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jfloatArray out_center, jfloatArray out_half_extent) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(jint, nCreateImpressNode)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  return view->GetNodeManager().CreateImpressNode();
}

JNI_METHOD_AOSP(void, nDestroyImpressNode)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetNodeManager().DestroyImpressNode(impress_node));
}

JNI_METHOD_AOSP(void, nSetImpressNodeParent)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node_child,
 jint impress_node_parent) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetNodeManager().SetImpressNodeParent(impress_node_child,
                                                       impress_node_parent));
}

JNI_METHOD_AOSP(jint, nGetImpressNodeParent)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  absl::StatusOr<int32_t> result =
      view->GetNodeManager().GetImpressNodeParent(impress_node);
  if (!result.ok()) {
    (void)imp::android::ThrowIfError(env, result.status());
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP(jint, nGetImpressNodeChildCount)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  absl::StatusOr<int32_t> result =
      view->GetNodeManager().GetImpressNodeChildCount(impress_node);
  if (!result.ok()) {
    (void)imp::android::ThrowIfError(env, result.status());
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP(jint, nGetImpressNodeChildAt)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint index) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  absl::StatusOr<int32_t> result =
      view->GetNodeManager().GetImpressNodeChildAt(impress_node, index);
  if (!result.ok()) {
    (void)imp::android::ThrowIfError(env, result.status());
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP(jstring, nGetImpressNodeName)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  absl::StatusOr<absl::string_view> result =
      view->GetNodeManager().GetImpressNodeName(impress_node);
  if (!result.ok()) {
    (void)imp::android::ThrowIfError(env, result.status());
    // Returned value does not matter since an exception was thrown.
    return nullptr;
  }
  return env->NewStringUTF(result->data());
}

JNI_METHOD_AOSP(void, nSetImpressNodeLocalTransform)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node, jfloat tx,
 jfloat ty, jfloat tz, jfloat qx, jfloat qy, jfloat qz, jfloat qw, jfloat sx,
 jfloat sy, jfloat sz) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

  imp::Transform<float> transform;
  transform.translation = {tx, ty, tz};
  transform.rotation = {qx, qy, qz, qw};
  transform.scale = {sx, sy, sz};

  (void)imp::android::ThrowIfError(
      env, view->GetNodeManager().SetImpressNodeLocalTransform(impress_node,
                                                               transform));
}

JNI_METHOD_AOSP(void, nGetImpressNodeLocalTransform)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jfloatArray out_transform) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

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

JNI_METHOD_AOSP(void, nSetImpressNodeRelativeTransform)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint relative_impress_node, jfloat tx, jfloat ty, jfloat tz, jfloat qx,
 jfloat qy, jfloat qz, jfloat qw, jfloat sx, jfloat sy, jfloat sz) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

  imp::Transform<float> transform;
  transform.translation = {tx, ty, tz};
  transform.rotation = {qx, qy, qz, qw};
  transform.scale = {sx, sy, sz};

  (void)imp::android::ThrowIfError(
      env, view->GetNodeManager().SetImpressNodeRelativeTransform(
               impress_node, relative_impress_node, transform));
}

JNI_METHOD_AOSP(void, nGetImpressNodeRelativeTransform)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jint relative_impress_node, jfloatArray out_transform) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

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

// TODO: (broken link) - Update this to return Status
JNI_METHOD_AOSP(jint, nCreateStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, int stereo_mode,
 int blending_mode, int content_security_level, jboolean use_super_sampling) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

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

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityCanvasShapeQuad)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id, jfloat width,
 jfloat height, jfloat corner_radius) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetStereoSurfaceEntityCanvasShape(
          node_id, imp::StereoSurface::Quad({width, height, corner_radius})));
}

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityCanvasShapeSphere)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jfloat radius) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager().SetStereoSurfaceEntityCanvasShape(
               node_id, imp::StereoSurface::Sphere({radius})));
}

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityCanvasShapeHemisphere)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jfloat radius) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager().SetStereoSurfaceEntityCanvasShape(
               node_id, imp::StereoSurface::Hemisphere({radius})));
}

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityCanvasShapeCustomMesh)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jobject left_positions, jobject left_texcoords, jobject left_indices,
 jobject right_positions, jobject right_texcoords, jobject right_indices,
 jint draw_mode) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntityColliderEnabled)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jboolean enable_collider) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetStereoSurfaceEntityColliderEnabled(
          node_id, enable_collider));
}

JNI_METHOD_AOSP(jobject, nGetSurfaceFromStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetStereoSurfaceEntitySurfaceSize)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id, jint width,
 jint height) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager()
          .SetSurfaceDimensionsForStereoSurfaceEntity(node_id, width, height));
}

JNI_METHOD_AOSP(void, nSetFeatherRadiusForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jfloat radius_x, jfloat radius_y) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetFeatherRadiusForStereoSurfaceEntity(
          node_id, {radius_x, radius_y}));
}

JNI_METHOD_AOSP(void, nSetStereoModeForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jint stereo_mode) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

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

JNI_METHOD_AOSP(void, nSetBlendingModeForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jint blending_mode) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

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

JNI_METHOD_AOSP(void, nSetContentColorMetadataForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jint color_standard, jint color_transfer, jint color_range,
 jint max_luminance) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

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

JNI_METHOD_AOSP(void, nResetContentColorMetadataForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager()
               .SetContentColorMetadataForStereoSurfaceEntity(node_id));
}

JNI_METHOD_AOSP(void, nSetPrimaryAlphaMaskForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jlong alpha_mask_token, jobject j_asset_loader) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetPrimaryAlphaMaskForStereoSurfaceEntity(
          node_id, alpha_mask_token));
}

JNI_METHOD_AOSP(void, nSetAuxiliaryAlphaMaskForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jlong alpha_mask_token, jobject j_asset_loader) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetStereoSurfaceManager()
               .SetAuxiliaryAlphaMaskForStereoSurfaceEntity(node_id,
                                                            alpha_mask_token));
}

// TODO: Add support for StereoSubViews which would take separate
// rectangles for the left and right eye.
JNI_METHOD_AOSP(void, nSetSubViewConfigForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id, jfloat bottom,
 jfloat left, jfloat right, jfloat top) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto unused = imp::android::ThrowIfError(
      env,
      view->GetStereoSurfaceManager().SetSubViewConfigForStereoSurfaceEntity(
          node_id, bottom, left, right, top));
}

JNI_METHOD_AOSP(void, nLoadTexture)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->GetTextureManager().LoadTexture(imp::GetString(env, path),
                                        std::move(asset_loader));
}

JNI_METHOD_AOSP(std::intptr_t, nBorrowReflectionTexture)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  absl::StatusOr<intptr_t> result =
      view->GetTextureManager().BorrowReflectionTexture();
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP(std::intptr_t, nGetReflectionTextureFromIbl)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong ibl_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  absl::StatusOr<intptr_t> result =
      view->GetTextureManager().GetReflectionTextureFromIbl(ibl_token);
  if (!imp::android::ThrowIfError(env, result).ok()) {
    // Returned value does not matter since an exception was thrown.
    return -1;
  }
  return *result;
}

JNI_METHOD_AOSP(void, nCreateWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jboolean is_alpha_map_version) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->GetWaterMaterialManager().CreateWaterMaterial(std::move(asset_loader),
                                                      is_alpha_map_version);
}

JNI_METHOD_AOSP(void, nSetReflectionMapOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jlong reflection_map, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetNormalMapOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jlong normal_map, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetNormalTilingOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_tiling) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetNormalTilingOnWaterMaterial(
               water_material, normal_tiling));
}

JNI_METHOD_AOSP(void, nSetNormalSpeedOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_speed) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetNormalSpeedOnWaterMaterial(
               water_material, normal_speed));
}

JNI_METHOD_AOSP(void, nSetAlphaStepMultiplierOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat alpha_step_multiplier) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetWaterMaterialManager().SetAlphaStepMultiplierOnWaterMaterial(
          water_material, alpha_step_multiplier));
}

JNI_METHOD_AOSP(void, nSetAlphaMapOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jlong alpha_map, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetNormalZOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_z) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetNormalZOnWaterMaterial(
               water_material, normal_z));
}

JNI_METHOD_AOSP(void, nSetNormalBoundaryOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_boundary) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetWaterMaterialManager().SetNormalBoundaryOnWaterMaterial(
               water_material, normal_boundary));
}

JNI_METHOD_AOSP(void, nCreateGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jint lighting_model, jint blend_mode, jint double_sided_mode) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetBaseColorTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong base_color_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetBaseColorUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetBaseColorUvTransformOnGenericMaterial(generic_material,
                                                         uv_transform));
}

JNI_METHOD_AOSP(void, nSetBaseColorFactorsOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat x, jfloat y, jfloat z, jfloat w) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetBaseColorFactorsOnGenericMaterial(
          generic_material, {x, y, z, w}));
}

JNI_METHOD_AOSP(void, nSetMetallicRoughnessTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong metallic_roughness_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetMetallicRoughnessUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetMetallicRoughnessUvTransformOnGenericMaterial(
                   generic_material, uv_transform));
}

JNI_METHOD_AOSP(void, nSetMetallicFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager().SetMetallicFactorOnGenericMaterial(
               generic_material, factor));
}

JNI_METHOD_AOSP(void, nSetRoughnessFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetRoughnessFactorOnGenericMaterial(
          generic_material, factor));
}

JNI_METHOD_AOSP(void, nSetNormalTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong normal_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetNormalUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetNormalUvTransformOnGenericMaterial(
          generic_material, uv_transform));
}

JNI_METHOD_AOSP(void, nSetNormalFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager().SetNormalFactorOnGenericMaterial(
               generic_material, factor));
}

JNI_METHOD_AOSP(void, nSetAmbientOcclusionTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong ambient_occlusion_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetAmbientOcclusionUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetAmbientOcclusionUvTransformOnGenericMaterial(
                   generic_material, uv_transform));
}

JNI_METHOD_AOSP(void, nSetAmbientOcclusionFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetAmbientOcclusionFactorOnGenericMaterial(generic_material,
                                                           factor));
}

JNI_METHOD_AOSP(void, nSetEmissiveTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong emissive_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetEmissiveUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetEmissiveUvTransformOnGenericMaterial(
          generic_material, uv_transform));
}

JNI_METHOD_AOSP(void, nSetEmissiveFactorsOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat x, jfloat y, jfloat z) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetEmissiveFactorsOnGenericMaterial(
          generic_material, {x, y, z}));
}

JNI_METHOD_AOSP(void, nSetClearcoatTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong clearcoat_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetClearcoatNormalTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong clearcoat_normal_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetClearcoatRoughnessTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong clearcoat_roughness_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetClearcoatFactorsOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat intensity, jfloat roughness, jfloat normal) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetClearcoatFactorsOnGenericMaterial(
          generic_material, {intensity, roughness, normal}));
}

JNI_METHOD_AOSP(void, nSetSheenColorTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong sheen_color_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetSheenColorFactorsOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat x, jfloat y, jfloat z) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetSheenColorFactorsOnGenericMaterial(
          generic_material, {x, y, z}));
}

JNI_METHOD_AOSP(void, nSetSheenRoughnessTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong sheen_roughness_texture, jint min_filter, jint mag_filter,
 jint wrap_mode_s, jint wrap_mode_t, jint wrap_mode_r, jint compare_mode,
 jint compare_func, jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetSheenRoughnessFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager()
          .SetSheenRoughnessFactorOnGenericMaterial(generic_material, factor));
}

JNI_METHOD_AOSP(void, nSetTransmissionTextureOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jlong transmission_texture, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
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

JNI_METHOD_AOSP(void, nSetTransmissionUvTransformOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12,
 jfloat m20, jfloat m21, jfloat m22) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  imp::mat3f uv_transform(m00, m01, m02, m10, m11, m12, m20, m21, m22);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager()
               .SetTransmissionUvTransformOnGenericMaterial(generic_material,
                                                            uv_transform));
}

JNI_METHOD_AOSP(void, nSetTransmissionFactorOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat factor) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetTransmissionFactorOnGenericMaterial(
          generic_material, factor));
}

JNI_METHOD_AOSP(void, nSetIndexOfRefractionOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat index_of_refraction) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env,
      view->GetGenericMaterialManager().SetIndexOfRefractionOnGenericMaterial(
          generic_material, index_of_refraction));
}

JNI_METHOD_AOSP(void, nSetAlphaCutoffOnGenericMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong generic_material,
 jfloat alpha_cutoff) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetGenericMaterialManager().SetAlphaCutoffOnGenericMaterial(
               generic_material, alpha_cutoff));
}

JNI_METHOD_AOSP(void, nDestroyNativeObject)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  view->DestroyNativeObject(handle);
}

JNI_METHOD_AOSP(void, nSetGltfModelNodeMaterialOverride)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id, jlong material,
 jint primitive_index) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().SetGltfModelNodeMaterialOverride(
               node_id, material, primitive_index));
}

JNI_METHOD_AOSP(void, nClearGltfModelNodeMaterialOverride)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jint primitive_index) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().ClearGltfModelNodeMaterialOverride(
               node_id, primitive_index));
}

JNI_METHOD_AOSP(void, nScheduleGltfReskinning)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetModelManager().ScheduleReskinning(impress_node));
}

JNI_METHOD_AOSP(void, nSetEnvironmentLight)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong ibl_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(
      env, view->GetSkyboxManager().SetEnvironmentLight(ibl_token));
}

JNI_METHOD_AOSP(void, nClearEnvironmentLight)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  view->GetSkyboxManager().ClearEnvironmentLight();
}

JNI_METHOD_AOSP(void, nDisposeAllResources)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  (void)imp::android::ThrowIfError(env, view->DisposeAllResources());
}

}  // extern "C"
