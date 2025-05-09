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
#include "apibindings/impress_api_view.h"
#include "apibindings/stereo_surface.h"
#include "core/common/buffer_access.h"
#include "core/common/jni_helpers.h"
#include "core/input/pointer_event_processor.h"
#include "core/media/media_type.h"

#define JNI_METHOD_ACTIVITY(return_type, method_name) \
  IMP_JNI return_type JNICALL                         \
      Java_com_google_ar_imp_apibindings_ImpressApiImpl_##method_name  // NOLINT

using ::imp::JniAllowlist;

namespace {

template <class T>
using ImpressApiImplAllowlist = JniAllowlist<T, imp::ImpressApiView>;

template <class T>
constexpr auto FromJava = &ImpressApiImplAllowlist<T>::FromJava;

template <typename T>
jobject ConvertStatusOrToKotlin(JNIEnv* env, const absl::StatusOr<T>& status_or,
                                const char* class_name,
                                const char* constructor_signature);

// LINT.IfChange(status_conversion)

// TODO: Refactor the status conversion to not call Java code from
// C++.
jobject ConvertErrorToKotlin(JNIEnv* env, const absl::Status& status) {
  std::string error_message(status.message().data(), status.message().size());
  jstring message = env->NewStringUTF(error_message.c_str());

  jclass specific_error_class;
  switch (status.code()) {
    case absl::StatusCode::kNotFound: {
      jclass not_found_class =
          env->FindClass("com/google/ar/imp/apibindings/Status$Error$NotFound");
      specific_error_class = not_found_class;
      break;
    }
    case absl::StatusCode::kInvalidArgument: {
      jclass invalid_argument_class = env->FindClass(
          "com/google/ar/imp/apibindings/Status$Error$InvalidArgument");
      specific_error_class = invalid_argument_class;
      break;
    }
    case absl::StatusCode::kInternal: {
      jclass internal_class =
          env->FindClass("com/google/ar/imp/apibindings/Status$Error$Internal");
      specific_error_class = internal_class;
      break;
    }
    default: {
      jclass error_class =
          env->FindClass("com/google/ar/imp/apibindings/Status$Error");
      specific_error_class = error_class;
      break;
    }
  }

  jmethodID constructor =
      env->GetMethodID(specific_error_class, "<init>", "(Ljava/lang/String;)V");
  jobject result = env->NewObject(specific_error_class, constructor, message);
  env->DeleteLocalRef(message);

  return result;
}

jobject ConvertStatusToKotlin(JNIEnv* env, const absl::Status& status) {
  if (status.ok()) {
    jclass success_class =
        env->FindClass("com/google/ar/imp/apibindings/Status$Success");
    jmethodID constructor = env->GetMethodID(success_class, "<init>", "()V");
    return env->NewObject(success_class, constructor);
  } else {
    return ConvertErrorToKotlin(env, status);
  }
}

jobject ConvertStatusOrIntToKotlin(JNIEnv* env,
                                   const absl::StatusOr<int>& status_or) {
  return ConvertStatusOrToKotlin(
      env, status_or,
      "com/google/ar/imp/apibindings/Status$SuccessWithIntValue", "(I)V");
}

jobject ConvertStatusOrLongToKotlin(
    JNIEnv* env, const absl::StatusOr<std::intptr_t>& status_or) {
  return ConvertStatusOrToKotlin(
      env, status_or,
      "com/google/ar/imp/apibindings/Status$SuccessWithLongValue", "(J)V");
}

template <typename T>
jobject ConvertStatusOrToKotlin(JNIEnv* env, const absl::StatusOr<T>& status_or,
                                const char* class_name,
                                const char* constructor_signature) {
  if (status_or.ok()) {
    jclass success_with_value_class = env->FindClass(class_name);
    jmethodID constructor = env->GetMethodID(success_with_value_class, "<init>",
                                             constructor_signature);

    jobject result =
        env->NewObject(success_with_value_class, constructor, *status_or);
    return result;
  } else {
    return ConvertErrorToKotlin(env, status_or.status());
  }
}

// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/apibindings/Status.kt:status_conversion)

}  // namespace

extern "C" {

// LINT.IfChange(api)

JNI_METHOD_ACTIVITY(void, nSetup)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  view->SetupImpressApiNative();
}

JNI_METHOD_ACTIVITY(jobject, nReleaseImageBasedLightingAsset)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong ibl_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  return ConvertStatusToKotlin(env,
                               view->ReleaseImageBasedLightingAsset(ibl_token));
}

JNI_METHOD_ACTIVITY(void, nLoadImageBasedLightingAssetFromPath)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->LoadImageBasedLightingAsset(imp::GetString(env, path),
                                    std::move(asset_loader));
}

JNI_METHOD_ACTIVITY(void, nLoadImageBasedLightingAssetFromByteArray)
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
  view->LoadImageBasedLightingAsset(data_cord, imp::GetString(env, key),
                                    std::move(asset_loader));
}

JNI_METHOD_ACTIVITY(void, nLoadGltfAssetFromPath)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jstring path) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->LoadGltfAsset(imp::GetString(env, path), std::move(asset_loader));
}

JNI_METHOD_ACTIVITY(void, nLoadGltfAssetFromByteArray)
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
  view->LoadGltfAsset(data_cord, imp::GetString(env, key),
                      std::move(asset_loader));
}

JNI_METHOD_ACTIVITY(jobject, nReleaseGltfAsset)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong gltf_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

  return ConvertStatusToKotlin(env, view->ReleaseGltfAsset(gltf_token));
}

JNI_METHOD_ACTIVITY(jobject, nInstanceGltfModel)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong gltf_token,
 jboolean enable_collider) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

  return ConvertStatusOrIntToKotlin(
      env, view->InstanceGltfModel(gltf_token, enable_collider));
}

JNI_METHOD_ACTIVITY(jobject, nSetGltfModelColliderEnabled)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong impress_node,
 jboolean enable_collider) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

  return ConvertStatusToKotlin(
      env, view->SetGltfModelColliderEnabled(impress_node, enable_collider));
}

JNI_METHOD_ACTIVITY(void, nAnimateGltfModel)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jstring animation_name, jboolean loop, jobject j_asset_animator) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_animator =
      std::make_unique<imp::AssetAnimator>(env, j_asset_animator);
  view->AnimateGltfModel(impress_node, imp::GetString(env, animation_name),
                         loop, std::move(asset_animator));
}

JNI_METHOD_ACTIVITY(jobject, nStopGltfModelAnimation)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  return ConvertStatusToKotlin(env, view->StopGltfModelAnimation(impress_node));
}

JNI_METHOD_ACTIVITY(jint, nCreateImpressNode)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

  return view->CreateImpressNode();
}

JNI_METHOD_ACTIVITY(jobject, nDestroyImpressNode)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

  return ConvertStatusToKotlin(env, view->DestroyImpressNode(impress_node));
}

JNI_METHOD_ACTIVITY(jobject, nSetImpressNodeParent)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node_child,
 jint impress_node_parent) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

  return ConvertStatusToKotlin(
      env, view->SetImpressNodeParent(impress_node_child, impress_node_parent));
}

// TODO: (broken link) - Update this to return Status
JNI_METHOD_ACTIVITY(jint, nCreateStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, int stereo_mode) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  return view->CreateStereoSurfaceEntity(
      static_cast<imp::MediaStereoMode>(stereo_mode));
}

// TODO: (broken link) - Update this to return Status
JNI_METHOD_ACTIVITY(void, nSetStereoSurfaceEntityCanvasShapeQuad)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id, jfloat width,
 jfloat height) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  view->SetStereoSurfaceEntityCanvasShape(
      node_id, imp::StereoSurface::Quad({width, height}));
}

// TODO: (broken link) - Update this to return Status
JNI_METHOD_ACTIVITY(void, nSetStereoSurfaceEntityCanvasShapeSphere)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jfloat radius) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  view->SetStereoSurfaceEntityCanvasShape(node_id,
                                          imp::StereoSurface::Sphere({radius}));
}

// TODO: (broken link) - Update this to return Status
JNI_METHOD_ACTIVITY(void, nSetStereoSurfaceEntityCanvasShapeHemisphere)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jfloat radius) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  view->SetStereoSurfaceEntityCanvasShape(
      node_id, imp::StereoSurface::Hemisphere({radius}));
}

JNI_METHOD_ACTIVITY(jobject, nGetSurfaceFromStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

  // Note that Impress' Android::Surface is a JNI wrapper around the Android
  // Surface class. This returns a (Java-managed) reference as a jobject.
  return view->GetSurfaceFromStereoSurfaceEntity(node_id)->WeakReference();
}

JNI_METHOD_ACTIVITY(void, nSetFeatherRadiusForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jfloat radius_x, jfloat radius_y) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  // TODO: add a check to make sure the node is a StereoSurface.
  view->SetFeatherRadiusForStereoSurfaceEntity(node_id, {radius_x, radius_y});
}

JNI_METHOD_ACTIVITY(void, nSetStereoModeForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jint stereo_mode) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);

  view->SetStereoModeForStereoSurfaceEntity(
      node_id, static_cast<imp::MediaStereoMode>(stereo_mode));
}

JNI_METHOD_ACTIVITY(void, nSetPrimaryAlphaMaskForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jlong alpha_mask_token, jobject j_asset_loader) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  view->SetPrimaryAlphaMaskForStereoSurfaceEntity(node_id, alpha_mask_token);
}

JNI_METHOD_ACTIVITY(void, nSetAuxiliaryAlphaMaskForStereoSurfaceEntity)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint node_id,
 jlong alpha_mask_token, jobject j_asset_loader) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  view->SetAuxiliaryAlphaMaskForStereoSurfaceEntity(node_id, alpha_mask_token);
}

JNI_METHOD_ACTIVITY(void, nLoadTexture)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jstring path, jint min_filter, jint mag_filter, jint wrap_mode_s,
 jint wrap_mode_t, jint wrap_mode_r, jint compare_mode, jint compare_func,
 jint anisotropyLog2) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);

  // Map the Java enum constants to the corresponding Filament enum values.
  filament::TextureSampler::MinFilter min_f =
      static_cast<filament::TextureSampler::MinFilter>(min_filter);
  filament::TextureSampler::MagFilter mag_f =
      static_cast<filament::TextureSampler::MagFilter>(mag_filter);
  filament::TextureSampler::WrapMode wrap_s =
      static_cast<filament::TextureSampler::WrapMode>(wrap_mode_s);
  filament::TextureSampler::WrapMode wrap_t =
      static_cast<filament::TextureSampler::WrapMode>(wrap_mode_t);
  filament::TextureSampler::WrapMode wrap_r =
      static_cast<filament::TextureSampler::WrapMode>(wrap_mode_r);
  filament::TextureSampler::CompareMode compare_m =
      static_cast<filament::TextureSampler::CompareMode>(compare_mode);
  filament::TextureSampler::CompareFunc compare_f =
      static_cast<filament::TextureSampler::CompareFunc>(compare_func);

  filament::TextureSampler native_sampler(min_f, mag_f);
  native_sampler.setWrapModeS(wrap_s);
  native_sampler.setWrapModeT(wrap_t);
  native_sampler.setWrapModeR(wrap_r);
  native_sampler.setCompareMode(compare_m, compare_f);
  native_sampler.setAnisotropy(1 << anisotropyLog2);

  view->LoadTexture(imp::GetString(env, path), std::move(native_sampler),
                    std::move(asset_loader));
}

JNI_METHOD_ACTIVITY(jobject, nBorrowReflectionTexture)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  return ConvertStatusOrLongToKotlin(env, view->BorrowReflectionTexture());
}

JNI_METHOD_ACTIVITY(jobject, nGetReflectionTextureFromIbl)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong ibl_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  return ConvertStatusOrLongToKotlin(
      env, view->GetReflectionTextureFromIbl(ibl_token));
}

JNI_METHOD_ACTIVITY(void, nCreateWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jobject j_asset_loader,
 jboolean is_alpha_map_version) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  auto asset_loader = std::make_unique<imp::AssetLoader>(env, j_asset_loader);
  view->CreateWaterMaterial(std::move(asset_loader), is_alpha_map_version);
}

JNI_METHOD_ACTIVITY(jobject, nSetReflectionCubeOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jlong reflection_cube) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  // TODO: Throw Java exceptions from C++ to avoid returning a
  // status when setting material parameters.
  return ConvertStatusToKotlin(env, view->SetReflectionCubeOnWaterMaterial(
                                        water_material, reflection_cube));
}

JNI_METHOD_ACTIVITY(jobject, nSetNormalMapOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jlong normal_map) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  // TODO: Throw Java exceptions from C++ to avoid returning a
  // status when setting material parameters.
  return ConvertStatusToKotlin(
      env, view->SetNormalMapOnWaterMaterial(water_material, normal_map));
}

JNI_METHOD_ACTIVITY(jobject, nSetNormalTilingOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_tiling) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  // TODO: Throw Java exceptions from C++ to avoid returning a
  // status when setting material parameters.
  return ConvertStatusToKotlin(
      env, view->SetNormalTilingOnWaterMaterial(water_material, normal_tiling));
}

JNI_METHOD_ACTIVITY(jobject, nSetNormalSpeedOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_speed) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  // TODO: Throw Java exceptions from C++ to avoid returning a
  // status when setting material parameters.
  return ConvertStatusToKotlin(
      env, view->SetNormalSpeedOnWaterMaterial(water_material, normal_speed));
}

JNI_METHOD_ACTIVITY(jobject, nSetAlphaStepMultiplierOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat alpha_step_multiplier) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  // TODO: Throw Java exceptions from C++ to avoid returning a
  // status when setting material parameters.
  return ConvertStatusToKotlin(env, view->SetAlphaStepMultiplierOnWaterMaterial(
                                        water_material, alpha_step_multiplier));
}

JNI_METHOD_ACTIVITY(jobject, nSetAlphaMapOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jlong alpha_map) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  // TODO: Throw Java exceptions from C++ to avoid returning a
  // status when setting material parameters.
  return ConvertStatusToKotlin(
      env, view->SetAlphaMapOnWaterMaterial(water_material, alpha_map));
}

JNI_METHOD_ACTIVITY(jobject, nSetNormalZOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_z) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  // TODO: Throw Java exceptions from C++ to avoid returning a
  // status when setting material parameters.
  return ConvertStatusToKotlin(
      env, view->SetNormalZOnWaterMaterial(water_material, normal_z));
}

JNI_METHOD_ACTIVITY(jobject, nSetNormalBoundaryOnWaterMaterial)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong water_material,
 jfloat normal_boundary) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  // TODO: Throw Java exceptions from C++ to avoid returning a
  // status when setting material parameters.
  return ConvertStatusToKotlin(env, view->SetNormalBoundaryOnWaterMaterial(
                                        water_material, normal_boundary));
}

JNI_METHOD_ACTIVITY(void, nDestroyNativeObject)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  view->DestroyNativeObject(handle);
}

JNI_METHOD_ACTIVITY(jobject, nSetMaterialOverride)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jint impress_node,
 jlong material, jstring mesh_name) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  return ConvertStatusToKotlin(
      env, view->SetMaterialOverride(impress_node, material,
                                     imp::GetString(env, mesh_name)));
}

JNI_METHOD_ACTIVITY(jobject, nSetEnvironmentLight)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle, jlong ibl_token) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  return ConvertStatusToKotlin(env, view->SetEnvironmentLight(ibl_token));
}

JNI_METHOD_ACTIVITY(jobject, nClearEnvironmentLight)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  return ConvertStatusToKotlin(env, view->ClearEnvironmentLight());
}

JNI_METHOD_ACTIVITY(jobject, nDisposeAllResources)
(JNIEnv* env, jclass /*clazz*/, jlong view_handle) {
  auto view = FromJava<imp::ImpressApiView>(view_handle);
  return ConvertStatusToKotlin(env, view->DisposeAllResources());
}

// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/apibindings/ImpressApiImpl.java:api)

}  // extern "C"
