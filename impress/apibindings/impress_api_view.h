/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_IMPRESS_API_VIEW_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_IMPRESS_API_VIEW_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <tuple>

#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "apibindings/asset_animator.h"
#include "apibindings/asset_loader.h"
#include "apibindings/asset_ptr_map.h"
#include "apibindings/bindings_material.h"
#include "apibindings/bindings_object.h"
#include "apibindings/bindings_texture.h"
#include "apibindings/stereo_surface.h"
#include "core/common/jni_helpers.h"
#include "core/media/media_type.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "imp.h"
#include "split_engine/materials/water_reflection_material.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

// Implements an Impress View for the ImpressJava API used by Jetpack XR.
class ImpressApiView : public View {
 public:
  // Converts a native pointer to a Java handle.
  template <class T>
  inline jlong ToJava(T* p);

  // Converts a Java handle to a native pointer.
  template <class T>
  inline T* FromJava(jlong n);

  // Sets a value parameter on the water reflection Split Engine material.
  template <typename ParameterValueT, typename SetterFn>
  absl::Status SetWaterMaterialValueParameter(std::intptr_t water_material,
                                              ParameterValueT value,
                                              SetterFn setter_fn);

  // Sets a texture parameter on the water reflection Split Engine material.
  template <typename SetterFn>
  absl::Status SetWaterMaterialTextureParameter(std::intptr_t water_material,
                                                std::intptr_t texture,
                                                SetterFn setter_fn);

  // Sets up the native side of the Impress API
  void SetupImpressApiNative();

  // Loads the asset pointer of an IBL asset from the local assets folder or
  // a remote URL, and returns a unique identifier for it when it is ready.
  void LoadImageBasedLightingAsset(absl::string_view path,
                                   std::unique_ptr<AssetLoader> asset_loader);
  // Loads the asset pointer of an IBL asset from a byte array, and returns a
  // unique identifier for it when it is ready.
  void LoadImageBasedLightingAsset(absl::Cord data, absl::string_view key,
                                   std::unique_ptr<AssetLoader> asset_loader);
  // Releases the asset pointer of a previously loaded image based lighting
  // asset.
  absl::Status ReleaseImageBasedLightingAsset(std::intptr_t ibl_token);
  // Loads the asset pointer of a glTF model from the local assets folder or
  // a remote URL, and resolves the asset loader when it is ready.
  void LoadGltfAsset(absl::string_view path,
                     std::unique_ptr<AssetLoader> asset_loader);
  // Loads the asset pointer of a glTF model from a absl::Cord, and resolves the
  // AssetLoader when it is ready. The data will be managed by the Impress
  // resource system and will be destroyed when the AssetPtr associated with the
  // data is destroyed.
  void LoadGltfAsset(absl::Cord data, absl::string_view key,
                     std::unique_ptr<AssetLoader> asset_loader);
  // Releases the asset pointer of previously loaded glTF asset.
  absl::Status ReleaseGltfAsset(std::intptr_t gltf_token);
  // Instantiates a glTF model from a previously loaded model and returns an
  // entity ID corresponding to the Impress node associated with the model.
  absl::StatusOr<int32_t> InstanceGltfModel(std::intptr_t gltf_token,
                                            bool enable_collider);
  // Attaches or detaches a collider based on enable_collider. (If the mesh
  // doesn't have a collider, one will be created and attached if
  // enable_collider is true)
  absl::Status SetGltfModelColliderEnabled(int32_t node, bool enable_collider);
  // Animates a glTF model and notifies the caller when the animation is
  // complete or fails.
  void AnimateGltfModel(int32_t node, absl::string_view animation_name,
                        bool loop,
                        std::unique_ptr<AssetAnimator> asset_animator);
  // Stops the animation of a glTF model or fails if the model is not animating
  // or does not exist.
  absl::Status StopGltfModelAnimation(int32_t node);
  // Creates an Impress node and returns a corresponding entity ID.
  int32_t CreateImpressNode();
  // Destroys an Impress node using its entity ID.
  absl::Status DestroyImpressNode(int32_t node);
  // Sets the parent of an Impress node using the entity IDs of the child and
  // parent nodes.
  absl::Status SetImpressNodeParent(int32_t child, int32_t parent);

  // Creates a new Impress node and attaches a StereoSurface component to it.
  // Returns the entity ID of the new node. A StereoSurfaceEntity can be
  // destroyed by calling DestroyImpressNode with the return value from this
  // method.
  int32_t CreateStereoSurfaceEntity(imp::MediaStereoMode stereo_mode);

  // Sets the canvas shape of a stereo surface using its entity ID.
  void SetStereoSurfaceEntityCanvasShape(
      int32_t node_id, imp::StereoSurface::CanvasShape canvas_shape);

  // Returns the surface associated with a stereo surface entity.
  android::Surface* GetSurfaceFromStereoSurfaceEntity(int32_t node_id);

  // Sets the Left/Right and Top/Bottom feather radius of a surface entity.
  void SetFeatherRadiusForStereoSurfaceEntity(
      int32_t node_id, const imp::float2& feather_radius);

  // Sets the stereo mode of a stereo surface entity.
  void SetStereoModeForStereoSurfaceEntity(int32_t node_id,
                                           imp::MediaStereoMode stereo_mode);
  // Sets an alpha mask on an stereo surface entity using an image asset.
  void SetPrimaryAlphaMaskForStereoSurfaceEntity(int32_t node_id,
                                                 int64_t alpha_mask_token);
  // Sets an alpha mask on an stereo surface entity using an image asset.
  void SetAuxiliaryAlphaMaskForStereoSurfaceEntity(int32_t node_id,
                                                   int64_t alpha_mask_token);
  // Loads a texture from the assets folder or a remote texture from a URL.
  void LoadTexture(absl::string_view path, filament::TextureSampler sampler,
                   std::unique_ptr<AssetLoader> asset_loader);

  // Borrows the reflection texture from the currently set environment IBL.
  absl::StatusOr<std::intptr_t> BorrowReflectionTexture();

  // Borrows the reflection texture from the given environment IBL.
  absl::StatusOr<std::intptr_t> GetReflectionTextureFromIbl(
      std::intptr_t ibl_token);

  // Creates a new regular or alpha map version of the water material, and
  // resolves the asset loader when it is ready.
  void CreateWaterMaterial(std::unique_ptr<AssetLoader> asset_loader,
                           bool is_alpha_map_version);

  // Destroys the native Impress object using its native handle.
  void DestroyNativeObject(std::intptr_t handle);

  // TODO: Refactor API bindings layer to be more modular so that
  // water material specific methods are not added to the ImpressApiView.
  // Sets the reflection cube for the water material.
  absl::Status SetReflectionCubeOnWaterMaterial(std::intptr_t water_material,
                                                std::intptr_t reflection_cube);

  // Sets the normal map for the water material.
  absl::Status SetNormalMapOnWaterMaterial(std::intptr_t water_material,
                                           std::intptr_t normal_map);

  // Sets the normal tiling for the water material.
  absl::Status SetNormalTilingOnWaterMaterial(std::intptr_t water_material,
                                              float normal_tiling);

  // Sets the normal speed for the water material.
  absl::Status SetNormalSpeedOnWaterMaterial(std::intptr_t water_material,
                                             float normal_speed);

  // Sets the alpha step multiplier for the water material.
  absl::Status SetAlphaStepMultiplierOnWaterMaterial(
      std::intptr_t water_material, float alpha_step_multiplier);

  // Sets the alpha map for the water material.
  absl::Status SetAlphaMapOnWaterMaterial(std::intptr_t water_material,
                                          std::intptr_t alpha_map);

  // Sets the normal z for the water material.
  absl::Status SetNormalZOnWaterMaterial(std::intptr_t water_material,
                                         float normal_z);

  // Sets the normal boundary for the water material.
  absl::Status SetNormalBoundaryOnWaterMaterial(std::intptr_t water_material,
                                                float normal_boundary);

  // Sets the material override for the mesh of a glTF model.
  absl::Status SetMaterialOverride(int32_t node_id, std::intptr_t material,
                                   absl::string_view mesh_name);

  // Sets the preferred IBL asset to be used by the system.
  absl::Status SetEnvironmentLight(std::intptr_t ibl_token);

  // Clears the preferred IBL asset to be used by the system.
  absl::Status ClearEnvironmentLight();

  // Disposes all resources associated with the Impress API view.
  absl::Status DisposeAllResources();

  std::unique_ptr<AssetPtrMap> asset_ptr_map_;

 protected:
  void Setup() override;
  void Update(const FrameTime& frame_time) override;

 private:
  // TODO: (broken link) - Refactor this per-node caching into a custom
  //                     AnimatorController component.
  absl::flat_hash_map<int32_t,
                      std::tuple<ComponentHandle<GltfAnimator>,
                                 std::optional<std::unique_ptr<AssetAnimator>>>>
      node_to_anim_ctx_;
  // We keep track of the bindings materials and textures separately so we can
  // destroy them in the right order (materials before textures).
  absl::flat_hash_set<std::intptr_t> bindings_material_set_;
  absl::flat_hash_set<std::intptr_t> bindings_texture_set_;
};

template <class T>
inline jlong ImpressApiView::ToJava(T* p) {
  return JniAllowlist<T, BindingsTexture, BindingsMaterial>::ToJava(p);
}

template <class T>
inline T* ImpressApiView::FromJava(jlong n) {
  return JniAllowlist<T, BindingsTexture, BindingsObject, BindingsMaterial,
                      android_xr::WaterReflectionMaterial>::FromJava(n);
};

template <typename ParameterValueT, typename SetterFn>
absl::Status ImpressApiView::SetWaterMaterialValueParameter(
    std::intptr_t water_material, ParameterValueT value, SetterFn setter_fn) {
  BindingsMaterial* bindings_material =
      FromJava<BindingsMaterial>(water_material);
  if (!bindings_material) {
    return absl::InvalidArgumentError("Provided material handle is not valid.");
  }

  MP_ASSIGN_OR_RETURN(
      android_xr::WaterReflectionMaterial * water_mat,
      bindings_material->GetMaterial<android_xr::WaterReflectionMaterial>());

  setter_fn(water_mat, value);
  return absl::OkStatus();
}

template <typename SetterFn>
absl::Status ImpressApiView::SetWaterMaterialTextureParameter(
    std::intptr_t water_material, std::intptr_t texture, SetterFn setter_fn) {
  BindingsMaterial* bindings_material =
      FromJava<BindingsMaterial>(water_material);
  if (!bindings_material) {
    return absl::InvalidArgumentError("Provided material handle is not valid.");
  }

  MP_ASSIGN_OR_RETURN(
      android_xr::WaterReflectionMaterial * water_mat,
      bindings_material->GetMaterial<android_xr::WaterReflectionMaterial>());

  BindingsTexture* bindings_texture = FromJava<BindingsTexture>(texture);
  if (!bindings_texture) {
    return absl::InvalidArgumentError("Provided texture handle is not valid.");
  }

  BorrowedTexturePtr borrowed_texture = bindings_texture->GetTexture();
  if (!borrowed_texture) {
    return absl::InvalidArgumentError(
        "Texture associated with handle is not valid.");
  }
  setter_fn(water_mat, borrowed_texture);
  return absl::OkStatus();
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_IMPRESS_API_VIEW_H_
