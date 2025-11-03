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

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
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
#include "core/common/hash.h"
#include "core/common/jni_helpers.h"
#include "core/common/owned_ptr.h"
#include "core/common/type_traits.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/generic_material_spec.h"
#include "core/media/media_color_space.h"
#include "core/media/media_type.h"
#include "core/render/content_security_level.h"
#include "core/split_engine/materials/split_engine_generic_material.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/view/platforms/android/wrappers/surface.h"
#include "imp.h"
#include "split_engine/materials/water_reflection_material.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

// Implements an Impress View for the ImpressJava API used by Jetpack XR.
class ImpressApiView : public View {
 public:
  // Texture token value which unsets an alpha mask on a stereo surface.
  constexpr static int kUnSetAlphaMaskToken = -1;

  // Converts a native pointer to a Java handle.
  template <class T>
  inline jlong ToJava(T* p);

  // Converts a Java handle to a native pointer.
  template <class T>
  inline T* FromJava(jlong n);

  // Sets a texture parameter on the water reflection Split Engine material.
  template <typename SetterFn>
  absl::Status SetWaterMaterialTextureParameter(std::intptr_t water_material,
                                                std::intptr_t texture,
                                                SetterFn setter_fn);

  // Sets a texture parameter on the generic Split Engine material.
  template <typename SetterFn>
  absl::Status SetGenericMaterialTextureParameter(
      std::intptr_t generic_material, std::intptr_t texture,
      std::optional<filament::TextureSampler> sampler, SetterFn setter_fn);

  // Returns the Split Engine material subtype from a bindings material handle.
  template <typename SplitEngineMaterialT>
  absl::StatusOr<SplitEngineMaterialT*> GetMaterialFromBindingsMaterial(
      std::intptr_t material_handle);

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
  // Returns the local space unscaled bounds of the glTF model's axis aligned
  // bounding box.
  absl::StatusOr<imp::Box> GetGltfModelLocalBounds(int32_t node);
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
  absl::StatusOr<int32_t> CreateStereoSurfaceEntity(
      MediaStereoMode stereo_mode, ContentSecurityLevel content_security_level,
      bool use_super_sampling);

  // Sets the canvas shape of a stereo surface using its entity ID.
  absl::Status SetStereoSurfaceEntityCanvasShape(
      int32_t node_id, StereoSurface::CanvasShape canvas_shape);

  // Attaches or detaches a collider based on enable_collider and the canvas
  // shape of the stereo surface.
  absl::Status SetStereoSurfaceEntityColliderEnabled(int32_t node_id,
                                                     bool enable_collider);

  // Returns the surface associated with a stereo surface entity.
  absl::StatusOr<android::Surface*> GetSurfaceFromStereoSurfaceEntity(
      int32_t node_id);

  // Updates the Surface Dimensions of a stereo surface entity - This is needed
  // to support android.graphics.Canvas methods on that Surface.
  absl::Status SetSurfaceDimensionsForStereoSurfaceEntity(int32_t node_id,
                                                          int32_t width,
                                                          int32_t height);

  // Sets the Left/Right and Top/Bottom feather radius of a surface entity.
  absl::Status SetFeatherRadiusForStereoSurfaceEntity(
      int32_t node_id, const float2& feather_radius);

  // Sets the stereo mode of a stereo surface entity.
  absl::Status SetStereoModeForStereoSurfaceEntity(int32_t node_id,
                                                   MediaStereoMode stereo_mode);

  // Sets an alpha mask on an stereo surface entity using an image asset.
  absl::Status SetPrimaryAlphaMaskForStereoSurfaceEntity(
      int32_t node_id, int64_t alpha_mask_token);

  // Sets an alpha mask on an stereo surface entity using an image asset.
  absl::Status SetAuxiliaryAlphaMaskForStereoSurfaceEntity(
      int32_t node_id, int64_t alpha_mask_token);

  // Configures the color space metadata for content rendered on the stereo
  // surface. When set to an unknown color space, the system will attempt a
  // best-effort color conversion. If specific color space parameters are
  // provided, these will be used to explicitly define the source color space
  // for backend color conversion.
  absl::Status SetContentColorMetadataForStereoSurfaceEntity(
      int32_t node_id, MediaColorSpace color_space = {});

  // Loads a texture from the assets folder or a remote texture from a URL.
  void LoadTexture(absl::string_view path,
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
  // Sets the reflection map for the water material.
  absl::Status SetReflectionMapOnWaterMaterial(
      std::intptr_t water_material, std::intptr_t reflection_map,
      std::optional<filament::TextureSampler> sampler);

  // Sets the normal map for the water material.
  absl::Status SetNormalMapOnWaterMaterial(
      std::intptr_t water_material, std::intptr_t normal_map,
      std::optional<filament::TextureSampler> sampler);

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
  absl::Status SetAlphaMapOnWaterMaterial(
      std::intptr_t water_material, std::intptr_t alpha_map,
      std::optional<filament::TextureSampler> sampler);

  // Sets the normal z for the water material.
  absl::Status SetNormalZOnWaterMaterial(std::intptr_t water_material,
                                         float normal_z);

  // Sets the normal boundary for the water material.
  absl::Status SetNormalBoundaryOnWaterMaterial(std::intptr_t water_material,
                                                float normal_boundary);

  // Creates a new generic material using a given spec, and resolves the asset
  // loader when it is ready.
  void CreateGenericMaterial(std::unique_ptr<AssetLoader> asset_loader,
                             imp::GenericMaterialSpec generic_material_spec);

  // Sets the base color texture for the generic material. This texture defines
  // the albedo or diffuse color of the material.
  absl::Status SetBaseColorTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t base_color_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the UV transformation matrix for the base color texture. This allows
  // for scaling, rotating, and translating the texture coordinates.
  absl::Status SetBaseColorUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform);

  // Sets the base color factors for the generic material. These factors
  // multiply the base color texture or define a uniform base color.
  absl::Status SetBaseColorFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float4& factors);

  // Sets the metallic-roughness texture for the generic material. This texture
  // defines the metallic and roughness properties of the material.
  absl::Status SetMetallicRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t metallic_roughness_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the UV transformation matrix for the metallic-roughness texture.
  // Controls how the metallic-roughness texture is mapped onto the surface.
  absl::Status SetMetallicRoughnessUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform);

  // Sets the metallic factor for the generic material. Controls the metalness
  // of the material, ranging from non-metal to metal.
  absl::Status SetMetallicFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor);

  // Sets the roughness factor for the generic material. Controls the surface
  // roughness, affecting the sharpness of reflections.
  absl::Status SetRoughnessFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor);

  // Sets the normal map texture for the generic material. This texture perturbs
  // the surface normals, creating detailed surface features.
  absl::Status SetNormalTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t normal_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the UV transformation matrix for the normal map texture. Adjusts the
  // mapping of the normal map texture.
  absl::Status SetNormalUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform);

  // Sets the factor of the normal map effect. Controls the strength of the
  // normal map's influence.
  absl::Status SetNormalFactorOnGenericMaterial(std::intptr_t generic_material,
                                                float factor);

  // Sets the ambient occlusion texture for the generic material. Simulates the
  // occlusion of ambient light by surface details.
  absl::Status SetAmbientOcclusionTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t ambient_occlusion_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the UV transformation matrix for the ambient occlusion texture.
  // Controls the mapping of the ambient occlusion texture.
  absl::Status SetAmbientOcclusionUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform);

  // Sets the factor of the ambient occlusion effect.
  absl::Status SetAmbientOcclusionFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor);

  // Sets the emissive texture for the generic material. Defines the light
  // emitted by the material.
  absl::Status SetEmissiveTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t emissive_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the UV transformation matrix for the emissive texture.
  absl::Status SetEmissiveUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform);

  // Sets the emissive color factors for the generic material. Multiplies the
  // emissive texture or defines a uniform emissive color.
  absl::Status SetEmissiveFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factors);

  // Sets the clearcoat texture for the generic material. Adds a clearcoat layer
  // to the material, affecting reflections.
  absl::Status SetClearcoatTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the clearcoat normal texture for the generic material. Perturbs the
  // normals of the clearcoat layer.
  absl::Status SetClearcoatNormalTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_normal_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the clearcoat roughness texture for the generic material. Controls the
  // roughness of the clearcoat layer.
  absl::Status SetClearcoatRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t clearcoat_roughness_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the clearcoat factors for the generic material. Multiplies the
  // clearcoat texture or defines a uniform clearcoat color:
  absl::Status SetClearcoatFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factor);

  // Sets the sheen color texture for the generic material. Defines the color of
  // the sheen effect, visible at grazing angles.
  absl::Status SetSheenColorTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t sheen_color_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the sheen color factors for the generic material. Multiplies the sheen
  // color texture or defines a uniform sheen color.
  absl::Status SetSheenColorFactorsOnGenericMaterial(
      std::intptr_t generic_material, const float3& factors);

  // Sets the sheen roughness texture for the generic material. Controls the
  // roughness of the sheen effect.
  absl::Status SetSheenRoughnessTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t sheen_roughness_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the sheen roughness factor for the generic material. Controls the
  // roughness of the sheen effect.
  absl::Status SetSheenRoughnessFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor);

  // Sets the transmission texture for the generic material. Defines the
  // transmission of light through the material.
  absl::Status SetTransmissionTextureOnGenericMaterial(
      std::intptr_t generic_material, std::intptr_t transmission_texture,
      std::optional<filament::TextureSampler> sampler);

  // Sets the UV transformation matrix for the transmission texture.
  absl::Status SetTransmissionUvTransformOnGenericMaterial(
      std::intptr_t generic_material, const mat3f& uv_transform);

  // Sets the transmission factor for the generic material. Controls the amount
  // of light transmitted through the material.
  absl::Status SetTransmissionFactorOnGenericMaterial(
      std::intptr_t generic_material, float factor);

  // Sets the index of refraction for the generic material. Defines how much
  // light bends when entering the material.
  absl::Status SetIndexOfRefractionOnGenericMaterial(
      std::intptr_t generic_material, float index_of_refraction);

  // Sets the alpha cutoff for the generic material. Defines the threshold for
  // transparency, used for cutout effects.
  absl::Status SetAlphaCutoffOnGenericMaterial(std::intptr_t generic_material,
                                               float alpha_cutoff);

  // Sets the material override for a node's mesh at a given primitive index.
  absl::Status SetMaterialOverride(int32_t node_id, std::intptr_t material,
                                   absl::string_view node_name,
                                   size_t primitive_index);

  // Clears the material override for a node's mesh at a given primitive index.
  absl::Status ClearMaterialOverride(int32_t node_id,
                                     absl::string_view node_name,
                                     size_t primitive_index);

  // Sets the preferred IBL asset to be used by the system.
  absl::Status SetEnvironmentLight(std::intptr_t ibl_token);

  // Clears the preferred IBL asset to be used by the system.
  absl::Status ClearEnvironmentLight();

  // Disposes all resources associated with the Impress API view.
  absl::Status DisposeAllResources();

  // Returns the asset pointer map.
  AssetPtrMap& GetAssetPtrMap() { return *asset_ptr_map_; }

  // Returns the bindings texture map.
  absl::flat_hash_map<std::intptr_t, OwnedTexturePtr>& GetBindingsTextureMap() {
    return bindings_texture_map_;
  }

 protected:
  void Setup() override;
  void Update(const FrameTime& frame_time) override;

 private:
  std::unique_ptr<AssetPtrMap> asset_ptr_map_;
  // TODO: (broken link) - Refactor this per-node caching into a custom
  //                     AnimatorController component.
  absl::flat_hash_map<int32_t,
                      std::tuple<ComponentHandle<GltfAnimator>,
                                 std::optional<std::unique_ptr<AssetAnimator>>>>
      node_to_anim_ctx_;
  absl::flat_hash_map<std::intptr_t, OwnedTexturePtr> bindings_texture_map_;
  absl::flat_hash_map<std::intptr_t,
                      OwnedPtr<split_engine::SplitEngineMaterial>>
      bindings_material_map_;

  absl::StatusOr<BorrowedTexturePtr> BorrowTexture(
      std::intptr_t texture_handle);

  absl::StatusOr<ComponentHandle<GltfMesh>> FindGltfMeshByNodeName(
      int32_t node_id, absl::string_view node_name);

  void DestroyUnusedMaterials(bool shutdown);
  void DestroyUnusedTextures(bool shutdown);
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

template <typename SetterFn>
absl::Status ImpressApiView::SetWaterMaterialTextureParameter(
    std::intptr_t water_material, std::intptr_t texture, SetterFn setter_fn) {
  MP_ASSIGN_OR_RETURN(
      android_xr::WaterReflectionMaterial * water_material_ptr,
      GetMaterialFromBindingsMaterial<android_xr::WaterReflectionMaterial>(
          water_material));
  MP_ASSIGN_OR_RETURN(BorrowedTexturePtr borrowed_texture, BorrowTexture(texture));

  // (void) is required in case setter_fn is [[nodiscard]], which is flagged as
  // an error in Bazel builds.
  (void)setter_fn(water_material_ptr, borrowed_texture);
  return absl::OkStatus();
}

template <typename SetterFn>
absl::Status ImpressApiView::SetGenericMaterialTextureParameter(
    std::intptr_t generic_material, std::intptr_t texture,
    std::optional<filament::TextureSampler> sampler, SetterFn setter_fn) {
  MP_ASSIGN_OR_RETURN(
      split_engine::SplitEngineGenericMaterial * generic_material_ptr,
      GetMaterialFromBindingsMaterial<split_engine::SplitEngineGenericMaterial>(
          generic_material));
  MP_ASSIGN_OR_RETURN(BorrowedTexturePtr borrowed_texture, BorrowTexture(texture));

  imp::GenericMaterialTextureParameter texture_parameter_payload;
  // Since we only set one texture at a time, we can assume the texture id is
  // always 1.
  uint64_t texture_id = 1;
  texture_parameter_payload.texture_id = texture_id;
  if (sampler.has_value()) {
    texture_parameter_payload.sampler = *sampler;
  }
  // (void) is required in case setter_fn is [[nodiscard]], which is flagged as
  // an error in Bazel builds.
  (void)setter_fn(
      generic_material_ptr, texture_parameter_payload,
      imp::TextureBorrower([expected_texture_id = texture_id,
                            borrowed_texture = std::move(borrowed_texture)](
                               uint64_t id) -> BorrowedTexturePtr {
        // The only call to this function should be with the one ID we
        // have.
        
        return borrowed_texture.WithNewLocation();
      }));
  return absl::OkStatus();
}

template <typename SplitEngineMaterialT>
absl::StatusOr<SplitEngineMaterialT*>
ImpressApiView::GetMaterialFromBindingsMaterial(std::intptr_t material_handle) {
  BindingsMaterial* bindings_material =
      FromJava<BindingsMaterial>(material_handle);
  if (!bindings_material) {
    return absl::InvalidArgumentError("Provided material handle is not valid.");
  }

  HashValue material_type_hash = bindings_material->GetTypeHash();
  HashValue expected_type_hash = type_traits::kTypeHash<SplitEngineMaterialT>;
  if (material_type_hash != expected_type_hash) {
    return absl::InvalidArgumentError(
        "Provided material handle is not of the correct type.");
  }

  split_engine::SplitEngineMaterial* base_material =
      &(*bindings_material_map_.at(material_handle));
  SplitEngineMaterialT* derived_material =
      static_cast<SplitEngineMaterialT*>(base_material);
  if (!derived_material) {
    return absl::InvalidArgumentError(
        "Material type hash matched, but static_cast failed.");
  }

  return derived_material;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_IMPRESS_API_VIEW_H_
