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

#include "absl/container/flat_hash_map.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "apibindings/asset_ptr_map.h"
#include "apibindings/bindings_material.h"
#include "apibindings/bindings_object.h"
#include "apibindings/bindings_texture.h"
#include "core/common/jni_helpers.h"
#include "core/common/owned_ptr.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "imp.h"

namespace imp {

class GenericMaterialManager;
class ModelManager;
class SkyboxManager;
class StereoSurfaceManager;
class TextureManager;
class WaterMaterialManager;

// Implements an Impress View for the ImpressJava API used by Jetpack XR.
// This view manages the core lifecycle and delegates specific functionalities
// to specialized manager classes.
class ImpressApiView : public View {
 public:
  ImpressApiView();
  virtual ~ImpressApiView();

  // Texture token value which unsets an alpha mask on a stereo surface.
  constexpr static int kUnSetAlphaMaskToken = -1;

  // Converts a native pointer to a Java handle.
  template <class T>
  inline jlong ToJava(T* p);

  // Converts a Java handle to a native pointer.
  template <class T>
  inline T* FromJava(jlong n);

  // Sets up the native side of the Impress API and initializes all managers.
  virtual void SetupImpressApiNative();

  // Creates an Impress node and returns a corresponding entity ID.
  int32_t CreateImpressNode();
  // Destroys an Impress node using its entity ID.
  absl::Status DestroyImpressNode(int32_t node);
  // Sets the parent of an Impress node using the entity IDs of the child and
  // parent nodes.
  absl::Status SetImpressNodeParent(int32_t child, int32_t parent);

  // Destroys a native Impress object (texture or material) using its handle.
  // This delegates the destruction to the appropriate manager.
  void DestroyNativeObject(std::intptr_t handle);

  // Disposes all resources associated with the Impress API view by delegating
  // to all managers.
  absl::Status DisposeAllResources();

  // Accessors for managers, allowing them to collaborate (e.g., for
  // materials to borrow textures).
  ModelManager& GetModelManager() { return *model_manager_; }
  SkyboxManager& GetSkyboxManager() { return *skybox_manager_; }
  StereoSurfaceManager& GetStereoSurfaceManager() {
    return *stereo_surface_manager_;
  }
  TextureManager& GetTextureManager() { return *texture_manager_; }
  WaterMaterialManager& GetWaterMaterialManager() {
    return *water_material_manager_;
  }
  GenericMaterialManager& GetGenericMaterialManager() {
    return *generic_material_manager_;
  }

  // Accessors for maps, allowing them to collaborate (e.g., for
  // textures to be shared across managers).
  AssetPtrMap& GetAssetPtrMap() { return *asset_ptr_map_; }

  // Returns the map of textures owned by the Impress API view which is used by
  // the TextureManager, GenericMaterialManager, and WaterMaterialManager.
  absl::flat_hash_map<std::intptr_t, OwnedTexturePtr>& GetBindingsTextureMap() {
    return bindings_texture_map_;
  }

  // Returns the map of materials owned by the Impress API view which is used by
  // the GenericMaterialManager and WaterMaterialManager.
  absl::flat_hash_map<std::intptr_t,
                      OwnedPtr<split_engine::SplitEngineMaterial>>&
  GetBindingsMaterialMap() {
    return bindings_material_map_;
  }

 protected:
  void Setup() override;
  void Update(const FrameTime& frame_time) override;

  std::unique_ptr<AssetPtrMap> asset_ptr_map_;
  std::unique_ptr<ModelManager> model_manager_;
  std::unique_ptr<SkyboxManager> skybox_manager_;
  std::unique_ptr<StereoSurfaceManager> stereo_surface_manager_;
  std::unique_ptr<TextureManager> texture_manager_;
  std::unique_ptr<WaterMaterialManager> water_material_manager_;
  std::unique_ptr<GenericMaterialManager> generic_material_manager_;
  absl::flat_hash_map<std::intptr_t, OwnedTexturePtr> bindings_texture_map_;
  absl::flat_hash_map<std::intptr_t,
                      OwnedPtr<split_engine::SplitEngineMaterial>>
      bindings_material_map_;

 private:
  void DestroyUnusedMaterials(bool shutdown);
  void DestroyUnusedTextures(bool shutdown);
};

template <class T>
inline jlong ImpressApiView::ToJava(T* p) {
  return JniAllowlist<T, BindingsTexture, BindingsMaterial>::ToJava(p);
}

template <class T>
inline T* ImpressApiView::FromJava(jlong n) {
  return JniAllowlist<T, BindingsTexture, BindingsObject,
                      BindingsMaterial>::FromJava(n);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_IMPRESS_API_VIEW_H_
