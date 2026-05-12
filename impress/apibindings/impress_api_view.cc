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

#include "apibindings/impress_api_view.h"

#include <cstdint>
#include <memory>
#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/View.h"
#include "apibindings/asset_ptr_map.h"
#include "apibindings/bindings_object.h"
#include "apibindings/generic_material_manager.h"
#include "apibindings/model_manager.h"
#include "apibindings/node_manager.h"
#include "apibindings/skybox_manager.h"
#include "apibindings/stereo_surface_manager.h"
#include "apibindings/texture_manager.h"
#include "apibindings/water_material_manager.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "core/view/utils/frame_time.h"

namespace imp {

// Forward declare for manager implementations.
std::unique_ptr<ModelManager> CreateModelManager(ImpressApiView& view);
std::unique_ptr<SkyboxManager> CreateSkyboxManager(ImpressApiView& view);
std::unique_ptr<StereoSurfaceManager> CreateStereoSurfaceManager(
    ImpressApiView& view);
std::unique_ptr<TextureManager> CreateTextureManager(ImpressApiView& view);
std::unique_ptr<WaterMaterialManager> CreateWaterMaterialManager(
    ImpressApiView& view);
std::unique_ptr<GenericMaterialManager> CreateGenericMaterialManager(
    ImpressApiView& view);
std::unique_ptr<NodeManager> CreateNodeManager(ImpressApiView& view);

ImpressApiView::ImpressApiView() = default;
ImpressApiView::~ImpressApiView() = default;

void ImpressApiView::SetupImpressApiNative() {
  asset_ptr_map_ = std::make_unique<AssetPtrMap>(*this);
  model_manager_ = CreateModelManager(*this);
  skybox_manager_ = CreateSkyboxManager(*this);
  stereo_surface_manager_ = CreateStereoSurfaceManager(*this);
  texture_manager_ = CreateTextureManager(*this);
  water_material_manager_ = CreateWaterMaterialManager(*this);
  generic_material_manager_ = CreateGenericMaterialManager(*this);
  node_manager_ = CreateNodeManager(*this);
}

void ImpressApiView::DestroyNativeObject(std::intptr_t handle) {
  // Creating a unique_ptr and letting it go out of scope will destroy the
  // bindings object, which will free the memory it was holding.
  std::unique_ptr<BindingsObject> bindings_object(
      FromJava<BindingsObject>(handle));
}

void ImpressApiView::DestroyUnusedMaterials(bool shutdown) {
  for (auto it = bindings_material_map_.begin();
       it != bindings_material_map_.end();) {
    if (shutdown) {
      // We also destroy the corresponding BindingsMaterial object when shutting
      // down the view.
      DestroyNativeObject(it->first);
    }

    // Looking for when the material is unused. We check for one instead of zero
    // because the temporary BorrowedMaterialPtr returned by GetMaterial
    // contributes one to the count.
    if (it->second->GetMaterial().GetBorrowedCount() == 1) {
      // Erasing the map entry also destroys the OwnedPtr of the material.
      bindings_material_map_.erase(it++);
    } else {
      ++it;
    }
  }
}

void ImpressApiView::DestroyUnusedTextures(bool shutdown) {
  for (auto it = bindings_texture_map_.begin();
       it != bindings_texture_map_.end();) {
    if (shutdown) {
      // We also destroy the corresponding BindingsTexture object when shutting
      // down the view.
      DestroyNativeObject(it->first);
    }
    if (it->second.GetBorrowedCount() == 0) {
      // Erasing the map entry also destroys the OwnedTexturePtr.
      bindings_texture_map_.erase(it++);
    } else {
      ++it;
    }
  }
}

absl::Status ImpressApiView::DisposeAllResources() {
  model_manager_->ResetAnimationContexts();
  // The order of destruction matters - we first destroy the glTF models, then
  // the materials which might be used in the glTF models, then the textures
  // which might be used in the materials.
  std::vector<NodeHandle> nodes_to_destroy;
  GetComponentManager().ForEach<GltfRenderer>(
      [&nodes_to_destroy](const GltfRenderer* gltf_renderer) {
        nodes_to_destroy.push_back(gltf_renderer->GetNode());
      });
  for (const auto& node : nodes_to_destroy) {
    DestroyNode(node);
  }
  asset_ptr_map_->DestroyGltfAssets();
  absl::Status status = asset_ptr_map_->DisposeIblAssets();
  if (!status.ok()) {
    return status;
  }
  DestroyUnusedMaterials(true);
  DestroyUnusedTextures(true);
  return absl::OkStatus();
}

void ImpressApiView::Setup() {
  // Setup View settings.
  filament::View* filament_view = GetHost()->GetView();
  filament_view->setPostProcessingEnabled(false);
}

void ImpressApiView::Update(const FrameTime& frame_time) {
  // We continually check if any of the bindings texture and materials are no
  // longer in use and destroy them if so.
  DestroyUnusedMaterials(false);
  DestroyUnusedTextures(false);

  // Update animation callbacks.
  model_manager_->Update(frame_time);
}

}  // namespace imp
