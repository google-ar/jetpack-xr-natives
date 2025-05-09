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

#include "extensions/skybox/skybox_renderer.h"

#include <cstdint>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/async/future.h"
#include "core/lighting/environment_light.h"
#include "core/render/texture.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/framework/lighting/light_manager.h"
#include "core/view/framework/render/mesh_factory.h"
#include "core/view/framework/render/mesh_renderer.h"
#include "core/view/utils/frame_time.h"
#include "extensions/skybox/skybox_assets.h"

namespace imp {

// Name of the parameter for the skybox cubemap texture in the skybox material.
constexpr absl::string_view kSkyboxParameter = "skybox";

// Rendering priority for the skybox is zero which means render first (behind
// everything else).
constexpr uint8_t kSkyboxPriority = 0;

Future<absl::Status> SkyboxRenderer::Setup() {
  return GetView()
      .GetAssetManager()
      .LoadMaterial(kSkyboxMaterialCmat)
      .Then([this](AssetPtr<MaterialAsset> material) mutable {
        // Create the MeshRenderer used to render the skybox.
        renderer_ = GetNode()->AddComponent<MeshRenderer>(
            MeshRenderer::FrustumCullingMode::kDisabled);
        renderer_->SetMesh(GetView().GetMeshFactory().CreateQuad());
        renderer_->SetShadowCastingMode(MeshRenderer::ShadowMode::kNone);
        renderer_->SetShadowReceivingMode(MeshRenderer::ShadowMode::kNone);
        renderer_->SetPriority(kSkyboxPriority);

        // Assign the skybox material to the MeshRenderer.
        renderer_->SetMaterial(
            GetView().GetMaterialFactory().CreateMaterial(material));

        // Disable until we have a reflections texture.
        renderer_->SetEnabled(false);
      });
}

void SkyboxRenderer::Update(const FrameTime& frame_time) {
  // Poll each frame to see if the reflections texture has changed.
  // There is no event/future based API to get this at the moment.
  const EnvironmentLight* environment_light =
      GetView().GetLightManager().GetEnvironmentLight();
  imp::Texture* reflections_texture = nullptr;
  if (environment_light) {
    reflections_texture =
        (*environment_light->GetReflectionIblAsset())->GetReflectionTexture();
  }

  if (reflections_texture != reflections_texture_) {
    reflections_texture_ = reflections_texture;

    // Set the texture on the material.
    // Doing it through the filament material instance directly because light
    // manager doesn't provide an Impress texture wrapper.
    renderer_->GetMaterial()->SetParameter(kSkyboxParameter.data(),
                                           reflections_texture);
    // Enable showing the skybox if the reflections texture exists.
    renderer_->SetEnabled(reflections_texture_ != nullptr);
  }
}

}  // namespace imp
