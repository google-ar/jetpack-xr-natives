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

#include "core/lighting/environment_light_factory.h"

#include <vector>

#include "absl/memory/memory.h"
#include "core/lighting/environment_light.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/math/vec.h"
#include "core/render/texture_factory.h"
#include "filament/filament/include/filament/IndirectLight.h"
#include "filament/filament/include/filament/Texture.h"

namespace imp {

EnvironmentLightPtr EnvironmentLightFactory::CreateEnvironmentLight(
    const AssetPtr<ImageBasedLightingAsset>& ibl_asset, float intensity,
    float3 tint) {
  return absl::WrapUnique(new EnvironmentLight(
      view_->GetSharedEngine(), ibl_asset, ibl_asset, intensity, tint));
}

EnvironmentLightPtr EnvironmentLightFactory::CreateEnvironmentLight(
    const AssetPtr<ImageBasedLightingAsset>& reflection_ibl_asset,
    const AssetPtr<ImageBasedLightingAsset>& sh_ibl_asset, float intensity,
    float3 tint) {
  return absl::WrapUnique(new EnvironmentLight(view_->GetSharedEngine(),
                                               reflection_ibl_asset,
                                               sh_ibl_asset, intensity, tint));
}

EnvironmentLightPtr EnvironmentLightFactory::CreateEnvironmentLight(
    float3 ambient_color, float intensity) {
  std::vector<float3> irradiance_data(9, kZero3);
  irradiance_data[0] = ambient_color;
  return absl::WrapUnique(new EnvironmentLight(view_->GetSharedEngine(),
                                               irradiance_data, intensity));
}

EnvironmentLightPtr EnvironmentLightFactory::WrapIndirectLight(
    filament::IndirectLight* indirect_light, bool transfer_texture_ownership) {
  if (transfer_texture_ownership) {
    return absl::WrapUnique(new EnvironmentLight(
        view_->GetSharedEngine(), indirect_light,
        view_->GetTextureFactory().WrapTexture(const_cast<filament::Texture*>(
            indirect_light->getReflectionsTexture())),
        view_->GetTextureFactory().WrapTexture(const_cast<filament::Texture*>(
            indirect_light->getIrradianceTexture()))));
  } else {
    return absl::WrapUnique(new EnvironmentLight(
        view_->GetSharedEngine(), indirect_light, nullptr, nullptr));
  }
}

}  // namespace imp
