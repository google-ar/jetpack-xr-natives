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

#include "core/lighting/environment_light.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "filament/filament/include/filament/IndirectLight.h"
#include "core/assets/asset_ptr.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"

namespace imp {

EnvironmentLight::~EnvironmentLight() {
  if (engine_ && indirect_light_) {
    engine_->destroy(indirect_light_);
  }
}

EnvironmentLight::EnvironmentLight(
    filament::Engine* engine,
    const AssetPtr<ImageBasedLightingAsset>& reflection_ibl_asset,
    const AssetPtr<ImageBasedLightingAsset>& irradiance_sh_ibl_asset,
    float intensity, float3 tint)
    : engine_(engine) {
  auto indirect_light_builder =
      filament::IndirectLight::Builder().intensity(intensity);

  if (irradiance_sh_ibl_asset->GetSphericalHarmonics()) {
    std::vector<float3> out_coefficients(
        irradiance_sh_ibl_asset->GetSphericalHarmonics()->coefficients.size());
    std::transform(
        irradiance_sh_ibl_asset->GetSphericalHarmonics()->coefficients.begin(),
        irradiance_sh_ibl_asset->GetSphericalHarmonics()->coefficients.end(),
        out_coefficients.begin(),
        [](float3 c) { return float3{c.x, c.y, c.z} / M_PI; });
    out_coefficients[0] *= tint;

    indirect_light_builder.irradiance(
        irradiance_sh_ibl_asset->GetSphericalHarmonics()->num_bands,
        out_coefficients.data());
    irradiance_sh_ibl_asset_ = irradiance_sh_ibl_asset;
  } else {
    IMP_LOG(imp::WARNING)
        << "Spherical harmonics data not found in irradiance_sh_ibl_asset. If "
           "this is not intended, please make sure the image based lighting "
           "asset is not excluding spherical harmonics data.";
  }

  if (reflection_ibl_asset->GetLightingCubemap()) {
    indirect_light_builder.reflections(
        reflection_ibl_asset->GetLightingCubemap()->GetTexture());
    reflection_ibl_asset_ = reflection_ibl_asset;
  } else if (reflection_ibl_asset->GetSkyboxCubemap()) {
    indirect_light_builder.reflections(
        reflection_ibl_asset->GetSkyboxCubemap()->GetTexture());
    reflection_ibl_asset_ = reflection_ibl_asset;
  }

  indirect_light_ = indirect_light_builder.build(*engine_);
}

EnvironmentLight::EnvironmentLight(filament::Engine* engine,
                                   std::vector<float3> sh_irradiance,
                                   float intensity)
    : engine_(engine) {
  auto indirect_light_builder =
      filament::IndirectLight::Builder().intensity(intensity);

  size_t num_bands = sqrt(sh_irradiance.size());
  indirect_light_builder.irradiance(num_bands, sh_irradiance.data());

  indirect_light_ = indirect_light_builder.build(*engine_);
}

EnvironmentLight::EnvironmentLight(filament::Engine* engine,
                                   filament::IndirectLight* indirect_light,
                                   TexturePtr reflection_cubemap,
                                   TexturePtr irradiance_cubemap)
    : engine_(engine),
      indirect_light_(indirect_light),
      owned_reflection_cubemap_(std::move(reflection_cubemap)),
      owned_irradiance_cubemap_(std::move(irradiance_cubemap)) {}

void EnvironmentLight::SetIntensity(float intensity) {
  GetIndirectLight()->setIntensity(intensity);
}

void EnvironmentLight::SetRotation(quatf rotation) {
  GetIndirectLight()->setRotation(mat3f(rotation));
}

mat3f EnvironmentLight::GetRotation() const {
  return GetIndirectLight()->getRotation();
}

}  // namespace imp
