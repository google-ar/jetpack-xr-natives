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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_LIGHTING_IMAGE_BASED_LIGHTING_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_LIGHTING_IMAGE_BASED_LIGHTING_H_

#include <memory>
#include <optional>
#include <vector>

#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndirectLight.h"
#include "core/assets/asset_ptr.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_or_unowned_memory.h"
#include "core/common/owned_ptr.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"

namespace imp {

class EnvironmentLightFactory;

// EnvironmentLight is a wrapper of filament::IndirectLight.
// Similar to imp::Material, EnvironmentLight should not be created directly
// and should be created through EnvironmentLightFactory.
//
// Please note that if an EnvironmentLight is constructed through
// EnvironmentLightFactory::WrapIndirectLight, the destructor will also clean up
// all the textures(irradiance, reflection) the wrapped filament::IndirectLight
// references.
class EnvironmentLight {
 public:
  ~EnvironmentLight();

  filament::IndirectLight* GetIndirectLight() const { return indirect_light_; }

  // Returns the ImageBasedLightingAsset if the reflection
  // comes from ImageBasedLightingAsset. Returns std::nullopt otherwise.
  std::optional<AssetPtr<ImageBasedLightingAsset>> GetReflectionIblAsset()
      const {
    return reflection_ibl_asset_;
  }

  // Returns the ImageBasedLightingAsset if the spherical harmonics for
  // irradiance comes from ImageBasedLightingAsset. Returns std::nullopt
  // otherwise.
  std::optional<AssetPtr<ImageBasedLightingAsset>> GetShIrradianceIblAsset()
      const {
    return irradiance_sh_ibl_asset_;
  }

  // Sets the intensity of the indirect light.
  void SetIntensity(float intensity);
  // Sets the rotation of the indirect light.
  void SetRotation(quatf rotation);
  // Gets the rotation of the indirect light.
  mat3f GetRotation() const;

 private:
  EnvironmentLight(
      filament::Engine* engine,
      const AssetPtr<ImageBasedLightingAsset>& reflection_ibl_asset,
      const AssetPtr<ImageBasedLightingAsset>& irradiance_sh_ibl_asset,
      float intensity, float3 tint);

  EnvironmentLight(filament::Engine* engine, std::vector<float3> sh_irradiance,
                   float intensity);

  EnvironmentLight(filament::Engine* engine,
                   filament::IndirectLight* indirect_light,
                   TexturePtr reflection_cubemap,
                   TexturePtr irradiance_cubemap);

  filament::Engine* engine_;

  std::optional<AssetPtr<ImageBasedLightingAsset>> irradiance_sh_ibl_asset_;
  std::optional<AssetPtr<ImageBasedLightingAsset>> reflection_ibl_asset_;

  filament::IndirectLight* indirect_light_;

  TexturePtr owned_reflection_cubemap_;
  TexturePtr owned_irradiance_cubemap_;

  friend class EnvironmentLightFactory;
};

using EnvironmentLightPtr = std::unique_ptr<EnvironmentLight>;
using OwnedEnvironmentLightPtr = OwnedPtr<EnvironmentLight>;
using BorrowedEnvironmentLightPtr = BorrowedPtr<EnvironmentLight>;
using OwnedOrBorrowedEnvironmentLightPtr = OwnedOrBorrowedPtr<EnvironmentLight>;
// Note: OwnedOrUnownedEnvironmentLight is legacy and should be migrated to
// the new Owned/Borrowed system.
using OwnedOrUnownedEnvironmentLight = OwnedOrUnownedMemory<EnvironmentLight>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_LIGHTING_IMAGE_BASED_LIGHTING_H_
