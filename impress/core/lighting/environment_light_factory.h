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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_IBL_FACTORY_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_IBL_FACTORY_H_

#include "core/lighting/environment_light.h"
#include "core/lighting/image_based_lighting_asset.h"
#include "core/math/vec.h"
#include "core/view/base_view.h"
#include "filament/filament/include/filament/IndirectLight.h"

namespace imp {

// Factory class for creating EnvironmentLight.
class EnvironmentLightFactory {
 public:
  explicit EnvironmentLightFactory(BaseView* view) : view_(view) {}
  // Create an EnvironmentLight object from ImageBasedLightingAsset.
  //
  // If spherical harmonics is available, the
  // EnvironmentLight will use it as irradiance.
  //
  // If lighting cubemap is available, the EnvironmentLight will use it as
  // reflections texture. If not and if skybox cubemap is available, the
  // EnvironmentLight will use it as reflections texture.
  //
  // This can also tint the lighting if `tint` is specified.
  // Please note that tint requires spherical harmonics data to be present in
  // the ImageBasedLightingAsset. Otherwise tint will be a no-op.
  EnvironmentLightPtr CreateEnvironmentLight(
      const AssetPtr<ImageBasedLightingAsset>& ibl_asset, float intensity,
      float3 tint = float3(1.0f, 1.0f, 1.0f));

  // Create an EnvironmentLight object from ImageBasedLightingAsset.
  //
  // `reflection_ibl_asset` will be used for reflection and `sh_ibl_asset` will
  // be used for irradiance.
  //
  // Please note that `reflection_ibl_asset` requires at
  // least one of lighting cubemap or skybox cubemap to be included and
  // `sh_ibl_asset` requires spherical_harmonics to be included for them to be
  // working properly.
  //
  // This can also tint the lighting if `tint` is specified.
  // Please note that tint requires spherical harmonics data to be present in
  // `sh_ibl_asset`. Otherwise tint will be a no-op.
  EnvironmentLightPtr CreateEnvironmentLight(
      const AssetPtr<ImageBasedLightingAsset>& reflection_ibl_asset,
      const AssetPtr<ImageBasedLightingAsset>& sh_ibl_asset, float intensity,
      float3 tint = float3(1.0f, 1.0f, 1.0f));

  // Create an EnvironmentLight object that can be used for ambient
  // lighting by specifying ambient color.
  EnvironmentLightPtr CreateEnvironmentLight(float3 ambient_color,
                                             float intensity);

  // Create an EnvironmentLight object that wraps an existing
  // filament::IndirectLight.
  // By default, transfer_texture_ownership is true, which makes the
  // EnvironmentLight the owner of the textures associated with the wrapped
  // IndirectLight, meaning that its destructor will also clean up all the
  // textures(irradiance, reflection) the wrapped IndirectLight references.
  EnvironmentLightPtr WrapIndirectLight(filament::IndirectLight* indirect_light,
                                        bool transfer_texture_ownership = true);

 private:
  BaseView* view_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_FRAMEWORK_ASSETS_IBL_FACTORY_H_
