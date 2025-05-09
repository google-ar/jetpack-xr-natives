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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_DEEPLIGHT_CONTROLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_DEEPLIGHT_CONTROLLER_H_

#include <memory>

#include "core/ar/android/ar_core_ptrs.h"
#include "core/ar/ar_hdr_lighting.h"
#include "core/lighting/environment_light_factory.h"
#include "core/math/vec.h"
#include "filament/filament/include/filament/Engine.h"

namespace imp::ar {

class DeeplightController {
 public:
  // DeeplightController should not outlive ar_core_adapter or engine.
  DeeplightController(filament::Engine* engine,
                      imp::EnvironmentLightFactory* env_light_factory);
  ~DeeplightController();

  DeeplightController(DeeplightController const&) = delete;
  void operator=(DeeplightController const&) = delete;

  void Update(ArSession_* session, ArConfig_* config, ArFrame_* frame);
  std::vector<float3> GetSphericalHarmonicsLighting();

  bool IsHdrLightingEnabled() const;

  std::unique_ptr<HdrLighting> GetHdrLighting(ArSession* session);

  float4 GetAmbientLighting(ArSession* session);

 private:
  void ReleaseCubemap();

  filament::Engine* engine_;
  imp::EnvironmentLightFactory* env_light_factory_;
  UniqueArLightEstimate ar_light_estimate_;

  float3 irradiance_data_[9];
  float3 main_light_direction_;
  float3 main_light_color_;
  float main_light_intensity_scalar_;
  ArImageCubemap ar_image_cubemap_;
  ArLightEstimationMode light_estimation_mode_;

  float4 ambient_light_estimate_;
};

}  // namespace imp::ar

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_DEEPLIGHT_CONTROLLER_H_
