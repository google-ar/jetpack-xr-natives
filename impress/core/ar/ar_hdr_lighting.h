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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_AR_HDR_LIGHTING_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_AR_HDR_LIGHTING_H_

#include <optional>

#include "core/lighting/environment_light.h"
#include "core/math/vec.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndirectLight.h"

namespace imp {
namespace ar {

// Describes lighting information from ArSessionNative classes.
struct HdrLighting {
  struct DirectionalLightInfo {
    float3 color;
    float intensity;
    float3 direction;
    bool cast_shadow = true;
  };

  imp::EnvironmentLightPtr environment_light;
  std::optional<DirectionalLightInfo> directional_light_info;
};

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_AR_HDR_LIGHTING_H_
