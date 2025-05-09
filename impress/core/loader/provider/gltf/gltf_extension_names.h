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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_EXTENSIONS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_EXTENSIONS_H_

#include "absl/strings/string_view.h"
namespace imp::loader::details {

constexpr absl::string_view kExtensionIor = "KHR_materials_ior";
constexpr absl::string_view kExtensionMask = "GOOG_mask";
constexpr absl::string_view kExtensionTransmission =
    "KHR_materials_transmission";
constexpr absl::string_view kExtensionLightPunctual = "KHR_lights_punctual";

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_EXTENSIONS_H_
