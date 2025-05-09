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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_LOOKUP_ANIMATION_POINTER_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_LOOKUP_ANIMATION_POINTER_H_

#include "core/common/optional_error.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_lookup.h"

namespace imp::loader::details::provider_gltf {

OptionalError AnimationPointerNodesLookup(const imp::gltf::Gltf &gltf,
                                          GltfLookup &lookup);

OptionalError AnimationPointerMaterialsLookup(const imp::gltf::Gltf &gltf,
                                              GltfLookup &lookup);

OptionalError AnimationPointerLightsLookup(const imp::gltf::Gltf &gltf,
                                           GltfLookup &lookup);

}  // namespace imp::loader::details::provider_gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_LOOKUP_ANIMATION_POINTER_H_
