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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_CONVERSIONS_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_CONVERSIONS_H_

#include "absl/status/statusor.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/loader/provider/gltf/gltf_lookup.h"

namespace imp::animation {

// Convert an animation from a loaded gltf file into a flabuffer representation.
absl::StatusOr<FlatBufferAccess<schemas::GltfAnimation>> GetAnimation(
    const imp::gltf::Gltf& gltf,
    const imp::loader::details::provider_gltf::GltfLookup& lookup,
    imp::loader::details::provider_gltf::AnimationId animation);

}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_CONVERSIONS_H_
