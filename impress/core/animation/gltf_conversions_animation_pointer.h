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

#ifndef THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_CONVERSIONS_ANIMATION_POINTER_H_
#define THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_CONVERSIONS_ANIMATION_POINTER_H_

#include "absl/types/optional.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/animation/gltf_conversions_helper.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_helpers.h"

namespace imp::animation {
using imp::loader::details::provider_gltf::LightPunctualId;

OptionalError SerializeMaterialAnimation(
    const imp::gltf::imp_proto::Gltf& gltf, const GltfLookup& lookup,
    MaterialId material, AnimationId animation,
    flatbuffers::FlatBufferBuilder* fbb,
    absl::optional<flatbuffers::Offset<animation::schemas::MaterialAnimation>>*
        out_offset,
    absl::optional<Domain>* out_domain);

OptionalError SerializeLightPunctualAnimation(
    const imp::gltf::imp_proto::Gltf& gltf, const GltfLookup& lookup,
    LightPunctualId light, AnimationId animation,
    flatbuffers::FlatBufferBuilder* fbb,
    absl::optional<
        flatbuffers::Offset<animation::schemas::LightPunctualAnimation>>*
        out_offset,
    absl::optional<Domain>* out_domain);

OptionalError GetLightAnimation(
    const imp::gltf::imp_proto::Gltf& gltf, const GltfLookup& lookup,
    imp::loader::details::provider_gltf::AnimationId animation,
    flatbuffers::FlatBufferBuilder* fbb,
    std::vector<
        flatbuffers::Offset<animation::schemas::LightPunctualAnimation>>&
        out_light_animations,
    std::vector<animation::schemas::LightAnimationTarget>& out_light_targets,
    float& start_time, float& end_time);
}  // namespace imp::animation

#endif  // THIRD_PARTY_IMPRESS_CORE_ANIMATION_GLTF_CONVERSIONS_ANIMATION_POINTER_H_
