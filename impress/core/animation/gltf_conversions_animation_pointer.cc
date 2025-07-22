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

#include "core/animation/gltf_conversions_animation_pointer.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "absl/strings/numbers.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/animation/gltf_conversions_helper.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/enum_flags.h"
#include "core/common/optional_error.h"
#include "core/common/typed_span.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_extension_names.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/loader/provider/gltf/gltf_lookup.h"
#include "core/loader/provider/gltf/texturable_parameter.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::animation {
namespace {
constexpr absl::string_view kPointer = "pointer";
constexpr absl::string_view kExtensions = "extensions";
constexpr absl::string_view kLights = "lights";

using loader::details::kExtensionLightPunctual;
using loader::details::provider_gltf::TexturableParameters;
using TextureTransformChannelMap = GltfLookup::TextureTransformChannelMap;
using TextureTransformChannelSet = GltfLookup::TextureTransformChannelSet;

template <typename T>
OptionalError FillChannelOfParameter(
    const imp::gltf::imp_proto::Gltf& gltf,
    const std::vector<AnimationSampler>& animation_samplers,
    TypedSpan<const imp::gltf::imp_proto::AnimationChannel>& animation_channels,
    absl::optional<Domain>& material_parameter_domain,
    flatbuffers::FlatBufferBuilder* fbb, T& type,
    flatbuffers::Offset<void>& offset, ChannelId channel) {
  absl::optional<Domain> domain;
  if (channel) {
    MP_RETURN_IF_ERROR(AddChannel(
        gltf, animation_samplers[*animation_channels[channel].sampler], fbb,
        &type, &offset, &domain));
  }
  material_parameter_domain = MergeDomains(material_parameter_domain, domain);
  return NoError();
}

OptionalError SerializeTextureTransformAnimation(
    const imp::gltf::imp_proto::Gltf& gltf,
    TypedSpan<const imp::gltf::imp_proto::AnimationChannel>& animation_channels,
    const std::vector<AnimationSampler>& animation_samplers,
    TextureTransformChannelSet& texture_transform_channels,
    TexturableParameters target_texturable_parameter,
    flatbuffers::FlatBufferBuilder* fbb,
    absl::optional<
        flatbuffers::Offset<animation::schemas::TextureTransformAnimation>>*
        out_offset,
    absl::optional<Domain>* out_domain) {
  absl::optional<Domain> texturable_parameter_domain;

  std::optional<ChannelId> offset_channel =
      texture_transform_channels.offset_channel;
  std::optional<ChannelId> rotation_channel =
      texture_transform_channels.rotation_channel;
  std::optional<ChannelId> scale_channel =
      texture_transform_channels.scale_channel;

  if (!offset_channel.has_value() && !rotation_channel.has_value() &&
      !scale_channel.has_value()) {
    return NoError();
  }

  schemas::ChannelFloat2 texture_transform_offset_type =
      schemas::ChannelFloat2::NONE;
  flatbuffers::Offset<void> texture_transform_offset;
  if (offset_channel.has_value()) {
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat2>(
        gltf, animation_samplers, animation_channels,
        texturable_parameter_domain, fbb, texture_transform_offset_type,
        texture_transform_offset, *offset_channel));
  }

  schemas::ChannelFloat texture_transform_rotation_type =
      schemas::ChannelFloat::NONE;
  flatbuffers::Offset<void> texture_transform_rotation;
  if (rotation_channel.has_value()) {
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat>(
        gltf, animation_samplers, animation_channels,
        texturable_parameter_domain, fbb, texture_transform_rotation_type,
        texture_transform_rotation, *rotation_channel));
  }

  schemas::ChannelFloat2 texture_transform_scale_type =
      schemas::ChannelFloat2::NONE;
  flatbuffers::Offset<void> texture_transform_scale;
  if (scale_channel.has_value()) {
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat2>(
        gltf, animation_samplers, animation_channels,
        texturable_parameter_domain, fbb, texture_transform_scale_type,
        texture_transform_scale, *scale_channel));
  }

  out_offset->emplace(schemas::CreateTextureTransformAnimation(
      *fbb, texture_transform_offset_type, texture_transform_offset,
      texture_transform_rotation_type, texture_transform_rotation,
      texture_transform_scale_type, texture_transform_scale,
      static_cast<uint16_t>(target_texturable_parameter)));

  *out_domain = texturable_parameter_domain;

  return NoError();
}

OptionalError GetTextureTransformAnimation(
    const imp::gltf::imp_proto::Gltf& gltf,
    TypedSpan<const imp::gltf::imp_proto::AnimationChannel>& animation_channels,
    const std::vector<AnimationSampler>& animation_samplers,
    TextureTransformChannelMap& texture_transforms,
    flatbuffers::FlatBufferBuilder* fbb,
    std::vector<
        flatbuffers::Offset<animation::schemas::TextureTransformAnimation>>&
        out_texture_transform_animations,
    absl::optional<Domain>* out_domain) {
  // Go through all the textures in order. If a texture transform animation is
  // found, add the animation and texture target to the respective vector.
  TexturableParameters all_texturable_parameters[12] = {
      TexturableParameters::kBaseColorTexture,
      TexturableParameters::kMetallicRoughnessTexture,
      TexturableParameters::kNormalTexture,
      TexturableParameters::kOcclusionTexture,
      TexturableParameters::kEmissiveTexture,
      TexturableParameters::kClearcoatTexture,
      TexturableParameters::kClearcoatRoughnessTexture,
      TexturableParameters::kClearcoatNormalTexture,
      TexturableParameters::kSheenColorTexture,
      TexturableParameters::kSheenColorRoughnessTexture,
      TexturableParameters::kIorTexture,
      TexturableParameters::kTransmissionTexture,
  };

  for (TexturableParameters texturable_parameter : all_texturable_parameters) {
    if (!texture_transforms[texturable_parameter].has_value()) continue;

    absl::optional<
        flatbuffers::Offset<animation::schemas::TextureTransformAnimation>>
        animation_offset;
    absl::optional<Domain> animation_domain;
    MP_RETURN_IF_ERROR(SerializeTextureTransformAnimation(
        gltf, animation_channels, animation_samplers,
        /*lookup table*/ *(texture_transforms[texturable_parameter]),
        /*target*/ texturable_parameter, fbb, &animation_offset,
        &animation_domain))
        << "Adding texture transform animation data";

    *out_domain = animation_domain;

    if (animation_offset.has_value()) {
      out_texture_transform_animations.push_back(animation_offset.value());
    }
  }
  return NoError();
}

}  // namespace

OptionalError SerializeMaterialAnimation(
    const imp::gltf::imp_proto::Gltf& gltf, const GltfLookup& lookup,
    MaterialId material, AnimationId animation,
    flatbuffers::FlatBufferBuilder* fbb,
    absl::optional<flatbuffers::Offset<animation::schemas::MaterialAnimation>>*
        out_offset,
    absl::optional<Domain>* out_domain) {
  const GltfLookup::MaterialChannelSet& material_channel_set =
      lookup.material_channel_sets[animation];
  ChannelId base_color_factor_channel =
      material_channel_set.base_color_factor_channel[material];
  ChannelId metallic_factor_channel =
      material_channel_set.metallic_factor_channel[material];
  ChannelId roughness_factor_channel =
      material_channel_set.roughness_factor_channel[material];
  ChannelId alpha_cutoff_channel =
      material_channel_set.alpha_cutoff_channel[material];
  ChannelId emissive_factor_channel =
      material_channel_set.emissive_factor_channel[material];
  ChannelId normal_texture_scale_channel =
      material_channel_set.normal_texture_scale_channel[material];
  ChannelId occlusion_texture_strength_channel =
      material_channel_set.occlusion_texture_strength_channel[material];
  ChannelId ior_channel = material_channel_set.ior_channel[material];
  ChannelId transmission_channel =
      material_channel_set.transmission_channel[material];
  const TextureTransformChannelMap& texture_transforms =
      material_channel_set.texture_transform_channels[material];

  TypedSpan<const imp::gltf::imp_proto::AnimationChannel> animation_channels(
      lookup.animations[animation].channels);
  const std::vector<AnimationSampler>& animation_samplers =
      lookup.animations[animation].samplers;

  absl::optional<Domain> material_parameter_domain;

  if (base_color_factor_channel || metallic_factor_channel ||
      roughness_factor_channel || alpha_cutoff_channel ||
      emissive_factor_channel || normal_texture_scale_channel ||
      occlusion_texture_strength_channel || ior_channel ||
      transmission_channel || !texture_transforms.empty()) {
    schemas::ChannelFloat4 base_color_type = schemas::ChannelFloat4::NONE;
    flatbuffers::Offset<void> base_color;
    absl::optional<Domain> base_color_domain;
    if (base_color_factor_channel) {
      MP_RETURN_IF_ERROR(AddChannel(
          gltf,
          animation_samplers[*animation_channels[base_color_factor_channel]
                                  .sampler],
          fbb, &base_color_type, &base_color, &base_color_domain));
    }
    material_parameter_domain = base_color_domain;

    schemas::ChannelFloat metallic_factor_type = schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> metallic_factor;
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat>(
        gltf, animation_samplers, animation_channels, material_parameter_domain,
        fbb, metallic_factor_type, metallic_factor, metallic_factor_channel));

    schemas::ChannelFloat roughness_factor_type = schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> roughness_factor;
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat>(
        gltf, animation_samplers, animation_channels, material_parameter_domain,
        fbb, roughness_factor_type, roughness_factor,
        roughness_factor_channel));

    schemas::ChannelFloat alpha_cutoff_type = schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> alpha_cutoff;
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat>(
        gltf, animation_samplers, animation_channels, material_parameter_domain,
        fbb, alpha_cutoff_type, alpha_cutoff, alpha_cutoff_channel));

    schemas::ChannelFloat3 emissive_factor_type = schemas::ChannelFloat3::NONE;
    flatbuffers::Offset<void> emissive_factor;
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat3>(
        gltf, animation_samplers, animation_channels, material_parameter_domain,
        fbb, emissive_factor_type, emissive_factor, emissive_factor_channel));

    schemas::ChannelFloat normal_texture_scale_type =
        schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> normal_texture_scale;
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat>(
        gltf, animation_samplers, animation_channels, material_parameter_domain,
        fbb, normal_texture_scale_type, normal_texture_scale,
        normal_texture_scale_channel));

    schemas::ChannelFloat occlusion_texture_strength_type =
        schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> occlusion_texture_strength;
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat>(
        gltf, animation_samplers, animation_channels, material_parameter_domain,
        fbb, occlusion_texture_strength_type, occlusion_texture_strength,
        occlusion_texture_strength_channel));

    schemas::ChannelFloat ior_type = schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> ior;
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat>(
        gltf, animation_samplers, animation_channels, material_parameter_domain,
        fbb, ior_type, ior, ior_channel));

    schemas::ChannelFloat transmission_type = schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> transmission;
    MP_RETURN_IF_ERROR(FillChannelOfParameter<schemas::ChannelFloat>(
        gltf, animation_samplers, animation_channels, material_parameter_domain,
        fbb, transmission_type, transmission, transmission_channel));

    std::vector<
        flatbuffers::Offset<animation::schemas::TextureTransformAnimation>>
        texture_transform_animations;
    absl::optional<Domain> texture_transform_domain;
    MP_RETURN_IF_ERROR(GetTextureTransformAnimation(
        gltf, animation_channels, animation_samplers,
        const_cast<TextureTransformChannelMap&>(texture_transforms), fbb,
        texture_transform_animations, &texture_transform_domain));

    out_offset->emplace(schemas::CreateMaterialAnimation(
        *fbb, base_color_type, base_color, metallic_factor_type,
        metallic_factor, roughness_factor_type, roughness_factor,
        alpha_cutoff_type, alpha_cutoff, emissive_factor_type, emissive_factor,
        normal_texture_scale_type, normal_texture_scale,
        occlusion_texture_strength_type, occlusion_texture_strength, ior_type,
        ior, transmission_type, transmission,
        fbb->CreateVector(texture_transform_animations)));
    *out_domain =
        MergeDomains(material_parameter_domain, texture_transform_domain);
  }

  return NoError();
}

OptionalError SerializeLightPunctualAnimation(
    const imp::gltf::imp_proto::Gltf& gltf, const GltfLookup& lookup,
    LightPunctualId light, AnimationId animation,
    flatbuffers::FlatBufferBuilder* fbb,
    absl::optional<
        flatbuffers::Offset<animation::schemas::LightPunctualAnimation>>*
        out_offset,
    absl::optional<Domain>* out_domain) {
  const GltfLookup::LightChannelSet& light_channel_set =
      lookup.light_channel_sets[animation];
  ChannelId color_channel = light_channel_set.color_channel[light];
  ChannelId intensity_channel = light_channel_set.intensity_channel[light];
  ChannelId range_channel = light_channel_set.range_channel[light];
  ChannelId inner_cone_angle_channel =
      light_channel_set.spot_inner_cone_angle_channel[light];
  ChannelId outer_cone_angle_channel =
      light_channel_set.spot_outer_cone_angle_channel[light];
  TypedSpan<const imp::gltf::imp_proto::AnimationChannel> animation_channels(
      lookup.animations[animation].channels);
  const std::vector<AnimationSampler>& animation_samplers =
      lookup.animations[animation].samplers;

  absl::optional<Domain> material_parameter_domain;

  if (color_channel || intensity_channel || range_channel ||
      inner_cone_angle_channel || outer_cone_angle_channel) {
    schemas::ChannelFloat3 color_type = schemas::ChannelFloat3::NONE;
    flatbuffers::Offset<void> color;
    absl::optional<Domain> color_domain;
    if (color_channel) {
      MP_RETURN_IF_ERROR(AddChannel(
          gltf, animation_samplers[*animation_channels[color_channel].sampler],
          fbb, &color_type, &color, &color_domain));
    }
    material_parameter_domain = color_domain;

    schemas::ChannelFloat intensity_type = schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> intensity;
    absl::optional<Domain> intensity_domain;
    if (intensity_channel) {
      MP_RETURN_IF_ERROR(AddChannel(
          gltf,
          animation_samplers[*animation_channels[intensity_channel].sampler],
          fbb, &intensity_type, &intensity, &intensity_domain));
    }
    material_parameter_domain =
        MergeDomains(material_parameter_domain, intensity_domain);

    schemas::ChannelFloat range_type = schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> range;
    absl::optional<Domain> range_domain;
    if (range_channel) {
      MP_RETURN_IF_ERROR(AddChannel(
          gltf, animation_samplers[*animation_channels[range_channel].sampler],
          fbb, &range_type, &range, &range_domain));
    }
    material_parameter_domain =
        MergeDomains(material_parameter_domain, range_domain);

    schemas::ChannelFloat inner_cone_angle_type = schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> inner_cone_angle;
    absl::optional<Domain> inner_cone_angle_domain;
    if (inner_cone_angle_channel) {
      MP_RETURN_IF_ERROR(AddChannel(
          gltf,
          animation_samplers[*animation_channels[inner_cone_angle_channel]
                                  .sampler],
          fbb, &inner_cone_angle_type, &inner_cone_angle,
          &inner_cone_angle_domain));
    }
    material_parameter_domain =
        MergeDomains(material_parameter_domain, inner_cone_angle_domain);

    schemas::ChannelFloat outer_cone_angle_type = schemas::ChannelFloat::NONE;
    flatbuffers::Offset<void> outer_cone_angle;
    absl::optional<Domain> outer_cone_angle_domain;
    if (outer_cone_angle_channel) {
      MP_RETURN_IF_ERROR(AddChannel(
          gltf,
          animation_samplers[*animation_channels[outer_cone_angle_channel]
                                  .sampler],
          fbb, &outer_cone_angle_type, &outer_cone_angle,
          &outer_cone_angle_domain));
    }
    material_parameter_domain =
        MergeDomains(material_parameter_domain, outer_cone_angle_domain);

    out_offset->emplace(schemas::CreateLightPunctualAnimation(
        *fbb, color_type, color, intensity_type, intensity, range_type, range,
        inner_cone_angle_type, inner_cone_angle, outer_cone_angle_type,
        outer_cone_angle));
    *out_domain = material_parameter_domain;
  }

  return NoError();
}

OptionalError GetLightAnimation(
    const imp::gltf::imp_proto::Gltf& gltf, const GltfLookup& lookup,
    imp::loader::details::provider_gltf::AnimationId animation,
    flatbuffers::FlatBufferBuilder* fbb,
    std::vector<
        flatbuffers::Offset<animation::schemas::LightPunctualAnimation>>&
        out_light_animations,
    std::vector<animation::schemas::LightAnimationTarget>& out_light_targets,
    float& start_time, float& end_time) {
  if (!gltf.extensions.lights_punctual) return NoError();
  enum class ScratchFlags : uint8_t {
    kInThisAnim = (1 << 0),
  };
  GltfLookup::LightLookup<Flags<ScratchFlags>> light_scratch_flags;
  light_scratch_flags.Pair(gltf.extensions.lights_punctual->lights);

  for (const imp::gltf::imp_proto::AnimationChannel& c :
       lookup.animations[animation].channels) {
    if (c.target.path != kPointer) {
      continue;
    }
    if (!c.target.extensions.animation_pointer.has_value()) {
      return Error("Animation pointer misses content.");
    }
    std::vector<std::string> tokens =
        absl::StrSplit(c.target.extensions.animation_pointer->pointer, '/');
    if (tokens.size() < 4) {
      return Error("Invalid light animation target.");
    }
    if (tokens[1] == kExtensions && tokens[2] == kExtensionLightPunctual &&
        tokens[3] == kLights) {
      size_t light_id;
      if (!absl::SimpleAtoi(tokens[4], &light_id)) {
        return Error("Invalid light id.");
      }
      LightPunctualId target = LightPunctualId::At(light_id);
      if (light_scratch_flags[target] & ScratchFlags::kInThisAnim) continue;
      light_scratch_flags[target] |= ScratchFlags::kInThisAnim;
    }
  }

  // Go through all the lights in order. If a light punctual animation is
  // found, add the animation and light target to the respective vector.
  for (size_t light_index = 0; light_index < lookup.lights.size();
       light_index++) {
    LightPunctualId light_id = LightPunctualId::At(light_index);
    if (!(light_scratch_flags[light_id] & ScratchFlags::kInThisAnim)) continue;

    absl::optional<
        flatbuffers::Offset<animation::schemas::LightPunctualAnimation>>
        animation_offset;
    absl::optional<Domain> animation_domain;
    MP_RETURN_IF_ERROR(
        SerializeLightPunctualAnimation(gltf, lookup, light_id, animation, fbb,
                                        &animation_offset, &animation_domain))
        << "Adding light punctual animation data";

    if (animation_domain) {
      start_time = std::min(start_time, animation_domain->min);
      end_time = std::max(end_time, animation_domain->max);
    }

    if (animation_offset.has_value()) {
      out_light_animations.push_back(animation_offset.value());
      out_light_targets.push_back(light_index);
    }
  }
  return NoError();
}

}  // namespace imp::animation
