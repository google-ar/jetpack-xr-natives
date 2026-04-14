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

#include "core/loader/provider/gltf/gltf_lookup_animation_pointer.h"

#include <cstddef>
#include <iomanip>
#include <string>
#include <vector>

#include "core/common/log.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "core/common/optional_error.h"
#include "core/common/paired_vector.h"
#include "core/common/platform_helpers.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_extension_names.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/loader/provider/gltf/gltf_lookup.h"
#include "core/loader/provider/gltf/texturable_parameter.h"

namespace imp::loader::details::provider_gltf {
namespace {
constexpr absl::string_view kTranslation = "translation";
constexpr absl::string_view kRotation = "rotation";
constexpr absl::string_view kScale = "scale";
constexpr absl::string_view kWeights = "weights";
constexpr absl::string_view kPointer = "pointer";
constexpr absl::string_view kNodes = "nodes";
constexpr absl::string_view kMaterials = "materials";
constexpr absl::string_view kPbrMetallicRoughness = "pbrMetallicRoughness";
constexpr absl::string_view kBaseColorFactor = "baseColorFactor";
constexpr absl::string_view kMetallicFactor = "metallicFactor";
constexpr absl::string_view kRoughnessFactor = "roughnessFactor";
constexpr absl::string_view kAlphaCutoff = "alphaCutoff";
constexpr absl::string_view kEmissiveFactor = "emissiveFactor";
constexpr absl::string_view kNormalTexture = "normalTexture";
constexpr absl::string_view kOcclusionTexture = "occlusionTexture";
constexpr absl::string_view kExtensions = "extensions";
constexpr absl::string_view kLights = "lights";
constexpr absl::string_view kColor = "color";
constexpr absl::string_view kIntensity = "intensity";
constexpr absl::string_view kRange = "range";
constexpr absl::string_view kSpot = "spot";
constexpr absl::string_view kInnerConeAngle = "innerConeAngle";
constexpr absl::string_view kOuterConeAngle = "outerConeAngle";
constexpr absl::string_view kBaseColorTexture = "baseColorTexture";
constexpr absl::string_view kMetallicRoughnessTexture =
    "metallicRoughnessTexture";
constexpr absl::string_view kTextureTransform = "KHR_texture_transform";
constexpr absl::string_view kOffset = "offset";

using ::imp::gltf::imp_proto::Animation;
using ::imp::gltf::imp_proto::AnimationChannel;
using MaterialChannelSet = GltfLookup::MaterialChannelSet;
using TextureTransformChannelSet = GltfLookup::TextureTransformChannelSet;

// Figures out the parameter of the texture that this animation targets at.
void AssignChannelIdToTextureTransformChannel(
    TextureTransformChannelSet &transform,
    const absl::string_view &texture_transform_channel_name,
    const ChannelId &channel_id) {
  if (texture_transform_channel_name == kOffset) {
    transform.offset_channel = channel_id;
  } else if (texture_transform_channel_name == kRotation) {
    transform.rotation_channel = channel_id;
  } else if (texture_transform_channel_name == kScale) {
    transform.scale_channel = channel_id;
  }
}

// Figures out the texture that this animation targets at.
void AssignChannelIdToTexture(absl::string_view texture_name,
                              absl::string_view texture_transform_channel_name,
                              MaterialChannelSet &material_channel_set,
                              const MaterialId &material,
                              const ChannelId &channel_id) {
  if (texture_name == kBaseColorTexture) {
    if (!material_channel_set.texture_transform_channels
             [material][TexturableParameters::kBaseColorTexture]) {
      material_channel_set
          .texture_transform_channels[material]
                                     [TexturableParameters::kBaseColorTexture] =
          TextureTransformChannelSet();
    }
    AssignChannelIdToTextureTransformChannel(
        *material_channel_set.texture_transform_channels
             [material][TexturableParameters::kBaseColorTexture],
        texture_transform_channel_name, channel_id);
  } else if (texture_name == kMetallicRoughnessTexture) {
    if (!material_channel_set.texture_transform_channels
             [material][TexturableParameters::kMetallicRoughnessTexture]) {
      material_channel_set.texture_transform_channels
          [material][TexturableParameters::kMetallicRoughnessTexture] =
          TextureTransformChannelSet();
    }
    AssignChannelIdToTextureTransformChannel(
        *material_channel_set.texture_transform_channels
             [material][TexturableParameters::kMetallicRoughnessTexture],
        texture_transform_channel_name, channel_id);
  }
  // TODO: Parsing data for other texture types.
}
}  // namespace

OptionalError AnimationPointerNodesLookup(
    const imp::gltf::imp_proto::Gltf &gltf, GltfLookup &lookup) {
  GltfLookup::ChannelSet default_channel_set;
  lookup.channel_sets.Pair(gltf.animations, default_channel_set);

  for (const Animation &anim : lookup.animations) {
    GltfLookup::ChannelSet &channel_set =
        lookup.channel_sets[lookup.animations.IdOf(anim)];
    channel_set.channels.Set(anim.channels);
    for (const AnimationChannel &channel : channel_set.channels) {
      auto channel_id = ChannelId(channel_set.channels.IdOf(channel));
      if (channel.target.path != kPointer) {
        continue;
      }

      if (!channel.target.extensions.animation_pointer.has_value()) {
        return Error("Animation pointer extension is missing.");
      }

      absl::string_view pointer =
          channel.target.extensions.animation_pointer->pointer;
      std::vector<std::string> tokens = absl::StrSplit(pointer, '/');
      if (tokens.size() < 4) {
        return Error("Pointer is too short %.*s, skipped.", pointer.size(),
                     pointer.data());
      }

      if (tokens[1] != kNodes) {
        continue;
      }

      size_t node_id;
      if (!absl::SimpleAtoi(tokens[2], &node_id)) {
        return Error("Invalid target node");
      }
      auto node = NodeId::At(node_id);
      if (!lookup.nodes.IsValid(node)) {
        return Error("Invalid target node");
      }

      absl::string_view channel_name = tokens[3];
      if (channel_name == kTranslation) {
        channel_set.translation_channels[node] = channel_id;
        lookup.self_flags[node] |= NodeGltfFlags::kIsAnimated;
      } else if (channel_name == kRotation) {
        channel_set.rotation_channels[node] = channel_id;
        lookup.self_flags[node] |= NodeGltfFlags::kIsAnimated;
      } else if (channel_name == kScale) {
        channel_set.scale_channels[node] = channel_id;
        lookup.self_flags[node] |= NodeGltfFlags::kIsAnimated;
      } else if (channel_name == kWeights) {
        channel_set.weights_channels[node] = channel_id;
        lookup.self_flags[node] |= NodeGltfFlags::kIsAnimated;
      } else {
        IMP_LOG(imp::INFO) << "Skipping unsupported channel target '"
                  << std::setw(channel_name.size()) << channel_name.data()
                  << "'";
      }
    }
  }

  return NoError();
}

OptionalError AnimationPointerMaterialsLookup(
    const imp::gltf::imp_proto::Gltf &gltf, GltfLookup &lookup) {
  GltfLookup::MaterialChannelSet default_material_channels_set;
  default_material_channels_set.base_color_factor_channel.Pair(gltf.materials);
  default_material_channels_set.metallic_factor_channel.Pair(gltf.materials);
  default_material_channels_set.roughness_factor_channel.Pair(gltf.materials);
  default_material_channels_set.alpha_cutoff_channel.Pair(gltf.materials);
  default_material_channels_set.emissive_factor_channel.Pair(gltf.materials);
  default_material_channels_set.normal_texture_scale_channel.Pair(
      gltf.materials);
  default_material_channels_set.occlusion_texture_strength_channel.Pair(
      gltf.materials);
  default_material_channels_set.ior_channel.Pair(gltf.materials);
  default_material_channels_set.transmission_channel.Pair(gltf.materials);
  default_material_channels_set.texture_transform_channels.Pair(gltf.materials);

  lookup.material_channel_sets.Pair(gltf.animations,
                                    default_material_channels_set);

  for (const Animation &anim : lookup.animations) {
    GltfLookup::MaterialChannelSet &material_channel_set =
        lookup.material_channel_sets[lookup.animations.IdOf(anim)];
    material_channel_set.channels.Set(anim.channels);
    for (const AnimationChannel &channel : material_channel_set.channels) {
      auto channel_id = ChannelId(material_channel_set.channels.IdOf(channel));
      if (channel.target.path != kPointer) continue;
      if (!channel.target.extensions.animation_pointer.has_value()) {
        return Error("Animation pointer extention is missing.");
      }

      absl::string_view pointer =
          channel.target.extensions.animation_pointer->pointer;
      std::vector<std::string> tokens = absl::StrSplit(pointer, '/');
      if (tokens.size() < 4) {
        return Error("Pointer is too short %.*s, skipped.", pointer.size(),
                     pointer.data());
      }

      if (tokens[1] != kMaterials) {
        continue;
      }

      size_t material_id;
      if (!absl::SimpleAtoi(tokens[2], &material_id)) {
        return Error("Invalid material id");
      }
      MaterialId material = MaterialId::At(material_id);
      if (!lookup.materials.IsValid(material)) {
        return Error("Invalid target material");
      }

      // Make sure there is a tokens[4] so that it can compile.
      if (tokens.size() == 4) {
        tokens.push_back(" ");
      }
      absl::string_view channel_name = tokens[3];
      absl::string_view sub_channel_name = tokens[4];
      if (channel_name == kPbrMetallicRoughness) {
        if (sub_channel_name == kBaseColorFactor) {
          material_channel_set.base_color_factor_channel[material] = channel_id;
        } else if (sub_channel_name == kMetallicFactor) {
          material_channel_set.metallic_factor_channel[material] = channel_id;
        } else if (sub_channel_name == kRoughnessFactor) {
          material_channel_set.roughness_factor_channel[material] = channel_id;
        } else if (tokens.size() >= 8 && tokens[5] == kExtensions &&
                   tokens[6] == kTextureTransform) {
          // Example path:
          // /materials/{}/pbrMetallicRoughness/{xxTexture}/extensions/KHR_texture_transform/<transform_type>
          const absl::string_view texture_transform_channel_name = tokens[7];
          AssignChannelIdToTexture(sub_channel_name,
                                   texture_transform_channel_name,
                                   material_channel_set, material, channel_id);
        }
      } else if (channel_name == kAlphaCutoff) {
        material_channel_set.alpha_cutoff_channel[material] = channel_id;
      } else if (channel_name == kEmissiveFactor) {
        material_channel_set.emissive_factor_channel[material] = channel_id;
      } else if (channel_name == kNormalTexture) {
        material_channel_set.normal_texture_scale_channel[material] =
            channel_id;
      } else if (channel_name == kOcclusionTexture) {
        material_channel_set.occlusion_texture_strength_channel[material] =
            channel_id;
      } else if (channel_name == kExtensions) {
        if (sub_channel_name == kExtensionIor) {
          if (lookup.materials[material].extensions.ior != nullptr &&
              lookup.materials[material].extensions.ior->ior.has_value()) {
            material_channel_set.ior_channel[material] = channel_id;
          } else {
            IMP_LOG(imp::WARNING) << "Skipping material ior animation, since the "
                            "material has no ior extension.";
          }
        } else if (sub_channel_name == kExtensionTransmission) {
          if (lookup.materials[material].extensions.transmission != nullptr &&
              lookup.materials[material]
                  .extensions.transmission->transmission_factor.has_value()) {
            material_channel_set.transmission_channel[material] = channel_id;
          } else {
            IMP_LOG(imp::WARNING) << "Skipping material transmission animation, since "
                            "the material has no transmission extension.";
          }
        } else {
          IMP_LOG(imp::WARNING) << "Skipping unsupported material extension target in '"
                       << std::setw(pointer.size()) << pointer.data() << "'";
        }
      } else {
        IMP_LOG(imp::WARNING) << "Skipping unsupported material channel target in '"
                     << std::setw(pointer.size()) << pointer.data() << "'";
      }
    }
  }
  return NoError();
}

OptionalError AnimationPointerLightsLookup(
    const imp::gltf::imp_proto::Gltf &gltf, GltfLookup &lookup) {
  if (gltf.extensions.lights_punctual.has_value()) {
    lookup.lights.Set(gltf.extensions.lights_punctual->lights);
    const std::vector<::imp::gltf::imp_proto::LightPunctual> &lights =
        gltf.extensions.lights_punctual->lights;
    GltfLookup::LightChannelSet default_light_channel_set;
    default_light_channel_set.color_channel.Pair(lights);
    default_light_channel_set.intensity_channel.Pair(lights);
    default_light_channel_set.range_channel.Pair(lights);
    default_light_channel_set.spot_inner_cone_angle_channel.Pair(lights);
    default_light_channel_set.spot_outer_cone_angle_channel.Pair(lights);
    lookup.light_channel_sets.Pair(gltf.animations, default_light_channel_set);

    for (const Animation &anim : lookup.animations) {
      GltfLookup::LightChannelSet &light_channel_set =
          lookup.light_channel_sets[lookup.animations.IdOf(anim)];
      light_channel_set.channels.Set(anim.channels);
      for (const AnimationChannel &channel : light_channel_set.channels) {
        auto channel_id = ChannelId(light_channel_set.channels.IdOf(channel));
        if (channel.target.path != kPointer) continue;
        if (!channel.target.extensions.animation_pointer.has_value()) {
          return Error("Animation pointer extention is missing.");
        }

        absl::string_view pointer =
            channel.target.extensions.animation_pointer->pointer;
        std::vector<std::string> tokens = absl::StrSplit(pointer, '/');
        if (tokens.size() < 4) {
          return Error("Pointer is too short %.*s, skipped.", pointer.size(),
                       pointer.data());
        }
        if (tokens[1] != kExtensions || tokens[2] != kExtensionLightPunctual ||
            tokens[3] != kLights) {
          continue;
        }

        size_t light_id;
        if (!absl::SimpleAtoi(tokens[4], &light_id)) {
          return Error("Invalid light id");
        }

        const absl::string_view channel_name = tokens[5];
        if (channel_name == kColor) {
          light_channel_set.color_channel[LightPunctualId::At(light_id)] =
              channel_id;
        } else if (channel_name == kIntensity) {
          light_channel_set.intensity_channel[LightPunctualId::At(light_id)] =
              channel_id;
        } else if (channel_name == kRange) {
          light_channel_set.range_channel[LightPunctualId::At(light_id)] =
              channel_id;
        } else if (channel_name == kSpot) {
          if (tokens[6] == kInnerConeAngle) {
            light_channel_set
                .spot_inner_cone_angle_channel[LightPunctualId::At(light_id)] =
                channel_id;
          } else if (tokens[6] == kOuterConeAngle) {
            light_channel_set
                .spot_outer_cone_angle_channel[LightPunctualId::At(light_id)] =
                channel_id;
          } else {
            return Error("Invalid pointer '%.*s'", pointer.size(),
                         pointer.data());
          }
        } else {
          return Error("Invalid pointer '%.*s'", pointer.size(),
                       pointer.data());
        }
      }
    }
  }

  return NoError();
}

}  // namespace imp::loader::details::provider_gltf
