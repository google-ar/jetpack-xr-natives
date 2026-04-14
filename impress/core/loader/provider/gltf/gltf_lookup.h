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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_LOOKUP_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_LOOKUP_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "core/common/enum_flags.h"
#include "core/common/optional_error.h"
#include "core/common/paired_vector.h"
#include "core/common/robin_map.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_span.h"
#include "core/common/typed_vector.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/loader/provider/gltf/texturable_parameter.h"
#include "core/math/math.h"
#include "core/math/transform.h"

namespace imp::loader::details::provider_gltf {

enum class NodeGltfFlags : uint16_t {
  kHasMesh = (1 << 0),
  kIsMeshOrSkinRootAncestor = (1 << 1),
  kIsAnimated = (1 << 2),
  kIsSampledBySkins = (1 << 3),
  kIsParentOfAnimated = (1 << 4),
  kIsSkinRoot = (1 << 5),
  kHasSkin = (1 << 6),
  kIsNamedLeaf = (1 << 7),
  kHasLightPunctual = (1 << 8),
  kHasAudioEmitter = (1 << 9),
  // TODO: Remove this when deprecating KHR_behavior.
  kHasKhrVisibility = (1 << 10),
  kHasKhrNodeVisibility = (1 << 11),
  kHasKhrNodeSelectability = (1 << 12),
  kHasKhrNodeHoverability = (1 << 13),
  kShouldBubble = kHasMesh | kIsSkinRoot | kHasLightPunctual | kHasAudioEmitter,
  kBoneMask = kHasMesh | kIsAnimated | kIsSampledBySkins | kIsParentOfAnimated |
              kIsSkinRoot | kIsMeshOrSkinRootAncestor | kIsNamedLeaf |
              kHasLightPunctual,
  kExportMask = kHasMesh | kHasSkin | kIsMeshOrSkinRootAncestor | kIsSkinRoot |
                kHasLightPunctual | kHasAudioEmitter | kHasKhrVisibility |
                kHasKhrNodeVisibility | kHasKhrNodeSelectability |
                kHasKhrNodeHoverability,
};

enum class TextureTransformChannels : uint16_t {
  kOffset = (1 << 0),
  kRotation = (1 << 1),
  kScale = (1 << 2),
};

struct GltfLookup {
  template <typename T>
  using NodeLookup = PairedVector<T, const imp::gltf::imp_proto::Node>;
  template <typename T>
  using SparseNodeLookup = absl::flat_hash_map<NodeId, T>;
  struct ChannelSet {
    TypedSpan<const gltf::imp_proto::AnimationChannel> channels;
    SparseNodeLookup<ChannelId> translation_channels;
    SparseNodeLookup<ChannelId> rotation_channels;
    SparseNodeLookup<ChannelId> scale_channels;
    SparseNodeLookup<ChannelId> weights_channels;
  };
  struct TextureTransformChannelSet {
    std::optional<ChannelId> offset_channel;
    std::optional<ChannelId> rotation_channel;
    std::optional<ChannelId> scale_channel;
  };
  using TextureTransformChannelMap =
      RobinMap<TexturableParameters, std::optional<TextureTransformChannelSet>>;
  template <typename T>
  using MaterialLookup = PairedVector<T, const gltf::imp_proto::Material>;
  struct MaterialChannelSet {
    TypedSpan<const gltf::imp_proto::AnimationChannel> channels;
    MaterialLookup<ChannelId> base_color_factor_channel;
    MaterialLookup<ChannelId> metallic_factor_channel;
    MaterialLookup<ChannelId> roughness_factor_channel;
    MaterialLookup<ChannelId> alpha_cutoff_channel;
    MaterialLookup<ChannelId> emissive_factor_channel;
    MaterialLookup<ChannelId> normal_texture_scale_channel;
    MaterialLookup<ChannelId> occlusion_texture_strength_channel;
    MaterialLookup<ChannelId> ior_channel;
    MaterialLookup<ChannelId> transmission_channel;
    MaterialLookup<TextureTransformChannelMap> texture_transform_channels;
  };
  template <typename T>
  using LightLookup = PairedVector<T, const gltf::imp_proto::LightPunctual>;
  struct LightChannelSet {
    TypedSpan<const gltf::imp_proto::AnimationChannel> channels;
    LightLookup<ChannelId> color_channel;
    LightLookup<ChannelId> intensity_channel;
    LightLookup<ChannelId> range_channel;
    LightLookup<ChannelId> spot_inner_cone_angle_channel;
    LightLookup<ChannelId> spot_outer_cone_angle_channel;
  };
  struct BoneEntry {
    NodeId node;
    std::vector<NodeId> children;
  };
  using BoneId = TypedIdWithSentinel<BoneEntry, uint32_t, kMaxValue<uint32_t>>;
  struct ExportEntry {
    using ArrayType = StructureOfArrays<BoneId, NodeId, uint32_t>;
    enum Fields {
      kBone,
      kNode,
      kNumChildren,
    };
    struct Proxy {
      template <size_t E>
      using Field = ArrayType::Field<E>;

      union {
        Field<kBone> bone;
        Field<kNode> node;
        Field<kNumChildren> num_children;
      };
    };
  };
  using ExportId =
      TypedIdWithSentinel<ExportEntry, uint32_t, kMaxValue<uint32_t>>;

  // Retrieves a value from a map by key, or a default-constructed value if not
  // found, without modifying the map.
  template <typename MapType, typename KeyType>
  static typename MapType::mapped_type GetValueOrDefault(const MapType& map,
                                                         KeyType key) {
    using ValueType = typename MapType::mapped_type;
    auto it = map.find(key);
    return it != map.end() ? it->second : ValueType();
  }

  NodeLookup<Flags<NodeGltfFlags>> self_flags;
  NodeLookup<NodeId> parents;
  PairedVector<ChannelSet, const imp::gltf::imp_proto::Animation> channel_sets;
  PairedVector<MaterialChannelSet, const imp::gltf::imp_proto::Animation>
      material_channel_sets;
  PairedVector<LightChannelSet, const imp::gltf::imp_proto::Animation>
      light_channel_sets;
  NodeLookup<PreciseTransform> local_transforms;
  NodeLookup<BoneId> bones;
  TypedVector<BoneEntry> bone_entries;
  std::vector<NodeId> bone_roots;
  NodeLookup<ExportId> exports;
  TypedSetVector<ExportEntry> export_entries;
  std::vector<NodeId> export_roots;

  // Type-safe view into our gltf data.
  TypedSpan<const gltf::imp_proto::Node> nodes;
  TypedSpan<const gltf::imp_proto::Accessor> accessors;
  TypedSpan<const gltf::imp_proto::Animation> animations;
  TypedSpan<const gltf::imp_proto::Material> materials;
  TypedSpan<const gltf::imp_proto::LightPunctual> lights;
};

OptionalError BuildGltfLookup(const gltf::imp_proto::Gltf& gltf,
                              const gltf::imp_proto::Scene& scene,
                              const loader::LoaderOptions& options,
                              GltfLookup* out_lookup);

}  // namespace imp::loader::details::provider_gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_LOOKUP_H_
