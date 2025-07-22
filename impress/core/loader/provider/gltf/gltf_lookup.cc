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

#include "core/loader/provider/gltf/gltf_lookup.h"

#include <algorithm>
#include <cstdint>
#include <iomanip>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "core/common/bit_vector.h"
#include "core/common/optional_error.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_tree.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/loader/provider/gltf/gltf_lookup_animation_pointer.h"
#include "core/math/math.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_gltf {
namespace {
constexpr absl::string_view kTranslation = "translation";
constexpr absl::string_view kRotation = "rotation";
constexpr absl::string_view kScale = "scale";
constexpr absl::string_view kWeights = "weights";
constexpr absl::string_view kPointer = "pointer";

using ::imp::gltf::imp_proto::Animation;
using ::imp::gltf::imp_proto::AnimationChannel;
using ::imp::gltf::imp_proto::Node;
using ::imp::gltf::imp_proto::Scene;
using ::imp::gltf::imp_proto::Skin;

bool IsAnimated(const GltfLookup &lookup, NodeId node) {
  const ChannelId kNilChannel = {};
  return absl::c_any_of(
      lookup.channel_sets,
      [&node, &kNilChannel](const GltfLookup::ChannelSet &channel_set) {
        return (channel_set.translation_channels[node] != kNilChannel ||
                channel_set.rotation_channels[node] != kNilChannel ||
                channel_set.scale_channels[node] != kNilChannel ||
                channel_set.weights_channels[node] != kNilChannel);
      });
}

absl::StatusOr<PreciseTransform> GetLocalTransform(const Node &node) {
  if (node.matrix.size() == 16) {
    const double *data = node.matrix.data();
    mat4 matrix{data[0],  data[1],  data[2],  data[3],   // ^
                data[4],  data[5],  data[6],  data[7],   // ^
                data[8],  data[9],  data[10], data[11],  // ^
                data[12], data[13], data[14], data[15]};
    PreciseTransform transform(matrix);
    if (!RoughlyEqual(matrix, transform.AsMat4())) {
      return Error(
          "Invalid matrix in node %s, The matrix must be desomposable to TRS "
          "properties.",
          node.name.data());
    }
    return transform;
  }

  // Extract glTF TRS.
  auto translation = double3(0);
  if (node.translation.size() == 3) {
    const double *data = node.translation.data();
    translation = double3{data[0], data[1], data[2]};
  }
  quatf rotation = kIdentityQuatf;
  if (node.rotation.size() == 4) {
    const float *data = node.rotation.data();
    // NOTE: intentional xyzw->wxyz conversion.
    rotation = quatf{data[3], data[0], data[1], data[2]};
  }
  auto scale = float3(1);
  if (node.scale.size() == 3) {
    const float *data = node.scale.data();
    scale = float3{data[0], data[1], data[2]};
  }

  return PreciseTransform(translation, rotation, scale);
}

NodeId GetBoneParent(const GltfLookup &lookup, NodeId node) {
  NodeId parent = lookup.parents[node];
  while (parent && !lookup.bones[parent]) {
    parent = lookup.parents[parent];
  }
  return parent;
}

OptionalError BuildExports(GltfLookup *out_lookup) {
  GltfLookup &lookup = *out_lookup;
  using ExportedBoneId = TypedId<GltfLookup::BoneEntry, uint32_t>;
  using ExportedBoneParentId = TypedParentId<GltfLookup::BoneEntry, uint32_t>;
  PairedVector<uint32_t, GltfLookup::BoneEntry> bone_child_counts(
      lookup.bone_entries.size());
  absl::c_transform(lookup.bone_entries, bone_child_counts.data(),
                    [](const GltfLookup::BoneEntry &entry) {
                      return static_cast<uint32_t>(entry.children.size());
                    });

  TypedDagTools<ExportedBoneId>::VisitParentThenChildren(
      PairedSpan<uint32_t, GltfLookup::BoneEntry>(
          absl::MakeSpan(bone_child_counts)),
      [&lookup](ExportedBoneId self, ExportedBoneParentId parent) {
        auto bone = self.CastTo<GltfLookup::BoneId>();
        auto node = lookup.bone_entries[self].node;
        if (lookup.self_flags[node] & NodeFlags::kExportMask) {
          lookup.exports[node] =
              lookup.export_entries.Append<GltfLookup::ExportId>(bone, node, 0);

          if (parent) {
            auto parent_bone = parent.CastTo<GltfLookup::BoneId>();
            auto parent_node = lookup.bone_entries[parent_bone].node;
            if (lookup.exports[parent_node]) {
              lookup.export_entries[lookup.exports[parent_node]].num_children++;
            }
          } else {
            lookup.export_roots.push_back(node);
          }
        }
      });

  return NoError();
}

OptionalError BuildGltfAnimationLookup(const imp::gltf::imp_proto::Gltf &gltf,
                                       GltfLookup &lookup) {
  GltfLookup::ChannelSet default_channel_set;
  default_channel_set.translation_channels.Pair(gltf.nodes);
  default_channel_set.rotation_channels.Pair(gltf.nodes);
  default_channel_set.scale_channels.Pair(gltf.nodes);
  default_channel_set.weights_channels.Pair(gltf.nodes);
  lookup.channel_sets.Pair(gltf.animations, default_channel_set);

  for (const Animation &anim : lookup.animations) {
    GltfLookup::ChannelSet &channel_set =
        lookup.channel_sets[lookup.animations.IdOf(anim)];
    channel_set.channels.Set(anim.channels);
    for (const AnimationChannel &channel : channel_set.channels) {
      auto channel_id = ChannelId(channel_set.channels.IdOf(channel));
      if (channel.target.node) {
        auto node = NodeId::At(*channel.target.node);
        if (!lookup.nodes.IsValid(node)) {
          return Error("Invalid target node");
        }
        if (channel.target.path == kTranslation) {
          channel_set.translation_channels[node] = channel_id;
        } else if (channel.target.path == kRotation) {
          channel_set.rotation_channels[node] = channel_id;
        } else if (channel.target.path == kScale) {
          channel_set.scale_channels[node] = channel_id;
        } else if (channel.target.path == kWeights) {
          channel_set.weights_channels[node] = channel_id;
        } else {
          IMP_LOG(imp::INFO) << "Skipping unsupported channel target '"
                    << std::setw(channel.target.path.size())
                    << channel.target.path.data() << "'";
        }
      } else {
        if (channel.target.path != kPointer) {
          return Error("Animation missing target node");
        }
      }
    }
  }

  // Parsing data for KHR_animation_pointer extension.
  MP_RETURN_IF_ERROR(AnimationPointerNodesLookup(gltf, lookup));
  MP_RETURN_IF_ERROR(AnimationPointerMaterialsLookup(gltf, lookup));
  MP_RETURN_IF_ERROR(AnimationPointerLightsLookup(gltf, lookup));

  return NoError();
}

}  // namespace

OptionalError BuildGltfLookup(const imp::gltf::imp_proto::Gltf &gltf,
                              const Scene &scene,
                              const loader::LoaderOptions &options,
                              GltfLookup *out_lookup) {
  GltfLookup &lookup = *out_lookup;

  lookup.self_flags.Pair(gltf.nodes);
  lookup.parents.Pair(gltf.nodes);
  lookup.local_transforms.Pair(gltf.nodes);
  lookup.bones.Pair(gltf.nodes);
  lookup.exports.Pair(gltf.nodes);
  lookup.nodes.Set(gltf.nodes);
  lookup.accessors.Set(gltf.accessors);
  lookup.animations.Set(gltf.animations);
  lookup.materials.Set(gltf.materials);

  if (!gltf.animations.empty()) {
    MP_RETURN_IF_ERROR(BuildGltfAnimationLookup(gltf, lookup));
  }

  for (const Skin &skin : gltf.skins) {
    for (auto node_index : skin.joints) {
      auto node = NodeId::At(node_index);
      if (!lookup.nodes.IsValid(node)) return Error("Invalid joints");
      lookup.self_flags[node] |= NodeFlags::kIsSampledBySkins;
    }
    if (skin.skeleton) {
      auto skeleton_node = NodeId::At(*skin.skeleton);
      if (!lookup.nodes.IsValid(skeleton_node))
        return Error("Invalid skeleton");
      lookup.self_flags[skeleton_node] |= NodeFlags::kIsSkinRoot;
    }
  }

  absl::InlinedVector<int, 32> work_stack;
  PairedBitVector<const Node> visited_by_work_stack;

  visited_by_work_stack.Resize(gltf.nodes.size());
  std::copy(scene.nodes.rbegin(), scene.nodes.rend(),
            std::back_inserter(work_stack));
  while (!work_stack.empty()) {
    auto node = NodeId(work_stack.back());
    if (!lookup.nodes.IsValid(node)) return Error("invalid node");
    if (visited_by_work_stack[node]) return Error("Cyclic graph");
    visited_by_work_stack[node] = true;
    work_stack.pop_back();

    // Queue our children (memoizing parent_index as we go)
    const auto &children = lookup.nodes[node].children;
    if (!children.empty()) {
      for (auto it = children.rbegin(); it != children.rend(); it++) {
        auto child_node = NodeId::At(*it);
        if (!lookup.nodes.IsValid(child_node)) return Error("invalid node");
        if (visited_by_work_stack[child_node]) {
          return Error("node graph had cycles");
        }

        if (IsAnimated(lookup, child_node)) {
          lookup.self_flags[node] |= NodeFlags::kIsParentOfAnimated;
        }
        work_stack.push_back(static_cast<int>(child_node));
        lookup.parents[child_node] = node;
      }
    }
    if (lookup.nodes[node].mesh.has_value()) {
      lookup.self_flags[node] |= NodeFlags::kHasMesh;
    }
    if (lookup.nodes[node].skin.has_value()) {
      lookup.self_flags[node] |= NodeFlags::kHasSkin;
    }
    if (!lookup.nodes[node].name.empty() &&
        lookup.nodes[node].children.empty()) {
      lookup.self_flags[node] |= NodeFlags::kIsNamedLeaf;
    }
    if (IsAnimated(lookup, node)) {
      lookup.self_flags[node] |= NodeFlags::kIsAnimated;
    }
    if (lookup.nodes[node].extensions.lights_punctual &&
        lookup.nodes[node].extensions.lights_punctual->light.has_value()) {
      lookup.self_flags[node] |= NodeFlags::kHasLightPunctual;
    }
    if (lookup.nodes[node].extensions.audio_extension &&
        lookup.nodes[node].extensions.audio_extension->emitter.has_value()) {
      lookup.self_flags[node] |= NodeFlags::kHasAudioEmitter;
    }

    if (lookup.nodes[node].extensions.khr_visibility) {
      lookup.self_flags[node] |= NodeFlags::kHasKhrVisibility;
    }

    if (lookup.nodes[node].extensions.khr_node_visibility) {
      lookup.self_flags[node] |= NodeFlags::kHasKhrNodeVisibility;
    }

    if (lookup.nodes[node].extensions.khr_node_selectability) {
      lookup.self_flags[node] |= NodeFlags::kHasKhrNodeSelectability;
    }

    if (lookup.nodes[node].extensions.khr_node_hoverability) {
      lookup.self_flags[node] |= NodeFlags::kHasKhrNodeHoverability;
    }

    if (!options.exclude_excess_nodes ||
        lookup.self_flags[node] & NodeFlags::kBoneMask) {
      lookup.bones[node] = lookup.bone_entries.Append<GltfLookup::BoneId>(
          GltfLookup::BoneEntry{node, {}});
      if (NodeId bone_parent = GetBoneParent(lookup, node)) {
        GltfLookup::BoneEntry &parent_entry =
            lookup.bone_entries[lookup.bones[bone_parent]];
        parent_entry.children.push_back(node);
      } else {
        lookup.bone_roots.push_back(node);
      }
    }

    // Cache our local transform.
    absl::StatusOr<PreciseTransform> local_transform =
        GetLocalTransform(lookup.nodes[node]);
    MP_ASSIGN_OR_RETURN(lookup.local_transforms[node],
                     GetLocalTransform(lookup.nodes[node]));

    // If a node has a mesh or a skin, then propagate the
    // IsMeshOrSkinRootAncestor to all of its parents (stop once we find they're
    // all set).
    if (lookup.self_flags[node] & NodeFlags::kShouldBubble) {
      auto parent = lookup.parents[node];
      auto self_flags = ToFlags(NodeFlags::kIsMeshOrSkinRootAncestor);
      while (parent &&
             ((lookup.self_flags[parent] & self_flags) != self_flags)) {
        lookup.self_flags[parent] |= self_flags;
        parent = lookup.parents[parent];
      }
    }
  }

  // Iterate over the bones we generated and come up with the narrower set of
  // export entities.
  MP_RETURN_IF_ERROR(BuildExports(out_lookup));

  return NoError();
}

}  // namespace imp::loader::details::provider_gltf
