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

#include "core/loader/provider/gltf/provider_gltf.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/escaping.h"
#include "absl/strings/match.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/math/include/math/scalar.h"
#include "core/animation/gltf_conversions.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/bit_vector.h"
#include "core/common/buffer_access.h"
#include "core/common/filament_helpers.h"
#include "core/common/file_helpers.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/optional_error.h"
#include "core/common/paired_vector.h"
#include "core/common/schemas/render_generated.h"
#include "core/common/typed_id.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_tree.h"
#include "core/common/typed_vector.h"
#include "core/loader/provider/details/gltf_provider.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/loader/provider/details/provider_details_common.h"
#include "core/loader/provider/details/vertex_attribute.h"
#include "core/loader/provider/extensions/behavior/behavior.proto.imp.h"
#include "core/loader/provider/extensions/behavior/loader_extension.h"
#include "core/loader/provider/extensions/gltf_extension_draco.h"
#include "core/loader/provider/extensions/gltf_extension_meshopt.h"
#include "core/loader/provider/extensions/interactivity/interactivity.proto.imp.h"
#include "core/loader/provider/extensions/interactivity/loader_extension.h"
#include "core/loader/provider/gltf/accessor_reader.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_animation.h"
#include "core/loader/provider/gltf/gltf_attribute.h"
#include "core/loader/provider/gltf/gltf_geometry.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/loader/provider/gltf/gltf_lookup.h"
#include "core/loader/provider/gltf/gltf_texture.h"
#include "core/loader/provider/gltf/parse_gltf.h"
#include "core/loader/provider/gltf/parsed_gltf.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/generic_material_schema_provider.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "robin_map/include/tsl/robin_map.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_gltf {
namespace {
using ::imp::gltf::Accessor;
using ::imp::gltf::Animation;
using ::imp::gltf::Gltf;
using ::imp::gltf::Material;
using ::imp::gltf::Primitive;
using ::imp::gltf::Scene;
using WeakSkinId = imp::TypedId<model::ModelData::SkinData, int>;

class ProtoGltfProvider : public GltfProvider {
 public:
  ~ProtoGltfProvider() override {}

  // Attempt to parse the gltf file, retrieving a tinygltf::Model.  This can
  // be used in subsequent calls to TryLoadGltf.
  OptionalError TryParseGltf(LoaderState* state) override;

  bool IsParsed() override;

  // Check a successful parsed model for pending resources.  This allows us to
  // flag missing resources encountered during a successful parsing, and
  // update the tracking information that allows us to maintain those links
  // when we attempt to load.
  bool HasPendingResources(LoaderState* state) override;

  // Attempt a single gltf load.
  absl::StatusOr<FlatBufferAccess<schemas::LoadedModel>> TryLoadGltf(
      LoaderState* state) override;

  const ParsedGltf* GetParsedGltf() const override {
    return parsed_gltf_.get();
  }

 private:
  std::unique_ptr<ParsedGltf> parsed_gltf_;
  std::unique_ptr<LoadedModelBuilder> builder_;
};

absl::Status ResolveDataResource(std::vector<BufferAccess>& owned,
                                 absl::string_view uri,
                                 absl::string_view* view) {
  if (!absl::StrContains(uri, ";base64,")) {
    return Error("unknown format");
  }
  // Handle base64 encoded data.
  auto data = uri.substr(uri.find(";base64,") + 8);
  std::string decoded;
  if (!absl::Base64Unescape(data, &decoded)) {
    return Error("base64 decoding failed");
  }
  owned.emplace_back(BufferAccess::Clone(
      reinterpret_cast<const uint8_t*>(decoded.data()), decoded.size()));
  *view = owned.back().StringView();
  return NoError();
}

absl::Status ResolveUriResource(
    absl::string_view directory,
    tsl::robin_map<std::string, BufferAccess>& resources,
    tsl::robin_map<std::string, std::string>& missing_resource_name_from_path,
    absl::string_view uri, absl::string_view* view) {
  const std::string kUpOneDirectory = "..";
  const std::string kDesiredPrefix = "./";
  std::string corrected_uri(uri);
  // gltf loader tracks multiple paths; ignore non-local ones.
  if (corrected_uri.substr(0, kDesiredPrefix.size()) != kDesiredPrefix) {
    corrected_uri = kDesiredPrefix + corrected_uri;
  }
  if (corrected_uri.find(kUpOneDirectory) != absl::string_view::npos) {
    return Error("attempting to load resource from non-local path");
  }

  if (auto it = resources.find(std::string(corrected_uri));
      it != resources.end()) {
    *view = it->second.StringView();
    return NoError();
  }

  missing_resource_name_from_path[JoinPath(
      directory, corrected_uri.substr(kDesiredPrefix.size()))] = corrected_uri;

  return NoError();
}
absl::Status ResolveResource(
    absl::string_view directory,
    tsl::robin_map<std::string, BufferAccess>& resources,
    tsl::robin_map<std::string, std::string>& missing_resource_name_from_path,
    std::vector<BufferAccess>& owned, absl::string_view uri,
    absl::string_view* view) {
  if (uri.empty() || !view->empty()) {
    return NoError();
  }
  if (absl::StartsWith(uri, "data:")) {
    return ResolveDataResource(owned, uri, view);
  }
  return ResolveUriResource(directory, resources,
                            missing_resource_name_from_path, uri, view);
}

OptionalError GetPartInfosFromNode(
    const GltfModel& model, const std::vector<Primitive>& primitives,
    GltfPrimitiveVector<ProcessedPrimitive>&& processed_primitives,
    LoadedModelBuilder* builder,
    std::vector<LoadedModelBuilder::PartData>* out_parts) {
  for (const Primitive& primitive : primitives) {
    size_t i = &primitive - &primitives.front();
    ProcessedPrimitive& processed = processed_primitives[i];

    // TRIANGLES is the default. glTF primitive.mode is an optional setting.
    using PrimitiveType = model::ModelData::PrimitiveType;
    PrimitiveType primitive_type = PrimitiveType::TRIANGLES;

    if (primitive.mode) {
      switch (*primitive.mode) {
        case imp::gltf::Primitive::LINES:
          primitive_type = PrimitiveType::LINES;
          break;
        case imp::gltf::Primitive::POINTS:
          primitive_type = PrimitiveType::POINTS;
          break;
        case imp::gltf::Primitive::TRIANGLES:
          primitive_type = PrimitiveType::TRIANGLES;
          break;
        default:
          return Error(
              "Unsupported mesh primitive mode found. Only POINTS, LINES, "
              "and TRIANGLES are supported.");
      }
    }

    const Material& gltf_material = model.GetMaterial(primitive.material);
    const Accessor& indices = model.GetAccessor(primitive.indices.value_or(0));

    using PartIndexCountType =
        decltype(model::ModelData::PartData::index_count);
    if (indices.count > std::numeric_limits<PartIndexCountType>::max()) {
      return Error("Too many indices on primitive");
    }

    size_t num_variants =
        model.Root().extensions.materials_variants.has_value()
            ? model.Root().extensions.materials_variants.value().variants.size()
            : 0;
    model::ModelData::MaterialsVariantsMappingLookup
        materials_variants_mappings(num_variants);

    if (primitive.extensions.materials_variants) {
      for (const auto& mapping :
           primitive.extensions.materials_variants->mappings) {
        LoadedModelBuilder::MaterialId pending_material =
            builder->GetMaterial(mapping.material);
        if (!pending_material) {
          return Error(
              "Invalid material specified in KHR_materials_variants mapping.");
        }

        auto variant_material =
            pending_material.CastTo<model::ModelData::MaterialId>();
        for (uint32_t variant_index : mapping.variants) {
          auto variant_id =
              model::ModelData::MaterialsVariantsId::At(variant_index);
          materials_variants_mappings[variant_id] = variant_material;
        }
      }
    }

    LoadedModelBuilder::MaterialId material =
        primitive.material ? builder->GetMaterial(*primitive.material)
                           : builder->GetMaterial(-1);

    out_parts->push_back(LoadedModelBuilder::PartData(
        std::string(gltf_material.name), 0,
        static_cast<PartIndexCountType>(indices.count),
        builder->AddVertexBuffer(std::move(processed.vertex_blocks),
                                 processed.vertex_count,
                                 static_cast<bool>(processed.skinning_buffer)),
        processed.index_buffer, material, primitive_type,
        std::move(materials_variants_mappings), processed.skinning_buffer,
        processed.morph_target_offset, processed.morph_target_count));
  }
  return NoError();
}

model::ModelData::RuntimeData PopulateRuntimeData(
    const GltfModel& model, const std::vector<Primitive>& primitives) {
  using RuntimeData = model::ModelData::RuntimeData;
  for (const Primitive& primitive : primitives) {
    if (model.GetMaterial(primitive.material).extensions.mask) {
      return RuntimeData::HighPriority();
    }
  }

  return RuntimeData::Default();
}

// TODO: When removing the legacy loader option, we can remove
// this function and use the local transform directly.
PreciseTransform GetExportTransform(const GltfLookup& lookup, NodeId node) {
  NodeId parent = lookup.parents[node];
  if (!parent || lookup.bones[parent] ||
      (lookup.self_flags[parent] & NodeFlags::kIsAnimated)) {
    return lookup.local_transforms[node];
  }

  mat4 parent_from_self = lookup.local_transforms[node].AsMat4();
  mat4f parent_from_self_rotation =
      Transform<float>(kZero3, lookup.local_transforms[node].rotation, kZero3)
          .AsMat4();

  std::vector<NodeId> included = {node};
  while (parent && !lookup.bones[parent] &&
         !(lookup.self_flags[parent] & NodeFlags::kIsAnimated)) {
    mat4 grand_parent_from_parent = lookup.local_transforms[parent].AsMat4();
    parent_from_self = grand_parent_from_parent * parent_from_self;

    mat4f grand_parent_from_parent_rotation =
        Transform<float>(float3(0.0f), lookup.local_transforms[parent].rotation,
                         float3(1.0f))
            .AsMat4();
    parent_from_self_rotation =
        grand_parent_from_parent_rotation * parent_from_self_rotation;

    included.push_back(parent);
    parent = lookup.parents[parent];
  }
  return PreciseTransform(parent_from_self);
}

absl::Status ConsumeSkeleton(const GltfLookup& lookup,
                             LoadedModelBuilder* builder) {
  builder->ReserveBones(lookup.bone_entries.size());
  for (const GltfLookup::BoneEntry& entry : lookup.bone_entries) {
    PreciseTransform local_transform = GetExportTransform(lookup, entry.node);
    std::string generated_name_storage;
    absl::string_view name = lookup.nodes[entry.node].name;
    if (name.empty()) {
      generated_name_storage =
          absl::StrFormat("Node[%d]", static_cast<int>(entry.node));
      name = generated_name_storage;
    }
    MP_RETURN_IF_ERROR(
        builder->AddBone(static_cast<uint16_t>(entry.children.size()),
                         local_transform, name, static_cast<int>(entry.node)));
  }

  return builder->FinishBones();
}

// Input data used to compute joint bounds on a skinned mesh primitive.
struct PrimitiveBoundsInputs {
  std::unique_ptr<AccessorReader> position;
  std::unique_ptr<AccessorReader> joints;
  std::unique_ptr<AccessorReader> weights;
};

template <typename JointType, typename WeightType>
OptionalError MergeBoundsFromPrimitive(
    PrimitiveBoundsInputs const& inputs,
    absl::Span<filament::Aabb> sampled_joint_index_to_bounds,
    BitVector& out_sampled_joint_in_use) {
  const DenseDataAccess position_data = inputs.position->GetData();
  const DenseDataAccess joints_data = inputs.joints->GetData();
  const DenseDataAccess weights_data = inputs.weights->GetData();

  for (auto i = 0u; i < inputs.position->GetCount(); ++i) {
    const float3& position = *position_data.At<float3>(i);
    const JointType& joints = *joints_data.At<JointType>(i);
    const WeightType& weights = *weights_data.At<WeightType>(i);
    // Joint info is required to be stored in vec4s per the glTF spec.
    auto static constexpr kMaxWeightedJointsPerVertex = 4u;
    auto static constexpr kNilWeight = typename WeightType::value_type{0};
    for (auto component_index = 0u;
         component_index < kMaxWeightedJointsPerVertex; ++component_index) {
      auto weight = weights[component_index];
      if (weight <= kNilWeight) continue;

      auto sampled_joint_index = joints[component_index];
      if (sampled_joint_index >= sampled_joint_index_to_bounds.size()) {
        return Error("Invalid joint index: %d", sampled_joint_index);
      }
      out_sampled_joint_in_use.Set(sampled_joint_index);

      auto& bounds = sampled_joint_index_to_bounds[sampled_joint_index];
      bounds.min = min(bounds.min, position);
      bounds.max = max(bounds.max, position);
    }
  }

  return NoError();
}

// Accumulates joint bounds for the skin, and stores them in out_bounds.
// out_bounds is expected to be large enough to store bounds for each joint,
// and to be uninitialized.
OptionalError GetSkinBounds(gltf::Gltf const& gltf, const GltfLookup& lookup,
                            gltf::Node const& node,
                            absl::Span<filament::Aabb> out_bounds,
                            BitVector& out_sampled_bone_in_use) {
  if (!node.mesh) {
    return Error("Missing skinning mesh on node: %.*s", node.name.size(),
                 node.name.data());
  }
  auto mesh_index = *node.mesh;

  if (mesh_index >= gltf.meshes.size()) {
    return Error("Invalid mesh index(%d) on node: %.*s", mesh_index,
                 node.name.size(), node.name.data());
  }
  auto& mesh = gltf.meshes[mesh_index];

  for (auto& primitive : mesh.primitives) {
    auto& attributes = primitive.attributes;
    auto position_iter = attributes.find("POSITION");
    auto joints_iter = attributes.find("JOINTS_0");
    auto weights_iter = attributes.find("WEIGHTS_0");
    if (position_iter == attributes.end() || joints_iter == attributes.end() ||
        weights_iter == attributes.end()) {
      return Error("Incomplete skinned mesh");
    }

    auto position_accessor_index = position_iter->second;
    auto joints_accessor_index = joints_iter->second;
    auto weights_accessor_index = weights_iter->second;

    auto primitive_bounds_inputs = PrimitiveBoundsInputs{};
    MP_ASSIGN_OR_RETURN(AccessorReader position,
                     AccessorReader::Create(gltf, position_accessor_index));
    primitive_bounds_inputs.position =
        std::make_unique<AccessorReader>(position);
    MP_ASSIGN_OR_RETURN(AccessorReader joints,
                     AccessorReader::Create(gltf, joints_accessor_index));
    primitive_bounds_inputs.joints = std::make_unique<AccessorReader>(joints);
    MP_ASSIGN_OR_RETURN(AccessorReader weights,
                     AccessorReader::Create(gltf, weights_accessor_index));
    primitive_bounds_inputs.weights = std::make_unique<AccessorReader>(weights);

    // Uses the appropriate merge function based on the joint component type.
    auto joints_component_type =
        primitive_bounds_inputs.joints->GetComponentType();
    auto weights_component_type =
        primitive_bounds_inputs.weights->GetComponentType();
    auto constexpr kComponentTypeUnsignedByte = 5121u;
    auto constexpr kComponentTypeUnsignedShort = 5123u;
    auto constexpr kComponentTypeFloat = 5126u;
    OptionalError merge_result = NoError();
    switch (joints_component_type) {
      case kComponentTypeUnsignedByte: {
        switch (weights_component_type) {
          case kComponentTypeUnsignedByte: {
            merge_result = MergeBoundsFromPrimitive<ubyte4, ubyte4>(
                primitive_bounds_inputs, out_bounds, out_sampled_bone_in_use);
            break;
          }
          case kComponentTypeFloat: {
            merge_result = MergeBoundsFromPrimitive<ubyte4, float4>(
                primitive_bounds_inputs, out_bounds, out_sampled_bone_in_use);
            break;
          }
          default:
            merge_result = Error("Invalid weights component type %d",
                                 weights_component_type);
        }
        break;
      }

      case kComponentTypeUnsignedShort:
        switch (weights_component_type) {
          case kComponentTypeUnsignedByte: {
            merge_result = MergeBoundsFromPrimitive<ushort4, ubyte4>(
                primitive_bounds_inputs, out_bounds, out_sampled_bone_in_use);
            break;
          }
          case kComponentTypeFloat: {
            merge_result = MergeBoundsFromPrimitive<ushort4, float4>(
                primitive_bounds_inputs, out_bounds, out_sampled_bone_in_use);
            break;
          }
          default:
            merge_result = Error("Invalid weights component type %d",
                                 weights_component_type);
        }
        break;

      default:
        merge_result =
            Error("Invalid joints component type: %d", joints_component_type);
    }
    MP_RETURN_IF_ERROR(merge_result);
  }

  return NoError();
}

absl::Status GetSkinInfoFromNode(
    const imp::gltf::Gltf& gltf, const GltfLookup& lookup,
    /*const PendingSkeleton& skeleton, */ NodeId node_id,
    std::vector<WeakSkinId>& skin_remap,
    LoadedModelBuilder* builder, /*PendingSkins* skins,*/
    model::ModelData::SkinId* out_skin, uint16_t* out_sampled_joint_count) {
  using BoneData = LoadedModelBuilder::BoneData;
  using BoneId = LoadedModelBuilder::BoneId;
  using BoneParentId = LoadedModelBuilder::BoneParentId;

  using BoneChildId = LoadedModelBuilder::BoneChildId;
  using JointData = LoadedModelBuilder::JointData;
  using JointId = LoadedModelBuilder::JointId;
  using WeakJointId = LoadedModelBuilder::WeakJointId;
  using JointParentId = LoadedModelBuilder::JointParentId;
  using JointChildId = LoadedModelBuilder::JointChildId;
  using SampledJointData = LoadedModelBuilder::SampledJointData;
  using WeakSampledJointId = LoadedModelBuilder::WeakSampledJointId;
  const TypedSetVector<BoneData>& bones = builder->Bones();
  auto& node = lookup.nodes[node_id];
  auto& skin = gltf.skins[*node.skin];
  size_t bone_count = bones.size();

  if (!skin_remap[*node.skin]) {
    LoadedModelBuilder::SampledJointLookup<mat4f> inverse_bind_poses;
    MP_RETURN_IF_ERROR(GetInverseBindPoses(gltf, skin, &inverse_bind_poses));

    GltfLookup::NodeLookup<WeakSampledJointId> sampled_joint_ids;
    sampled_joint_ids.resize(gltf.nodes.size());
    GltfLookup::NodeLookup<JointId> node_joint_ids;
    sampled_joint_ids.resize(gltf.nodes.size());
    TypedVector<LoadedModelBuilder::SampledJointData> sampled_joints;
    sampled_joints.reserve(skin.joints.size());
    PairedBitVector<BoneData> sampled_or_parent;
    PairedVector<WeakSampledJointId, BoneData> sampled_joint;
    PairedVector<WeakJointId, BoneData> joint;
    sampled_or_parent.Resize(bone_count);
    sampled_joint.resize(bone_count);
    joint.resize(bone_count);

    for (const uint32_t& joint_node_index : skin.joints) {
      auto joint_node = NodeId(static_cast<int>(joint_node_index));
      if (!lookup.bones.IsValid(joint_node)) {
        return Error("invalid skin1");
      }
      // Our lookup bones match our skeleton bones which match our joint bones.
      GltfLookup::BoneId joint_bone = lookup.bones[joint_node];
      if (!joint_bone)
        return Error(
            "Skin %d: sampled joint #%d points to node %d which is not a bone",
            node.skin.value(),
            static_cast<uint16_t>(&joint_node_index - &skin.joints.front()),
            static_cast<int>(joint_node));
      BoneId bone = joint_bone.CastTo<BoneId>();
      if (!bone) {
        IMP_LOG(imp::FATAL) << "Internal error";
      }

      auto bone_parents = bones.Span<BoneData::Fields::kParent>();
      sampled_joint[bone] =
          WeakSampledJointId::At(&joint_node_index - &skin.joints.front());
      for (BoneParentId iter = bone; iter; iter = bone_parents[iter]) {
        if (sampled_or_parent.Get(iter)) break;
        sampled_or_parent.Set(iter);
      }
    }

    TypedSetVector<JointData> joints;

    sampled_or_parent.ForEachBit<BoneId>(
        [&joint, &joints, &sampled_or_parent, &sampled_joint,
         bone_num_childrens = bones.Span<BoneData::Fields::kNumChildren>(),
         bone_first_children = bones.Span<BoneData::Fields::kFirstChild>(),
         bone_next_siblings =
             bones.Span<BoneData::Fields::kNextSibling>()](BoneId bone) {
          // correct num_children and append.

          uint16_t num_children = bone_num_childrens[bone];
          if (num_children) {
            BoneChildId child = bone_first_children[bone];
            while (child) {
              if (!sampled_or_parent.Get(child)) --num_children;
              child = bone_next_siblings[child];
            }
          }
          WeakSampledJointId weak_sampled_joint = sampled_joint[bone];

          joint[bone] =
              joints.Append(num_children, JointParentId(), JointChildId(),
                            JointChildId(), bone, weak_sampled_joint);
        });

    for (int joint_node_index : skin.joints) {
      auto joint_node = NodeId(joint_node_index);
      GltfLookup::BoneId joint_bone = lookup.bones[joint_node];
      BoneId bone = joint_bone.CastTo<BoneId>();
      sampled_joint_ids[joint_node] = sampled_joints.Append<WeakSampledJointId>(
          SampledJointData{joint[bone]});
    }

    *out_sampled_joint_count = sampled_joints.size();
    joints.reserve(bone_count);
    for (auto bone_id : bones.Ids<BoneId>()) {
      uint16_t bone_num_children = bones[bone_id].num_children;
      NodeId bone_node =
          lookup.bone_entries[bone_id.CastTo<GltfLookup::BoneId>()].node;
      joints.push_back(bone_num_children, JointParentId{}, JointChildId{},
                       JointChildId{}, bone_id, sampled_joint_ids[bone_node]);
    }

    // Which bone (regardless of 'bone' in our enclosing EntityInfo) is the root
    // transform when computing our sampled transforms.
    auto target_export = lookup.exports[node_id];
    auto skeleton_node = skin.skeleton ? NodeId::At(*skin.skeleton) : NodeId();
    auto skeleton_export =
        skeleton_node ? lookup.exports[skeleton_node] : GltfLookup::ExportId();
    using EntityId = model::ModelData::EntityId;
    auto target = target_export.CastTo<EntityId>();

    TypedDagTools<JointId>::ExpandGraph(joints.Span<JointData::kNumChildren>(),
                                        joints.Span<JointData::kParent>(),
                                        joints.Span<JointData::kFirstChild>(),
                                        joints.Span<JointData::kNextSibling>());

    TypedSetVector<model::ModelData::SkinnedEntityData> skinned_entities;

    LoadedModelBuilder::SampledJointLookup<filament::Aabb> bounds(
        skin.joints.size());
    PairedBitVector<SampledJointData> usage;
    usage.Resize(skin.joints.size());
    if (auto status =
            GetSkinBounds(gltf, lookup, node, absl::MakeSpan(bounds), usage);
        !status.ok()) {
      IMP_LOG(imp::ERROR) << "Error getting skin bounds: "
                 << std::setw(static_cast<int>(status.message().size()))
                 << status.message().data();
    }

    skinned_entities.push_back(target, std::move(bounds), std::move(usage));

    MP_ASSIGN_OR_RETURN(
        *out_skin,
        builder->AddSkin(model::ModelData::SkinData{
            .sampled_joints = std::move(sampled_joints),
            .inverse_bind_poses = std::move(inverse_bind_poses),
            .joints = std::move(joints),
            .skinned_entities = std::move(skinned_entities),
            .pose_root =
                skeleton_export
                    ? skeleton_export.CastTo<model::ModelData::WeakEntityId>()
                    : model::ModelData::WeakEntityId(),
        }));

    skin_remap[*node.skin] = *out_skin;

  } else {
    using EntityId = model::ModelData::EntityId;
    auto target_export = lookup.exports[node_id];
    auto target = target_export.CastTo<EntityId>();

    *out_sampled_joint_count = skin.joints.size();
    *out_skin = skin_remap[*node.skin];

    LoadedModelBuilder::SampledJointLookup<filament::Aabb> bounds(
        skin.joints.size());
    PairedBitVector<LoadedModelBuilder::SampledJointData> usage;
    usage.Resize(skin.joints.size());
    MP_RETURN_IF_ERROR(
        GetSkinBounds(gltf, lookup, node, absl::MakeSpan(bounds), usage));
    MP_RETURN_IF_ERROR(builder->AddSkinEntity(*out_skin, target, std::move(bounds),
                                           std::move(usage)));
  }

  return NoError();
}

OptionalError ProtoGltfProvider::TryParseGltf(LoaderState* state_ptr) {
  auto parsed = std::make_unique<ParsedGltf>();
  std::optional<imp::gltf::Gltf> gltf;
  MP_RETURN_IF_ERROR(
      provider_gltf::TryParseGltf(state_ptr->primary_resource_, gltf));
  parsed->gltf = std::move(gltf.value());

  MP_RETURN_IF_ERROR(ResolveResources(state_ptr->directory_, state_ptr->resources_,
                                   state_ptr->missing_resource_name_from_path_,
                                   parsed->gltf, parsed->owned));

  parsed_gltf_ = std::move(parsed);
  return NoError();
}

bool ProtoGltfProvider::IsParsed() { return parsed_gltf_ != nullptr; }

bool ProtoGltfProvider::HasPendingResources(LoaderState* state_ptr) {
  const auto& gltf = parsed_gltf_->gltf;
  return absl::c_any_of(gltf.buffers,
                        [](const imp::gltf::Buffer& buffer) {
                          return !buffer.uri.empty() && buffer.access.empty();
                        }) ||
         absl::c_any_of(gltf.images,
                        [](const imp::gltf::Image& image) {
                          return !image.uri.empty() && image.access.empty();
                        }) ||
         (gltf.extensions.audio_extension &&
          absl::c_any_of(gltf.extensions.audio_extension->audio,
                         [](const imp::gltf::AudioExtension::Audio& audio) {
                           return !audio.uri.empty() && audio.access.empty();
                         }));
}

OptionalError AddMissingAttributes(
    LoadedModelBuilder& builder,
    GltfPrimitiveVector<ProcessedPrimitive>* primitives) {
  for (ProcessedPrimitive& primitive : *primitives) {
    VertexAttributeMask missing_attributes;
    for (auto material_index : primitive.required_materials) {
      LoadedModelBuilder::MaterialId material_id =
          builder.GetMaterial(material_index);
      auto required_attributes = builder.GetRequiredAttributes(material_id);
      // Non-generic materials have no required attributes to fill.
      if (!required_attributes.has_value()) {
        continue;
      }

      for (uint8_t attribute_index = static_cast<uint8_t>(VertexAttribute::MIN);
           attribute_index <= static_cast<uint8_t>(VertexAttribute::MAX);
           attribute_index++) {
        auto attribute = static_cast<VertexAttribute>(attribute_index);
        if (required_attributes->Test(attribute)) {
          if (!absl::c_any_of(
                  primitive.vertex_blocks,
                  [&attribute](const LoadedModelBuilder::VertexBlock& block) {
                    return absl::c_any_of(
                        block.attributes,
                        [&attribute](const schemas::VertexAttributeInfo&
                                         block_attribute_info) {
                          auto block_attribute =
                              block_attribute_info.attribute();
                          switch (attribute) {
                            case VertexAttribute::POSITION: {
                              return block_attribute ==
                                     schemas::VertexAttribute::POSITION;
                            }
                            case VertexAttribute::NORMAL: {
                              return block_attribute ==
                                     schemas::VertexAttribute::TANGENTS;
                            }
                            case VertexAttribute::TANGENT: {
                              return block_attribute ==
                                     schemas::VertexAttribute::TANGENTS;
                            }
                            case VertexAttribute::TEXCOORD_0: {
                              return block_attribute ==
                                     schemas::VertexAttribute::UV0;
                            }
                            case VertexAttribute::TEXCOORD_1: {
                              return block_attribute ==
                                     schemas::VertexAttribute::UV1;
                            }
                            case VertexAttribute::COLOR_0: {
                              return block_attribute ==
                                     schemas::VertexAttribute::COLOR;
                            }
                            case VertexAttribute::JOINTS_0: {
                              return block_attribute ==
                                     schemas::VertexAttribute::BONE_INDICES;
                            }
                            case VertexAttribute::WEIGHTS_0: {
                              return block_attribute ==
                                     schemas::VertexAttribute::BONE_WEIGHTS;
                            }
                            case VertexAttribute::MORPH_POSITION_0: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_POSITION_0;
                            }
                            case VertexAttribute::MORPH_NORMAL_0: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_TANGENTS_0;
                            }
                            case VertexAttribute::MORPH_TANGENT_0: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_TANGENTS_0;
                            }

                            case VertexAttribute::MORPH_POSITION_1: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_POSITION_1;
                            }
                            case VertexAttribute::MORPH_NORMAL_1: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_TANGENTS_1;
                            }
                            case VertexAttribute::MORPH_TANGENT_1: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_TANGENTS_1;
                            }

                            case VertexAttribute::MORPH_POSITION_2: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_POSITION_2;
                            }
                            case VertexAttribute::MORPH_NORMAL_2: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_TANGENTS_2;
                            }
                            case VertexAttribute::MORPH_TANGENT_2: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_TANGENTS_2;
                            }

                            case VertexAttribute::MORPH_POSITION_3: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_POSITION_3;
                            }
                            case VertexAttribute::MORPH_NORMAL_3: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_TANGENTS_3;
                            }
                            case VertexAttribute::MORPH_TANGENT_3: {
                              return block_attribute ==
                                     schemas::VertexAttribute::MORPH_TANGENTS_3;
                            }
                          }
                        });
                  })) {
            missing_attributes.Set(attribute);
          }
        }
      }
    }
    if (missing_attributes.Any()) {
      // Conjure a new zero'd out block that provides the missing attributes.
      std::vector<schemas::VertexAttributeInfo> attributes;
      uint32_t stride = 0;
      for (uint8_t attribute_index = static_cast<uint8_t>(VertexAttribute::MIN);
           attribute_index <= static_cast<uint8_t>(VertexAttribute::MAX);
           attribute_index++) {
        auto attribute = static_cast<VertexAttribute>(attribute_index);
        if (!missing_attributes.Test(attribute)) {
          continue;
        }
        switch (attribute) {
          case VertexAttribute::MORPH_POSITION_0:
          case VertexAttribute::MORPH_POSITION_1:
          case VertexAttribute::MORPH_POSITION_2:
          case VertexAttribute::MORPH_POSITION_3:
          case VertexAttribute::MORPH_NORMAL_0:
          case VertexAttribute::MORPH_NORMAL_1:
          case VertexAttribute::MORPH_NORMAL_2:
          case VertexAttribute::MORPH_NORMAL_3:
          case VertexAttribute::POSITION:
          case VertexAttribute::JOINTS_0:
          case VertexAttribute::WEIGHTS_0:
          case VertexAttribute::NORMAL: {
            return Error("Don't know how to conjure missing attribute %d",
                         static_cast<int>(attribute_index));
            break;
          }
          case VertexAttribute::MORPH_TANGENT_0:
          case VertexAttribute::MORPH_TANGENT_1:
          case VertexAttribute::MORPH_TANGENT_2:
          case VertexAttribute::MORPH_TANGENT_3: {
            attributes.push_back(schemas::VertexAttributeInfo(
                GetVertexAttribute(attribute), schemas::AttributeType::UBYTE4,
                stride, true));
            stride += sizeof(ubyte4);
            break;
          }
          case VertexAttribute::TANGENT: {
            attributes.push_back(schemas::VertexAttributeInfo(
                schemas::VertexAttribute::TANGENTS,
                schemas::AttributeType::UBYTE4, stride, true));
            stride += sizeof(ubyte4);
            break;
          }
          case VertexAttribute::TEXCOORD_0: {
            attributes.push_back(schemas::VertexAttributeInfo(
                schemas::VertexAttribute::UV0, schemas::AttributeType::USHORT2,
                stride, true));
            stride += sizeof(ushort2);
            break;
          }
          case VertexAttribute::TEXCOORD_1: {
            attributes.push_back(schemas::VertexAttributeInfo(
                schemas::VertexAttribute::UV1, schemas::AttributeType::USHORT2,
                stride, true));
            stride += sizeof(ushort2);
            break;
          }
          case VertexAttribute::COLOR_0: {
            attributes.push_back(schemas::VertexAttributeInfo(
                schemas::VertexAttribute::COLOR, schemas::AttributeType::UBYTE4,
                stride, true));
            stride += sizeof(ubyte4);
            break;
          }
        }
      }
      if (!attributes.empty()) {
        if (stride == 0 || (stride % 4) != 0) {
          return Error("Computed an invalid stride");
        }

        BufferAccess fake_attributes;
        const auto fake_attributes_size = primitive.vertex_count * stride;
        uint8_t* fake_attributes_data =
            BufferAccess::Create(fake_attributes_size, &fake_attributes);
        if (!fake_attributes_data) {
          return Error("Failed to allocate buffer!");
        }
        // Sets placeholder value to 0xFF so that vertex color data will be
        // interpreted as white.
        memset(fake_attributes_data, 0xFF, fake_attributes_size);
        primitive.vertex_blocks.push_back(LoadedModelBuilder::VertexBlock(
            std::move(attributes), std::move(fake_attributes), stride));
      }
    }
  }
  return NoError();
}

absl::StatusOr<FlatBufferAccess<schemas::LoadedModel>>
ProtoGltfProvider::TryLoadGltf(LoaderState* state_ptr) {
  imp::gltf::Gltf& gltf = parsed_gltf_->gltf;
  GltfModel model(&gltf);

  // Re-create the builder on each attempt.
  builder_ = std::make_unique<LoadedModelBuilder>();

  const int scene_index = gltf.scene;
  if (gltf.scenes.size() <= scene_index) {
    return Error("Gltf file did not specify scene; nothing to load.");
  }
  const Scene& scene = gltf.scenes[scene_index];

  MP_RETURN_IF_ERROR(ResolveResources(state_ptr->directory_, state_ptr->resources_,
                                   state_ptr->missing_resource_name_from_path_,
                                   parsed_gltf_->gltf, parsed_gltf_->owned));
  if (HasPendingResources(state_ptr)) {
    return Error("Resources still pending");
  }

  MP_ASSIGN_OR_RETURN(BufferAccess decoded_data,
                   extensions::ResolveMeshOpt(&gltf));
  if (!decoded_data.Empty()) {
    parsed_gltf_->owned.push_back(std::move(decoded_data));
  }
  MP_ASSIGN_OR_RETURN(std::vector<BufferAccess> draco_buffers,
                   extensions::ResolveDraco(&gltf));
  for (BufferAccess& buffer : draco_buffers) {
    parsed_gltf_->owned.push_back(std::move(buffer));
  }

  GltfLookup lookup;
  MP_RETURN_IF_ERROR(BuildGltfLookup(gltf, scene, state_ptr->options_, &lookup));

  // Build the skeleton for our scene.
  MP_RETURN_IF_ERROR(ConsumeSkeleton(lookup, builder_.get()));

  if (gltf.extensions.lights_punctual) {
    for (const auto& light : gltf.extensions.lights_punctual->lights) {
      static constexpr absl::string_view kDirectionalString = "directional";
      static constexpr absl::string_view kPointString = "point";
      static constexpr absl::string_view kSpotString = "spot";
      static constexpr float kDefaultIntensity = 1;
      // Range is meant to be infinite if unspecified:
      // https://github.com/KhronosGroup/glTF/tree/master/extensions/2.0/Khronos/KHR_lights_punctual#light-shared-properties
      static constexpr float kDefaultRange = 1000;

      schemas::Color color;
      schemas::LightPunctualType type;
      schemas::SpotConeAngles spot_cone_angles;

      if (light.type == kDirectionalString) {
        type = schemas::LightPunctualType::DIRECTIONAL;
      } else if (light.type == kPointString) {
        type = schemas::LightPunctualType::POINT;
      } else if (light.type == kSpotString) {
        static constexpr float kDefaultInner = 0;
        static constexpr float kDefaultOuter = filament::math::F_PI_2;
        type = schemas::LightPunctualType::SPOT;
        spot_cone_angles =
            light.spot
                ? schemas::SpotConeAngles(
                      light.spot->inner_cone_angle.value_or(kDefaultInner),
                      light.spot->outer_cone_angle.value_or(kDefaultOuter))
                : schemas::SpotConeAngles(kDefaultInner, kDefaultOuter);
      } else {
        return Error("Invalid punctual light type: %.*s", light.type.size(),
                     light.type.data());
      }
      if (light.color.empty()) {
        color = schemas::Color(1, 1, 1);
      } else if (light.color.size() == 3) {
        color = schemas::Color(light.color[0], light.color[1], light.color[2]);
      } else {
        return Error("Invalid number of color components in punctual light");
      }
      builder_->AddLightPunctual(
          light.name, color, light.intensity.value_or(kDefaultIntensity), type,
          light.range.value_or(kDefaultRange), spot_cone_angles);
    }
  }

  if (gltf.extensions.materials_variants) {
    for (const auto& variant : gltf.extensions.materials_variants->variants) {
      builder_->AddMaterialsVariants(variant.name);
    }
  }

  if (gltf.extensions.audio_extension) {
    LoadedModelBuilder::AudioEmitterOffsets audio_emitter_offsets;
    LoadedModelBuilder::AudioSourceOffsets audio_source_offsets;
    LoadedModelBuilder::AudioOffsets audio_offsets;
    std::vector<uint16_t> scene_emitters;

    for (const auto& emitter : gltf.extensions.audio_extension->emitters) {
      schemas::AudioEmitterType emitter_type =
          schemas::AudioEmitterType::GLOBAL;
      if (emitter.type == "global") {
        emitter_type = schemas::AudioEmitterType::GLOBAL;
      } else if (emitter.type == "positional") {
        emitter_type = schemas::AudioEmitterType::POSITIONAL;
      } else {
        return Error("Invalid emitter type: %s", emitter.type);
      }

      std::vector<uint16_t> indices;
      indices.reserve(emitter.sources.size());
      for (uint16_t index : emitter.sources) {
        indices.push_back(index);
      }

      // Default values from
      // https://github.com/omigroup/gltf-extensions/blob/main/extensions/2.0/KHR_audio/schema/emitter.schema.json
      audio_emitter_offsets.push_back(builder_->AddAudioEmitter(
          emitter.name, emitter_type, emitter.gain.value_or(1.0), indices));
    }

    // Default values from
    // https://github.com/omigroup/gltf-extensions/blob/main/extensions/2.0/KHR_audio/schema/source.schema.json
    for (const auto& source : gltf.extensions.audio_extension->sources) {
      audio_source_offsets.push_back(builder_->AddAudioSource(
          source.name, source.auto_play.value_or(false),
          source.gain.value_or(1.0), source.loop.value_or(false),
          static_cast<uint16_t>(source.audio)));
    }

    for (const auto& audio : gltf.extensions.audio_extension->audio) {
      BufferAccess audio_data;
      if (audio.buffer_view) {
        absl::StatusOr<BufferAccess> audio_buffer =
            BufferAccessFromBufferView(gltf, *audio.buffer_view);

        if (!audio_buffer.ok()) {
          return audio_buffer.status();
        }

        audio_data = std::move(*audio_buffer);
      } else {
        audio_data = BufferAccess::Wrap(
            reinterpret_cast<const uint8_t*>(audio.access.data()),
            audio.access.size());
      }
      audio_offsets.push_back(builder_->AddAudio(audio_data));
    }

    if (scene.extensions.audio_extension) {
      for (uint32_t scene_audio_emitter :
           scene.extensions.audio_extension->emitters) {
        scene_emitters.push_back(static_cast<uint16_t>(scene_audio_emitter));
      }
    }

    builder_->AddAudioExtension(audio_emitter_offsets, audio_source_offsets,
                                audio_offsets, scene_emitters);
  }

  // The gltf behavior extension listens to the Behavior message being visited
  // and returns if it's not supported, so Behavior will already have been
  // created. We need to check if it contains anything, in this case knowing if
  // there's any nodes will suffice.
  if (gltf.extensions.behavior && !gltf.extensions.behavior->nodes.empty()) {
    std::unique_ptr<BehaviorLoaderExtension> behavior_loader_extension =
        builder_->CreateBehaviorLoaderExtension();
    absl::StatusOr<BehaviorOffset> behavior_offset =
        behavior_loader_extension->AddBehavior(*gltf.extensions.behavior);
    if (!behavior_offset.ok()) {
      return behavior_offset.status();
    }

    builder_->AddBehavior(*std::move(behavior_offset));
  }

  if (gltf.extensions.interactivity &&
      !gltf.extensions.interactivity->graphs.empty()) {
    std::unique_ptr<InteractivityLoaderExtension>
        interactivity_loader_extension =
            builder_->CreateInteractivityLoaderExtension();
    absl::StatusOr<InteractivityOffset> interactivity_offset =
        interactivity_loader_extension->AddInteractivity(
            *gltf.extensions.interactivity);
    if (!interactivity_offset.ok()) {
      return interactivity_offset.status();
    }

    builder_->AddInteractivity(*std::move(interactivity_offset));
  }

  filament::Box scene_bounds = NilBounds();
  size_t entry_count = lookup.export_entries.size();
  const auto entry_nodes =
      lookup.export_entries.Span<GltfLookup::ExportEntry::kNode>();
  const auto entry_child_counts =
      lookup.export_entries.Span<GltfLookup::ExportEntry::kNumChildren>();
  builder_->ReserveEntities(entry_count);
  auto bone_root_transforms = builder_->BoneRootTransforms();

  std::vector<WeakSkinId> skin_remap(gltf.skins.size());

  for (auto entry : lookup.export_entries.Ids<GltfLookup::ExportId>()) {
    using PartData = model::ModelData::PartData;
    using RuntimeData = model::ModelData::RuntimeData;
    NodeId node = entry_nodes[entry];
    GltfLookup::BoneId bone = lookup.bones[node];
    LoadedModelBuilder::LightPunctualId light_punctual;
    model::ModelData::AudioEmitterId audio_emitter;
    model::ModelData::SkinId skin;
    model::ModelData::MorphTargetBufferId morph_target_buffer;
    std::vector<PartData> parts;
    std::optional<filament::Box> bounds;
    std::optional<RuntimeData> runtime;
    absl::string_view name;
    std::vector<float> morph_target_weights;
    uint16_t sampled_joint_count = 0;
    std::optional<model::ModelData::NodeVisibility> node_visibility;
    std::optional<model::ModelData::NodeSelectability> node_selectability;
    std::optional<model::ModelData::NodeHoverability> node_hoverability;

    // Get morph_target_weights from weights set on the node.
    if (!lookup.nodes[node].weights.empty()) {
      if (lookup.nodes[node].weights.size() > filament::MAX_MORPH_TARGETS) {
        IMP_LOG(imp::WARNING) << lookup.nodes[node].weights.size()
                     << " default weights for morph targets were specified on "
                        "a node, but only "
                     << filament::MAX_MORPH_TARGETS
                     << " morph targets can be supported.";
      }
      for (size_t i = 0, count = std::min(lookup.nodes[node].weights.size(),
                                          filament::MAX_MORPH_TARGETS);
           i < count; ++i) {
        morph_target_weights.push_back(lookup.nodes[node].weights[i]);
      }
    }

    if (lookup.self_flags[node] & NodeFlags::kHasSkin) {
      MP_RETURN_IF_ERROR(GetSkinInfoFromNode(gltf, lookup, node, skin_remap,
                                          builder_.get(), &skin,
                                          &sampled_joint_count));
    }

    if (lookup.self_flags[node] & NodeFlags::kHasLightPunctual) {
      auto light_index = *lookup.nodes[node].extensions.lights_punctual->light;
      light_punctual = LoadedModelBuilder::LightPunctualId::At(light_index);
    }

    if (lookup.self_flags[node] & NodeFlags::kHasAudioEmitter) {
      uint16_t audio_emitter_index =
          *lookup.nodes[node].extensions.audio_extension->emitter;
      audio_emitter = model::ModelData::AudioEmitterId::At(audio_emitter_index);
    }

    if (lookup.self_flags[node] & NodeFlags::kHasMesh) {
      int mesh = *lookup.nodes[node].mesh;
      Gltf2AttributeMask attribute_mask;
      const std::vector<Primitive>& mesh_primitives =
          gltf.meshes[mesh].primitives;

      GltfPrimitiveVector<ProcessedPrimitive> processed_primitives;
      filament::Box mesh_bounds;
      const mat4& mesh_transform =
          bone_root_transforms[bone.CastTo<LoadedModelBuilder::BoneId>()];

      // Assign morph_target_weights from mesh weights if not set already.
      if (morph_target_weights.empty() && !gltf.meshes[mesh].weights.empty()) {
        if (gltf.meshes[mesh].weights.size() > filament::MAX_MORPH_TARGETS) {
          IMP_LOG(imp::WARNING) << gltf.meshes[mesh].weights.size()
                       << " default weights for morph targets were specified "
                          "on a mesh, but only "
                       << filament::MAX_MORPH_TARGETS
                       << " morph targets can be supported.";
        }

        for (size_t i = 0, count = std::min(gltf.meshes[mesh].weights.size(),
                                            filament::MAX_MORPH_TARGETS);
             i < count; ++i) {
          morph_target_weights.push_back(gltf.meshes[mesh].weights[i]);
        }
      }

      MP_RETURN_IF_ERROR(ProcessPrimitives(gltf, mesh_primitives, mesh_transform,
                                        sampled_joint_count, builder_.get(),
                                        &processed_primitives, &mesh_bounds,
                                        &morph_target_buffer));

      // Process textures must happen first since materials record entries which
      // will cause later texture processing to be skipped.
      MP_RETURN_IF_ERROR(ProcessTextureInfoFromNode(
          *builder_, model, mesh_primitives, processed_primitives,
          attribute_mask, state_ptr->options_.compression_type,
          state_ptr->options_.use_lite_materials));

      // Create generic material schemas for all materials used by the model.
      MP_RETURN_IF_ERROR(CreateGenericMaterialSchemas(
          *builder_, model, mesh_primitives, processed_primitives,
          attribute_mask, state_ptr->options_.use_lite_materials));

      MP_RETURN_IF_ERROR(AddMissingAttributes(*builder_, &processed_primitives));

      MP_RETURN_IF_ERROR(GetPartInfosFromNode(model, mesh_primitives,
                                           std::move(processed_primitives),
                                           builder_.get(), &parts));

      bounds = mesh_bounds;
      scene_bounds.unionSelf(TransformBounds(mesh_bounds, mesh_transform));
      name = gltf.meshes[mesh].name;

      runtime.emplace(PopulateRuntimeData(model, mesh_primitives));
    }

    if (lookup.self_flags[node] & NodeFlags::kHasKhrVisibility) {
      node_visibility = model::ModelData::NodeVisibility{
          .visible = lookup.nodes[node].extensions.khr_visibility->visible};
    }

    if (lookup.self_flags[node] & NodeFlags::kHasKhrNodeVisibility) {
      node_visibility = model::ModelData::NodeVisibility{
          .visible =
              lookup.nodes[node].extensions.khr_node_visibility->visible};
    }

    if (lookup.self_flags[node] & NodeFlags::kHasKhrNodeVisibility &
        NodeFlags::kHasKhrVisibility) {
      IMP_LOG(imp::WARNING) << "Node " << name
                   << " has both KHR_visibility and KHR_node_visibility "
                      "extensions. KHR_node_visibility will be used.";
    }

    if (lookup.self_flags[node] & NodeFlags::kHasKhrNodeSelectability) {
      node_selectability = model::ModelData::NodeSelectability{
          .selectable =
              lookup.nodes[node].extensions.khr_node_selectability->selectable};
    }

    if (lookup.self_flags[node] & NodeFlags::kHasKhrNodeHoverability) {
      node_hoverability = model::ModelData::NodeHoverability{
          .hoverable =
              lookup.nodes[node].extensions.khr_node_hoverability->hoverable};
    }

    MP_ASSIGN_OR_RETURN(
        LoadedModelBuilder::EntityId entity,
        builder_->AddEntity(
            bone.CastTo<LoadedModelBuilder::BoneId>(), skin,
            morph_target_buffer, std::move(morph_target_weights),
            light_punctual.CastTo<imp::model::ModelData::LightPunctualId>(),
            audio_emitter, std::move(parts), bounds, runtime,
            entry_child_counts[entry], name, static_cast<int>(node),
            node_visibility, node_selectability, node_hoverability));
    (void)(entity);
  }

  if (state_ptr->options_.remove_shadow_planes) {
    builder_->RemoveShadowPlanes(scene_bounds);
  }
  MP_RETURN_IF_ERROR(builder_->FinishEntities());
  MP_RETURN_IF_ERROR(builder_->FinishSkins());

  for (const Animation& animation : lookup.animations) {
    FlatBufferAccess<animation::schemas::GltfAnimation> fb_storage;
    MP_ASSIGN_OR_RETURN(
        fb_storage, animation::GetAnimation(gltf, lookup,
                                            lookup.animations.IdOf(animation)));
    MP_ASSIGN_OR_RETURN(LoadedModelBuilder::AnimationId animation_id,
                     builder_->AddAnimation(std::move(fb_storage)));
    (void)(animation_id);
  }

  return builder_->Finish();
}

}  // namespace

std::unique_ptr<GltfProvider> CreateGltfProvider() {
  return std::make_unique<ProtoGltfProvider>();
}

absl::Status ResolveResources(
    absl::string_view directory,
    tsl::robin_map<std::string, BufferAccess>& resources,
    tsl::robin_map<std::string, std::string>& missing_resource_name_from_path,
    imp::gltf::Gltf& gltf, std::vector<BufferAccess>& owned) {
  // Resolve buffer URIs.
  for (auto& buffer : gltf.buffers) {
    MP_RETURN_IF_ERROR(ResolveResource(directory, resources,
                                    missing_resource_name_from_path, owned,
                                    buffer.uri, &buffer.access));

    if (!buffer.access.empty() && buffer.access.size() < buffer.byte_length) {
      return Error("buffer_length must be <= data size");
    }
    // Correct for uninitialized byte_length.
    if (!buffer.access.empty() && buffer.byte_length == 0) {
      buffer.byte_length = buffer.access.size();
    }
  }
  // Resolve image URIs.
  for (auto& image : gltf.images) {
    MP_RETURN_IF_ERROR(ResolveResource(directory, resources,
                                    missing_resource_name_from_path, owned,
                                    image.uri, &image.access));
  }

  // Resolve audio URIs.
  if (gltf.extensions.audio_extension) {
    for (auto& audio : gltf.extensions.audio_extension->audio) {
      MP_RETURN_IF_ERROR(ResolveResource(directory, resources,
                                      missing_resource_name_from_path, owned,
                                      audio.uri, &audio.access));
    }
  }
  return absl::OkStatus();
}

}  // namespace imp::loader::details::provider_gltf
