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

#include "core/loader/provider/details/loaded_model_builder.h"

#include <sys/types.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/libs/math/include/math/mat4.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"
#include "core/common/bit_vector.h"
#include "core/common/buffer_access.h"
#include "core/common/data_helpers.h"
#include "core/common/filament_helpers.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/paired_span.h"
#include "core/common/schemas/math_generated.h"
#include "core/common/schemas/render_generated.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_tree.h"
#include "core/image/image_contents.h"
#include "core/loader/provider/details/vertex_attribute.h"
#include "core/loader/provider/extensions/gltf_extension_interactivity.h"
#include "core/loader/provider/extensions/interactivity/loader_extension.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/generic_material_spec.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/math/flatbuffer_support.h"
#include "core/math/mat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/model/entity_data.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details {
namespace {
using ::flatbuffers::FlatBufferBuilder;
using ::flatbuffers::String;
template <typename T>
using Offset = ::flatbuffers::Offset<T>;
template <typename T>
using Vector = ::flatbuffers::Vector<T>;

bool VertexAttributesMatch(const schemas::VertexAttributeInfo& lhs,
                           const schemas::VertexAttributeInfo& rhs) {
  return lhs.attribute() == rhs.attribute() && lhs.type() == rhs.type() &&
         lhs.offset() == rhs.offset() && lhs.normalized() == rhs.normalized();
}

bool VertexBlocksMatch(const LoadedModelBuilder::VertexBlock& lhs,
                       const LoadedModelBuilder::VertexBlock& rhs) {
  if (!absl::c_equal(lhs.attributes, rhs.attributes, VertexAttributesMatch))
    return false;
  if (lhs.stride != rhs.stride) return false;
  if (lhs.buffer.Size() != rhs.buffer.Size()) return false;
  if (0 != memcmp(lhs.buffer.Data(), rhs.buffer.Data(), lhs.buffer.Size()))
    return false;
  return true;
}

Offset<schemas::BoundsInfo> CreateBoundsInfo(
    FlatBufferBuilder& fbb, const std::optional<filament::Box>& bounds) {
  if (bounds) {
    const auto center = bounds->center;
    const auto half_extent = bounds->halfExtent;
    imp::schemas::Box box(flatbuffers::Pack(center),
                          flatbuffers::Pack(half_extent));

    return schemas::CreateBoundsInfo(fbb, &box);
  } else {
    return Offset<schemas::BoundsInfo>{};
  }
}

Offset<schemas::RuntimeInfo> CreateRuntimeInfo(
    FlatBufferBuilder& fbb,
    const std::optional<imp::model::ModelData::RuntimeData>& runtime) {
  if (runtime) {
    return schemas::CreateRuntimeInfo(
        fbb, static_cast<schemas::RenderInfoFlags>(runtime->flags.RawValue()),
        runtime->priority);
  } else {
    return Offset<schemas::RuntimeInfo>{};
  }
}

Offset<schemas::NodeVisibility> CreateNodeVisibility(
    FlatBufferBuilder& fbb,
    const std::optional<imp::model::ModelData::NodeVisibility>& visibility) {
  if (visibility) {
    if (visibility->visible.has_value()) {
      schemas::Bool visible(visibility->visible.value());
      return schemas::CreateNodeVisibility(fbb, &visible);
    }
    return schemas::CreateNodeVisibility(fbb);
  } else {
    return Offset<schemas::NodeVisibility>{};
  }
}

Offset<schemas::NodeSelectability> CreateNodeSelectability(
    FlatBufferBuilder& fbb,
    const std::optional<imp::model::ModelData::NodeSelectability>&
        selectability) {
  if (selectability) {
    if (selectability->selectable.has_value()) {
      schemas::Bool selectable(selectability->selectable.value());
      return schemas::CreateNodeSelectability(fbb, &selectable);
    }
    return schemas::CreateNodeSelectability(fbb);
  } else {
    return Offset<schemas::NodeSelectability>{};
  }
}

Offset<schemas::NodeHoverability> CreateNodeHoverability(
    FlatBufferBuilder& fbb,
    const std::optional<imp::model::ModelData::NodeHoverability>&
        hoverability) {
  if (hoverability) {
    if (hoverability->hoverable.has_value()) {
      schemas::Bool hoverable(hoverability->hoverable.value());
      return schemas::CreateNodeHoverability(fbb, &hoverable);
    }
    return schemas::CreateNodeHoverability(fbb);
  } else {
    return Offset<schemas::NodeHoverability>{};
  }
}

}  // namespace

LoadedModelBuilder::VertexBufferId LoadedModelBuilder::AddVertexBuffer(
    std::vector<VertexBlock> blocks, size_t vertex_count,
    bool advanced_skinning) {
  auto existing = absl::c_find_if(
      vertex_buffers_, [&blocks = blocks, vertex_count, advanced_skinning](
                           const VertexBuffer& existing_vertex_buffer) {
        if (existing_vertex_buffer.vertex_count != vertex_count ||
            existing_vertex_buffer.advanced_skinning != advanced_skinning)
          return false;

        return absl::c_equal(blocks, existing_vertex_buffer.blocks,
                             VertexBlocksMatch);
      });
  if (existing != vertex_buffers_.end()) {
    return vertex_buffers_.IdOf<VertexBufferId>(*existing);
  }
  return vertex_buffers_.Append<VertexBufferId>(
      VertexBuffer(std::move(blocks), vertex_count, advanced_skinning));
}

LoadedModelBuilder::MorphTargetBufferId
LoadedModelBuilder::AddMorphTargetBuffer(
    std::vector<LoadedModelBuilder::MorphTargetBlock> blocks,
    size_t vertex_count) {
  std::vector<Offset<schemas::MorphTargetAttributeInfo>> block_offsets(
      blocks.size());
  absl::c_transform(
      blocks, block_offsets.data(), [this](const MorphTargetBlock& block) {
        return schemas::CreateMorphTargetAttributeInfo(
            fbb_,
            fbb_.CreateVector(block.positions.Data(), block.positions.Size()),
            fbb_.CreateVector(block.tangents.Data(), block.tangents.Size()),
            block.texcoords0.Empty()
                ? 0
                : fbb_.CreateVector(block.texcoords0.Data(),
                                    block.texcoords0.Size()));
      });
  return morph_target_buffer_offsets_.Append<MorphTargetBufferId>(
      schemas::CreateMorphTargetBufferInfo(
          fbb_, fbb_.CreateVector(block_offsets), vertex_count));
}

LoadedModelBuilder::SkinningBufferId LoadedModelBuilder::AddSkinningBuffer(
    const std::vector<float2>& bone_indices_and_weights) {
  return skinning_buffer_offsets_.Append<SkinningBufferId>(
      schemas::CreateSkinningBufferInfo(
          fbb_, fbb_.CreateVectorOfNativeStructs<schemas::Float2>(
                    bone_indices_and_weights.data(),
                    bone_indices_and_weights.size())));
}

LoadedModelBuilder::IndexBufferId LoadedModelBuilder::AddIndexBuffer(
    std::optional<DenseDataAccess>& indices_data, size_t index_count) {
  BufferAccess indices_access;
  schemas::IndexType type;
  if (!indices_data) {
    // Generate default triangulation if none provided.
    size_t conjured_index_count = index_count;
    bool use_32bit_indices = conjured_index_count > kMaxValue<uint16_t>;
    size_t conjured_stride =
        use_32bit_indices ? sizeof(uint32_t) : sizeof(uint16_t);
    uint8_t* conjured_indices_ptr = BufferAccess::Create(
        conjured_stride * conjured_index_count, &indices_access);
    if (use_32bit_indices) {
      uint32_t* conjured_indices =
          reinterpret_cast<uint32_t*>(conjured_indices_ptr);
      std::iota(conjured_indices, conjured_indices + conjured_index_count, 0);
    } else {
      uint16_t* conjured_indices =
          reinterpret_cast<uint16_t*>(conjured_indices_ptr);
      std::iota(conjured_indices, conjured_indices + conjured_index_count, 0);
    }
    type = use_32bit_indices ? schemas::IndexType::UINT
                             : schemas::IndexType::USHORT;
  } else {
    // Special case for byte layout
    if (indices_data->GetStride() == 1) {
      uint16_t* new_index_data =
          reinterpret_cast<uint16_t*>(BufferAccess::Create(
              sizeof(uint16_t) * indices_data->GetCount(), &indices_access));
      std::copy_n(indices_data->ReadRawData<uint8_t>(),
                  indices_data->GetCount(), new_index_data);
    } else {
      indices_access = BufferAccess::Wrap(
          indices_data->ReadRawData<uint8_t>(),
          indices_data->GetCount() * indices_data->GetStride());
    }
    type = (indices_data->GetStride() == 4) ? schemas::IndexType::UINT
                                            : schemas::IndexType::USHORT;
  }
  auto existing = absl::c_find_if(
      index_buffer_offsets_,
      [this, &indices_access,
       &type](const Offset<schemas::IndexBufferInfo>& index_buffer_offset) {
        const schemas::IndexBufferInfo* index_buffer_info =
            GetTemporaryPointer(fbb_, index_buffer_offset);
        return index_buffer_info->type() == type &&
               absl::c_equal(indices_access.StringView(),
                             *index_buffer_info->buffer());
      });
  if (existing != index_buffer_offsets_.end()) {
    return index_buffer_offsets_.IdOf<IndexBufferId>(*existing);
  }

  if (indices_access.Size() == 0) return {};

  return index_buffer_offsets_.Append<IndexBufferId>(
      schemas::CreateIndexBufferInfo(
          fbb_, type,
          fbb_.CreateVector(indices_access.Data(), indices_access.Size())));
}

absl::StatusOr<LoadedModelBuilder::ImageOffset>
LoadedModelBuilder::AddImageData(const ImageData& image_data) {
  if (auto buffer_access = std::get_if<BufferAccess>(&image_data)) {
    return schemas::CreateImageFileData(
               fbb_,
               fbb_.CreateVector(buffer_access->Data(), buffer_access->Size()))
        .Union();
  } else {
    auto compressed_image_contents =
        std::get_if<image::CompressedImageContents>(&image_data);
    if (!compressed_image_contents)
      return absl::InternalError("Missing Compressed Image contents");
    return schemas::CreateCompressedImageData(
               fbb_, compressed_image_contents->width,
               compressed_image_contents->height,
               static_cast<uint32_t>(compressed_image_contents->format),
               fbb_.CreateVector(compressed_image_contents->buffer.data(),
                                 compressed_image_contents->buffer.size()))
        .Union();
  }
}

absl::Span<const uint8_t> GetImageDataSpan(
    const LoadedModelBuilder::ImageData& image_data) {
  if (const auto* value =
          std::get_if<image::EncodedImageContents>(&image_data)) {
    return absl::Span<const uint8_t>(value->Data(), value->Size());
  }
  if (const auto* value =
          std::get_if<image::CompressedImageContents>(&image_data)) {
    return absl::Span<const uint8_t>(value->buffer.data(),
                                     value->buffer.size());
  }
  return {};
}

model::TextureId LoadedModelBuilder::AddTexture(
    uint16_t lookup_index, absl::string_view image_name,
    schemas::TextureInfoFlags texture_info_flags, ImageData image_data) {
  auto it = absl::c_find_if(textures_, [image_name, &image_data = image_data](
                                           const Texture& texture) {
    if (texture.image_name != image_name) return false;
    if (texture.image_data.index() != image_data.index()) return false;

    return absl::c_equal(GetImageDataSpan(texture.image_data),
                         GetImageDataSpan(image_data));
  });
  TextureId texture_id;
  if (it != textures_.end()) {
    texture_id = textures_.IdOf<TextureId>(*it);
  } else {
    texture_id = textures_.Append<TextureId>(
        Texture(image_name, texture_info_flags, std::move(image_data)));
  }
  texture_from_lookup_index_[lookup_index] = texture_id;
  return texture_id;
}

LoadedModelBuilder::TextureId LoadedModelBuilder::GetTexture(
    uint16_t lookup_index) const {
  auto itr = texture_from_lookup_index_.find(lookup_index);
  if (itr == texture_from_lookup_index_.end()) {
    return TextureId{};
  }
  return itr->second;
}

model::MaterialId LoadedModelBuilder::AddMaterial(
    uint16_t lookup_index,
    flatbuffers::Offset<schemas::MaterialInfo> material_offset) {
  MaterialId material_id =
      material_offsets_.Append<MaterialId>(material_offset);
  material_from_lookup_index_[lookup_index] = material_id;
  return material_id;
}

LoadedModelBuilder::MaterialId LoadedModelBuilder::GetMaterial(
    uint16_t lookup_index) const {
  auto itr = material_from_lookup_index_.find(lookup_index);
  if (itr == material_from_lookup_index_.end()) {
    return MaterialId{};
  }
  return itr->second;
}

LoadedModelBuilder::SkinId LoadedModelBuilder::GetSkin(
    uint32_t lookup_index) const {
  auto itr = skin_from_lookup_index_.find(lookup_index);
  if (itr == skin_from_lookup_index_.end()) {
    return SkinId{};
  }
  return itr->second;
}

std::optional<VertexAttributeMask> LoadedModelBuilder::GetRequiredAttributes(
    MaterialId material) {
  const schemas::MaterialInfo* schema =
      GetTemporaryPointer(fbb_, material_offsets_[material]);
  VertexAttributeMask result;
  result.Set(VertexAttribute::POSITION);
  result.Set(VertexAttribute::TEXCOORD_0);
  result.Set(VertexAttribute::TEXCOORD_1);
  result.Set(VertexAttribute::COLOR_0);
  if (schema->material()->spec()->lighting_model() ==
      schemas::GenericMaterialLightingModel::Lit) {
    result.Set(VertexAttribute::TANGENT);
  }
  return result;
}

LoadedModelBuilder::MaterialsVariantsId
LoadedModelBuilder::AddMaterialsVariants(absl::string_view name) {
  return materials_variants_offsets_.Append<MaterialsVariantsId>(
      schemas::CreateMaterialsVariantsInfo(
          fbb_, fbb_.CreateString(name.data(), name.size())));
}

void LoadedModelBuilder::ReserveBones(size_t size) { bones_.reserve(size); }

absl::Status LoadedModelBuilder::AddBone(uint16_t num_children,
                                         PreciseTransform local_transform,
                                         absl::string_view name,
                                         uint16_t node_index) {
  if (bones_.size() >= bones_.capacity())
    return absl::InternalError("Overflow of reserved bones");

  bones_.push_back(num_children, BoneParentId{}, BoneChildId{}, BoneChildId{},
                   local_transform, mat4{}, std::string(name), node_index);
  return absl::OkStatus();
}

absl::Status LoadedModelBuilder::FinishBones() {
  if (!bones_.size()) return absl::InternalError("Invalid Skeleton");
  auto child_counts = bones_.Span<BoneData::Fields::kNumChildren>();
  auto parents = bones_.Span<BoneData::Fields::kParent>();
  auto first_children = bones_.Span<BoneData::Fields::kFirstChild>();
  auto next_siblings = bones_.Span<BoneData::Fields::kNextSibling>();

  TypedDagTools<model::BoneId>::ExpandGraph(child_counts, parents,
                                            first_children, next_siblings);

  MP_RETURN_IF_ERROR(TypedDagTools<model::BoneId>::VerifyGraph(
      child_counts, parents, first_children, next_siblings));

  auto root_transforms = bones_.Span<BoneData::Fields::kRootTransform>();
  auto local_transforms = bones_.Span<BoneData::Fields::kLocalTransform>();

  absl::c_transform(parents, local_transforms, root_transforms.data(),
                    [&root_transforms](const model::BoneParentId& parent,
                                       const PreciseTransform& trs) {
                      if (parent) {
                        return root_transforms[parent] * trs.AsMat4();
                      } else {
                        return trs.AsMat4();
                      }
                    });
  return absl::OkStatus();
}

void LoadedModelBuilder::ReserveEntities(size_t size) {
  entities_.reserve(size);
}

absl::StatusOr<LoadedModelBuilder::EntityId> LoadedModelBuilder::AddEntity(
    BoneId bone, SkinId skin, MorphTargetBufferId morph_target_buffer,
    std::vector<float> node_morph_target_weights,
    std::vector<float> mesh_morph_target_weights,
    LightPunctualId light_punctual, AudioEmitterId audio_emitter,
    std::vector<PartData> parts, std::optional<filament::Box> bounds,
    std::optional<RuntimeData> runtime, int child_count, absl::string_view name,
    uint16_t original_index, int original_mesh_index, int original_skin_index,
    std::optional<NodeVisibility> node_visibility,
    std::optional<NodeSelectability> node_selectability,
    std::optional<NodeHoverability> node_hoverability) {
  if (entities_.size() >= entities_.capacity()) {
    return absl::InternalError("Overflow of reserved bones");
  }

  if (parts.empty() == bounds.has_value() ||
      parts.empty() == runtime.has_value()) {
    return absl::InvalidArgumentError("bounds/runtime not paired with parts");
  }

  return entities_.Append(
      std::move(child_count), EntityParentId{}, EntityChildId{},
      EntityChildId{}, std::move(parts), std::move(bone), std::move(skin),
      std::move(morph_target_buffer), std::move(node_morph_target_weights),
      std::move(mesh_morph_target_weights), std::move(light_punctual),
      audio_emitter, std::move(bounds), std::move(runtime), std::string(name),
      std::move(original_index), std::move(original_mesh_index),
      std::move(original_skin_index), std::move(node_visibility),
      std::move(node_selectability), std::move(node_hoverability));
}

absl::Status LoadedModelBuilder::FinishEntities() {
  if (entities_.capacity() != entities_.size())
    return absl::InternalError("Invalid entities");
  if (entities_.empty()) return absl::OkStatus();
  auto child_counts = entities_.Span<EntityData::Fields::kNumChildren>();
  auto parents = entities_.Span<EntityData::Fields::kParent>();
  auto first_children = entities_.Span<EntityData::Fields::kFirstChild>();
  auto next_siblings = entities_.Span<EntityData::Fields::kNextSibling>();

  TypedDagTools<EntityId>::ExpandGraph(child_counts, parents, first_children,
                                       next_siblings);
  MP_RETURN_IF_ERROR(TypedDagTools<EntityId>::VerifyGraph(
      child_counts, parents, first_children, next_siblings));
  return absl::OkStatus();
}

absl::StatusOr<LoadedModelBuilder::SkinId> LoadedModelBuilder::AddSkin(
    uint32_t lookup_index, model::ModelData::SkinData skin_data) {
  if (skins_.size() > std::numeric_limits<int16_t>::max()) {
    return absl::InternalError("Too many skins");
  }
  SkinId skin_id = skins_.Append<SkinId>(std::move(skin_data));
  skin_from_lookup_index_[lookup_index] = skin_id;
  return skin_id;
}

absl::Status LoadedModelBuilder::AddSkinEntity(
    SkinId skin, EntityId target,
    SampledJointLookup<filament::Aabb> sampled_joint_bounds,
    PairedBitVector<model::ModelData::SampledJointData> sampled_joint_in_use) {
  skins_[skin].skinned_entities.push_back(
      target, std::move(sampled_joint_bounds), std::move(sampled_joint_in_use));
  return absl::OkStatus();
}

absl::Status LoadedModelBuilder::FinishSkins() {
  using SkinnedEntityData = model::ModelData::SkinnedEntityData;
  using JointData = model::ModelData::JointData;
  using JointParentId = model::ModelData::JointParentId;
  using JointChildId = model::ModelData::JointChildId;
  using SampledJointId = model::ModelData::SampledJointId;

  skin_offsets_.resize(skins_.size());
  for (const auto& [skin_index, skin_id] : skin_from_lookup_index_) {
    const model::ModelData::SkinData& skin = skins_[skin_id];
    auto joint_count = skin.joints.size();
    auto sampled_joint_count = skin.sampled_joints.size();
    auto skinned_entity_count = skin.skinned_entities.size();
    Offset<Vector<const schemas::SampledJointInfo*>> sampled_joints_offset =
        fbb_.CreateVectorOfStructs<schemas::SampledJointInfo>(
            sampled_joint_count,
            [sampled_joints = skin.sampled_joints.data()](
                size_t i, schemas::SampledJointInfo* sampled_joint) {
              *sampled_joint = schemas::SampledJointInfo(
                  static_cast<uint16_t>(sampled_joints[i].joint));
            });
    if (skin.inverse_bind_poses.size() != sampled_joint_count) {
      return absl::InternalError("Invalid skin inverse bind poses");
    }
    Offset<Vector<const schemas::Mat4f*>> inverse_bind_poses_offset =
        fbb_.CreateVectorOfNativeStructs<schemas::Mat4f>(
            skin.inverse_bind_poses.data(), skin.inverse_bind_poses.size());
    Offset<Vector<const schemas::SkinnedEntityTargetInfo*>>
        skinned_entity_targets_offset =
            fbb_.CreateVectorOfStructs<schemas::SkinnedEntityTargetInfo>(
                skinned_entity_count,
                [skinned_entity_targets =
                     skin.skinned_entities
                         .data<SkinnedEntityData::Fields::kTarget>()](
                    size_t i,
                    schemas::SkinnedEntityTargetInfo* skinned_entity_target) {
                  *skinned_entity_target = schemas::SkinnedEntityTargetInfo(
                      static_cast<uint16_t>(skinned_entity_targets[i]));
                });
    Offset<Vector<uint16_t>> joint_child_counts_offset = fbb_.CreateVector(
        skin.joints.data<JointData::Fields::kNumChildren>(), joint_count);
    Offset<Vector<uint16_t>> joint_parents_offset = fbb_.CreateVector<uint16_t>(
        reinterpret_cast<const JointParentId::ValueType*>(
            skin.joints.data<JointData::Fields::kParent>()),
        joint_count);
    Offset<Vector<uint16_t>> joint_first_children_offset =
        fbb_.CreateVector<uint16_t>(
            reinterpret_cast<const JointChildId::ValueType*>(

                skin.joints.data<JointData::Fields::kFirstChild>()),
            joint_count);
    Offset<Vector<uint16_t>> joint_next_siblings_offset =
        fbb_.CreateVector<uint16_t>(
            reinterpret_cast<const JointChildId::ValueType*>(
                skin.joints.data<JointData::Fields::kNextSibling>()),
            joint_count);
    Offset<Vector<uint16_t>> joint_sources_offset = fbb_.CreateVector<uint16_t>(
        joint_count,
        [joint_sources = skin.joints.data<JointData::Fields::kSource>()](
            size_t i) { return static_cast<uint16_t>(joint_sources[i]); });
    Offset<Vector<int16_t>> joint_targets_offset = fbb_.CreateVector<int16_t>(
        joint_count,
        [joint_targets = skin.joints.data<JointData::Fields::kTarget>()](
            size_t i) { return static_cast<int16_t>(joint_targets[i]); });

    std::vector<Offset<schemas::SkinnedEntityBoundsInfo>>
        item_sampled_bone_bounds(skinned_entity_count);
    absl::c_transform(
        absl::Span<const SampledJointLookup<filament::Aabb>>(
            skin.skinned_entities
                .data<SkinnedEntityData::Fields::kSampledJointBounds>(),
            skinned_entity_count),
        item_sampled_bone_bounds.data(),
        [this](const SampledJointLookup<filament::Aabb>& sampled_bone_bounds) {
          std::vector<schemas::Box> bounds;
          bounds.reserve(sampled_bone_bounds.size());
          absl::c_transform(sampled_bone_bounds, std::back_inserter(bounds),
                            [](const filament::Aabb& aabb) {
                              return schemas::Box{
                                  flatbuffers::Pack(aabb.center()),
                                  flatbuffers::Pack(aabb.extent())};
                            });

          return schemas::CreateSkinnedEntityBoundsInfo(
              fbb_, fbb_.CreateVectorOfStructs(bounds));
        });
    Offset<Vector<Offset<schemas::SkinnedEntityBoundsInfo>>>
        skinned_entity_joint_bounds_offset =
            fbb_.CreateVector(item_sampled_bone_bounds);

    std::vector<Offset<schemas::SkinnedEntityJointUsageInfo>>
        item_sampled_bone_usage(skinned_entity_count);
    absl::c_transform(
        absl::Span<const PairedBitVector<SampledJointId::ReferredType>>(
            skin.skinned_entities
                .data<SkinnedEntityData::Fields::kSampledJointInUse>(),
            skinned_entity_count),
        item_sampled_bone_usage.data(),
        [this](const PairedBitVector<SampledJointId::ReferredType>&
                   sampled_bone_usage) {
          absl::Span<const uint32_t> words = sampled_bone_usage.Words();
          return schemas::CreateSkinnedEntityJointUsageInfo(
              fbb_, fbb_.CreateVector(words.data(), words.size()));
        });

    Offset<Vector<Offset<schemas::SkinnedEntityJointUsageInfo>>>
        skinned_entity_joint_usage_offset =
            fbb_.CreateVector(item_sampled_bone_usage);
    skin_offsets_[skin_id] = schemas::CreateSkinInfo(
        fbb_, sampled_joints_offset, inverse_bind_poses_offset,
        joint_child_counts_offset, joint_parents_offset,
        joint_first_children_offset, joint_next_siblings_offset,
        joint_sources_offset, joint_targets_offset,
        skinned_entity_targets_offset, skinned_entity_joint_bounds_offset,
        skinned_entity_joint_usage_offset, static_cast<int32_t>(skin.pose_root),
        skin_index);
  }
  return absl::OkStatus();
}

const TypedSetVector<LoadedModelBuilder::BoneData>& LoadedModelBuilder::Bones()
    const {
  return bones_;
}

PairedSpan<const mat4, LoadedModelBuilder::BoneData>
LoadedModelBuilder::BoneRootTransforms() const {
  return bones_.Span<BoneData::Fields::kRootTransform>();
}

LoadedModelBuilder::LightPunctualId LoadedModelBuilder::AddLightPunctual(
    absl::string_view name, const schemas::Color& color, float intensity,
    schemas::LightPunctualType type, float range,
    const schemas::SpotConeAngles& spot_cone_angles) {
  return light_punctual_offsets_.Append<LightPunctualId>(
      CreateLightPunctualInfo(fbb_, fbb_.CreateString(name.data(), name.size()),
                              &color, intensity, type, range,
                              &spot_cone_angles));
}

absl::StatusOr<LoadedModelBuilder::AnimationId>
LoadedModelBuilder::AddAnimation(AnimationAccess access) {
  return animations_.Append<AnimationId>(std::move(access));
}

void LoadedModelBuilder::RemoveShadowPlanes(const filament::Box& scene_bounds) {
  // Expect shadow planes to be very thin in height, on the order of 1mm
  constexpr float kShadowPlaneHeightThreshold = 1.0e-4f;
  // ... and to be not-thin in the other dimensions.
  constexpr float kShadowPlaneSizeThreshold = 1.0e-2f;
  // To avoid false-positives, ignore scenes without sufficient height.
  constexpr float kSceneHeightThreshold = 1.0e-2f;

  const float scene_height = scene_bounds.halfExtent.y * 2.0f;
  if (scene_height < kSceneHeightThreshold) {
    return;
  }

  PairedBitVector<MaterialId::ReferredType> is_shadow_plane;
  bool any_shadow_plane = false;

  absl::c_transform(
      material_offsets_, std::back_inserter(is_shadow_plane),
      [&scene_bounds, &any_shadow_plane,
       &fbb = fbb_](MaterialOffset material_offset) {
        const schemas::MaterialInfo* generic_material_info =
            GetTemporaryPointer(fbb, material_offset);

        GenericMaterialSpec spec = GenericMaterialSpec::FromFlatbuffer(
            *generic_material_info->material()->spec());
        const schemas::BoundsInfo* material_bounds_info =
            generic_material_info->material()->bounds();
        auto material_bounds = filament::Box{
            flatbuffers::UnPack(material_bounds_info->bounds()->center()),
            flatbuffers::UnPack(material_bounds_info->bounds()->half_extent())};

        // Non-transparent materials are not shadow planes.
        if (spec.GetBlendMode() !=
            schemas::GenericMaterialBlendMode::Transparent) {
          return false;
        }
        float material_height = material_bounds.halfExtent.y * 2.0f;
        float material_size = std::max(material_bounds.halfExtent.x,
                                       material_bounds.halfExtent.z) *
                              2.0f;
        // Ignore materials that are too tall to be shadow planes.
        if (material_height > kShadowPlaneHeightThreshold) return false;
        // ... or not wide enough.
        if (material_size < kShadowPlaneSizeThreshold) return false;

        // Ignore materials that don't cleanly decompose into bounds fractions.
        absl::StatusOr<float3> min_fraction =
            ToBoundsFraction(scene_bounds, material_bounds.getMin());
        if (!min_fraction.ok()) return false;
        absl::StatusOr<float3> max_fraction =
            ToBoundsFraction(scene_bounds, material_bounds.getMax());
        if (!max_fraction.ok()) return false;

        // Ignore materials that don't occupy the bottom of the scene bounds.
        float squared_error = distance2(min_fraction.value(), float3{0, 0, 0}) +
                              distance2(max_fraction.value(), float3{1, 0, 1});
        const float kErrorThreshold = 1.0e-2f;
        if (squared_error > kErrorThreshold) return false;

        any_shadow_plane = true;
        return true;
      });
  if (any_shadow_plane) {
    absl::c_for_each(
        entities_.Span<EntityData::kParts>(),
        [&is_shadow_plane](std::vector<PartData>& parts) {
          // Skip the copy if this entity doesn't actually have shadow planes.
          if (!absl::c_any_of(parts, [&is_shadow_plane](const PartData& part) {
                return is_shadow_plane[part.material];
              }))
            return;

          std::vector<PartData> new_parts;
          absl::c_for_each(
              parts, [&is_shadow_plane, &new_parts](PartData& part) {
                if (!is_shadow_plane[part.material]) {
                  new_parts.push_back(PartData{
                      part.name, part.index_offset, part.index_count,
                      part.vertex_buffer, part.index_buffer, part.material,
                      part.original_material_index, part.primitive_type,
                      std::move(part.materials_variants_mappings),
                      part.skinning_buffer, part.morph_target_buffer_offset,
                      part.morph_target_buffer_count});
                }
              });
          std::swap(parts, new_parts);
        });
  }
}

LoadedModelBuilder::AudioEmitterOffset LoadedModelBuilder::AddAudioEmitter(
    absl::string_view name, schemas::AudioEmitterType type, float gain,
    const std::vector<uint16_t>& source_indices,
    AudioEmitterPositionalOffset positional_offset) {
  // TODO: Add support for positional audio
  return schemas::CreateAudioEmitter(
      fbb_, fbb_.CreateString(name.data(), name.size()),
      schemas::AudioEmitterType::GLOBAL, gain,
      fbb_.CreateVector(source_indices), positional_offset);
}
LoadedModelBuilder::AudioEmitterPositionalOffset
LoadedModelBuilder::AddAudioEmitterPositional(
    float cone_inner_angle, float cone_outer_angle, float cone_outer_gain,
    schemas::AudioEmitterDistanceModel distance_model, float max_distance,
    float ref_distance, float rolloff_factor) {
  return schemas::CreateAudioEmitterPositional(
      fbb_, cone_inner_angle, cone_outer_angle, cone_outer_gain, distance_model,
      max_distance, ref_distance, rolloff_factor);
}
LoadedModelBuilder::AudioSourceOffset LoadedModelBuilder::AddAudioSource(
    absl::string_view name, bool auto_play, float gain, bool loop,
    uint16_t audio) {
  return schemas::CreateAudioSource(fbb_,
                                    fbb_.CreateString(name.data(), name.size()),
                                    auto_play, gain, loop, audio);
}
LoadedModelBuilder::AudioOffset LoadedModelBuilder::AddAudio(
    const BufferAccess& audio_buffer) {
  return schemas::CreateAudio(
      fbb_, fbb_.CreateVector(audio_buffer.Data(), audio_buffer.Size()));
}

void LoadedModelBuilder::AddAudioExtension(
    const AudioEmitterOffsets& audio_emitter_offsets,
    const AudioSourceOffsets& audio_source_offsets,
    const AudioOffsets& audio_offsets,
    const std::vector<uint16_t>& scene_emitters) {
  audio_extension_offset_ = schemas::CreateAudioExtension(
      fbb_, CreateVector<schemas::AudioEmitter>(fbb_, audio_emitter_offsets),
      CreateVector<schemas::AudioSource>(fbb_, audio_source_offsets),
      CreateVector<schemas::Audio>(fbb_, audio_offsets),
      fbb_.CreateVector(scene_emitters));
}

std::unique_ptr<InteractivityLoaderExtension>
LoadedModelBuilder::CreateInteractivityLoaderExtension() {
  return extensions::CreateInteractivityLoaderExtension(fbb_);
}

void LoadedModelBuilder::AddInteractivity(
    const InteractivityOffset& interactivity_offset) {
  interactivity_offset_ = interactivity_offset;
}

absl::StatusOr<Offset<schemas::LoadedModel>> LoadedModelBuilder::Serialize() {
  auto bone_child_counts_offset = CreateVector<uint16_t>(
      fbb_, bones_.Span<BoneData::Fields::kNumChildren>());
  auto bone_parents_offset =
      CreateVector<uint16_t>(fbb_, bones_.Span<BoneData::Fields::kParent>());
  auto bone_first_children_offset = CreateVector<uint16_t>(
      fbb_, bones_.Span<BoneData::Fields::kFirstChild>());
  auto bone_next_siblings_offset = CreateVector<uint16_t>(
      fbb_, bones_.Span<BoneData::Fields::kNextSibling>());
  auto bone_local_transforms_offset =
      CreateVectorOfNativeStructs<schemas::PreciseTransform>(
          fbb_, bones_.Span<BoneData::Fields::kLocalTransform>());
  auto bone_root_transforms_offset = CreateVectorOfNativeStructs<schemas::Mat4>(
      fbb_, bones_.Span<BoneData::Fields::kRootTransform>());
  auto bone_names_offset =
      CreateVectorOfStrings(fbb_, bones_.Span<BoneData::Fields::kName>());
  auto bone_node_indices_offset =
      CreateVector<uint16_t>(fbb_, bones_.Span<BoneData::Fields::kNodeIndex>());

  Offset<schemas::SkeletonInfo> skeleton = schemas::CreateSkeletonInfo(
      fbb_, bone_child_counts_offset, bone_parents_offset,
      bone_first_children_offset, bone_next_siblings_offset,
      bone_local_transforms_offset, bone_root_transforms_offset,
      bone_names_offset, bone_node_indices_offset);

  std::vector<EntityOffset> entity_offsets(entities_.size());
  std::transform(
      entities_.begin(), entities_.end(), entity_offsets.data(),
      [&fbb = fbb_](
          EntityData::ArrayType::IteratorValueRef entity) -> EntityOffset {
        const std::vector<PartData>& parts =
            entity.get<EntityData::Fields::kParts>();
        std::vector<PartOffset> part_offsets(parts.size());
        absl::c_transform(
            parts, part_offsets.data(), [&fbb](const PartData& part) {
              Offset<Vector<MaterialId::ValueType>>
                  materials_variants_mappings_offset = fbb.CreateVector(
                      reinterpret_cast<const MaterialId::ValueType*>(
                          part.materials_variants_mappings.data()),
                      part.materials_variants_mappings.size());

              return schemas::CreatePartInfo(
                  fbb, fbb.CreateString(part.name), part.index_offset,
                  part.index_count, uint16_t{part.vertex_buffer},
                  uint16_t{part.index_buffer}, uint16_t{part.material},
                  part.original_material_index,
                  static_cast<uint8_t>(part.primitive_type),
                  materials_variants_mappings_offset,
                  uint16_t{part.skinning_buffer},
                  part.morph_target_buffer_offset,
                  part.morph_target_buffer_count);
            });

        const std::vector<float>& node_morph_target_weights =
            entity.get<EntityData::Fields::kNodeMorphTargetWeights>();
        const std::vector<float>& mesh_morph_target_weights =
            entity.get<EntityData::Fields::kMeshMorphTargetWeights>();
        return schemas::CreateEntityInfo(
            fbb, fbb.CreateString(entity.get<EntityData::Fields::kName>()),
            uint16_t{entity.get<EntityData::Fields::kBone>()},
            int16_t{entity.get<EntityData::Fields::kSkin>()},
            uint16_t{entity.get<EntityData::Fields::kMorphTargetBuffer>()},
            fbb.CreateVector(node_morph_target_weights.data(),
                             node_morph_target_weights.size()),
            fbb.CreateVector(mesh_morph_target_weights.data(),
                             mesh_morph_target_weights.size()),
            int16_t{entity.get<EntityData::Fields::kLightPunctual>()},
            uint16_t{entity.get<EntityData::Fields::kAudioEmitter>()},
            fbb.CreateVector(part_offsets),
            CreateBoundsInfo(fbb,
                             entity.get<EntityData::Fields::kLocalBounds>()),
            CreateRuntimeInfo(fbb, entity.get<EntityData::Fields::kRuntime>()),
            entity.get<EntityData::Fields::kNumChildren>(),
            uint16_t{entity.get<EntityData::Fields::kOriginalIndex>()},
            int16_t{entity.get<EntityData::Fields::kOriginalMeshIndex>()},
            int16_t{entity.get<EntityData::Fields::kOriginalSkinIndex>()},
            CreateNodeVisibility(
                fbb, entity.get<EntityData::Fields::kNodeVisibility>()),
            CreateNodeSelectability(
                fbb, entity.get<EntityData::Fields::kNodeSelectability>()),
            CreateNodeHoverability(
                fbb, entity.get<EntityData::Fields::kNodeHoverability>()));
      });

  auto entity_parents_offset = CreateVector<uint16_t>(
      fbb_, entities_.Span<EntityData::Fields::kParent>());
  auto entity_first_children_offset = CreateVector<uint16_t>(
      fbb_, entities_.Span<EntityData::Fields::kFirstChild>());
  auto entity_next_siblings_offset = CreateVector<uint16_t>(
      fbb_, entities_.Span<EntityData::Fields::kNextSibling>());

  Offset<schemas::EntityGraphInfo> entity_graph =
      schemas::CreateEntityGraphInfo(
          fbb_, fbb_.CreateVector(entity_offsets), entity_parents_offset,
          entity_first_children_offset, entity_next_siblings_offset);

  std::vector<VertexBufferOffset> vertex_buffer_offsets(vertex_buffers_.size());
  absl::c_transform(
      vertex_buffers_, vertex_buffer_offsets.data(),
      [&fbb = fbb_](const VertexBuffer& vertex_buffer) {
        std::vector<VertexBlockOffset> vertex_block_offsets(
            vertex_buffer.blocks.size());
        absl::c_transform(
            vertex_buffer.blocks, vertex_block_offsets.data(),
            [&fbb](const VertexBlock& block) {
              return schemas::CreateVertexBlockInfo(
                  fbb, fbb.CreateVectorOfStructs(block.attributes),
                  fbb.CreateVector(block.buffer.Data(), block.buffer.Size()),
                  block.stride);
            });
        return schemas::CreateVertexBufferInfo(
            fbb, fbb.CreateVector(vertex_block_offsets),
            vertex_buffer.vertex_count, vertex_buffer.advanced_skinning);
      });

  TextureOffsets texture_offsets;
  ImageTypes image_types;
  ImageOffsets image_offsets;

  for (auto& texture : textures_) {
    auto image_type_id = image_types.Append<TextureId>(
        std::holds_alternative<BufferAccess>(texture.image_data)
            ? schemas::ImageInfo::ImageFileData
            : schemas::ImageInfo::CompressedImageData);
    MP_ASSIGN_OR_RETURN(ImageOffset image_offset,
                     AddImageData(texture.image_data));
    auto image_id = image_offsets.Append<TextureId>(image_offset);
    if (image_type_id != image_id)
      return absl::InternalError("Mismatched arrays");

    auto texture_id = texture_offsets.Append<TextureId>(
        CreateTextureInfo(fbb_, fbb_.CreateString(texture.image_name),
                          texture.texture_info_flags));
    if (texture_id != image_id) return absl::InternalError("Mismatched arrays");
  }

  Offset<Vector<VertexBufferOffset>> vertex_buffers = fbb_.CreateVector(
      vertex_buffer_offsets.data(), vertex_buffer_offsets.size());
  Offset<Vector<IndexBufferOffset>> index_buffers = fbb_.CreateVector(
      index_buffer_offsets_.data(), index_buffer_offsets_.size());
  Offset<Vector<TextureOffset>> textures =
      fbb_.CreateVector(texture_offsets.data(), texture_offsets.size());
  Offset<Vector<ImageType>> images_types =
      fbb_.CreateVector(image_types.data(), image_types.size());
  Offset<Vector<ImageOffset>> images =
      fbb_.CreateVector(image_offsets.data(), image_offsets.size());

  Offset<Vector<Offset<schemas::SkinInfo>>> skins =
      fbb_.CreateVector(skin_offsets_.data(), skin_offsets_.size());
  Offset<Vector<Offset<schemas::MaterialInfo>>> materials =
      CreateVector<schemas::MaterialInfo>(fbb_, material_offsets_);

  AnimationOffsets animation_offsets(animations_.size());
  absl::c_transform(animations_, animation_offsets.data(),
                    [&fbb = fbb_](const AnimationAccess& access) {
                      Offset<Vector<uint8_t>> buffer =
                          fbb.CreateVector<uint8_t>(access.Buffer().Data(),
                                                    access.Buffer().Size());
                      return schemas::CreateGltfAnimationInfo(fbb, buffer);
                    });

  Offset<Vector<Offset<schemas::GltfAnimationInfo>>> animations =
      CreateVector<schemas::GltfAnimationInfo>(fbb_, animation_offsets);
  Offset<Vector<Offset<schemas::LightPunctualInfo>>> lights_punctual =
      CreateVector<schemas::LightPunctualInfo>(fbb_, light_punctual_offsets_);
  Offset<Vector<Offset<schemas::MaterialsVariantsInfo>>> materials_variants =
      CreateVector<schemas::MaterialsVariantsInfo>(fbb_,
                                                   materials_variants_offsets_);
  Offset<Vector<Offset<schemas::MorphTargetBufferInfo>>> morph_target_buffers =
      CreateVector<schemas::MorphTargetBufferInfo>(
          fbb_, morph_target_buffer_offsets_);
  Offset<Vector<Offset<schemas::SkinningBufferInfo>>> skinning_buffers =
      CreateVector<schemas::SkinningBufferInfo>(fbb_, skinning_buffer_offsets_);

  return CreateLoadedModel(
      fbb_, skeleton, entity_graph, skins, vertex_buffers, index_buffers,
      materials, textures, images_types, images, animations, lights_punctual,
      materials_variants, morph_target_buffers, skinning_buffers,
      audio_extension_offset_, interactivity_offset_);
}

flatbuffers::FlatBufferBuilder& LoadedModelBuilder::GetFlatBufferBuilder() {
  return fbb_;
}

absl::StatusOr<LoadedModelBuilder::LoadedModelAccess>
LoadedModelBuilder::Finish() {
  MP_ASSIGN_OR_RETURN(Offset<schemas::LoadedModel> root, Serialize());
  fbb_.Finish(root);
  FlatBufferAccess<schemas::LoadedModel> result;
  MP_RETURN_IF_ERROR(CreateFlatBufferAccess(&fbb_, &result));
  return result;
}

}  // namespace imp::loader::details
