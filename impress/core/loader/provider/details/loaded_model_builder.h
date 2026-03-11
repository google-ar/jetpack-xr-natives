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

#ifndef THIRD_PARTY_ARCORE_AR_IMP_CORE_LOADER_PROVIDER_DETAILS_LOADED_MODEL_BUILDER_H_
#define THIRD_PARTY_ARCORE_AR_IMP_CORE_LOADER_PROVIDER_DETAILS_LOADED_MODEL_BUILDER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Box.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/string.h"
#include "flatbuffers/vector.h"
#include "core/animation/schemas/gltf_animation_generated.h"
#include "core/common/bit_vector.h"
#include "core/common/buffer_access.h"
#include "core/common/flatbuffer_helpers.h"
#include "core/common/paired_span.h"
#include "core/common/paired_vector.h"
#include "core/common/robin_map.h"
#include "core/common/schemas/render_generated.h"
#include "core/common/typed_id.h"
#include "core/common/typed_set_vector.h"
#include "core/image/image_contents.h"
#include "core/loader/provider/details/vertex_attribute.h"
#include "core/loader/provider/extensions/behavior/loader_extension.h"
#include "core/loader/provider/extensions/interactivity/loader_extension.h"
#include "core/loader/provider/extensions/interactivity/schemas/interactivity_generated.h"
#include "core/loader/provider/gltf/dense_data_access.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/math/mat.h"
#include "core/math/quat.h"
#include "core/math/transform.h"
#include "core/math/vec.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "core/model/skin_data.h"

namespace imp::loader::details {

// Wraps a Flatbuffer Builder to incrementally build a LoadedModel flatbuffer.
// The wrapped builder has one hard-and-fast requirement that drives the design:
// Arrays must be added to the flatbuffer in a one-shot with complete data.
class LoadedModelBuilder {
 public:
  // Output type; a buffer holding a serialized LoadedModel.
  using LoadedModelAccess = imp::FlatBufferAccess<schemas::LoadedModel>;
  // Flatbuffer aliases
  using String = flatbuffers::String;
  template <typename T>
  using Offset = flatbuffers::Offset<T>;
  template <typename T>
  using Vector = flatbuffers::Vector<T>;
  // TypedId aliases
  using BoneData = model::BoneData;
  using WeakBoneId = model::WeakBoneId;
  using BoneId = model::BoneId;
  using BoneParentId = model::BoneParentId;
  using BoneChildId = model::BoneChildId;
  template <typename T>
  using BoneLookup = model::BoneLookup<T>;

  using EntityData = model::ModelData::EntityData;
  using EntityId = model::ModelData::EntityId;
  using EntityParentId = model::ModelData::EntityParentId;
  using EntityChildId = model::ModelData::EntityChildId;
  using WeakEntityId = model::ModelData::WeakEntityId;
  template <typename T>
  using EntityLookup = model::ModelData::EntityLookup<T>;
  using EntityOffset = Offset<schemas::EntityInfo>;

  using PartData = model::ModelData::PartData;
  using PartOffset = Offset<schemas::PartInfo>;

  using JointData = model::ModelData::JointData;
  using JointId = model::ModelData::JointId;
  using WeakJointId = model::ModelData::WeakJointId;
  using JointParentId = model::ModelData::JointParentId;
  using JointChildId = model::ModelData::JointChildId;
  template <typename T>
  using JointLookup = model::ModelData::JointLookup<T>;

  using SampledJointData = model::ModelData::SampledJointData;
  using SampledJointId = model::ModelData::SampledJointId;
  using WeakSampledJointId = model::ModelData::WeakSampledJointId;
  template <typename T>
  using SampledJointLookup = model::ModelData::SampledJointLookup<T>;
  using SampledJointBitVector = PairedBitVector<SampledJointId::ReferredType>;

  using RuntimeData = model::ModelData::RuntimeData;
  using PrimitiveType = model::ModelData::PrimitiveType;
  using MaterialsVariantsMappingLookup =
      model::ModelData::MaterialsVariantsMappingLookup;

  using VertexBufferId = model::ModelData::VertexBufferId;
  using VertexBlockOffset = Offset<schemas::VertexBlockInfo>;
  using VertexBufferOffset = Offset<schemas::VertexBufferInfo>;
  using VertexBufferOffsets =
      PairedVector<VertexBufferOffset, VertexBufferId::ReferredType>;

  using IndexBufferId = model::ModelData::IndexBufferId;
  using IndexBufferOffset = Offset<schemas::IndexBufferInfo>;
  using IndexBufferOffsets =
      PairedVector<IndexBufferOffset, IndexBufferId::ReferredType>;

  using TextureId = model::ModelData::TextureId;
  using TextureOffset = Offset<schemas::TextureInfo>;
  using TextureOffsets = PairedVector<TextureOffset, TextureId::ReferredType>;

  using ImageType = schemas::ImageInfo;
  using ImageTypes = PairedVector<ImageType, TextureId::ReferredType>;
  using ImageOffset = Offset<void>;
  using ImageOffsets = PairedVector<ImageOffset, TextureId::ReferredType>;

  using SamplerId = model::ModelData::SamplerId;
  using SamplerOffset = Offset<schemas::TextureSampler>;
  using SamplerOffsets = PairedVector<SamplerOffset, SamplerId::ReferredType>;

  using SkinId = model::ModelData::SkinId;
  using SkinOffset = Offset<schemas::SkinInfo>;
  using SkinOffsets = PairedVector<SkinOffset, SkinId::ReferredType>;
  template <typename T>
  using SkinLookup = model::ModelData::SkinLookup<T>;

  using MaterialTextureId = model::ModelData::MaterialTextureId;

  using MaterialParameterId = model::ModelData::MaterialParameterId;
  using MaterialParameterOffset = Offset<schemas::MaterialParamInfo>;
  using MaterialParameterOffsets =
      PairedVector<MaterialParameterOffset, MaterialParameterId::ReferredType>;

  using MaterialId = model::ModelData::MaterialId;
  using MaterialOffset = Offset<schemas::MaterialInfo>;
  using MaterialOffsets =
      PairedVector<MaterialOffset, MaterialId::ReferredType>;

  using AnimationId = TypedId<imp::gltf::imp_proto::Animation, int16_t>;
  using AnimationOffset = Offset<schemas::GltfAnimationInfo>;
  using AnimationOffsets =
      PairedVector<AnimationOffset, AnimationId::ReferredType>;
  using AnimationAccess = FlatBufferAccess<animation::schemas::GltfAnimation>;

  using LightPunctualId = model::ModelData::LightPunctualId;
  using LightPunctualOffset = Offset<schemas::LightPunctualInfo>;
  using LightPunctualOffsets =
      PairedVector<LightPunctualOffset, LightPunctualId::ReferredType>;

  using MaterialsVariantsId = model::ModelData::MaterialsVariantsId;
  using MaterialsVariantsOffset = Offset<schemas::MaterialsVariantsInfo>;
  using MaterialsVariantsOffsets =
      PairedVector<MaterialsVariantsOffset, MaterialsVariantsId::ReferredType>;

  using MorphTargetBufferId = model::ModelData::MorphTargetBufferId;
  using MorphTargetBufferOffset = Offset<schemas::MorphTargetBufferInfo>;
  using MorphTargetBufferOffsets =
      PairedVector<MorphTargetBufferOffset, MorphTargetBufferId::ReferredType>;

  using SkinningBufferId = model::ModelData::SkinningBufferId;
  using SkinningBufferOffset = Offset<schemas::SkinningBufferInfo>;
  using SkinningBufferOffsets =
      PairedVector<SkinningBufferOffset, SkinningBufferId::ReferredType>;

  using ImageData =
      std::variant<image::EncodedImageContents, image::CompressedImageContents>;

  using AudioEmitterId = model::ModelData::AudioEmitterId;
  using AudioEmitterOffset = Offset<schemas::AudioEmitter>;
  using AudioEmitterPositionalOffset = Offset<schemas::AudioEmitterPositional>;
  using AudioEmitterOffsets =
      PairedVector<AudioEmitterOffset, AudioEmitterId::ReferredType>;
  using AudioSourceId = model::ModelData::AudioSourceId;
  using AudioSourceOffset = Offset<schemas::AudioSource>;
  using AudioSourceOffsets =
      PairedVector<AudioSourceOffset, AudioSourceId::ReferredType>;
  using AudioId = model::ModelData::AudioId;
  using AudioOffset = Offset<schemas::Audio>;
  using AudioOffsets = PairedVector<AudioOffset, AudioId::ReferredType>;
  using AudioExtensionOffset = Offset<schemas::AudioExtension>;

  using BehaviorOffset = Offset<schemas::Behavior>;
  using InteractivityOffset = Offset<schemas::Interactivity>;

  using NodeVisibility = model::ModelData::NodeVisibility;
  using NodeSelectability = model::ModelData::NodeSelectability;
  using NodeHoverability = model::ModelData::NodeHoverability;

  struct VertexBlock {
    VertexBlock(std::vector<schemas::VertexAttributeInfo> in_attributes,
                BufferAccess in_buffer, uint32_t in_stride)
        : attributes(std::move(in_attributes)),
          buffer(std::move(in_buffer)),
          stride(in_stride) {}
    VertexBlock(const schemas::VertexAttributeInfo &in_attribute,
                BufferAccess in_buffer, uint32_t in_stride)
        : attributes(1, in_attribute),
          buffer(std::move(in_buffer)),
          stride(in_stride) {}

    std::vector<schemas::VertexAttributeInfo> attributes;
    BufferAccess buffer;
    uint32_t stride;

    static VertexBlock Positions(BufferAccess buffer) {
      return VertexBlock(schemas::VertexAttributeInfo(
                             schemas::VertexAttribute::POSITION,
                             schemas::AttributeType::FLOAT3, 0, false),
                         std::move(buffer), sizeof(float3));
    }
    static VertexBlock Tangents(BufferAccess buffer) {
      return VertexBlock(schemas::VertexAttributeInfo(
                             schemas::VertexAttribute::TANGENTS,
                             schemas::AttributeType::FLOAT4, 0, false),
                         std::move(buffer), sizeof(quatf));
    }
    static VertexBlock PrimaryUvFloats(BufferAccess buffer) {
      return VertexBlock(schemas::VertexAttributeInfo(
                             schemas::VertexAttribute::UV0,
                             schemas::AttributeType::FLOAT2, 0, false),
                         std::move(buffer), sizeof(float2));
    }
    static VertexBlock SecondaryUvShorts(BufferAccess buffer) {
      return VertexBlock(
          schemas::VertexAttributeInfo(schemas::VertexAttribute::UV1,
                                       schemas::AttributeType::USHORT2, 0, 1),
          std::move(buffer), sizeof(ushort2));
    }
    static VertexBlock Colors(BufferAccess buffer) {
      return VertexBlock(
          schemas::VertexAttributeInfo(schemas::VertexAttribute::COLOR,
                                       schemas::AttributeType::UBYTE4, 0, true),
          std::move(buffer), sizeof(ubyte4));
    }

    // move-only
    VertexBlock(const VertexBlock &) = delete;
    VertexBlock &operator=(const VertexBlock &rhs) = delete;
    VertexBlock(VertexBlock &&rhs) = default;
    VertexBlock &operator=(VertexBlock &&rhs) = default;
  };

  struct MorphTargetBlock {
    MorphTargetBlock(BufferAccess in_positions, BufferAccess in_tangents)
        : positions(std::move(in_positions)),
          tangents(std::move(in_tangents)) {}
    BufferAccess positions;
    BufferAccess tangents;
  };

  struct VertexBuffer {
    std::vector<VertexBlock> blocks;
    size_t vertex_count;
    bool advanced_skinning;

    VertexBuffer(std::vector<VertexBlock> in_blocks, size_t in_vertex_count,
                 bool in_advanced_skinning)
        : blocks(std::move(in_blocks)),
          vertex_count(in_vertex_count),
          advanced_skinning(in_advanced_skinning) {}

    VertexBuffer(const VertexBuffer &) = delete;
    VertexBuffer &operator=(const VertexBuffer &rhs) = delete;
    VertexBuffer(VertexBuffer &&rhs) = default;
    VertexBuffer &operator=(VertexBuffer &&rhs) = default;
  };

  struct Texture {
    std::string image_name;
    schemas::TextureInfoFlags texture_info_flags;
    ImageData image_data;

    Texture(absl::string_view in_image_name,
            schemas::TextureInfoFlags in_texture_info_flags,
            ImageData in_image_data)
        : image_name(in_image_name),
          texture_info_flags(in_texture_info_flags),
          image_data(std::move(in_image_data)) {}

    Texture(const Texture &) = delete;
    Texture &operator=(const Texture &rhs) = delete;
    Texture(Texture &&rhs) = default;
    Texture &operator=(Texture &&rhs) = default;
  };

  LoadedModelBuilder() = default;

  // Checks to see if a given buffer (defined as a set of blocks) has already
  // been added to the flatbuffer, otherwise adds it.
  VertexBufferId AddVertexBuffer(std::vector<VertexBlock> blocks,
                                 size_t vertex_count, bool advanced_skinning);

  // Checks to see if a given buffer (defined with an attribute layout) has
  // already been added to the flatbuffer, otherwise adds it.
  IndexBufferId AddIndexBuffer(std::optional<DenseDataAccess> &indices_data,
                               size_t index_count);

  MorphTargetBufferId AddMorphTargetBuffer(std::vector<MorphTargetBlock> blocks,
                                           size_t vertex_count);

  SkinningBufferId AddSkinningBuffer(
      const std::vector<float2> &bone_indices_and_weights);

  // Adds a encoded or compressed texture to the flatbuffer.
  // Lookup index is the index of the texture in the original gltf schema.
  // Returns the index of the added texture which may not be the same as the
  // lookup index. This index can be used to refer to the texture in other
  // functions.
  TextureId AddTexture(uint16_t lookup_index, absl::string_view image_name,
                       schemas::TextureInfoFlags texture_info_flags,
                       ImageData image_data);

  // Returns the index of a previously added texture given the lookup index.
  TextureId GetTexture(uint16_t lookup_index) const;

  // Adds a material using previously added parameters and textures.
  MaterialId AddMaterial(
      uint16_t lookup_index,
      flatbuffers::Offset<schemas::MaterialInfo> material_offset);

  // Returns the index of a previously added material given the lookup index.
  MaterialId GetMaterial(uint16_t lookup_index) const;

  // Returns the index of a previously added skin given the lookup index.
  SkinId GetSkin(uint32_t lookup_index) const;

  // Returns a mask of all the vertex attributes required by a given material.
  std::optional<VertexAttributeMask> GetRequiredAttributes(MaterialId material);

  MaterialsVariantsId AddMaterialsVariants(absl::string_view name);

  // Reserves structure-of-array storage for the bones used by the model.
  void ReserveBones(size_t size);

  // Adds the next bone whose space was previously reserved.
  absl::Status AddBone(uint16_t num_children, PreciseTransform local_transform,
                       absl::string_view name, uint16_t node_index);

  // Computes remainder of bone data (root transforms, topology)
  absl::Status FinishBones();

  // Reserves structure-of-array storage for the entities defined in the model.
  void ReserveEntities(size_t size);
  // Adds a previously reserved entity using previously added parts.
  absl::StatusOr<EntityId> AddEntity(
      BoneId bone, SkinId skin, MorphTargetBufferId morph_target_buffer,
      std::vector<float> node_morph_target_weights,
      std::vector<float> mesh_morph_target_weights,
      LightPunctualId light_punctual, AudioEmitterId audio_emitter,
      std::vector<PartData> parts, std::optional<filament::Box> bounds,
      std::optional<RuntimeData> runtime, int child_count,
      absl::string_view name, uint16_t original_index = 0,
      int original_mesh_index = -1, int original_skin_index = -1,
      std::optional<NodeVisibility> node_visibility = std::nullopt,
      std::optional<NodeSelectability> node_selectability = std::nullopt,
      std::optional<NodeHoverability> node_hoverability = std::nullopt);
  absl::Status FinishEntities();

  using WeakSkinId = imp::TypedId<model::SkinData, int>;

  // Adds a SkinInfo to the model and returns an ID to it.
  absl::StatusOr<SkinId> AddSkin(uint32_t lookup_index,
                                 model::ModelData::SkinData skin_data);

  absl::Status AddSkinEntity(
      SkinId skin, EntityId target,
      SampledJointLookup<filament::Aabb> sampled_joint_bounds,
      PairedBitVector<model::ModelData::SampledJointData> sampled_joint_in_use);

  absl::Status FinishSkins();

  const TypedSetVector<BoneData> &Bones() const;
  PairedSpan<const mat4, BoneData> BoneRootTransforms() const;

  LightPunctualId AddLightPunctual(
      absl::string_view name, const schemas::Color &color, float intensity,
      schemas::LightPunctualType type, float range,
      const schemas::SpotConeAngles &spot_cone_angles);

  absl::StatusOr<AnimationId> AddAnimation(AnimationAccess access);

  void RemoveShadowPlanes(const filament::Box &scene_bounds);

  AudioEmitterOffset AddAudioEmitter(
      absl::string_view name, schemas::AudioEmitterType type, float gain,
      const std::vector<uint16_t> &source_indices,
      AudioEmitterPositionalOffset positional_offset = 0);
  AudioEmitterPositionalOffset AddAudioEmitterPositional(
      float cone_inner_angle, float cone_outer_angle, float cone_outer_gain,
      schemas::AudioEmitterDistanceModel distance_model, float max_distance,
      float ref_distance, float rolloff_factor);
  AudioSourceOffset AddAudioSource(absl::string_view name, bool auto_play,
                                   float gain, bool loop, uint16_t audio);
  AudioOffset AddAudio(const BufferAccess &audio_buffer);

  void AddAudioExtension(const AudioEmitterOffsets &audio_emitter_offsets,
                         const AudioSourceOffsets &audio_source_offsets,
                         const AudioOffsets &audio_offsets,
                         const std::vector<uint16_t> &scene_emitters);

  void AddBehavior(const BehaviorOffset &behavior_offset);

  std::unique_ptr<BehaviorLoaderExtension> CreateBehaviorLoaderExtension();

  void AddInteractivity(const InteractivityOffset &interactivity_offset);

  std::unique_ptr<InteractivityLoaderExtension>
  CreateInteractivityLoaderExtension();

  flatbuffers::FlatBufferBuilder &GetFlatBufferBuilder();

  // Concludes building and returns an error or a built result.
  absl::StatusOr<LoadedModelAccess> Finish();

 protected:
  absl::StatusOr<ImageOffset> AddImageData(const ImageData &image_data);
  absl::StatusOr<flatbuffers::Offset<schemas::LoadedModel>> Serialize();

  flatbuffers::FlatBufferBuilder fbb_;

  TypedSetVector<model::BoneData> bones_;
  TypedSetVector<model::ModelData::EntityData> entities_;
  SkinLookup<model::ModelData::SkinData> skins_;
  PairedVector<Texture, TextureId::ReferredType> textures_;

  PairedBitVector<MaterialId::ReferredType> is_shadow_plane_material_;
  PairedVector<AnimationAccess, AnimationId::ReferredType> animations_;

  PairedVector<VertexBuffer, VertexBufferId::ReferredType> vertex_buffers_;
  IndexBufferOffsets index_buffer_offsets_;
  SkinOffsets skin_offsets_;
  MaterialOffsets material_offsets_;
  LightPunctualOffsets light_punctual_offsets_;
  MaterialsVariantsOffsets materials_variants_offsets_;
  MorphTargetBufferOffsets morph_target_buffer_offsets_;
  AudioExtensionOffset audio_extension_offset_;
  BehaviorOffset behavior_offset_;
  InteractivityOffset interactivity_offset_;
  SkinningBufferOffsets skinning_buffer_offsets_;

  // Lookup structures to accelerate searches.
  RobinMap<std::string, TextureId> dest_from_name_texture_cache_;
  RobinMap<uint16_t, TextureId> texture_from_lookup_index_;
  RobinMap<uint16_t, MaterialId> material_from_lookup_index_;
  RobinMap<uint32_t, SkinId> skin_from_lookup_index_;
};

}  // namespace imp::loader::details
#endif  // THIRD_PARTY_ARCORE_AR_IMP_CORE_LOADER_PROVIDER_DETAILS_LOADED_MODEL_BUILDER_H_
