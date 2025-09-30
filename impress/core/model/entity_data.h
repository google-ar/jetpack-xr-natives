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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_ENTITY_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_ENTITY_DATA_H_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/strings/cord.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/common/data_helpers.h"
#include "core/common/enum_flags.h"
#include "core/common/paired_vector.h"
#include "core/common/typed_id.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_tree.h"
#include "core/common/typed_vector.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/material_param_value.h"
#include "core/math/math.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/shared_data.h"
#include "core/model/skeleton_data.h"
#include "core/model/skin_data.h"
#include "core/render/texture.h"
#include "core/view/utils/string_map.h"

namespace imp::model {

using PrimitiveType = filament::RenderableManager::PrimitiveType;
using VertexBufferId =
    TypedIdWithSentinel<filament::VertexBuffer*, uint16_t, kMaxValue<uint16_t>>;
using IndexBufferId =
    TypedIdWithSentinel<filament::IndexBuffer*, uint16_t, kMaxValue<uint16_t>>;
using MorphTargetBufferId = TypedIdWithSentinel<filament::MorphTargetBuffer*,
                                                uint16_t, kMaxValue<uint16_t>>;

using PrimitiveType = filament::RenderableManager::PrimitiveType;
using VertexBufferId =
    TypedIdWithSentinel<filament::VertexBuffer*, uint16_t, kMaxValue<uint16_t>>;
using IndexBufferId =
    TypedIdWithSentinel<filament::IndexBuffer*, uint16_t, kMaxValue<uint16_t>>;
using MorphTargetBufferId = TypedIdWithSentinel<filament::MorphTargetBuffer*,
                                                uint16_t, kMaxValue<uint16_t>>;

using SamplerId = TypedIdWithSentinel<filament::TextureSampler, uint16_t,
                                      kMaxValue<uint16_t>>;
using TextureId =
    TypedIdWithSentinel<OwnedTexturePtr, uint16_t, kMaxValue<uint16_t>>;

template <typename T>
using SamplerLookup = PairedVector<T, SamplerId::ReferredType>;
template <typename T>
using TextureLookup = PairedVector<T, TextureId::ReferredType>;

struct MaterialTexture {
  MaterialTexture(TextureId in_texture, SamplerId in_sampler)
      : texture(in_texture), sampler(in_sampler) {}

  TextureId texture;
  SamplerId sampler;
};

using MaterialTextureId =
    TypedIdWithSentinel<MaterialTexture, uint16_t, kMaxValue<uint16_t>>;

struct MaterialParameter {
  using ParamVariant =
      std::variant<float, float2, float3, float4,  //
                   int, int2, int3, int4,          //
                   bool, bool2, bool3, bool4,      //
                   mat3f, std::vector<mat3f>, TextureAndSampler>;

  MaterialParameter(absl::string_view in_name, ParamVariant in_value)
      : name(in_name), value(std::move(in_value)) {}

  std::string name;
  ParamVariant value;

  inline bool operator==(const MaterialParameter& other) const {
    return name == other.name && value == other.value;
  }
};

using MaterialParameterId =
    TypedIdWithSentinel<MaterialParameter, uint16_t, kMaxValue<uint16_t>>;

using MaterialId =
    TypedIdWithSentinel<GenericMaterialPtr, uint16_t, kMaxValue<uint16_t>>;

using MaterialId =
    TypedIdWithSentinel<GenericMaterialPtr, uint16_t, kMaxValue<uint16_t>>;

// TODO: (broken link) - Remove MaterialConfig entirely.
struct MaterialConfig {
  MaterialConfig(absl::string_view in_name,
                 std::vector<MaterialParameter> in_params,
                 TypedVector<MaterialTexture> in_textures,
                 StringMap<int> in_sampler_index_lookup)
      : name(in_name),
        params(std::move(in_params)),
        textures(std::move(in_textures)) {}

  std::string name;
  std::vector<MaterialParameter> params;
  TypedVector<MaterialTexture> textures;
  StringMap<int> sampler_index_lookup;
};

template <typename T>
using MaterialLookup = PairedVector<T, MaterialId::ReferredType>;

using MeshVertexDataLookup =
    PairedVector<MeshVertexDataPtr, VertexBufferId::ReferredType>;
using MeshIndexDataLookup =
    PairedVector<MeshIndexDataPtr, IndexBufferId::ReferredType>;

struct MaterialsVariantsData;
using MaterialsVariantsId = TypedId<MaterialsVariantsData, uint16_t>;

struct MaterialsVariantsData {
  std::string name;
};
using MaterialsVariantsMappingLookup =
    PairedVector<MaterialId, MaterialsVariantsId::ReferredType>;

struct SkinningBufferData;
using SkinningBufferId =
    TypedIdWithSentinel<SkinningBufferData, uint16_t, kMaxValue<uint16_t>>;

struct SkinningBufferData {
  std::vector<float2> bone_indices_and_weights;
};

struct LightPunctualData;
using LightPunctualId = TypedId<LightPunctualData, int16_t>;

struct LightPunctualData {
  std::string name;
  filament::math::float3 color;
  float intensity;
  schemas::LightPunctualType type;
  float range;
  std::optional<filament::math::float2> spot_cone_angles;
};

struct AudioEmitterData;
struct AudioSourceData;
struct AudioData;

using AudioEmitterId = TypedIdWithSentinel<AudioEmitterData, uint16_t>;
using AudioSourceId = TypedIdWithSentinel<AudioSourceData, uint16_t>;
using AudioId = TypedIdWithSentinel<AudioData, uint16_t>;

// AudioEmitterData contains data on how/which AudioSource should be set up
// for playing audio.
// Please note that although positional audio can be specified in KHR_audio
// extension, impress currently does not support positional audio and all the
// positional audio data from gltf will be ignored. All positional audio will
// be treated as global audio.
struct AudioEmitterData {
  std::string name;
  schemas::AudioEmitterType type;
  float gain;
  std::vector<AudioSourceId> audio_sources;
  // TODO: Add support for positional emitter.
};

struct AudioSourceData {
  std::string name;
  bool auto_play;
  float gain;
  bool loop;
  AudioId audio;
};

struct AudioData {
  absl::Cord data;
};

// A Model has Entities and Entities have Parts (in their mesh renderer).
// A Part ~== A Draw Call (material+vertex buffer+index buffer)
struct PartData {
  PartData(std::string in_name, uint32_t in_index_offset,
           uint32_t in_index_count, VertexBufferId in_vertex_buffer,
           IndexBufferId in_index_buffer, MaterialId in_material,
           PrimitiveType in_primitive_type,
           MaterialsVariantsMappingLookup in_materials_variants_mappings,
           SkinningBufferId in_skinning_buffer,
           uint32_t in_morph_target_buffer_offset,
           uint32_t in_morph_target_buffer_count)
      : name(std::move(in_name)),
        index_offset(in_index_offset),
        index_count(in_index_count),
        vertex_buffer(in_vertex_buffer),
        index_buffer(in_index_buffer),
        material(in_material),
        primitive_type(in_primitive_type),
        materials_variants_mappings(std::move(in_materials_variants_mappings)),
        skinning_buffer(in_skinning_buffer),
        morph_target_buffer_offset(in_morph_target_buffer_offset),
        morph_target_buffer_count(in_morph_target_buffer_count) {}

  std::string name;
  uint32_t index_offset;
  uint32_t index_count;
  VertexBufferId vertex_buffer;
  IndexBufferId index_buffer;
  MaterialId material;
  PrimitiveType primitive_type;
  MaterialsVariantsMappingLookup materials_variants_mappings;
  SkinningBufferId skinning_buffer;
  uint32_t morph_target_buffer_offset;
  uint32_t morph_target_buffer_count;
};

enum class RenderFlags : uint8_t {
  DoNotCastShadows = (1 << 0),
  DoNotReceiveShadows = (1 << 1),
  DisableFrustumCulling = (1 << 2),
  Empty = 0,
};
struct RuntimeData {
  RuntimeData(Flags<RenderFlags> in_flags, uint8_t in_priority)
      : flags(in_flags), priority(in_priority) {}

  Flags<RenderFlags> flags;
  uint8_t priority;

  static RuntimeData Default() {
    // All flags off; render priority of 4.
    return RuntimeData{{}, 4};
  }

  static RuntimeData HighPriority() { return RuntimeData{{}, 3}; }
};

// Data for KHR_node_visibility.
struct NodeVisibility {
  std::optional<bool> visible;
};

// Data for KHR_node_selectability.
struct NodeSelectability {
  std::optional<bool> selectable;
};

// Data for KHR_node_hoverability.
struct NodeHoverability {
  std::optional<bool> hoverable;
};

// EntityData describes a single entity in a model.
using EntityParentId = TypedParentId<EntityData, uint16_t>;
using EntityChildId = TypedDescendantId<EntityData, uint16_t>;
template <typename T>
using EntityLookup = PairedVector<T, EntityData>;
struct EntityData {
  // Declare the SoA type (with field types for each field).
  using ArrayType = StructureOfArrays<
      uint16_t, EntityParentId, EntityChildId, EntityChildId,
      std::vector<PartData>, BoneId, SkinId, MorphTargetBufferId,
      std::vector<float>, LightPunctualId, AudioEmitterId,
      std::optional<filament::Box>, std::optional<RuntimeData>, std::string,
      uint16_t, std::optional<NodeVisibility>, std::optional<NodeSelectability>,
      std::optional<NodeHoverability>>;
  // Declare an enum to access fields (e.g. the iterator type in
  // utils::StructureOfArrays uses a tuple style get<>() API).
  enum Fields {
    kNumChildren,
    kParent,
    kFirstChild,
    kNextSibling,
    kParts,
    kBone,
    kSkin,
    kMorphTargetBuffer,
    kMorphTargetWeights,
    kLightPunctual,
    kAudioEmitter,
    kLocalBounds,
    kRuntime,
    kName,
    kOriginalIndex,
    kNodeVisibility,
    kNodeSelectability,
    kNodeHoverability,
  };
  union Proxy {
    template <size_t E>
    using Field = ArrayType::Field<E>;
    Field<kNumChildren> num_children;
    Field<kParent> parent;
    Field<kFirstChild> first_child;
    Field<kNextSibling> next_sibling;
    Field<kParts> parts;
    Field<kBone> bone;
    Field<kSkin> skin;
    Field<kMorphTargetBuffer> morph_target_buffer;
    Field<kMorphTargetWeights> morph_target_weights;
    Field<kLightPunctual> light_punctual;
    Field<kAudioEmitter> audio_emitter;
    Field<kLocalBounds> local_bounds;
    Field<kRuntime> runtime;
    Field<kName> name;
    Field<kOriginalIndex> original_index;
    Field<kNodeVisibility> node_visibility;
    Field<kNodeSelectability> node_selectability;
    Field<kNodeHoverability> node_hoverability;
  };
};

}  // namespace imp::model

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_ENTITY_DATA_H_
