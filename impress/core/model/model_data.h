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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MODEL_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MODEL_DATA_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Box.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_vector.h"
#include "core/material_library/material_param_value.h"
#include "core/math/math.h"
#include "core/model/behavior_data.h"
#include "core/model/entity_data.h"
#include "core/model/interactivity_data.h"
#include "core/model/joint_data.h"
#include "core/model/shared_data.h"
#include "core/model/skeleton_data.h"
#include "core/model/skin_data.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"

namespace imp::model {

class ModelData {
 public:
  // TODO: (broken link) - delete these using redefinitions.
  using EntityData = EntityData;
  using EntityId = EntityId;
  using WeakEntityId = WeakEntityId;
  template <typename T>
  using EntityLookup = EntityLookup<T>;
  using EntityParentId = EntityParentId;
  using EntityChildId = EntityChildId;

  using JointData = JointData;
  using JointId = JointId;
  using WeakJointId = WeakJointId;
  template <typename T>
  using JointLookup = JointLookup<T>;
  using JointChildId = JointChildId;
  using JointParentId = JointParentId;

  using SkinData = SkinData;
  using SkinId = SkinId;
  using SkinningBufferId = SkinningBufferId;
  template <typename T>
  using SkinLookup = SkinLookup<T>;
  using SkinnedEntityData = SkinnedEntityData;
  using SkinnedEntityId = SkinnedEntityId;
  using SkinningBufferData = SkinningBufferData;

  using SampledJointData = SampledJointData;
  using SampledJointId = SampledJointId;
  using WeakSampledJointId = WeakSampledJointId;
  template <typename T>
  using SampledJointLookup = SampledJointLookup<T>;

  using MaterialId = MaterialId;
  template <typename T>
  using MaterialLookup = MaterialLookup<T>;
  using MaterialsVariantsMappingLookup = MaterialsVariantsMappingLookup;
  using MaterialsVariantsId = MaterialsVariantsId;
  using MaterialParameter = MaterialParameter;
  using MaterialParameterId = MaterialParameterId;
  using MaterialTexture = MaterialTexture;
  using MaterialTextureId = MaterialTextureId;
  using TextureId = TextureId;
  using SamplerId = SamplerId;
  using MaterialConfig = MaterialConfig;
  using MaterialsVariantsData = MaterialsVariantsData;

  using VertexBufferId = VertexBufferId;
  using IndexBufferId = IndexBufferId;
  using MorphTargetBufferId = MorphTargetBufferId;
  using PrimitiveType = PrimitiveType;
  using PartData = PartData;
  using MeshVertexDataLookup = MeshVertexDataLookup;
  using MeshIndexDataLookup = MeshIndexDataLookup;
  using RuntimeData = RuntimeData;
  using RenderFlags = RenderFlags;

  using NodeVisibility = NodeVisibility;
  using NodeSelectability = NodeSelectability;
  using NodeHoverability = NodeHoverability;

  using AudioSourceData = AudioSourceData;
  using AudioSourceId = AudioSourceId;
  using AudioData = AudioData;
  using AudioId = AudioId;
  using AudioEmitterData = AudioEmitterData;
  using AudioEmitterId = AudioEmitterId;

  using LightPunctualData = LightPunctualData;
  using LightPunctualId = LightPunctualId;

  using BehaviorData = BehaviorData;
  using BehaviorCustomEventId = BehaviorCustomEventId;
  using BehaviorVariableId = BehaviorVariableId;
  using BehaviorNodeId = BehaviorNodeId;
  using BehaviorNodeValueId = BehaviorNodeValueId;
  using BehaviorNodeConfigurationId = BehaviorNodeConfigurationId;
  using BehaviorNodeFlowId = BehaviorNodeFlowId;

  using InteractivityData = InteractivityData;
  using InteractivityGraphData = InteractivityData::GraphData;
  using InteractivityGraphId = InteractivityGraphId;
  using InteractivityEventId = InteractivityEventId;
  using InteractivityVariableId = InteractivityVariableId;
  using InteractivityNodeId = InteractivityNodeId;
  using InteractivityNodeValueId = InteractivityNodeValueId;
  using InteractivityNodeConfigurationId = InteractivityNodeConfigurationId;
  using InteractivityNodeFlowId = InteractivityNodeFlowId;
  using InteractivityTypeId = InteractivityTypeId;

  using InteractivityDeclarationId = InteractivityDeclarationId;

  ModelData(imp::BaseView* view, filament::Engine* engine,
            TypedSetVector<EntityData> entities, TypedVector<SkinData> skins,
            TypedVector<LightPunctualData> lights_punctual,
            TypedVector<MaterialsVariantsData> materials_variants,
            SkeletonData skeleton,
            TypedVector<filament::VertexBuffer*> vertex_buffers,
            TypedVector<filament::IndexBuffer*> index_buffers,
            TypedVector<filament::MorphTargetBuffer*> morph_target_buffers,
            TypedVector<OwnedTexturePtr> textures,
            TypedVector<GenericMaterialPtr> materials,
            absl::flat_hash_map<uint16_t, MaterialId> material_id_lookup,
            absl::flat_hash_map<uint32_t, SkinId> skin_id_lookup,
            TypedVector<SkinningBufferData> skinning_buffers,
            MeshVertexDataLookup stored_vertex_data,
            MeshIndexDataLookup stored_index_data,
            MaterialLookup<MaterialConfig> material_configs,
            TypedVector<AudioEmitterData> audio_emitters,
            TypedVector<AudioSourceData> audio_sources,
            TypedVector<AudioData> audios,
            std::vector<AudioEmitterId> scene_audio_emitters,
            std::optional<BehaviorData> behavior,
            std::optional<InteractivityData> interactivity);

  // Disposes of all filament resources using saved engine pointer.
  ~ModelData();

  filament::Box GetAxisAlignedBounds() const;
  filament::Engine* Engine() const;
  const TypedSetVector<EntityData>& Entities() const;
  const TypedVector<OwnedTexturePtr>& Textures() const;
  const TypedVector<GenericMaterialPtr>& Materials() const;
  const TypedVector<MaterialsVariantsData>& MaterialsVariants() const;
  // TODO: (broken link) - Remove MaterialConfig entirely.
  const MaterialLookup<MaterialConfig>& MaterialConfigs() const;
  const GenericMaterial* GetMaterial(absl::string_view material_name) const;
  MaterialId GetMaterialId(uint16_t material_index) const;
  SkinId GetSkinId(uint32_t skin_index) const;
  const SkeletonData& Skeleton() const;
  const TypedVector<SkinData>& Skins() const;
  const TypedVector<LightPunctualData>& LightsPunctual() const;
  const TypedVector<filament::VertexBuffer*>& VertexBuffers() const;
  const TypedVector<filament::IndexBuffer*>& IndexBuffers() const;
  const TypedVector<filament::MorphTargetBuffer*>& MorphTargetBuffers() const;
  // This is only used if the glTF has >4 weights, otherwise this is empty.
  const TypedVector<SkinningBufferData>& SkinningBuffers() const;
  const MeshVertexDataLookup& GetStoredVertexData() const;
  const MeshIndexDataLookup& GetStoredIndexData() const;
  const TypedVector<AudioEmitterData>& AudioEmitters() const;
  const TypedVector<AudioSourceData>& AudioSources() const;
  const TypedVector<AudioData>& Audios() const;
  const std::vector<AudioEmitterId>& SceneAudioEmitters() const;
  const BehaviorData* Behavior() const;
  const InteractivityData* Interactivity() const;

 private:
  imp::BaseView* view_;
  filament::Engine* engine_;
  TypedSetVector<EntityData> entities_;
  TypedVector<SkinData> skins_;
  TypedVector<LightPunctualData> lights_punctual_;
  TypedVector<MaterialsVariantsData> materials_variants_;
  SkeletonData skeleton_;
  TypedVector<filament::VertexBuffer*> vertex_buffers_;
  TypedVector<filament::IndexBuffer*> index_buffers_;
  TypedVector<filament::MorphTargetBuffer*> morph_target_buffers_;
  TypedVector<OwnedTexturePtr> textures_;
  TypedVector<GenericMaterialPtr> materials_;
  absl::flat_hash_map<uint16_t, MaterialId> material_id_lookup_;
  absl::flat_hash_map<uint32_t, SkinId> skin_id_lookup_;
  TypedVector<SkinningBufferData> skinning_buffers_;

  // Vertex and index information stored on CPU
  MeshVertexDataLookup stored_vertex_data_;
  MeshIndexDataLookup stored_index_data_;
  MaterialLookup<MaterialConfig> material_configs_;

  TypedVector<AudioEmitterData> audio_emitters_;
  TypedVector<AudioSourceData> audio_sources_;
  TypedVector<AudioData> audios_;
  std::vector<AudioEmitterId> scene_audio_emitters_;

  std::optional<BehaviorData> behavior_;

  std::optional<InteractivityData> interactivity_;
};

}  // namespace imp::model

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MODEL_DATA_H_
