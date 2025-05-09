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

#include <cstddef>
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
#include "core/material_library/generic_material.h"
#include "core/material_library/material_param_value.h"
#include "core/math/math.h"
#include "core/model/behavior_data.h"
#include "core/model/entity_data.h"
#include "core/model/interactivity_data.h"
#include "core/model/joint_data.h"
#include "core/model/shared_data.h"
#include "core/model/skeleton_data.h"
#include "core/model/skin_data.h"
#include "core/view/base_view.h"

namespace imp::model {

class ModelData {
 public:
  using EntityData = ::imp::model::EntityData;
  using EntityId = ::imp::model::EntityId;
  using WeakEntityId = ::imp::model::WeakEntityId;
  template <typename T>
  using EntityLookup = ::imp::model::EntityLookup<T>;
  using EntityParentId = ::imp::model::EntityParentId;
  using EntityChildId = ::imp::model::EntityChildId;

  using JointData = ::imp::model::JointData;
  using JointId = ::imp::model::JointId;
  using WeakJointId = ::imp::model::WeakJointId;
  template <typename T>
  using JointLookup = ::imp::model::JointLookup<T>;
  using JointChildId = ::imp::model::JointChildId;
  using JointParentId = ::imp::model::JointParentId;

  using SkinData = ::imp::model::SkinData;
  using SkinId = ::imp::model::SkinId;
  using SkinningBufferId = ::imp::model::SkinningBufferId;
  template <typename T>
  using SkinLookup = ::imp::model::SkinLookup<T>;
  using SkinnedEntityData = ::imp::model::SkinnedEntityData;
  using SkinnedEntityId = ::imp::model::SkinnedEntityId;
  using SkinningBufferData = ::imp::model::SkinningBufferData;

  using SampledJointData = ::imp::model::SampledJointData;
  using SampledJointId = ::imp::model::SampledJointId;
  using WeakSampledJointId = ::imp::model::WeakSampledJointId;
  template <typename T>
  using SampledJointLookup = ::imp::model::SampledJointLookup<T>;

  using MaterialId = ::imp::MaterialId;
  template <typename T>
  using MaterialLookup = ::imp::model::MaterialLookup<T>;
  using MaterialsVariantsMappingLookup =
      ::imp::model::MaterialsVariantsMappingLookup;
  using MaterialsVariantsId = ::imp::model::MaterialsVariantsId;
  using MaterialParameter = ::imp::MaterialParameter;
  using MaterialParameterId = ::imp::MaterialParameterId;
  using MaterialTexture = ::imp::MaterialTexture;
  using MaterialTextureId = ::imp::MaterialTextureId;
  using TextureId = ::imp::TextureId;
  using SamplerId = ::imp::SamplerId;
  using MaterialConfig = ::imp::model::MaterialConfig;
  using MaterialsVariantsData = ::imp::model::MaterialsVariantsData;

  using VertexBufferId = ::imp::model::VertexBufferId;
  using IndexBufferId = ::imp::model::IndexBufferId;
  using MorphTargetBufferId = ::imp::model::MorphTargetBufferId;
  using PrimitiveType = ::imp::model::PrimitiveType;
  using PartData = ::imp::model::PartData;
  using MeshVertexDataLookup = ::imp::model::MeshVertexDataLookup;
  using MeshIndexDataLookup = ::imp::model::MeshIndexDataLookup;
  using RuntimeData = ::imp::model::RuntimeData;
  using RenderFlags = ::imp::model::RenderFlags;

  using NodeVisibility = ::imp::model::NodeVisibility;
  using NodeSelectability = ::imp::model::NodeSelectability;
  using NodeHoverability = ::imp::model::NodeHoverability;

  using AudioSourceData = ::imp::model::AudioSourceData;
  using AudioSourceId = ::imp::model::AudioSourceId;
  using AudioData = ::imp::model::AudioData;
  using AudioId = ::imp::model::AudioId;
  using AudioEmitterData = ::imp::model::AudioEmitterData;
  using AudioEmitterId = ::imp::model::AudioEmitterId;

  using LightPunctualData = ::imp::model::LightPunctualData;
  using LightPunctualId = ::imp::model::LightPunctualId;

  using BehaviorData = ::imp::model::BehaviorData;
  using BehaviorCustomEventId = ::imp::model::BehaviorCustomEventId;
  using BehaviorVariableId = ::imp::model::BehaviorVariableId;
  using BehaviorNodeId = ::imp::model::BehaviorNodeId;
  using BehaviorNodeValueId = ::imp::model::BehaviorNodeValueId;
  using BehaviorNodeConfigurationId = ::imp::model::BehaviorNodeConfigurationId;
  using BehaviorNodeFlowId = ::imp::model::BehaviorNodeFlowId;

  using InteractivityData = ::imp::model::InteractivityData;
  using InteractivityGraphData = ::imp::model::InteractivityData::GraphData;
  using InteractivityGraphId = ::imp::model::InteractivityGraphId;
  using InteractivityEventId = ::imp::model::InteractivityEventId;
  using InteractivityVariableId = ::imp::model::InteractivityVariableId;
  using InteractivityNodeId = ::imp::model::InteractivityNodeId;
  using InteractivityNodeValueId = ::imp::model::InteractivityNodeValueId;
  using InteractivityNodeConfigurationId =
      ::imp::model::InteractivityNodeConfigurationId;
  using InteractivityNodeFlowId = ::imp::model::InteractivityNodeFlowId;

  using InteractivityDeclarationId = ::imp::model::InteractivityDeclarationId;

  ModelData(imp::BaseView* view, filament::Engine* engine,
            TypedSetVector<EntityData> entities, TypedVector<SkinData> skins,
            TypedVector<LightPunctualData> lights_punctual,
            TypedVector<MaterialsVariantsData> materials_variants,
            SkeletonData skeleton,
            TypedVector<filament::VertexBuffer*> vertex_buffers,
            TypedVector<filament::IndexBuffer*> index_buffers,
            TypedVector<filament::MorphTargetBuffer*> morph_target_buffers,
            TypedVector<filament::Texture*> textures,
            TypedVector<GenericMaterialPtr> materials,
            absl::flat_hash_map<uint16_t, MaterialId> material_id_lookup,
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
  const TypedVector<filament::Texture*>& Textures() const;
  const TypedVector<GenericMaterialPtr>& Materials() const;
  const TypedVector<MaterialsVariantsData>& MaterialsVariants() const;
  // TODO: (broken link) - Remove MaterialConfig entirely.
  const MaterialLookup<MaterialConfig>& MaterialConfigs() const;
  const GenericMaterial* GetMaterial(absl::string_view material_name) const;
  MaterialId GetMaterialId(uint16_t material_index) const;
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
  TypedVector<filament::Texture*> textures_;
  TypedVector<GenericMaterialPtr> materials_;
  absl::flat_hash_map<uint16_t, MaterialId> material_id_lookup_;
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
