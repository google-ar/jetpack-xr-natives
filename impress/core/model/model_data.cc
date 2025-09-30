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

#include "core/model/model_data.h"

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/Engine.h"
#include "core/common/filament_helpers.h"
#include "core/common/paired_span.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_vector.h"
#include "core/material_library/generic_material.h"
#include "core/material_library/material_param_value.h"
#include "core/math/math.h"
#include "core/model/behavior_data.h"
#include "core/model/entity_data.h"
#include "core/model/interactivity_data.h"
#include "core/model/skeleton_data.h"
#include "core/model/skin_data.h"
#include "core/render/texture.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"

namespace imp::model {

using ::filament::LinearColorA;

ModelData::ModelData(
    imp::BaseView* view, filament::Engine* engine,
    TypedSetVector<EntityData> entities, TypedVector<SkinData> skins,
    TypedVector<LightPunctualData> lights_punctual,
    TypedVector<MaterialsVariantsData> materials_variants,
    SkeletonData skeleton, TypedVector<filament::VertexBuffer*> vertex_buffers,
    TypedVector<filament::IndexBuffer*> index_buffers,
    TypedVector<filament::MorphTargetBuffer*> morph_target_buffers,
    TypedVector<OwnedTexturePtr> textures,
    TypedVector<GenericMaterialPtr> materials,
    absl::flat_hash_map<uint16_t, MaterialId> material_id_lookup,
    TypedVector<SkinningBufferData> skinning_buffers,
    MeshVertexDataLookup stored_vertex_data,
    MeshIndexDataLookup stored_index_data,
    MaterialLookup<MaterialConfig> material_configs,
    TypedVector<AudioEmitterData> audio_emitters,
    TypedVector<AudioSourceData> audio_sources, TypedVector<AudioData> audios,
    std::vector<AudioEmitterId> scene_audio_emitters,
    std::optional<BehaviorData> behavior,
    std::optional<InteractivityData> interactivity)

    : view_(view),
      engine_(engine),
      entities_(std::move(entities)),
      skins_(std::move(skins)),
      lights_punctual_(std::move(lights_punctual)),
      materials_variants_(std::move(materials_variants)),
      skeleton_(std::move(skeleton)),
      vertex_buffers_(std::move(vertex_buffers)),
      index_buffers_(std::move(index_buffers)),
      morph_target_buffers_(std::move(morph_target_buffers)),
      textures_(std::move(textures)),
      materials_(std::move(materials)),
      material_id_lookup_(std::move(material_id_lookup)),
      skinning_buffers_(std::move(skinning_buffers)),
      stored_vertex_data_(std::move(stored_vertex_data)),
      stored_index_data_(std::move(stored_index_data)),
      material_configs_(std::move(material_configs)),
      audio_emitters_(std::move(audio_emitters)),
      audio_sources_(std::move(audio_sources)),
      audios_(std::move(audios)),
      scene_audio_emitters_(std::move(scene_audio_emitters)),
      behavior_(std::move(behavior)),
      interactivity_(std::move(interactivity)) {}

ModelData::~ModelData() {
  // TODO: Make a helper destroy function so that we don't forget
  // to remove resources in split engine.
  split_engine::SplitEngineSerializer* serializer =
      view_->GetSplitEngineSerializer();

  for (auto* vertex_buffer : vertex_buffers_) {
    if (serializer != nullptr) {
      serializer->RemoveVertexBuffer(vertex_buffer);
    }
    engine_->destroy(vertex_buffer);
  }
  for (auto* index_buffer : index_buffers_) {
    if (serializer != nullptr) {
      serializer->RemoveIndexBuffer(index_buffer);
    }
    engine_->destroy(index_buffer);
  }
  for (auto* morph_target_buffer : morph_target_buffers_) {
    if (serializer != nullptr) {
      serializer->RemoveMorphTargetBuffer(morph_target_buffer);
    }
    engine_->destroy(morph_target_buffer);
  }
}

filament::Box ModelData::GetAxisAlignedBounds() const {
  PairedSpan<const mat4, BoneData> root_transforms =
      skeleton_.bones.Span<BoneData::kRootTransform>();
  // PairedSpan<const mat4f,
  filament::Box merged_bounds = absl::c_accumulate(
      entities_.Ids<EntityId>(), NilBounds(),
      [&root_transforms, this](filament::Box merged_bounds, EntityId entity) {
        auto proxy = entities_[entity];
        if (const std::vector<PartData>& parts = proxy.parts; parts.empty()) {
          return merged_bounds;
        }
        const absl::optional<filament::Box>& local_bounds = proxy.local_bounds;
        const BoneId bone = proxy.bone;
        filament::Box root_bounds =
            TransformBounds(*local_bounds, root_transforms[bone]);
        return merged_bounds.unionSelf(root_bounds);
      });

  if (min(merged_bounds.getMin(), merged_bounds.getMax()) ==
      merged_bounds.getMax()) {
    return filament::Box{};
  }
  return merged_bounds;
}

filament::Engine* ModelData::Engine() const { return engine_; }

const TypedSetVector<EntityData>& ModelData::Entities() const {
  return entities_;
}

const TypedVector<OwnedTexturePtr>& ModelData::Textures() const {
  return textures_;
}

const TypedVector<GenericMaterialPtr>& ModelData::Materials() const {
  return materials_;
}

const TypedVector<MaterialsVariantsData>& ModelData::MaterialsVariants() const {
  return materials_variants_;
}

// TODO: (broken link) - Remove MaterialConfig entirely.
const MaterialLookup<MaterialConfig>& ModelData::MaterialConfigs() const {
  return material_configs_;
}

const GenericMaterial* ModelData::GetMaterial(
    absl::string_view material_name) const {
  for (auto material_id : materials_.Ids<imp::model::ModelData::MaterialId>()) {
    const imp::GenericMaterialPtr& material = materials_[material_id];
    if (material->GetName() != material_name) continue;
    return material.get();
  }
  return nullptr;
}

MaterialId ModelData::GetMaterialId(uint16_t material_index) const {
  auto it = material_id_lookup_.find(material_index);
  if (it == material_id_lookup_.end()) {
    return MaterialId{};
  }
  return it->second;
}

const SkeletonData& ModelData::Skeleton() const { return skeleton_; }

const TypedVector<SkinData>& ModelData::Skins() const { return skins_; }

const TypedVector<LightPunctualData>& ModelData::LightsPunctual() const {
  return lights_punctual_;
}

const TypedVector<filament::VertexBuffer*>& ModelData::VertexBuffers() const {
  return vertex_buffers_;
}

const TypedVector<filament::IndexBuffer*>& ModelData::IndexBuffers() const {
  return index_buffers_;
}

const TypedVector<filament::MorphTargetBuffer*>& ModelData::MorphTargetBuffers()
    const {
  return morph_target_buffers_;
}

// This is only used if the glTF has >4 weights, otherwise this is empty.
const TypedVector<SkinningBufferData>& ModelData::SkinningBuffers() const {
  return skinning_buffers_;
}

const MeshVertexDataLookup& ModelData::GetStoredVertexData() const {
  return stored_vertex_data_;
}

const MeshIndexDataLookup& ModelData::GetStoredIndexData() const {
  return stored_index_data_;
}

const TypedVector<AudioEmitterData>& ModelData::AudioEmitters() const {
  return audio_emitters_;
}

const TypedVector<AudioSourceData>& ModelData::AudioSources() const {
  return audio_sources_;
}

const TypedVector<AudioData>& ModelData::Audios() const { return audios_; }

const std::vector<AudioEmitterId>& ModelData::SceneAudioEmitters() const {
  return scene_audio_emitters_;
}

const BehaviorData* ModelData::Behavior() const {
  if (!behavior_.has_value()) {
    return nullptr;
  }

  return &behavior_.value();
}

const InteractivityData* ModelData::Interactivity() const {
  if (!interactivity_.has_value()) {
    return nullptr;
  }

  return &interactivity_.value();
}

}  // namespace imp::model
