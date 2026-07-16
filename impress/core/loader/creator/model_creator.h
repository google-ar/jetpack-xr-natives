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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_MODEL_CREATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_MODEL_CREATOR_H_

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/async/future.h"
#include "core/common/paired_vector.h"
#include "core/common/typed_set_vector.h"
#include "core/common/typed_vector.h"
#include "core/image/image_contents.h"
#include "core/loader/creator/inflight_creation.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/material_library/material_package.h"
#include "core/material_library/material_param_value.h"
#include "core/model/model_data.h"
#include "core/model/skeleton_data.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"

namespace imp::loader::details {
// Transient object that manages creating a ModelData.  Holds created GPU
// resources until completion, so they can be released if a failure occurs.
class ModelCreator {
 public:
  explicit ModelCreator(filament::Engine* engine,
                        LoaderOptions loader_options = LoaderOptions());
  ~ModelCreator();

  Future<absl::Status> LoadAll(
      BaseView& view, const schemas::LoadedModel* model,
      MaterialPackage* material_package,
      std::vector<std::unique_ptr<image::ImageContents>> images,
      std::optional<absl::string_view> name = std::nullopt);

  absl::StatusOr<std::unique_ptr<model::ModelData>> CreateModelData(
      imp::BaseView* view);

  absl::Status TryComplete();
  bool IsFullyLoaded();
  bool HasPendingWork();
  void WhenFullyLoaded(std::function<void()> cb);
  void RemoveWhenFullyLoadedCallback();

 private:
  Future<absl::Status> LoadAllInternal(
      BaseView& view, const schemas::LoadedModel* model,
      MaterialPackage* material_package,
      std::vector<std::unique_ptr<image::ImageContents>> images,
      std::optional<absl::string_view> name = std::nullopt);

  Future<absl::Status> CreateModelResources(
      BaseView& view, MaterialPackage* material_package,
      const schemas::LoadedModel* model,
      std::vector<std::unique_ptr<image::ImageContents>> images,
      std::optional<absl::string_view> name = std::nullopt);

  filament::Engine* const engine_;
  LoaderOptions loader_options_;
  absl::Status status_;

  TypedVector<filament::VertexBuffer*> vertex_buffers_;
  TypedVector<filament::IndexBuffer*> index_buffers_;
  TypedVector<filament::MorphTargetBuffer*> morph_target_buffers_;
  PairedVector<OwnedTexturePtr, filament::MorphTargetBuffer*>
      morph_target_uv0_textures_;
  TypedVector<OwnedTexturePtr> textures_;
  TypedVector<GenericMaterialPtr> materials_;
  absl::flat_hash_map<uint16_t, model::ModelData::MaterialId>
      material_id_lookup_;
  absl::flat_hash_map<uint32_t, model::ModelData::SkinId> skin_id_lookup_;
  TypedVector<model::ModelData::SkinningBufferData> skinning_buffers_;
  model::ModelData::MeshVertexDataLookup stored_vertex_data_;
  model::ModelData::MeshIndexDataLookup stored_index_data_;
  model::ModelData::MaterialLookup<model::ModelData::MaterialConfig>
      material_config_info_;

  TypedSetVector<model::ModelData::EntityData> entities_;

  TypedVector<model::ModelData::SkinData> skins_;
  TypedVector<model::ModelData::LightPunctualData> lights_punctual_;
  TypedVector<model::ModelData::MaterialsVariantsData> materials_variants_;

  std::optional<model::SkeletonData> skeleton_;
  InflightCreation inflight_creation_;

  TypedVector<model::ModelData::AudioEmitterData> audio_emitters_;
  TypedVector<model::ModelData::AudioSourceData> audio_sources_;
  TypedVector<model::ModelData::AudioData> audios_;
  std::vector<model::ModelData::AudioEmitterId> scene_audio_emitters_;

  std::optional<model::ModelData::InteractivityData> interactivity_;
};

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_MODEL_CREATOR_H_
