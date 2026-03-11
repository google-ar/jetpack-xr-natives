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

#include "core/view/framework/assets/gltf_asset.h"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/synchronization/mutex.h"
#include "absl/time/time.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/collision/collision_accelerator_provider.h"
#include "core/collision/mesh_collision_accelerator.h"
#include "core/common/optional_error.h"
#include "core/common/typed_vector.h"
#include "core/geometry/shapes/box.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/model_data.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/gltf_asset_loader.h"
#include "core/view/utils/string_map.h"

namespace imp {

// Specifies which material variants to precompile and if the loading needs to
// wait for the high priority variants' compilation.
const MaterialPreCompileOptions& GltfAsset::kDefaultMaterialPreCompileOptions =
    *new MaterialPreCompileOptions{};

Future<std::unique_ptr<GltfAsset>> GltfAsset::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future, GltfAssetLoader* loader,
    GltfAsset::LoadOptions options) {
  return loader->Load(view, asset_url, resource_future, std::move(options));
}

absl::Duration GltfAsset::LoadEvent::GetTotalDuration() const {
  return end_time - start_time;
}

absl::Duration GltfAsset::LoadEvent::GetTotalDownloadDuration() const {
  return (std::max(end_download_model_time, end_download_materials_time) -
          start_download_materials_and_model_time) +
         (end_download_deps_time - start_download_deps_time);
}

absl::Duration GltfAsset::LoadEvent::GetTotalParseDuration() const {
  return (end_parse_time - start_parse_time) -
         (end_download_deps_time - start_download_deps_time);
}

GltfAsset::GltfAsset() = default;

GltfAsset::GltfAsset(std::unique_ptr<model::ModelData> model_data,
                     TypedVector<GltfAnimDataPtr>&& animations,
                     AnimLookup<std::string>&& animation_names,
                     StringMap<AnimId>&& animation_name_lookup,
                     GenericMaterialListing&& shared_materials)
    : model_data_(std::move(model_data)),
      animations_(std::move(animations)),
      animation_names_(std::move(animation_names)),
      animation_name_lookup_(std::move(animation_name_lookup)),
      shared_materials_(std::move(shared_materials)) {}

size_t GltfAsset::AnimationCount() const { return animation_names_.size(); }

const model::ModelData& GltfAsset::GetModelData() const { return *model_data_; }

const MeshCollisionAccelerator* GltfAsset::GetMeshCollisionAccelerator(
    model::ModelData::EntityId entity_id) const {
  if (!mesh_collision_creation_future_.has_value() ||
      !mesh_collision_creation_future_->Ready()) {
    return nullptr;
  }
  absl::MutexLock mesh_collision_accelerators_lock(
      mesh_collision_accelerators_mutex_);
  auto it = mesh_collision_accelerators_.find(entity_id);
  if (it != mesh_collision_accelerators_.end()) {
    return it->second.get();
  }
  return nullptr;
}

absl::Span<const std::string> GltfAsset::GetAnimNames() const {
  return absl::MakeSpan(animation_names_.data(), animation_names_.size());
}

GltfAsset::AnimId GltfAsset::GetAnimId(absl::string_view anim_name) const {
  auto it = animation_name_lookup_.find(anim_name);
  if (it != animation_name_lookup_.end()) {
    return it.value();
  } else {
    return AnimId{};
  }
}

const animation::GltfAnimation* GltfAsset::GetGltfAnimData(
    AnimId anim_id) const {
  if (!animations_.IsValid(anim_id)) {
    return nullptr;
  }
  return animations_[anim_id].get();
}

const GenericMaterialListing& GltfAsset::GetSharedMaterials() const {
  return shared_materials_;
}

GltfAsset::Builder::Builder(size_t animation_count) noexcept
    : model_data_(),
      animations_(animation_count),
      animation_names_(animation_count),
      animation_name_lookup_() {}

GltfAsset::Builder::Builder(Builder&& rhs) noexcept = default;
GltfAsset::Builder::~Builder() noexcept = default;
GltfAsset::Builder& GltfAsset::Builder::operator=(Builder&& rhs) noexcept =
    default;

GltfAsset::Builder& GltfAsset::Builder::Model(
    std::unique_ptr<model::ModelData> model_data) noexcept {
  model_data_ = std::move(model_data);
  return *this;
}

GltfAsset::Builder& GltfAsset::Builder::Animation(
    size_t animation_index, absl::string_view name,
    GltfAnimDataPtr animation) noexcept {
  auto anim_id = AnimId::At(animation_index);
  animations_[anim_id] = std::move(animation);
  animation_names_[anim_id] = static_cast<std::string>(name);
  animation_name_lookup_[static_cast<std::string>(name)] = anim_id;
  return *this;
}

GltfAsset::Builder& GltfAsset::Builder::SharedMaterials(
    GenericMaterialListing shared_materials) noexcept {
  shared_materials_ = std::move(shared_materials);
  return *this;
}

void GltfAsset::BuildMeshCollisionAccelerators(
    CollisionAcceleratorProvider& collision_accelerator_provider) {
  if (mesh_collision_creation_future_.has_value()) {
    return;
  }
  mesh_collision_creation_future_ = Future<absl::Status>::Schedule(
      [this, collision_accelerator_provider]() mutable -> absl::Status {
        BuildMeshCollisionAcceleratorsInternal(collision_accelerator_provider);
        return absl::OkStatus();
      },
      /*schedule_on_executor=*/Executor::Type::kBackground);
}

void GltfAsset::BuildMeshCollisionAcceleratorsInternal(
    CollisionAcceleratorProvider& collision_accelerator_provider) {
  absl::MutexLock lock(mesh_data_availability_mutex_);
  auto& stored_vertex_data = model_data_->GetStoredVertexData();
  auto& stored_index_data = model_data_->GetStoredIndexData();
  if (!stored_index_data.empty() && !stored_vertex_data.empty()) {
    auto& entities = model_data_->Entities();
    for (auto entity_id :
         model_data_->Entities().Ids<model::ModelData::EntityId>()) {
      const model::ModelData::EntityData::Proxy& entity_data =
          entities[entity_id];
      const absl::optional<Box>& local_bounds = entity_data.local_bounds;
      model::ModelData::SkinId skin_id = entity_data.skin;
      model::ModelData::MorphTargetBufferId morph_target_buffer =
          entity_data.morph_target_buffer;
      // No bound means the entity is not a mesh.
      if (!local_bounds.has_value() || skin_id || morph_target_buffer) {
        continue;
      }
      const std::vector<model::ModelData::PartData>& entity_parts =
          entity_data.parts;
      std::vector<MeshVertexAndIndexData> mesh_vertex_and_index_data;
      for (size_t part_index = 0; part_index < entity_parts.size();
           ++part_index) {
        const model::ModelData::PartData& part = entity_parts[part_index];
        mesh_vertex_and_index_data.push_back(MeshVertexAndIndexData{
            .vertex_data = stored_vertex_data[part.vertex_buffer].get(),
            .index_data = stored_index_data[part.index_buffer].get()});
      }
      absl::MutexLock mesh_collision_accelerators_lock(
          mesh_collision_accelerators_mutex_);
      mesh_collision_accelerators_[entity_id] =
          collision_accelerator_provider.GetMeshCollisionAccelerator(
              mesh_vertex_and_index_data, *local_bounds);
    }
  }
}

absl::StatusOr<std::unique_ptr<GltfAsset>> GltfAsset::Builder::Build() {
  if (!model_data_) return Error("No model");

  return absl::WrapUnique<GltfAsset>(new GltfAsset(
      std::move(model_data_), std::move(animations_),
      std::move(animation_names_), std::move(animation_name_lookup_),
      std::move(shared_materials_)));
}

GltfAsset::~GltfAsset() {
  // Wait until mesh collision accelerator creation futures are done.
  absl::MutexLock mesh_data_lock(mesh_data_availability_mutex_);
  model_data_.reset();
  absl::MutexLock mesh_collision_accelerators_lock(
      mesh_collision_accelerators_mutex_);
  mesh_collision_accelerators_.clear();
}

}  // namespace imp
