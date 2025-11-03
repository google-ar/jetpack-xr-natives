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

#include "core/split_engine/split_engine_renderable_info.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/math.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"
#include "core/render/base_renderable_manager.h"
#include "core/split_engine/shared/split_engine_defines.h"
#include "core/split_engine/skinning_helpers.h"
#include "core/split_engine/split_engine_filament_resource_ptrs.h"

namespace imp::split_engine {

namespace {

using RenderableInstance = filament::RenderableManager::Instance;

}  // namespace

absl::Status SplitEngineRenderableInfo::Setup(BridgeId bridge_id) {
  BaseRenderableManager& rm = GetView().GetRenderableManager();
  RenderableInstance renderable = rm.GetInstance(GetEntity());
  if (!renderable) {
    return absl::FailedPreconditionError(
        "Cannot add SplitEngineRenderableInfo to a node without a "
        "renderable.");
  }

  size_t primitive_count =
      GetView().GetRenderableManager().GetPrimitiveCount(renderable);
  primitives_.resize(primitive_count);

  bridge_id_ = bridge_id;

  return absl::OkStatus();
}

absl::Status SplitEngineRenderableInfo::SetPrimitiveMeshData(
    size_t primitive_index, MeshVertexData* vertex_data,
    MeshIndexData* index_data) {
  if (primitive_index >= primitives_.size()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Primitive index %d is out of bounds.", primitive_index));
  }

  primitives_[primitive_index].unskinned_mesh_data =
      MeshVertexAndIndexData{vertex_data, index_data};

  // If there is vertex data and it has bone indices, create a skinned vertex
  // data object so that we can store the skinned data.
  if (vertex_data) {
    const VertexFormat kVertexFormat =
        vertex_data->GetDescription().vertex_format;
    const std::optional<size_t> bone_indices_attr_id =
        kVertexFormat.GetIndexForAttribute(
            VertexFormat::VertexAttribute::BONE_INDICES);
    if (bone_indices_attr_id) {
      primitives_[primitive_index].skinned_vertex_data =
          std::make_unique<MeshVertexData>(vertex_data->GetDescription());
    }
  } else {
    // Reset the skinned vertex data if there is no vertex data.
    primitives_[primitive_index].skinned_vertex_data.reset();
  }

  // Reset this so that it can be re-created on the next call to GetAllMeshData.
  all_mesh_data_.reset();

  return absl::OkStatus();
}

void SplitEngineRenderableInfo::UpdateBones(
    const std::vector<mat4f>& bone_transforms) {
  for (PrimitiveInfo& primitive : primitives_) {
    if (!primitive.unskinned_mesh_data.vertex_data ||
        !primitive.unskinned_mesh_data.index_data ||
        !primitive.skinned_vertex_data) {
      continue;
    }

    UpdateSkinning(primitive.unskinned_mesh_data,
                   *primitive.skinned_vertex_data, bone_transforms);
  }
}

void SplitEngineRenderableInfo::SetMorphTargetBuffer(
    BorrowedMorphTargetBufferPtr morph_target_buffer) {
  morph_target_buffer_ = morph_target_buffer;
}

void SplitEngineRenderableInfo::SetVertexBuffer(
    BorrowedVertexBufferPtr vertex_buffer) {
  vertex_buffer_ = vertex_buffer;
}

void SplitEngineRenderableInfo::SetIndexBuffer(
    BorrowedIndexBufferPtr index_buffer) {
  index_buffer_ = index_buffer;
}

absl::Span<const MeshVertexAndIndexData>
SplitEngineRenderableInfo::GetAllMeshData() {
  if (all_mesh_data_) {
    return *all_mesh_data_;
  }

  std::vector<MeshVertexAndIndexData>& mesh_data = all_mesh_data_.emplace();
  mesh_data.reserve(primitives_.size());

  // Gather together all the mesh data for each primitive if it's available.
  // It may not be available if the data wasn't saved outside of the GPU or
  // a primitive has no geometry assigned.
  for (const PrimitiveInfo& primitive : primitives_) {
    if (!primitive.unskinned_mesh_data.vertex_data ||
        !primitive.unskinned_mesh_data.index_data) {
      continue;
    }

    // Use the skinned vertex data if it exists, otherwise use the unskinned
    // vertex data.
    mesh_data.push_back({primitive.skinned_vertex_data
                             ? primitive.skinned_vertex_data.get()
                             : primitive.unskinned_mesh_data.vertex_data,
                         primitive.unskinned_mesh_data.index_data});
  }

  return mesh_data;
}

void SplitEngineRenderableInfo::SetSkinningBoneCount(
    uint32_t skinning_bone_count) noexcept {
  skinning_bone_count_ = skinning_bone_count;
}

void SplitEngineRenderableInfo::SetRenderableBounds(
    const imp::Box& renderable_bounds) {
  renderable_bounds_ = renderable_bounds;
}

uint32_t SplitEngineRenderableInfo::GetSkinningBoneCount() const noexcept {
  return skinning_bone_count_;
}

const imp::Box& SplitEngineRenderableInfo::GetRenderableBounds() const {
  return renderable_bounds_;
}

BridgeId SplitEngineRenderableInfo::GetBridgeId() const noexcept {
  return bridge_id_;
}

absl::Status SplitEngineRenderableInfo::SetMaterialInstance(
    size_t primitive_index, BorrowedMaterialPtr material_instance) {
  if (primitive_index >= primitives_.size()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Primitive index %u is out of bounds.", primitive_index));
  }

  primitives_[primitive_index].material_instance = material_instance;

  return absl::OkStatus();
}

}  // namespace imp::split_engine
