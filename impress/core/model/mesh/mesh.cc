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

#include "core/model/mesh/mesh.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/collision/bvh.h"
#include "core/geometry/shapes/box.h"
#include "core/math/aabb_helpers.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_gpu_data.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"

namespace imp {

Mesh::~Mesh() {
  if (parent_mesh_) {
    parent_mesh_->RemoveSubmeshCount();
  }
  if (submesh_count_ > 0) {
    // Log a non-fatal error here. This object is wrapped in an ownedptr
    // (since all submeshes are borrowed ptrs, the parent must be an ownedptr)
    // so when it is destroyed the existence of borrowed submeshes will log
    // additional information that may be helpful in debugging this issue.
    IMP_LOG(imp::ERROR) << "Mesh destructor called on a Mesh with submeshes.";
  }
}

// TODO: Support UpdateMeshData in SplitEngine or throw an error.
void Mesh::UpdateMeshData(MeshDataPtr mesh_data, AabbSource aabb_source,
                          bool store_mesh_data_on_cpu) {
  if (!mesh_data_gpu_) {
    IMP_LOG(imp::FATAL)
        << "UpdateMeshData called on a submesh that does not own its MeshData.";
    return;
  }
  if (mesh_data_gpu_->GetDescription() != mesh_data->GetDescription()) {
    IMP_LOG(imp::FATAL) << "SetMeshData with mismatched MeshDescription.";
    return;
  }

  if (aabb_source == AabbSource::kCalculated) {
    aabb_ = CalculateAabb(mesh_data.get());
  }
  mesh_range_.offset = 0;
  mesh_range_.count = mesh_data->GetDescription().index_count;

  if (store_mesh_data_on_cpu) {
    mesh_data_ = std::move(mesh_data);
    mesh_data_gpu_->UploadMeshDataToGpu(mesh_data_.get());
    if (IsCollisionAccelerationStructureEnabled()) {
      BuildCollisionAccelerationStructureInternal();
    }
  } else {
    mesh_data_gpu_->UploadMeshDataToGpu(std::move(mesh_data));
  }
}

void Mesh::UpdateMeshData(MeshData* mesh_data, AabbSource aabb_source) {
  if (!mesh_data_gpu_) {
    IMP_LOG(imp::FATAL) << "Cannot update mesh data on a submesh.";
    return;
  }
  if (mesh_data_gpu_->GetDescription() != mesh_data->GetDescription()) {
    IMP_LOG(imp::FATAL) << "SetMeshData with mismatched MeshDescription.";
    return;
  }

  if (aabb_source == AabbSource::kCalculated) {
    aabb_ = CalculateAabb(mesh_data);
  }
  mesh_range_.offset = 0;
  mesh_range_.count = mesh_data->GetDescription().index_count;

  mesh_data_gpu_->UploadMeshDataToGpu(mesh_data);
}

void Mesh::SetIndexRange(size_t offset, size_t count) {
  MeshRange parent_mesh_data_range =
      parent_mesh_ ? parent_mesh_->GetMeshDataRange() : GetMeshDataRange();
  int32_t parent_mesh_index_max =
      parent_mesh_data_range.offset + parent_mesh_data_range.count;
  int32_t verified_offset = fmin(fmax(offset, 0), parent_mesh_index_max);
  int32_t verified_count = fmin(count, parent_mesh_index_max - verified_offset);
  if (verified_offset != offset || verified_count != count) {
    IMP_LOG(imp::FATAL) << "SetIndexRange with invalid range.";
    return;
  }

  mesh_range_.offset = offset;
  mesh_range_.count = count;
  MeshData* mesh_data = GetMeshData();
  if (mesh_data) {
    aabb_ = CalculateAabb(mesh_data, mesh_range_.offset, mesh_range_.count);

    if (IsCollisionAccelerationStructureEnabled()) {
      BuildCollisionAccelerationStructureInternal();
    }
  }
}

MeshRange Mesh::GetMeshRange() const { return mesh_range_; }

MeshRange Mesh::GetMeshDataRange() const {
  return IsSubmesh()
             ? parent_mesh_->GetMeshDataRange()
             : MeshRange{.offset = 0,
                         .count = static_cast<int32_t>(
                             mesh_data_gpu_->GetDescription().index_count)};
}

const Box& Mesh::GetAabb() const { return aabb_; }

Box Mesh::CalculateAabb(MeshData* mesh_data, size_t offset, size_t count) {
  size_t index_max = mesh_data->GetDescription().index_count;
  size_t index_render_offset = std::min(offset, index_max);
  size_t index_render_count = std::min(count, index_max - index_render_offset);

  AabbCalculator aabb_calculator;
  MeshVertexData* vertex_data = mesh_data->GetVertexData();
  MeshIndexData* index_data = mesh_data->GetIndexData();

  for (size_t i = index_render_offset;
       i < index_render_offset + index_render_count; i++) {
    uint32_t id = index_data->GetDescription().index_type ==
                          MeshDescription::IndexType::USHORT
                      ? index_data->IndexAt<uint16_t>(i)
                      : index_data->IndexAt<uint32_t>(i);
    aabb_calculator.AddVertex(vertex_data->VertexAttributeAt<float3>(
        id, VertexFormat::VertexAttribute::POSITION));
  }

  return aabb_calculator.GetAabb();
}

MeshData* Mesh::GetMeshData() {
  if (parent_mesh_) {
    return parent_mesh_->GetMeshData();
  }
  return mesh_data_.get();
}

// TODO: Make the following 3 functions private.
filament::VertexBuffer* Mesh::GetVertexBuffer() {
  if (parent_mesh_) {
    return parent_mesh_->GetVertexBuffer();
  }
  return mesh_data_gpu_->GetVertexBuffer();
}

filament::IndexBuffer* Mesh::GetIndexBuffer() {
  if (parent_mesh_) {
    return parent_mesh_->GetIndexBuffer();
  }
  return mesh_data_gpu_->GetIndexBuffer();
}

filament::RenderableManager::PrimitiveType Mesh::GetPrimitiveType() {
  if (parent_mesh_) {
    return parent_mesh_->GetPrimitiveType();
  }
  return mesh_data_gpu_->GetPrimitiveType();
}

bool Mesh::IsCollisionAccelerationStructureEnabled() const {
  if (collision_acceleration_structure_ ||
      !prepare_collision_acceleration_future_.Ready()) {
    return true;
  }
  return false;
}

void Mesh::EnableCollisionAccelerationStructure(bool enable) {
  if (enable) {
    if (!IsCollisionAccelerationStructureEnabled() && GetMeshData()) {
      BuildCollisionAccelerationStructureInternal();
    }
  } else {
    collision_acceleration_structure_.reset();
    prepare_collision_acceleration_future_ =
        Future<absl::Status>(absl::OkStatus());
  }
}

void Mesh::BuildCollisionAccelerationStructureInternal() {
  prepare_collision_acceleration_future_ = Future<absl::Status>::Schedule(
      [this]() {
        MeshVertexAndIndexData mesh_vertex_and_index_data{
            .vertex_data = mesh_data_->GetVertexData(),
            .index_data = mesh_data_->GetIndexData()};
        Bvh::Options options;
        options.intersect_backfaces = true;
        collision_acceleration_structure_ = std::make_unique<Bvh>(
            mesh_vertex_and_index_data, options, GetMeshRange());
        return absl::OkStatus();
      },
      {.executor = Executor::Type::kBackground});
}

Bvh* Mesh::GetCollisionAccelerationStructure() const {
  return collision_acceleration_structure_.get();
}

Mesh::Mesh(MeshGpuDataPtr mesh_data_gpu, MeshDataPtr mesh_data, const Box& aabb)
    : mesh_data_gpu_(std::move(mesh_data_gpu)),
      mesh_data_(std::move(mesh_data)) {
  mesh_range_.offset = 0;
  mesh_range_.count = mesh_data_gpu_->GetDescription().index_count;
  aabb_ = aabb;
}

Mesh::Mesh(BorrowedMeshPtr parent_mesh, int index_render_offset,
           int index_render_count, const Box& aabb) {
  if (parent_mesh->IsSubmesh()) {
    IMP_LOG(imp::FATAL) << "Cannot create a submesh from a submesh.";
    return;
  }

  parent_mesh_ = parent_mesh;
  parent_mesh_->AddSubmeshCount();
  mesh_range_.offset = index_render_offset;
  mesh_range_.count = index_render_count;
  aabb_ = aabb;
}
}  // namespace imp
