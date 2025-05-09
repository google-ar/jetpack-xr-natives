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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_H_

#include <cstddef>
#include <limits>
#include <memory>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/async/future.h"
#include "core/collision/bvh.h"
#include "core/common/owned_or_unowned_memory.h"
#include "core/common/owned_ptr.h"
#include "core/geometry/shapes/box.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_gpu_data.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"

namespace imp {

class MeshFactory;

// Mesh is a container of MeshGpuData and MeshData (on cpu, optional). It also
// contains all the information that is needed for rendering, such as AABB,
// index render range, etc.
//
// Mesh can be a top-level mesh or a submesh. Submesh is a mesh that is created
// from the same mesh data as its parent mesh but with a different rendering
// range and AABB.
//
// Mesh can be created from MeshFactory. Top-level mesh can be updated with
// new mesh data, if it has no submeshes.
class Mesh {
 public:
  // Indicates the source of AABB.
  enum class AabbSource {
    // AABB is assigned by the user.
    kAssignedOrUnchanged,
    // AABB is calculated from mesh data and tight.
    kCalculated
  };

  ~Mesh();

  // Use this to update the mesh data on the GPU, which reuses the previously
  // allocated buffers.
  //
  // recalculate_aabb: This can be useful if the mesh data has changed
  // significantly and the previous AABB is no longer valid.
  //
  // NOTE: Rendering range will be reset to the whole mesh.
  void UpdateMeshData(
      MeshData* mesh_data,
      AabbSource aabb_source = AabbSource::kAssignedOrUnchanged);

  // Use this to update the mesh data on the GPU, which reuses the previously
  // allocated buffers. It also takes ownership of the mesh data.
  //
  // recalculate_aabb: This can be useful if the mesh data has changed
  // significantly and the previous AABB is no longer valid.
  //
  // store_mesh_data_on_cpu: if true the mesh data will be stored in the Mesh
  // object.
  //
  // NOTE: Rendering range will be reset to the whole mesh.
  void UpdateMeshData(MeshDataPtr mesh_data,
                      AabbSource aabb_source = AabbSource::kAssignedOrUnchanged,
                      bool store_mesh_data_on_cpu = false);

  // NOTE: After using this function, call
  // MeshRenderer::ApplyAllMeshPropertyChanges() to show the changes of the
  // mesh visually.
  // NOTE: the user is responsible to provide a valid indices range.
  void SetIndexRange(size_t offset, size_t count);

  // Assigns new AABB to the mesh. This will overwrite the existing AABB anyway.
  void AssignAabb(Box aabb) { aabb_ = aabb; }

  // Calculates the AABB of the mesh data in the given range.
  static Box CalculateAabb(MeshData* mesh_data, size_t offset = 0,
                           size_t count = std::numeric_limits<size_t>::max());

  // Returns the AABB of the mesh. The AABB can be assigned by the user or
  // calculated according to the mesh data within rendering range.
  const Box& GetAabb() const;

  // Returns the index render offset or count or both, which is a subset or
  // whole of the mesh data range.
  size_t GetIndexRenderOffset() const { return mesh_range_.offset; }
  size_t GetIndexRenderCount() const { return mesh_range_.count; }
  MeshRange GetMeshRange() const;

  // Returns the range of the mesh data.
  MeshRange GetMeshDataRange() const;

  MeshData* GetMeshData();
  filament::VertexBuffer* GetVertexBuffer();
  filament::IndexBuffer* GetIndexBuffer();
  filament::RenderableManager::PrimitiveType GetPrimitiveType();

  bool IsSubmesh() const { return parent_mesh_ != nullptr; }

  // Enabling the collision acceleration structure will result in automatically
  // rebuilding it when the mesh is updated. Disabling it will remove the
  // acceleration structure and make corresponding MeshCollider fallback to use
  // per-triangle intersection if applicable.
  void EnableCollisionAccelerationStructure(bool enable);

  void DisableCollisionAccelerationStructure();
  Bvh* GetCollisionAccelerationStructure() const;

 private:
  Mesh(MeshGpuDataPtr mesh_data_gpu, MeshDataPtr mesh_data, const Box& aabb);

  // Create a submesh based on the same mesh data. Submesh
  // does not own the mesh data and should not be used after the original mesh
  // is destroyed.
  // NOTE: the user is responsible to provide a valid indices range.
  Mesh(BorrowedPtr<Mesh> parent_mesh, int index_render_offset,
       int index_render_count, const Box& aabb);

  // Keep track of the number of submeshes to delete the mesh data when the
  // number of submeshes is zero.
  void AddSubmeshCount() { submesh_count_++; }
  void RemoveSubmeshCount() { submesh_count_--; }

  // Returns true if the collision acceleration structure is enabled.
  bool IsCollisionAccelerationStructureEnabled() const;

  // Schedules the building of the collision acceleration structure. Assumes
  // that the mesh data is available on the CPU.
  void BuildCollisionAccelerationStructureInternal();

  // Only the main mesh can own the following data.
  MeshGpuDataPtr mesh_data_gpu_;
  MeshDataPtr mesh_data_;

  // Those variables varies for each mesh or submesh.
  MeshRange mesh_range_;
  Box aabb_;

  BorrowedPtr<Mesh> parent_mesh_;
  size_t submesh_count_ = 0;

  Future<absl::Status> prepare_collision_acceleration_future_ =
      Future<absl::Status>(absl::OkStatus());
  std::unique_ptr<Bvh> collision_acceleration_structure_;

  friend class MeshFactory;
};

// For now, we only support move semantics and single ownership.
using MeshPtr ABSL_DEPRECATED(
    "Prefer using OwnedMeshPtr instead. See "
    "(broken link).") = std::unique_ptr<Mesh>;
using OwnedOrUnownedMesh = OwnedOrUnownedMemory<Mesh>;

// Track lifetime of meshes using OwnedPtr and BorrowedPtr.
using BorrowedMeshPtr = BorrowedPtr<Mesh>;
using OwnedMeshPtr = OwnedPtr<Mesh>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_H_
