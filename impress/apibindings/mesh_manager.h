/*
 * Copyright 2026 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_MESH_MANAGER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_MESH_MANAGER_H_

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "apibindings/bindings_custom_mesh.h"
#include "apibindings/bindings_mesh_buffer.h"
#include "core/geometry/shapes/box.h"

namespace imp {

class ImpressApiView;

// Manages Custom Mesh resources (BindingsMeshBuffer, CustomMesh, CustomMesh
// node) for the Jetpack XR Scene.
class MeshManager {
 public:
  virtual ~MeshManager() = default;

  // Creates a BindingsMeshBuffer and returns a handle to it.
  virtual absl::StatusOr<std::intptr_t> CreateMeshBuffer(
      const BindingsMeshBuffer::CreateOptions& options) = 0;

  // Destroys a BindingsMeshBuffer with the given handle.
  virtual absl::Status DestroyMeshBuffer(std::intptr_t mesh_buffer_handle) = 0;

  // Updates vertex data in a mesh buffer.
  virtual absl::Status UpdateMeshBufferVertexData(
      std::intptr_t mesh_buffer_handle, int32_t buffer_index,
      int32_t offset_in_bytes, absl::Span<const uint8_t> data) = 0;

  // Updates index data in a mesh buffer.
  virtual absl::Status UpdateMeshBufferIndexData(
      std::intptr_t mesh_buffer_handle, int32_t offset_in_bytes,
      absl::Span<const uint8_t> data) = 0;

  // Creates a BindingsCustomMesh and returns a handle to it.
  // The bounding_box is optional, and if not provided, the AABB of the
  // mesh buffer will be used.
  virtual absl::StatusOr<std::intptr_t> CreateCustomMesh(
      std::intptr_t mesh_buffer_handle,
      absl::Span<const BindingsCustomMesh::Subset> subsets,
      std::optional<Box> bounding_box) = 0;

  // Destroys a CustomMesh.
  virtual absl::Status DestroyCustomMesh(std::intptr_t custom_mesh_handle) = 0;

  // Creates a CustomMesh node and returns its entity ID.
  virtual absl::StatusOr<int32_t> CreateCustomMeshNode(
      std::intptr_t custom_mesh_handle,
      const std::vector<std::intptr_t>& material_handles,
      int32_t bone_count) = 0;

  // Updates bone transforms for a custom mesh node.
  virtual absl::Status UpdateCustomMeshNodeBoneTransforms(
      int32_t impress_node, int32_t offset,
      absl::Span<const float> transforms) = 0;

  // Sets the material for a specific submesh of a CustomMesh node.
  virtual absl::Status SetCustomMeshNodeMaterial(
      int32_t node_entity_id, int32_t submesh_index,
      std::intptr_t material_handle) = 0;

  // Destroys all managed buffers and meshes.
  virtual void DestroyAllResources() = 0;

  // Factory function to create a MeshManager instance.
  static std::unique_ptr<MeshManager> Create(ImpressApiView& view);
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_MESH_MANAGER_H_
