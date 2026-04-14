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

#include "apibindings/mesh_manager.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

#include "absl/cleanup/cleanup.h"
#include "absl/container/flat_hash_map.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "filament/libs/utils/include/utils/Entity.h"
#include "apibindings/bindings_custom_mesh.h"
#include "apibindings/bindings_material.h"
#include "apibindings/bindings_mesh_buffer.h"
#include "apibindings/impress_api_view.h"
#include "core/common/small_source_location.h"
#include "core/geometry/shapes/box.h"
#include "core/math/mat.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/render/mesh_renderer.h"

namespace imp {

namespace {

class MeshManagerImpl : public MeshManager {
 public:
  explicit MeshManagerImpl(ImpressApiView& view);

  absl::StatusOr<std::intptr_t> CreateMeshBuffer(
      const BindingsMeshBuffer::CreateOptions& options) override;

  absl::Status DestroyMeshBuffer(std::intptr_t handle) override;

  absl::Status UpdateMeshBufferVertexData(
      std::intptr_t handle, int32_t buffer_index, int32_t offset_in_bytes,
      absl::Span<const uint8_t> data) override;

  absl::Status UpdateMeshBufferIndexData(
      std::intptr_t handle, int32_t offset_in_bytes,
      absl::Span<const uint8_t> data) override;

  absl::StatusOr<std::intptr_t> CreateCustomMesh(
      std::intptr_t mesh_buffer_handle,
      absl::Span<const BindingsCustomMesh::Subset> subsets,
      std::optional<Box> bounding_box = std::nullopt) override;

  absl::Status DestroyCustomMesh(std::intptr_t handle) override;

  absl::StatusOr<int32_t> CreateCustomMeshNode(
      std::intptr_t custom_mesh_handle,
      const std::vector<std::intptr_t>& material_handles,
      int32_t bone_count) override;

  absl::Status UpdateCustomMeshNodeBoneTransforms(
      int32_t impress_node, int32_t offset,
      absl::Span<const float> transforms) override;

  absl::Status SetCustomMeshNodeMaterial(
      int32_t node_entity_id, int32_t submesh_index,
      std::intptr_t material_handle) override;

  void DestroyAllResources() override;

 private:
  ImpressApiView& view_;
  absl::flat_hash_map<std::intptr_t, std::shared_ptr<BindingsMeshBuffer>>
      buffers_;
  absl::flat_hash_map<std::intptr_t, std::unique_ptr<BindingsCustomMesh>>
      custom_meshes_;
};

MeshManagerImpl::MeshManagerImpl(ImpressApiView& view) : view_(view) {}

absl::StatusOr<std::intptr_t> MeshManagerImpl::CreateMeshBuffer(
    const BindingsMeshBuffer::CreateOptions& options) {
  auto buffer = std::make_unique<BindingsMeshBuffer>(view_, options);
  std::intptr_t handle = reinterpret_cast<std::intptr_t>(buffer.get());
  buffers_[handle] = std::move(buffer);
  return handle;
}

absl::Status MeshManagerImpl::DestroyMeshBuffer(std::intptr_t handle) {
  if (buffers_.erase(handle)) {
    return absl::OkStatus();
  }
  return absl::NotFoundError("MeshBuffer not found.");
}

absl::Status MeshManagerImpl::UpdateMeshBufferVertexData(
    std::intptr_t handle, int32_t buffer_index, int32_t offset_in_bytes,
    absl::Span<const uint8_t> data) {
  auto it = buffers_.find(handle);
  if (it == buffers_.end()) {
    return absl::NotFoundError("MeshBuffer not found.");
  }
  return it->second->UpdateVertexData(buffer_index, offset_in_bytes, data);
}

absl::Status MeshManagerImpl::UpdateMeshBufferIndexData(
    std::intptr_t handle, int32_t offset_in_bytes,
    absl::Span<const uint8_t> data) {
  auto it = buffers_.find(handle);
  if (it == buffers_.end()) {
    return absl::NotFoundError("MeshBuffer not found.");
  }
  return it->second->UpdateIndexData(offset_in_bytes, data);
}

absl::StatusOr<std::intptr_t> MeshManagerImpl::CreateCustomMesh(
    std::intptr_t mesh_buffer_handle,
    absl::Span<const BindingsCustomMesh::Subset> subsets,
    std::optional<Box> bounding_box) {
  auto it = buffers_.find(mesh_buffer_handle);
  if (it == buffers_.end()) {
    return absl::NotFoundError("MeshBuffer not found.");
  }

  auto custom_mesh = std::make_unique<BindingsCustomMesh>(it->second, subsets,
                                                          bounding_box, view_);
  std::intptr_t handle = reinterpret_cast<std::intptr_t>(custom_mesh.get());
  custom_meshes_[handle] = std::move(custom_mesh);
  return handle;
}

absl::Status MeshManagerImpl::DestroyCustomMesh(std::intptr_t handle) {
  if (custom_meshes_.erase(handle)) {
    return absl::OkStatus();
  }
  return absl::NotFoundError("CustomMesh not found.");
}

absl::StatusOr<int32_t> MeshManagerImpl::CreateCustomMeshNode(
    std::intptr_t custom_mesh_handle,
    const std::vector<std::intptr_t>& material_handles, int32_t bone_count) {
  auto it = custom_meshes_.find(custom_mesh_handle);
  if (it == custom_meshes_.end()) {
    return absl::NotFoundError("CustomMesh not found.");
  }
  BindingsCustomMesh* mesh = it->second.get();
  if (!mesh) {
    return absl::InternalError("CustomMesh pointer is null.");
  }

  size_t submesh_count = mesh->GetSubMeshCount();
  if (submesh_count != material_handles.size()) {
    return absl::InvalidArgumentError(
        "Number of submeshes does not match number of materials.");
  }

  NodeHandle node = view_.CreateNode();
  auto node_cleanup =
      absl::MakeCleanup([this, node] { view_.DestroyNode(node); });

  MeshRenderer::SetupOptions options;
  options.primitive_count = submesh_count;
  options.num_bones = bone_count;
  auto mesh_renderer = node->AddComponent<MeshRenderer>(options);

  // Set meshes and materials for each partition.
  for (size_t i = 0; i < submesh_count; ++i) {
    mesh_renderer->SetMesh(mesh->BorrowSubMeshAt(i), i);
    BindingsMaterial* bindings_material =
        view_.FromJava<BindingsMaterial>(material_handles[i]);
    if (!bindings_material) {
      return absl::InvalidArgumentError("Invalid material handle.");
    }
    mesh_renderer->SetMaterial(
        bindings_material->GetMaterial(SmallSourceLocation::Current()), i);
  }

  std::move(node_cleanup).Cancel();
  return node.GetEntity().getId();
}

absl::Status MeshManagerImpl::UpdateCustomMeshNodeBoneTransforms(
    int32_t impress_node, int32_t offset, absl::Span<const float> transforms) {
  NodeHandle node(utils::Entity::import(impress_node));
  if (!node) {
    return absl::NotFoundError("Impress Node not found.");
  }

  ComponentHandle<MeshRenderer> renderer = node->GetComponent<MeshRenderer>();
  if (!renderer) {
    return absl::NotFoundError("MeshRenderer not found on the node.");
  }

  if (transforms.size() % 16 != 0) {
    return absl::InvalidArgumentError(
        "Transforms array size must be a multiple of 16.");
  }

  size_t num_bones = transforms.size() / 16;
  absl::Span<const imp::mat4f> new_bones(
      reinterpret_cast<const imp::mat4f*>(transforms.data()), num_bones);

  return renderer->UpdateBoneTransformsInRange(new_bones, offset);
}

absl::Status MeshManagerImpl::SetCustomMeshNodeMaterial(
    int32_t node_entity_id, int32_t submesh_index,
    std::intptr_t material_handle) {
  utils::Entity entity = utils::Entity::import(node_entity_id);
  NodeHandle node(entity);
  if (!node.IsValid()) {
    return absl::NotFoundError("Node not found.");
  }

  ComponentHandle<MeshRenderer> mesh_renderer =
      node->GetComponent<MeshRenderer>();
  if (!mesh_renderer.IsValid()) {
    return absl::NotFoundError("MeshRenderer not found on the node.");
  }

  if (submesh_index < 0 || static_cast<size_t>(submesh_index) >=
                               mesh_renderer->GetPrimitiveCount()) {
    return absl::OutOfRangeError("Submesh index out of range.");
  }

  BindingsMaterial* bindings_material =
      view_.FromJava<BindingsMaterial>(material_handle);
  if (!bindings_material) {
    return absl::InvalidArgumentError("Invalid material handle.");
  }

  mesh_renderer->SetMaterial(
      bindings_material->GetMaterial(SmallSourceLocation::Current()),
      submesh_index);

  return absl::OkStatus();
}

void MeshManagerImpl::DestroyAllResources() {
  custom_meshes_.clear();
  buffers_.clear();
}

}  // namespace

std::unique_ptr<MeshManager> MeshManager::Create(ImpressApiView& view) {
  return std::make_unique<MeshManagerImpl>(view);
}

}  // namespace imp
