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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_GPU_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_GPU_DATA_H_

#include <memory>

#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/view/base_view.h"

namespace imp {

class MeshFactory;

// A wrapper around filament::Vertex/IndexBuffer that automatically cleans them
// up. These shouldn't be created directly, but instead created from the
// MeshFactory and accessed using MeshGpuDataPtr.
class MeshGpuData {
 public:
  ~MeshGpuData();

  // Uploads the mesh data from the CPU to the GPU, and keep no copy on the CPU.
  // Use this to update the mesh data on the GPU, which reuses the previously
  // allocated buffers.
  void UploadMeshDataToGpu(MeshDataPtr mesh_data);

  // Make a copy of the mesh data and upload to the GPU. Use this to update
  // the mesh data on the GPU, which reuses the previously allocated buffers.
  void UploadMeshDataToGpu(MeshData* mesh_data);

  filament::VertexBuffer* GetVertexBuffer();
  filament::IndexBuffer* GetIndexBuffer();
  filament::RenderableManager::PrimitiveType GetPrimitiveType();
  MeshDescription GetDescription() { return description_; }

 private:
  MeshGpuData(imp::BaseView& view, const MeshDescription& description,
              filament::VertexBuffer* vertex_buffer,
              filament::IndexBuffer* index_buffer,
              filament::RenderableManager::PrimitiveType primitive_type);

  imp::BaseView& view_;
  MeshDescription description_;
  filament::VertexBuffer* vertex_buffer_;
  filament::IndexBuffer* index_buffer_;
  filament::RenderableManager::PrimitiveType primitive_type_;

  friend class MeshFactory;
};

using MeshGpuDataPtr = std::unique_ptr<MeshGpuData>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_GPU_DATA_H_
