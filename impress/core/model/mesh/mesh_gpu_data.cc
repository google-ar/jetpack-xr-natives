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

#include "core/model/mesh/mesh_gpu_data.h"

#include "core/common/log.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"

namespace imp {

MeshGpuData::MeshGpuData(
    imp::BaseView& view, const MeshDescription& description,
    filament::VertexBuffer* vertex_buffer, filament::IndexBuffer* index_buffer,
    filament::RenderableManager::PrimitiveType primitive_type)
    : view_(view),
      description_(description),
      vertex_buffer_(vertex_buffer),
      index_buffer_(index_buffer),
      primitive_type_(primitive_type) {}

MeshGpuData::~MeshGpuData() {
  if (filament::Engine* engine = view_.GetSharedEngine()) {
    split_engine::SplitEngineSerializer* serializer =
        view_.GetSplitEngineSerializer();
    if (vertex_buffer_) {
      if (serializer) {
        serializer->RemoveVertexBuffer(vertex_buffer_);
      }
      engine->destroy(vertex_buffer_);
    }
    if (index_buffer_) {
      if (serializer) {
        serializer->RemoveIndexBuffer(index_buffer_);
      }
      engine->destroy(index_buffer_);
    }
  }
  vertex_buffer_ = nullptr;
  index_buffer_ = nullptr;
}

void MeshGpuData::UploadMeshDataToGpu(MeshDataPtr mesh_data) {
  if (description_ != mesh_data->GetDescription()) {
    IMP_LOG(imp::FATAL) << "UpdateMeshDataOnGpu with mismatched MeshDescription.";
    return;
  }

  filament::Engine* engine = view_.GetSharedEngine();
  vertex_buffer_->setBufferAt(*engine, 0, mesh_data->MoveVertexData(), 0);
  index_buffer_->setBuffer(*engine, mesh_data->MoveIndexData(), 0);
}

void MeshGpuData::UploadMeshDataToGpu(MeshData* mesh_data) {
  if (mesh_data == nullptr) {
    IMP_LOG(imp::FATAL) << "Called CopyDataToGpu with no MeshData.";
    return;
  }

  if (description_ != mesh_data->GetDescription()) {
    IMP_LOG(imp::FATAL) << "UpdateMeshDataOnGpu with mismatched MeshDescription.";
    return;
  }
  filament::Engine* engine = view_.GetSharedEngine();
  vertex_buffer_->setBufferAt(*engine, 0, mesh_data->CopyVertexData(), 0);
  index_buffer_->setBuffer(*engine, mesh_data->CopyIndexData(), 0);
}

filament::VertexBuffer* MeshGpuData::GetVertexBuffer() {
  return vertex_buffer_;
}

filament::IndexBuffer* MeshGpuData::GetIndexBuffer() { return index_buffer_; }

filament::RenderableManager::PrimitiveType MeshGpuData::GetPrimitiveType() {
  return primitive_type_;
}

}  // namespace imp
