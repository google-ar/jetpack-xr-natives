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

#include "core/model/mesh/mesh_vertex_data.h"

#include <cstddef>
#include <cstdint>
#include <utility>

#include "core/common/log.h"
#include "absl/synchronization/mutex.h"
#include "core/model/mesh/mesh_data_helper.h"
#include "core/model/mesh/mesh_description.h"

namespace imp {

MeshVertexData::MeshVertexData(const MeshDescription& description)
    : description_(description) {
  size_t buffer_size =
      description.vertex_format.GetVertexSize() * description.vertex_count;

  // Allocated with new, this is deleted by MeshDataBufferDescriptorDeleter when
  // the last copy of the buffer is destroyed.
  copy_counter_ = new imp_internal::MeshDataCopyCounter();

  void* buffer = new uint8_t[buffer_size];
  if (!buffer) {
    IMP_LOG(imp::FATAL) << "Failed to allocate vertex buffer of size " << buffer_size;
  }

  vertex_data_ = BufferDescriptor(
      buffer, buffer_size, &imp_internal::MeshDataBufferDescriptorDeleter,
      copy_counter_);
}

const MeshDescription& MeshVertexData::GetDescription() const {
  return description_;
}

MeshVertexData::BufferDescriptor MeshVertexData::CopyVertexData() const {
  CheckVerticesNotMoved();
  absl::MutexLock lock(&copy_counter_->mu);
  ++copy_counter_->copies;
  return BufferDescriptor(vertex_data_.buffer, vertex_data_.size,
                          &imp_internal::MeshDataBufferDescriptorDeleter,
                          copy_counter_);
}

MeshVertexData::BufferDescriptor MeshVertexData::MoveVertexData() {
  CheckVerticesNotMoved();
  description_.vertex_count = 0;
  return std::move(vertex_data_);
}

void MeshVertexData::TruncateVertices(size_t new_count) {
  CheckVerticesNotMoved();
  size_t new_size = description_.vertex_format.GetVertexSize() * new_count;
  if (new_size < vertex_data_.size) {
    description_.vertex_count = new_count;
    vertex_data_.size = new_size;
  }
}

void MeshVertexData::CheckVerticesNotMoved() const {
  if (vertex_data_.buffer == nullptr) {
    IMP_LOG(imp::FATAL) << "The vertex buffer has been moved!";
  }
}

}  // namespace imp
