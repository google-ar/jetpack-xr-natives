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
#include <cstring>
#include <utility>

#include "core/common/log.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/span.h"
#include "core/model/mesh/mesh_data_helper.h"
#include "core/model/mesh/mesh_description.h"

namespace imp {

MeshVertexData::MeshVertexData(const MeshDescription& description)
    : description_(description) {
  size_t attribute_groups_count =
      description.vertex_format.GetAttributeGroupsCount();
  vertex_data_.reserve(attribute_groups_count);
  copy_counter_.reserve(attribute_groups_count);
  for (int group_idx = 0; group_idx < attribute_groups_count; ++group_idx) {
    size_t buffer_size = description.vertex_format.GetVertexSize(group_idx) *
                         description.vertex_count;

    // Allocated with new, this is deleted by MeshDataBufferDescriptorDeleter
    // when the last copy of the buffer is destroyed.
    copy_counter_.push_back(new imp_internal::MeshDataCopyCounter());

    void* buffer = new uint8_t[buffer_size];
    if (!buffer) {
      IMP_LOG(imp::FATAL) << "Failed to allocate vertex buffer of size " << buffer_size;
    }

    vertex_data_.push_back(BufferDescriptor(
        buffer, buffer_size, &imp_internal::MeshDataBufferDescriptorDeleter,
        copy_counter_[group_idx]));
  }
}

const MeshDescription& MeshVertexData::GetDescription() const {
  return description_;
}

MeshVertexData::BufferDescriptor MeshVertexData::CopyVertexData(
    size_t group_idx) const {
  CheckVerticesNotMoved(group_idx);
  absl::MutexLock lock(copy_counter_[group_idx]->mu);
  ++copy_counter_[group_idx]->copies;
  return BufferDescriptor(
      vertex_data_[group_idx].buffer, vertex_data_[group_idx].size,
      &imp_internal::MeshDataBufferDescriptorDeleter, copy_counter_[group_idx]);
}

MeshVertexData::BufferDescriptor MeshVertexData::MoveVertexData(
    size_t group_idx) {
  CheckVerticesNotMoved(group_idx);
  return std::move(vertex_data_[group_idx]);
}

void MeshVertexData::TruncateVertices(size_t new_count) {
  for (int group_idx = 0; group_idx < vertex_data_.size(); ++group_idx) {
    CheckVerticesNotMoved(group_idx);
    size_t new_size =
        description_.vertex_format.GetVertexSize(group_idx) * new_count;
    if (new_size < vertex_data_[group_idx].size) {
      description_.vertex_count = new_count;
      vertex_data_[group_idx].size = new_size;
    }
  }
}

void MeshVertexData::UpdateData(size_t group_idx, size_t offset,
                                absl::Span<const uint8_t> new_data) {
  CheckVerticesNotMoved(group_idx);
  if (offset + new_data.size() > vertex_data_[group_idx].size) {
    IMP_LOG(imp::FATAL) << "MeshVertexData UpdateData out of bounds: offset=" << offset
               << " size=" << new_data.size()
               << " max=" << vertex_data_[group_idx].size;
  }
  std::memcpy(static_cast<uint8_t*>(vertex_data_[group_idx].buffer) + offset,
              new_data.data(), new_data.size());
}

void MeshVertexData::CheckVerticesNotMoved(size_t group_idx) const {
  if (vertex_data_[group_idx].buffer == nullptr) {
    IMP_LOG(imp::FATAL) << "The vertex buffer has been moved!";
  }
}

}  // namespace imp
