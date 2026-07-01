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

#include "core/model/mesh/mesh_index_data.h"

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

MeshIndexData::MeshIndexData(const MeshDescription& description)
    : description_(description) {
  size_t buffer_size = description.GetIndexSize() * description.index_count;

  // Allocated with new, this is deleted by MeshDataBufferDescriptorDeleter when
  // the last copy of the buffer is destroyed.
  copy_counter_ = new imp_internal::MeshDataCopyCounter();

  index_data_ = BufferDescriptor(new uint8_t[buffer_size], buffer_size,
                                 &imp_internal::MeshDataBufferDescriptorDeleter,
                                 copy_counter_);
}

const MeshDescription& MeshIndexData::GetDescription() const {
  return description_;
}

MeshIndexData::BufferDescriptor MeshIndexData::CopyIndexData() const {
  CheckIndicesNotMoved();
  absl::MutexLock lock(copy_counter_->mu);
  ++copy_counter_->copies;
  return BufferDescriptor(index_data_.buffer, index_data_.size,
                          &imp_internal::MeshDataBufferDescriptorDeleter,
                          copy_counter_);
}

MeshIndexData::BufferDescriptor MeshIndexData::MoveIndexData() {
  CheckIndicesNotMoved();
  description_.index_count = 0;
  return std::move(index_data_);
}

void MeshIndexData::TruncateIndices(size_t new_count) {
  CheckIndicesNotMoved();
  size_t new_size = description_.GetIndexSize() * new_count;
  if (new_size < index_data_.size) {
    description_.index_count = new_count;
    index_data_.size = new_size;
  }
}

void MeshIndexData::UpdateData(size_t offset,
                               absl::Span<const uint8_t> new_data) {
  CheckIndicesNotMoved();
  if (offset + new_data.size() > index_data_.size) {
    IMP_LOG(imp::FATAL) << "MeshIndexData UpdateData out of bounds: offset=" << offset
               << " size=" << new_data.size() << " max=" << index_data_.size;
  }
  std::memcpy(static_cast<uint8_t*>(index_data_.buffer) + offset,
              new_data.data(), new_data.size());
}

void MeshIndexData::CheckIndicesNotMoved() const {
  if (index_data_.buffer == nullptr) {
    IMP_LOG(imp::FATAL) << "The index buffer has been moved!";
  }
}

}  // namespace imp
