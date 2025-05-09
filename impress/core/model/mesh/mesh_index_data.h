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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_INDEX_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_INDEX_DATA_H_

#include <cstddef>
#include <memory>

#include "core/common/log.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/BufferDescriptor.h"
#include "core/model/mesh/mesh_data_helper.h"
#include "core/model/mesh/mesh_description.h"

namespace imp {

// A data buffer for indices. It holds the underlying data buffers,
// and can either create read-only copies or be moved from once. It cannot be
// copied or moved, so prefer to use MeshIndexDataPtr.
class MeshIndexData {
 public:
  using BufferDescriptor = filament::backend::BufferDescriptor;

  explicit MeshIndexData(const MeshDescription& description);

  const MeshDescription& GetDescription() const;

  template <typename T>
  absl::Span<T> Indices();

  // Access the index data as type T, which must match the size described by
  // the description's IndexType.
  template <typename T>
  T& IndexAt(size_t index);

  // Get a non-owning copy of the data.  The BufferDescriptors can be passed
  // between threads and will safely update the copy counts in this class.
  //
  // Note, even though this method is const, the returned BufferDescriptor
  // doesn't maintain that const.
  BufferDescriptor CopyIndexData() const;

  // Moves out the data.  IMPORTANT! The respective buffers in MeshData are
  // empty after these calls.
  BufferDescriptor MoveIndexData();

  // Reduces the size of the buffers to avoid uploading the extra data to the
  // device. The `new_count` must be smaller than the current count, or this
  // function will do nothing.
  // IMPORTANT! Truncation does nothing for any existing copies of the
  // index data.
  void TruncateIndices(size_t new_count);

 private:
  // If the indices are moved, fatals.
  void CheckIndicesNotMoved() const;
  MeshDescription description_;
  BufferDescriptor index_data_;
  imp_internal::MeshDataCopyCounter* copy_counter_ = nullptr;
};

template <typename T>
absl::Span<T> MeshIndexData::Indices() {
  CheckIndicesNotMoved();
  const auto t_size = sizeof(T);
  const auto index_size = description_.GetIndexSize();
  if (t_size != index_size) {
    IMP_LOG(imp::FATAL) << "Type size " << t_size << " does not match index size "
               << index_size;
  }
  return absl::MakeSpan(static_cast<T*>(index_data_.buffer),
                        description_.index_count);
}

template <typename T>
T& MeshIndexData::IndexAt(size_t index) {
  auto indices = Indices<T>();
  if (index >= description_.index_count) {
    IMP_LOG(imp::FATAL) << "Index " << index
               << " out of bounds: " << description_.index_count;
  }
  return indices[index];
}

using MeshIndexDataPtr = std::unique_ptr<MeshIndexData>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_INDEX_DATA_H_
