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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_DATA_H_

#include <cstddef>
#include <memory>

#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/BufferDescriptor.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"

namespace imp {

// A data buffer for vertices and indices. It holds the underlying data buffers,
// and can either create read-only copys or be moved from once. It cannot be
// copied or moved, so prefer to use MeshDataPtr.
//
// ---------------------------------------------------------------------------
// Thread safety.
// ---------------------------------------------------------------------------
// This class is Thread-compatible ((broken link)) with some caveats.
// Get*Data() creates BufferDescriptor copies that can be passed to other
// threads and deleted independently. However, they don't enforce const access
// to their buffer pointer.
// Also, IsSafeToDestroy() won't prevent more copies from being created, so the
// thread evaluating IsSafeToDestroy() should also be the only thread allowed to
// call Get*Data().
class MeshData {
 public:
  using BufferDescriptor = filament::backend::BufferDescriptor;

  // Allocates new buffers to store the specified vertices and indices.
  explicit MeshData(const MeshDescription& description);

  const MeshDescription& GetDescription() const;

  // Access the vertex data as type T, which must match the size described by
  // the description's VertexFormat.
  template <typename T>
  T& VertexAt(size_t index, size_t group_idx = 0);

  template <typename T>
  absl::Span<T> Vertices(size_t group_idx = 0);

  // Access the |attribute| data as type |T| at |index|. |attribute| must exist
  // in the MeshDescription's VertexFormat, and |T| must be the same size as
  // the type for that attribute.
  template <typename T>
  T& VertexAttributeAt(size_t index, VertexFormat::VertexAttribute attribute);

  // Access the |attribute| data as type |T| at |index| with the given
  // |attribute_offset|. |T| must be the same size as the type for that
  // attribute.
  //
  // Faster than VertexAttributeAt(index, attribute) if you already know the
  // offset. Requires a little bit more care to use correctly by getting the
  // attribute offset from the vertex format in the MeshDescription.
  template <typename T>
  T& VertexAttributeAt(size_t index, size_t attribute_offset,
                       size_t group_idx = 0);
  template <typename T>
  T& VertexAttributeAt(size_t index, VertexFormat::AttributeKey key);
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
  BufferDescriptor CopyVertexData(size_t group_idx = 0) const;
  BufferDescriptor CopyIndexData() const;

  // Moves out the data.  IMPORTANT! The respective buffers in MeshData are
  // empty after these calls.
  BufferDescriptor MoveVertexData(size_t group_idx = 0);
  BufferDescriptor MoveIndexData();

  // Reduces the size of the buffers to avoid uploading the extra data to the
  // device. The `new_count` must be smaller than the current count, or this
  // function will do nothing.
  // IMPORTANT! Truncation does nothing for any existing copies of the
  // index/vertex data.
  void TruncateVertices(size_t new_count);
  void TruncateIndices(size_t new_count);

  // Return underlying vertex and index data.
  MeshVertexData* GetVertexData() { return &vertex_data_; }
  MeshIndexData* GetIndexData() { return &index_data_; }

 private:
  MeshDescription description_;
  MeshVertexData vertex_data_;
  MeshIndexData index_data_;
};

template <typename T>
absl::Span<T> MeshData::Vertices(size_t group_idx) {
  return vertex_data_.Vertices<T>(group_idx);
}

template <typename T>
T& MeshData::VertexAt(size_t index, size_t group_idx) {
  return vertex_data_.VertexAt<T>(index, group_idx);
}

template <typename T>
T& MeshData::VertexAttributeAt(size_t index,
                               VertexFormat::VertexAttribute attribute) {
  return vertex_data_.VertexAttributeAt<T>(index, attribute);
}

template <typename T>
T& MeshData::VertexAttributeAt(size_t index, size_t attribute_offset,
                               size_t group_idx) {
  return vertex_data_.VertexAttributeAt<T>(index, attribute_offset, group_idx);
}

template <typename T>
T& MeshData::VertexAttributeAt(size_t index, VertexFormat::AttributeKey key) {
  return vertex_data_.VertexAttributeAt<T>(index, key);
}

template <typename T>
absl::Span<T> MeshData::Indices() {
  return index_data_.Indices<T>();
}

template <typename T>
T& MeshData::IndexAt(size_t index) {
  return index_data_.IndexAt<T>(index);
}

using MeshDataPtr = std::unique_ptr<MeshData>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_DATA_H_
