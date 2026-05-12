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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_VERTEX_DATA_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_VERTEX_DATA_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/common/log.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/BufferDescriptor.h"
#include "core/model/mesh/mesh_data_helper.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/vertex_format.h"

namespace imp {

// A data buffer for vertices. It holds the underlying data buffers,
// and can either create read-only copies or be moved from once. It cannot be
// copied or moved, so prefer to use MeshVertexDataPtr.
class MeshVertexData {
 public:
  using BufferDescriptor = filament::backend::BufferDescriptor;

  explicit MeshVertexData(const MeshDescription& description);

  const MeshDescription& GetDescription() const;

  template <typename T>
  absl::Span<T> Vertices(size_t group_idx = 0);

  // Access the vertex data as type T, which must match the size described by
  // the description's VertexFormat.
  template <typename T>
  T& VertexAt(size_t vertex_index, size_t group_idx = 0);

  // Access the |attribute| data as type |T| at |index|. |attribute| must exist
  // in the MeshDescription's VertexFormat, and |T| must be the same size as
  // the type for that attribute.
  template <typename T>
  T& VertexAttributeAt(size_t vertex_index,
                       VertexFormat::VertexAttribute attribute);

  // Access the |attribute| data as type |T| at |index| with the given
  // |attribute_offset|. |T| must be the same size as the type for that
  // attribute.
  //
  // Faster than VertexAttributeAt(index, attribute) if you already know the
  // offset. Requires a little bit more care to use correctly by getting the
  // attribute offset from the vertex format in the MeshDescription.
  template <typename T>
  const T& VertexAttributeAt(size_t vertex_index, size_t attribute_offset,
                             size_t group_idx = 0) const;

  template <typename T>
  const T& VertexAttributeAt(size_t vertex_index,
                             VertexFormat::AttributeKey key) const;

  template <typename T>
  T& VertexAttributeAt(size_t vertex_index, size_t attribute_offset,
                       size_t group_idx = 0);
  template <typename T>
  T& VertexAttributeAt(size_t vertex_index, VertexFormat::AttributeKey key);

  // Get a non-owning copy of the data.  The BufferDescriptors can be passed
  // between threads and will safely update the copy counts in this class.
  //
  // Note, even though this method is const, the returned BufferDescriptor
  // doesn't maintain that const.
  BufferDescriptor CopyVertexData(size_t group_idx = 0) const;

  // Moves out the data.  IMPORTANT! The respective buffers in MeshData are
  // empty after these calls.
  BufferDescriptor MoveVertexData(size_t group_idx = 0);

  // Reduces the size of the buffers to avoid uploading the extra data to the
  // device. The `new_count` must be smaller than the current count, or this
  // function will do nothing.
  // IMPORTANT! Truncation does nothing for any existing copies of the
  // vertex data.
  void TruncateVertices(size_t new_count);

 private:
  // If the vertices are moved, fatals.
  void CheckVerticesNotMoved(size_t group_idx) const;
  MeshDescription description_;
  std::vector<BufferDescriptor> vertex_data_;
  std::vector<imp_internal::MeshDataCopyCounter*> copy_counter_;
};

template <typename T>
absl::Span<T> MeshVertexData::Vertices(size_t group_idx) {
  CheckVerticesNotMoved(group_idx);
  const auto t_size = sizeof(T);
  const auto vertex_size = description_.vertex_format.GetVertexSize(group_idx);
  if (t_size != vertex_size) {
    IMP_LOG(imp::FATAL) << "Type size " << t_size << " does not match vertex size "
               << vertex_size;
  }
  return absl::MakeSpan(static_cast<T*>(vertex_data_[group_idx].buffer),
                        description_.vertex_count);
}

template <typename T>
T& MeshVertexData::VertexAt(size_t vertex_index, size_t group_idx) {
  auto vertices = Vertices<T>(group_idx);
  if (vertex_index >= description_.vertex_count) {
    IMP_LOG(imp::FATAL) << "Vertex index " << vertex_index
               << " out of bounds: " << description_.vertex_count;
  }
  return vertices[vertex_index];
}

template <typename T>
T& MeshVertexData::VertexAttributeAt(size_t vertex_index,
                                     VertexFormat::VertexAttribute attribute) {
  const VertexFormat& vertex_format = description_.vertex_format;
  absl::optional<VertexFormat::AttributeKey> key =
      vertex_format.GetKeyForAttribute(attribute);
  if (!key) {
    IMP_LOG(imp::FATAL) << "Attribute " << attribute
               << " does not exist in MeshDescription";
  }
  CheckVerticesNotMoved(key->group_index);
  size_t t_size = sizeof(T);
  if (t_size != key->attribute_size) {
    IMP_LOG(imp::FATAL) << "Type size " << t_size << " does not match attribute size "
               << key->attribute_size;
  }
  if (vertex_index >= description_.vertex_count) {
    IMP_LOG(imp::FATAL) << "Index " << vertex_index
               << " out of bounds: " << description_.vertex_count;
  }

  return VertexAttributeAt<T>(vertex_index, *key);
}

template <typename T>
T& MeshVertexData::VertexAttributeAt(size_t vertex_index,
                                     size_t attribute_offset,
                                     size_t group_idx) {
  size_t total_offset =
      vertex_index * description_.vertex_format.GetVertexSize(group_idx) +
      attribute_offset;
  return reinterpret_cast<T&>(
      static_cast<uint8_t*>(vertex_data_[group_idx].buffer)[total_offset]);
}

template <typename T>
T& MeshVertexData::VertexAttributeAt(size_t vertex_index,
                                     VertexFormat::AttributeKey key) {
  size_t total_offset =
      vertex_index * description_.vertex_format.GetVertexSize(key.group_index) +
      key.attribute_offset;
  return reinterpret_cast<T&>(static_cast<uint8_t*>(
      vertex_data_[key.group_index].buffer)[total_offset]);
}

template <typename T>
const T& MeshVertexData::VertexAttributeAt(size_t vertex_index,
                                           size_t attribute_offset,
                                           size_t group_idx) const {
  return const_cast<MeshVertexData*>(this)->VertexAttributeAt<T>(
      vertex_index, attribute_offset, group_idx);
}

template <typename T>
const T& MeshVertexData::VertexAttributeAt(
    size_t vertex_index, VertexFormat::AttributeKey key) const {
  return const_cast<MeshVertexData*>(this)->VertexAttributeAt<T>(vertex_index,
                                                                 key);
}

using MeshVertexDataPtr = std::unique_ptr<MeshVertexData>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_VERTEX_DATA_H_
