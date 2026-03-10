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

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_MESH_BUFFER_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_MESH_BUFFER_H_

#include <cstdint>
#include <vector>

#include "absl/status/status.h"
#include "absl/types/span.h"
#include "core/model/mesh/mesh.h"

// Forward Declaration
namespace imp {
class ImpressApiView;
class VertexFormat;
}  // namespace imp

namespace imp {

// A buffer for holding vertex and index data for a mesh.
// This class manages vertex and index buffers for dynamic mesh data that can be
// updated frequently. The vertex data is stored in an interleaved format, where
// all attributes for a single vertex are stored contiguously in memory.
// The layout of these attributes is defined by the VertexLayout provided at
// construction.
// The data passed into UpdateVertexData and UpdateIndexData is copied once
// into a buffer that is then passed to Impress to update the mesh data.
class MeshBuffer {
 public:
  // Vertex attributes that can be used to construct the vertex layout.
  enum class VertexAttribute {
    kPosition,
    kNormal,
    kColor,
    kUv0,
    kUv1,
    kBoneIndices,
    kBoneWeights,
  };

  // Vertex attribute types.
  enum class VertexAttributeType {
    kFloat,
    kFloat2,
    kFloat3,
    kFloat4,
    kUByte4Norm,
    kUByte4,
  };

  // A single vertex attribute descriptor.
  struct VertexAttributeDescriptor {
    VertexAttribute attribute;
    VertexAttributeType type;
    // The index of the buffer this attribute belongs to. uint8_t
    // matches imp::VertexFormat::AttributeInfo it is converted to.
    uint8_t buffer_index = 0;
  };

  // A layout for the vertex data.
  struct VertexLayout {
    std::vector<VertexAttributeDescriptor> attributes;
  };

  // Creates a new MeshBuffer with the given layout, maximum number of vertices,
  // and maximum number of indices.
  MeshBuffer(ImpressApiView& view, const VertexLayout& layout,
             int32_t max_vertices, int32_t max_indices);

  // Updates the vertex data for the mesh.
  // The offset_in_bytes must be a multiple of the vertex stride.
  // The data size must be a multiple of the vertex stride.
  absl::Status UpdateVertexData(int32_t buffer_index, int32_t offset_in_bytes,
                                absl::Span<const uint8_t> data);

  // Updates the index data for the mesh.
  // The offset_in_bytes must be a multiple of the index stride (4 bytes).
  // The data size must be a multiple of the index stride (4 bytes).
  absl::Status UpdateIndexData(int32_t offset_in_bytes,
                               absl::Span<const uint8_t> data);

  // Returns the maximum number of vertices that can be stored in the mesh
  // buffer.
  int32_t GetMaxVertices() const { return max_vertices_; }

  // Returns the maximum number of indices that can be stored in the mesh
  // buffer.
  int32_t GetMaxIndices() const { return max_indices_; }

  // Returns the root mesh for the mesh buffer.
  BorrowedMeshPtr BorrowRootMesh() { return root_mesh_.Borrow(); }

  // Returns the vertex layout of the mesh buffer.
  const VertexLayout& GetLayout() const { return layout_; }

 private:
  friend class MeshBufferTest;

  void CreateRootMesh();
  absl::Status UploadVertexData(int32_t buffer_index, int32_t offset,
                                int32_t size, const void* data);
  absl::Status UploadIndexData(int32_t offset, int32_t size, const void* data);

  ImpressApiView& view_;
  VertexLayout layout_;

  const int32_t max_vertices_;
  const int32_t max_indices_;

  OwnedMeshPtr root_mesh_;

  // Per-buffer data
  struct BufferInfo {
    int32_t stride_in_bytes = 0;
    int32_t output_stride_in_bytes = 0;
    // Indicates whether float3 normal to float4 tangent conversion is
    // necessary.
    bool needs_normal_conversion = false;
  };
  std::vector<BufferInfo> buffer_info_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_MESH_BUFFER_H_
