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

#include "apibindings/mesh_buffer.h"

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/types/span.h"
#include "filament/filament/backend/include/backend/BufferDescriptor.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/RenderableManager.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "apibindings/impress_api_view.h"
#include "core/common/owned_ptr.h"
#include "core/geometry/shapes/box.h"
#include "core/math/math.h"
#include "core/model/mesh/mesh.h"
#include "core/model/mesh/mesh_data.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_factory.h"
#include "core/model/mesh/vertex_format.h"
#include "core/window/filament_host.h"

namespace imp {

namespace {

using ::filament::IndexBuffer;
using ::filament::RenderableManager;
using ::filament::VertexBuffer;

filament::VertexAttribute ToFilamentAttribute(
    MeshBuffer::VertexAttribute attribute) {
  switch (attribute) {
    case MeshBuffer::VertexAttribute::kPosition:
      return filament::VertexAttribute::POSITION;
    case MeshBuffer::VertexAttribute::kNormal:
      return filament::VertexAttribute::TANGENTS;
    case MeshBuffer::VertexAttribute::kColor:
      return filament::VertexAttribute::COLOR;
    case MeshBuffer::VertexAttribute::kUv0:
      return filament::VertexAttribute::UV0;
    case MeshBuffer::VertexAttribute::kUv1:
      return filament::VertexAttribute::UV1;
    case MeshBuffer::VertexAttribute::kBoneIndices:
      return filament::VertexAttribute::BONE_INDICES;
    case MeshBuffer::VertexAttribute::kBoneWeights:
      return filament::VertexAttribute::BONE_WEIGHTS;
  }
}

VertexBuffer::AttributeType ToFilamentType(
    MeshBuffer::VertexAttributeType type) {
  switch (type) {
    case MeshBuffer::VertexAttributeType::kFloat:
      return VertexBuffer::AttributeType::FLOAT;
    case MeshBuffer::VertexAttributeType::kFloat2:
      return VertexBuffer::AttributeType::FLOAT2;
    case MeshBuffer::VertexAttributeType::kFloat3:
      return VertexBuffer::AttributeType::FLOAT3;
    case MeshBuffer::VertexAttributeType::kFloat4:
      return VertexBuffer::AttributeType::FLOAT4;
    case MeshBuffer::VertexAttributeType::kUByte4Norm:
      return VertexBuffer::AttributeType::UBYTE4;
    case MeshBuffer::VertexAttributeType::kUByte4:
      return VertexBuffer::AttributeType::UBYTE4;
  }
}

size_t GetTypeSize(MeshBuffer::VertexAttributeType type) {
  switch (type) {
    case MeshBuffer::VertexAttributeType::kFloat:
      return 4;
    case MeshBuffer::VertexAttributeType::kFloat2:
      return 8;
    case MeshBuffer::VertexAttributeType::kFloat3:
      return 12;
    case MeshBuffer::VertexAttributeType::kFloat4:
      return 16;
    case MeshBuffer::VertexAttributeType::kUByte4Norm:
      return 4;
    case MeshBuffer::VertexAttributeType::kUByte4:
      return 4;
  }
}

imp::VertexFormat::AttributeInfo ToImpressAttributeInfo(
    const MeshBuffer::VertexAttributeDescriptor& attr) {
  bool normalized = (attr.type == MeshBuffer::VertexAttributeType::kUByte4Norm);
  return imp::VertexFormat::AttributeInfo{ToFilamentAttribute(attr.attribute),
                                          ToFilamentType(attr.type), normalized,
                                          attr.buffer_index};
}

// Copies vertex data from a source buffer to a destination buffer, converting
// float3 normals to float4 tangents in the process. This is necessary because
// Impress uses float4 tangents to represent normals, while clients may provide
// float3 normals.
void ConvertAndCopyVertexData(const void* src_data, void* dst_buffer,
                              int vertex_count, int src_stride, int dst_stride,
                              const MeshBuffer::VertexLayout& layout,
                              int buffer_index) {
  const uint8_t* src = static_cast<const uint8_t*>(src_data);
  uint8_t* dst = static_cast<uint8_t*>(dst_buffer);

  int pre_normal_size = 0;
  bool normal_found = false;

  // Calculate pre normal size for the specific buffer
  for (const auto& attr : layout.attributes) {
    if (attr.buffer_index != buffer_index) continue;

    if (attr.attribute == MeshBuffer::VertexAttribute::kNormal) {
      
      normal_found = true;
      break;
    }
    pre_normal_size += GetTypeSize(attr.type);
  }

  

  const int post_normal_size =
      src_stride - pre_normal_size - sizeof(imp::float3);

  for (int i = 0; i < vertex_count; ++i) {
    const uint8_t* v_src = src + i * src_stride;
    uint8_t* v_dst = dst + i * dst_stride;

    // Copy data before normal
    if (pre_normal_size > 0) {
      std::memcpy(v_dst, v_src, pre_normal_size);
    }

    // Convert normal
    imp::float3 normal;
    std::memcpy(&normal, v_src + pre_normal_size, sizeof(imp::float3));
    imp::quatf tangent = imp::NormalToTangent(normal);
    std::memcpy(v_dst + pre_normal_size, &tangent, sizeof(tangent));

    // Copy data after normal
    if (post_normal_size > 0) {
      std::memcpy(v_dst + pre_normal_size + sizeof(tangent),
                  v_src + pre_normal_size + sizeof(imp::float3),
                  post_normal_size);
    }
  }
}

}  // namespace

MeshBuffer::MeshBuffer(ImpressApiView& view,
                       const MeshBuffer::VertexLayout& layout,
                       int32_t max_vertices, int32_t max_indices)
    : view_{view},
      layout_{layout},
      max_vertices_{max_vertices},
      max_indices_{max_indices} {
  // Determine number of buffers
  int max_buffer_index = -1;
  for (const MeshBuffer::VertexAttributeDescriptor& attr : layout_.attributes) {
    if (attr.buffer_index > max_buffer_index) {
      max_buffer_index = attr.buffer_index;
    }
  }
  int num_buffers = max_buffer_index + 1;
  constexpr int kMaxVertexBuffers = filament::backend::MAX_VERTEX_BUFFER_COUNT;
  if (num_buffers > kMaxVertexBuffers) {
    IMP_LOG(imp::FATAL) << "Number of vertex buffers exceeds maximum of "
               << kMaxVertexBuffers;
  }
  buffer_info_.resize(num_buffers);

  // Calculate strides
  for (const MeshBuffer::VertexAttributeDescriptor& attr : layout_.attributes) {
    BufferInfo& info = buffer_info_[attr.buffer_index];
    info.stride_in_bytes += GetTypeSize(attr.type);
    if (attr.attribute == VertexAttribute::kNormal &&
        attr.type == VertexAttributeType::kFloat3) {
      info.needs_normal_conversion = true;
      // Convert float3 normal to float4 tangent
      info.output_stride_in_bytes += 16;
    } else {
      info.output_stride_in_bytes += GetTypeSize(attr.type);
    }
  }
  CreateRootMesh();
}

absl::Status MeshBuffer::UpdateVertexData(int32_t buffer_index,
                                          int32_t offset_in_bytes,
                                          absl::Span<const uint8_t> data) {
  if (buffer_index < 0 || buffer_index >= buffer_info_.size()) {
    return absl::InvalidArgumentError("Invalid buffer index.");
  }
  if (offset_in_bytes < 0 ||
      static_cast<size_t>(offset_in_bytes) + data.size() >
          static_cast<size_t>(max_vertices_) *
              buffer_info_[buffer_index].stride_in_bytes) {
    return absl::InvalidArgumentError("Vertex update out of bounds.");
  }

  return UploadVertexData(buffer_index, offset_in_bytes, data.size(),
                          data.data());
}

absl::Status MeshBuffer::UpdateIndexData(int32_t offset_in_bytes,
                                         absl::Span<const uint8_t> data) {
  if (offset_in_bytes < 0 || offset_in_bytes + data.size() > max_indices_ * 4) {
    return absl::InvalidArgumentError("Index update out of bounds.");
  }
  return UploadIndexData(offset_in_bytes, data.size(), data.data());
}

void MeshBuffer::CreateRootMesh() {
  imp::VertexFormat format;
  for (const auto& attr : layout_.attributes) {
    auto info = ToImpressAttributeInfo(attr);
    if (buffer_info_[attr.buffer_index].needs_normal_conversion &&
        attr.attribute == VertexAttribute::kNormal) {
      info.type = VertexBuffer::AttributeType::FLOAT4;
    }
    format.AppendAttribute(info);
  }

  imp::MeshDescription description;
  description.vertex_format = format;
  description.vertex_count = max_vertices_;
  description.index_count = max_indices_;
  description.index_type = IndexBuffer::IndexType::UINT;

  auto mesh_data = std::make_unique<MeshData>(description);

  // MeshBuffer is a container for vertex data. By itself, it does not
  // define the geometry. The bounds will be defined by the custom mesh
  // that will reference the MeshBuffer.
  const imp::Box kEmptyBounds;
  MeshFactory factory(view_);
  root_mesh_ = factory.CreateByMovingMeshData(
      RenderableManager::PrimitiveType::TRIANGLES, std::move(mesh_data),
      kEmptyBounds, MeshFactory::MeshDataStorageMode::kDiscardMeshData);
}

absl::Status MeshBuffer::UploadVertexData(int32_t buffer_index, int32_t offset,
                                          int32_t size, const void* data) {
  if (!root_mesh_) return absl::FailedPreconditionError("Root mesh not ready.");
  if (buffer_index < 0 || buffer_index >= buffer_info_.size()) {
    return absl::InvalidArgumentError("Invalid buffer index.");
  }
  const BufferInfo& info = buffer_info_[buffer_index];
  int32_t stride = info.stride_in_bytes;
  int32_t output_stride = info.output_stride_in_bytes;

  if (stride == 0) {
    return absl::InvalidArgumentError(
        "Vertex buffer at index " + std::to_string(buffer_index) +
        " has zero stride, which indicates that it is not used.");
  }
  if (offset % stride != 0) {
    return absl::InvalidArgumentError(
        "Vertex buffer offset must be aligned to vertex stride.");
  }
  if (size % stride != 0) {
    return absl::InvalidArgumentError(
        "Vertex buffer size must be aligned to vertex stride.");
  }
  VertexBuffer* vb = root_mesh_->GetVertexBuffer();
  if (!vb) return absl::InternalError("No VertexBuffer.");

  int vertex_count = size / stride;
  size_t buffer_size = vertex_count * output_stride;
  void* buffer = new uint8_t[buffer_size];

  if (info.needs_normal_conversion) {
    ConvertAndCopyVertexData(data, buffer, vertex_count, stride, output_stride,
                             layout_, buffer_index);

    // Adjust offset for GPU buffer
    offset = (offset / stride) * output_stride;
  } else {
    std::memcpy(buffer, data, size);
  }

  filament::backend::BufferDescriptor desc(
      buffer, buffer_size, [](void* buffer, size_t size, void* user) {
        delete[] static_cast<uint8_t*>(buffer);
      });

  vb->setBufferAt(*view_.GetHost()->GetEngine(), buffer_index, std::move(desc),
                  offset);
  return absl::OkStatus();
}

absl::Status MeshBuffer::UploadIndexData(int32_t offset, int32_t size,
                                         const void* data) {
  if (!root_mesh_) return absl::FailedPreconditionError("Root mesh not ready.");
  IndexBuffer* ib = root_mesh_->GetIndexBuffer();
  if (!ib) return absl::InternalError("No IndexBuffer.");

  void* buffer = new uint8_t[size];
  std::memcpy(buffer, data, size);

  filament::backend::BufferDescriptor desc(
      buffer, size, [](void* buffer, size_t size, void* user) {
        delete[] static_cast<uint8_t*>(buffer);
      });

  ib->setBuffer(*view_.GetHost()->GetEngine(), std::move(desc), offset);
  return absl::OkStatus();
}

}  // namespace imp
