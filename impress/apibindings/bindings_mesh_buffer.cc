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

#include "apibindings/bindings_mesh_buffer.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

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
#include "core/math/aabb_helpers.h"
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
    BindingsMeshBuffer::VertexAttribute attribute) {
  switch (attribute) {
    case BindingsMeshBuffer::VertexAttribute::kPosition:
      return filament::VertexAttribute::POSITION;
    case BindingsMeshBuffer::VertexAttribute::kNormal:
      return filament::VertexAttribute::TANGENTS;
    case BindingsMeshBuffer::VertexAttribute::kColor:
      return filament::VertexAttribute::COLOR;
    case BindingsMeshBuffer::VertexAttribute::kUv0:
      return filament::VertexAttribute::UV0;
    case BindingsMeshBuffer::VertexAttribute::kUv1:
      return filament::VertexAttribute::UV1;
    case BindingsMeshBuffer::VertexAttribute::kBoneIndices:
      return filament::VertexAttribute::BONE_INDICES;
    case BindingsMeshBuffer::VertexAttribute::kBoneWeights:
      return filament::VertexAttribute::BONE_WEIGHTS;
  }
}

VertexBuffer::AttributeType ToFilamentType(
    BindingsMeshBuffer::VertexAttributeType type) {
  switch (type) {
    case BindingsMeshBuffer::VertexAttributeType::kFloat:
      return VertexBuffer::AttributeType::FLOAT;
    case BindingsMeshBuffer::VertexAttributeType::kFloat2:
      return VertexBuffer::AttributeType::FLOAT2;
    case BindingsMeshBuffer::VertexAttributeType::kFloat3:
      return VertexBuffer::AttributeType::FLOAT3;
    case BindingsMeshBuffer::VertexAttributeType::kFloat4:
      return VertexBuffer::AttributeType::FLOAT4;
    case BindingsMeshBuffer::VertexAttributeType::kUByte4Norm:
      return VertexBuffer::AttributeType::UBYTE4;
    case BindingsMeshBuffer::VertexAttributeType::kUByte4:
      return VertexBuffer::AttributeType::UBYTE4;
  }
}

imp::VertexFormat::AttributeInfo ToImpressAttributeInfo(
    const BindingsMeshBuffer::VertexAttributeDescriptor& attr) {
  bool normalized =
      (attr.type == BindingsMeshBuffer::VertexAttributeType::kUByte4Norm);
  return imp::VertexFormat::AttributeInfo{
      .attribute = ToFilamentAttribute(attr.attribute),
      .type = ToFilamentType(attr.type),
      .normalized = normalized,
      .attribute_group_override = attr.buffer_index,
      .byte_offset = attr.byte_offset,
  };
}

// Copies vertex data from a source buffer to a destination buffer, converting
// float3 normals to float4 tangents in the process. This is necessary because
// Impress uses float4 tangents to represent normals, while clients may provide
// float3 normals.
void ConvertAndCopyVertexData(const void* src_data, void* dst_buffer,
                              int vertex_count, int src_stride, int dst_stride,
                              const imp::VertexFormat& input_format,
                              const imp::VertexFormat& output_format,
                              int buffer_index) {
  const uint8_t* src = static_cast<const uint8_t*>(src_data);
  uint8_t* dst = static_cast<uint8_t*>(dst_buffer);

  auto input_normal_key =
      input_format.GetKeyForAttribute(filament::VertexAttribute::TANGENTS);
  auto output_normal_key =
      output_format.GetKeyForAttribute(filament::VertexAttribute::TANGENTS);

  
  

  int input_normal_offset = input_normal_key->attribute_offset;
  int output_normal_offset = output_normal_key->attribute_offset;

  struct AttrCopy {
    size_t in_offset;
    size_t out_offset;
    size_t size;
  };
  std::vector<AttrCopy> attrs_to_copy;
  size_t num_attrs = input_format.GetNumAttributes(buffer_index);
  for (size_t j = 0; j < num_attrs; ++j) {
    const imp::VertexFormat::AttributeInfo& in_attr =
        input_format.GetAttributeAt(j, buffer_index);
    if (in_attr.attribute == filament::VertexAttribute::TANGENTS) {
      continue;
    }
    attrs_to_copy.push_back({
        input_format.GetAttributeOffsetAt(j, buffer_index),
        output_format.GetAttributeOffsetAt(j, buffer_index),
        imp::VertexFormat::GetAttributeSize(in_attr),
    });
  }

  for (int i = 0; i < vertex_count; ++i) {
    const uint8_t* v_src = src + i * src_stride;
    uint8_t* v_dst = dst + i * dst_stride;

    // Convert normal
    imp::float3 normal;
    std::memcpy(&normal, v_src + input_normal_offset, sizeof(imp::float3));
    imp::quatf tangent = imp::NormalToTangent(normal);
    std::memcpy(v_dst + output_normal_offset, &tangent, sizeof(tangent));

    // Copy other attributes
    for (const AttrCopy& attr : attrs_to_copy) {
      std::memcpy(v_dst + attr.out_offset, v_src + attr.in_offset, attr.size);
    }
  }
}

}  // namespace

BindingsMeshBuffer::BindingsMeshBuffer(
    ImpressApiView& view, const BindingsMeshBuffer::CreateOptions& options)
    : view_{view},
      layout_{options.layout},
      max_vertices_{options.max_vertices},
      max_indices_{options.max_indices} {
  // Determine number of buffers
  int max_buffer_index = -1;
  for (const BindingsMeshBuffer::VertexAttributeDescriptor& attr :
       layout_.attributes) {
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

  // Determine if each buffer needs normal conversion, and find position buffer.
  for (const BindingsMeshBuffer::VertexAttributeDescriptor& attr :
       layout_.attributes) {
    if (attr.attribute == VertexAttribute::kNormal &&
        attr.type == VertexAttributeType::kFloat3) {
      buffer_info_[attr.buffer_index].needs_normal_conversion = true;
    }
    if (attr.attribute == VertexAttribute::kPosition) {
      position_buffer_index_ = attr.buffer_index;
    }
  }
  

  // Calculate strides and formats
  for (const BindingsMeshBuffer::VertexAttributeDescriptor& attr :
       layout_.attributes) {
    const BufferInfo& info = buffer_info_[attr.buffer_index];

    input_format_.AppendAttribute(ToImpressAttributeInfo(attr));

    imp::VertexFormat::AttributeInfo out_info = ToImpressAttributeInfo(attr);
    if (info.needs_normal_conversion) {
      // When repacking normals, we tightly pack the output format to avoid
      // padding and simplify the repacking process. Custom byte offsets
      // specified in the input layout are ignored for the GPU buffer.
      out_info.byte_offset = imp::VertexFormat::AttributeInfo::kUnset;
      if (attr.attribute == VertexAttribute::kNormal) {
        out_info.type = VertexBuffer::AttributeType::FLOAT4;
      }
    }
    output_format_.AppendAttribute(out_info);
  }

  for (int i = 0; i < num_buffers; ++i) {
    if (i < layout_.strides.size() && layout_.strides[i] > 0) {
      input_format_.SetGroupByteStride(i, layout_.strides[i]);
      // Only set the output stride if normal conversion is not necessary.
      // If normal conversion is necessary, we tightly pack the output buffer to
      // avoid padding and simplify the repacking process, so we don't set a
      // custom stride and let VertexFormat compute it.
      if (!buffer_info_[i].needs_normal_conversion) {
        output_format_.SetGroupByteStride(i, layout_.strides[i]);
      }
    }
    buffer_info_[i].stride_in_bytes = input_format_.GetVertexSize(i);
    buffer_info_[i].output_stride_in_bytes = output_format_.GetVertexSize(i);
  }

  // If max_vertices_ is not set, calculate it from the initial vertex data.
  // We compute the minimum across all non-empty buffers.
  if (max_vertices_ == 0) {
    size_t num_buffers_to_check =
        std::min(options.initial_vertex_data.size(), buffer_info_.size());
    std::optional<int32_t> max_vertices;
    for (size_t i = 0; i < num_buffers_to_check; ++i) {
      const auto& info = buffer_info_[i];
      if (info.stride_in_bytes > 0 && !options.initial_vertex_data[i].empty()) {
        int32_t vertex_count =
            options.initial_vertex_data[i].size() / info.stride_in_bytes;
        if (!max_vertices.has_value() || vertex_count < *max_vertices) {
          max_vertices = vertex_count;
        }
      }
    }
    if (max_vertices.has_value()) {
      max_vertices_ = *max_vertices;
    }
  }

  // If max_indices_ is not set, calculate it from the initial index data.
  if (max_indices_ == 0) {
    max_indices_ = options.initial_index_data.size() / sizeof(uint32_t);
  }

  CreateRootMesh(options.initial_vertex_data, options.initial_index_data);
}

absl::Status BindingsMeshBuffer::UpdateVertexData(
    int32_t buffer_index, int32_t offset_in_bytes,
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

absl::Status BindingsMeshBuffer::UpdateIndexData(
    int32_t offset_in_bytes, absl::Span<const uint8_t> data) {
  if (offset_in_bytes < 0 ||
      offset_in_bytes + data.size() > max_indices_ * sizeof(uint32_t)) {
    return absl::InvalidArgumentError("Index update out of bounds.");
  }
  return UploadIndexData(offset_in_bytes, data.size(), data.data());
}

void BindingsMeshBuffer::CreateRootMesh(
    const std::vector<absl::Span<const uint8_t>>& initial_vertex_data,
    absl::Span<const uint8_t> initial_index_data) {
  imp::MeshDescription description;
  description.vertex_format = output_format_;
  description.vertex_count = max_vertices_;
  description.index_count = max_indices_;
  description.index_type = IndexBuffer::IndexType::UINT;

  auto mesh_data = std::make_unique<MeshData>(description);

  int num_position_vertices = 0;
  // Populate initial vertex data
  int num_vert_buffers_to_copy =
      std::min(initial_vertex_data.size(), buffer_info_.size());
  for (int i = 0; i < num_vert_buffers_to_copy; ++i) {
    if (initial_vertex_data[i].empty()) {
      // Skip empty buffers.
      continue;
    }

    const BufferInfo& info = buffer_info_[i];
    int32_t stride = info.stride_in_bytes;
    if (stride == 0) {
      continue;
    }

    int32_t output_stride = info.output_stride_in_bytes;
    int vertex_count = initial_vertex_data[i].size() / stride;
    vertex_count = std::min(vertex_count, static_cast<int>(max_vertices_));
    if (i == position_buffer_index_) {
      num_position_vertices = vertex_count;
    }

    if (vertex_count == 0) {
      continue;
    }

    void* dst_buffer = &mesh_data->VertexAttributeAt<uint8_t>(0, 0, i);

    if (info.needs_normal_conversion) {
      ConvertAndCopyVertexData(initial_vertex_data[i].data(), dst_buffer,
                               vertex_count, stride, output_stride,
                               input_format_, output_format_, i);
    } else {
      std::memcpy(dst_buffer, initial_vertex_data[i].data(),
                  vertex_count * stride);
    }
  }

  // Populate initial index data
  if (!initial_index_data.empty()) {
    int index_count = initial_index_data.size() / sizeof(uint32_t);
    index_count = std::min(index_count, static_cast<int>(max_indices_));
    if (index_count > 0) {
      std::memcpy(mesh_data->Indices<uint32_t>().data(),
                  initial_index_data.data(), index_count * sizeof(uint32_t));
    }
  }

  Box initial_aabb = Box{};
  // If position data is available, calculate the AABB.
  // Meshes using this mesh buffer may override this AABB.
  if (num_position_vertices > 0) {
    AabbCalculator aabb_calculator;
    auto pos_key =
        input_format_.GetKeyForAttribute(filament::VertexAttribute::POSITION);
    
    int position_offset = pos_key->attribute_offset;
    const uint8_t* src_data =
        initial_vertex_data[position_buffer_index_].data();
    int32_t stride = buffer_info_[position_buffer_index_].stride_in_bytes;
    for (int i = 0; i < num_position_vertices; ++i) {
      imp::float3 position;
      std::memcpy(&position, src_data + i * stride + position_offset,
                  sizeof(imp::float3));
      aabb_calculator.AddVertex(position);
    }
    initial_aabb = aabb_calculator.GetAabb();
  }

  MeshFactory factory(view_);
  root_mesh_ = factory.CreateByMovingMeshData(
      RenderableManager::PrimitiveType::TRIANGLES, std::move(mesh_data),
      initial_aabb, MeshFactory::MeshDataStorageMode::kDiscardMeshData);
}

absl::Status BindingsMeshBuffer::UploadVertexData(int32_t buffer_index,
                                                  int32_t offset, int32_t size,
                                                  const void* data) {
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
                             input_format_, output_format_, buffer_index);

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

absl::Status BindingsMeshBuffer::UploadIndexData(int32_t offset, int32_t size,
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
