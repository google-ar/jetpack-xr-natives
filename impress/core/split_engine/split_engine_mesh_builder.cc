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

#include "core/split_engine/split_engine_mesh_builder.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/BufferDescriptor.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "core/math/vec.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/model/mesh/mesh_builder.h"
#include "core/split_engine/flatbuffer_size_calculator.h"
#include "core/split_engine/split_engine_serializer.h"
#include "split_engine/schemas/split_engine_data_generated.h"

namespace imp::split_engine {

SplitEngineVertexBufferBuilder::SplitEngineVertexBufferBuilder() noexcept
    : enable_buffer_objects_(false), advanced_skinning_(false) {}

SplitEngineVertexBufferBuilder& SplitEngineVertexBufferBuilder::BufferCount(
    uint8_t bufferCount) noexcept {
  return *this;
}

SplitEngineVertexBufferBuilder& SplitEngineVertexBufferBuilder::VertexCount(
    uint32_t vertexCount) noexcept {
  return *this;
}

SplitEngineVertexBufferBuilder&
SplitEngineVertexBufferBuilder::EnableBufferObjectsInternal(
    bool enabled) noexcept {
  enable_buffer_objects_ = enabled;
  return *this;
}

SplitEngineVertexBufferBuilder&
SplitEngineVertexBufferBuilder::AdvancedSkinningInternal(
    bool enabled) noexcept {
  advanced_skinning_ = enabled;
  return *this;
}

// TODO: this requires things being built in the right order, i.e.
// have to do all the Attributes for a buffer and then the buffer itself before
// moving to the next bufferIndex.
SplitEngineVertexBufferBuilder&
SplitEngineVertexBufferBuilder::AttributeInternal(
    filament::VertexAttribute attribute, uint8_t bufferIndex,
    filament::VertexBuffer::AttributeType attributeType, uint32_t byteOffset,
    uint8_t byteStride, bool normalized) noexcept {
  if (bufferIndex >= attributes_.size()) {
    // We require an in-order build.
    assert(bufferIndex == attributes_.size());
    attributes_.emplace_back();
    strides_.emplace_back(byteStride);
  }
  // All strides must match.
  assert(strides_[bufferIndex] == byteStride);

  attributes_[bufferIndex].emplace_back(
      static_cast<android_xr::schemas::VertexAttribute>(attribute),
      static_cast<android_xr::schemas::AttributeType>(attributeType),
      byteOffset, normalized);

  return *this;
}

SplitEngineVertexBufferBuilder&
SplitEngineVertexBufferBuilder::BufferAtInternal(
    filament::Engine& engine, uint8_t bufferIndex,
    filament::backend::BufferDescriptor&& buffer,
    uint32_t byteOffset) noexcept {
  // TODO: VertexBufferInfo doesn't have an offset field.
  assert(byteOffset == 0);

  buffer_descriptors_[bufferIndex] = std::move(buffer);
  return *this;
}

SplitEngineVertexBufferBuilder&
SplitEngineVertexBufferBuilder::VertexAccessFlags(
    uint8_t vertexAccessFlags) noexcept {
  vertex_access_flags_ = vertexAccessFlags;
  return *this;
}

SplitEngineVertexBufferBuilder& SplitEngineVertexBufferBuilder::Name(
    absl::string_view name) noexcept {
  // TODO: Implement this.
  return *this;
}

filament::VertexBuffer* SplitEngineVertexBufferBuilder::Build(
    filament::Engine& engine) noexcept {
  // This should never be called.
  IMP_LOG(imp::FATAL) << "SplitEngineVertexBufferBuilder::Build() is not implemented";
  return nullptr;
}

void SplitEngineVertexBufferBuilder::ContributeBufferSize(
    FlatbufferSizeCalculator& calculator) noexcept {
  for (std::pair<const unsigned char, filament::backend::BufferDescriptor>&
           buffer_descriptor : buffer_descriptors_) {
    int bufferIndex = buffer_descriptor.first;
    calculator.AddAttributeVector(attributes_[bufferIndex].size());
    calculator.AddVector(buffer_descriptor.second.size, sizeof(uint8_t));
    calculator.AddVertexBlockInfo();
  }
  calculator.AddReferenceVector(buffer_descriptors_.size());
  calculator.AddVertexBufferInfo();
  calculator.AddVertexBuffer();
}

flatbuffers::Offset<android_xr::schemas::VertexBuffer>
SplitEngineVertexBufferBuilder::SerializeVertexBuffer(
    flatbuffers::FlatBufferBuilder& builder) noexcept {
  std::vector<flatbuffers::Offset<android_xr::schemas::VertexBlockInfo>>
      vertex_blocks;

  if (vertex_blocks.empty()) {
    vertex_blocks.resize(attributes_.size());
  }
  for (std::pair<const unsigned char, filament::backend::BufferDescriptor>&
           buffer_descriptor : buffer_descriptors_) {
    int bufferIndex = buffer_descriptor.first;
    filament::backend::BufferDescriptor& buffer = buffer_descriptor.second;

    vertex_blocks[bufferIndex] = android_xr::schemas::CreateVertexBlockInfo(
        builder, builder.CreateVectorOfStructs(attributes_[bufferIndex]),
        builder.CreateVector(reinterpret_cast<uint8_t*>(buffer.buffer),
                             buffer.size),
        strides_[bufferIndex]);
  }

  return android_xr::schemas::CreateVertexBuffer(
      builder, SplitEngineSerializer::GetId(vertex_buffer_),
      android_xr::schemas::CreateVertexBufferInfo(
          builder, builder.CreateVector(vertex_blocks),
          vertex_buffer_->getVertexCount(), advanced_skinning_),
      static_cast<android_xr::schemas::VertexAccessFlags>(
          vertex_access_flags_));
}

void SplitEngineVertexBufferBuilder::Finalize(
    filament::VertexBuffer* vertex_buffer) noexcept {
  vertex_buffer_ = vertex_buffer;
}

SplitEngineIndexBufferBuilder::SplitEngineIndexBufferBuilder() noexcept {}

SplitEngineIndexBufferBuilder& SplitEngineIndexBufferBuilder::IndexCount(
    uint32_t indexCount) noexcept {
  return *this;
}
SplitEngineIndexBufferBuilder& SplitEngineIndexBufferBuilder::BufferType(
    filament::IndexBuffer::IndexType indexType) noexcept {
  index_type_ = indexType;
  return *this;
}
SplitEngineIndexBufferBuilder& SplitEngineIndexBufferBuilder::BufferInternal(
    filament::Engine& engine, filament::IndexBuffer::BufferDescriptor&& buffer,
    uint32_t byteOffset) noexcept {
  buffer_ = std::move(buffer);

  return *this;
}

SplitEngineIndexBufferBuilder& SplitEngineIndexBufferBuilder::StoreIndexData(
    bool store_index_data) noexcept {
  store_index_data_ = store_index_data;
  return *this;
}

SplitEngineIndexBufferBuilder& SplitEngineIndexBufferBuilder::Name(
    absl::string_view name) noexcept {
  // TODO: Implement this.
  return *this;
}

filament::IndexBuffer* SplitEngineIndexBufferBuilder::Build(
    filament::Engine& engine) noexcept {
  // This should never be called.
  IMP_LOG(imp::FATAL) << "SplitEngineIndexBufferBuilder::Build() is not implemented";
  return nullptr;
}

void SplitEngineIndexBufferBuilder::Finalize(
    filament::IndexBuffer* index_buffer) noexcept {
  index_buffer_ = index_buffer;
}

::flatbuffers::Offset<::flatbuffers::Vector<
    flatbuffers::Offset<android_xr::schemas::IndexBuffer>>>
SplitEngineMeshBuilder::SerializeIndexBuffers(
    flatbuffers::FlatBufferBuilder& builder) noexcept {
  std::vector<flatbuffers::Offset<android_xr::schemas::IndexBuffer>>
      index_buffers;
  for (std::unique_ptr<imp::split_engine::SplitEngineIndexBufferBuilder>&
           index_buffer : index_buffers_) {
    flatbuffers::Offset<android_xr::schemas::IndexBuffer> offset =
        index_buffer->SerializeIndexBuffer(builder);
    index_buffers.push_back(offset);
  }
  return builder.CreateVector(index_buffers);
}

flatbuffers::Offset<android_xr::schemas::IndexBuffer>
SplitEngineIndexBufferBuilder::SerializeIndexBuffer(
    flatbuffers::FlatBufferBuilder& builder) noexcept {
  android_xr::schemas::IndexType schema_index_type;
  switch (index_type_) {
    default:
    case filament::IndexBuffer::IndexType::USHORT:
      schema_index_type = android_xr::schemas::IndexType::USHORT;
      break;
    case filament::IndexBuffer::IndexType::UINT:
      schema_index_type = android_xr::schemas::IndexType::UINT;
      break;
  }

  flatbuffers::Offset<android_xr::schemas::IndexBufferInfo> serialized_ibinfo =
      android_xr::schemas::CreateIndexBufferInfo(
          builder, schema_index_type,
          builder.CreateVector(reinterpret_cast<uint8_t*>(buffer_.buffer),
                               buffer_.size));

  return android_xr::schemas::CreateIndexBuffer(
      builder, SplitEngineSerializer::GetId(index_buffer_), serialized_ibinfo,
      store_index_data_);
}

void SplitEngineIndexBufferBuilder::ContributeBufferSize(
    FlatbufferSizeCalculator& calculator) noexcept {
  calculator.AddVector(buffer_.size, sizeof(uint8_t));
  calculator.AddIndexBufferInfo();
  calculator.AddIndexBuffer();
}

SplitEngineMorphTargetBufferBuilder::
    SplitEngineMorphTargetBufferBuilder() noexcept {}

SplitEngineMorphTargetBufferBuilder&
SplitEngineMorphTargetBufferBuilder::VertexCount(size_t vertexCount) noexcept {
  vertex_count_ = vertexCount;
  return *this;
}

SplitEngineMorphTargetBufferBuilder& SplitEngineMorphTargetBufferBuilder::Count(
    size_t count) noexcept {
  attribute_count_ = count;
  return *this;
}

SplitEngineMorphTargetBufferBuilder&
SplitEngineMorphTargetBufferBuilder::PositionsAt(size_t target_index,
                                                 const float3* positions,
                                                 size_t count,
                                                 size_t offset) noexcept {
  const uint8_t* positions_data =
      reinterpret_cast<const uint8_t*>(positions + offset);
  const size_t positions_size = count * sizeof(float3);

  positions_.emplace_back(target_index, positions_data, positions_size);

  return *this;
}

SplitEngineMorphTargetBufferBuilder&
SplitEngineMorphTargetBufferBuilder::TangentsAt(size_t target_index,
                                                const short4* tangents,
                                                size_t count,
                                                size_t offset) noexcept {
  const uint8_t* tangents_data =
      reinterpret_cast<const uint8_t*>(tangents + offset);
  const size_t tangents_size = count * sizeof(short4);

  tangents_.emplace_back(target_index, tangents_data, tangents_size);

  return *this;
}

filament::MorphTargetBuffer*
SplitEngineMorphTargetBufferBuilder::Build() noexcept {
  // This should never be called.
  IMP_LOG(imp::FATAL)
      << "SplitEngineMorphTargetBufferBuilder::Build() is not implemented";
  return nullptr;
}

flatbuffers::Offset<android_xr::schemas::MorphTargetBuffer>
SplitEngineMorphTargetBufferBuilder::SerializeMorphTargetBuffer(
    flatbuffers::FlatBufferBuilder& builder) noexcept {
  std::vector<Attribute> attributes;
  attributes.resize(attribute_count_);

  for (AttributeData& position : positions_) {
    attributes[position.index].positions =
        builder.CreateVector(position.data, position.size);
  }
  for (AttributeData& tangent : tangents_) {
    attributes[tangent.index].tangents =
        builder.CreateVector(tangent.data, tangent.size);
  }

  std::vector<
      flatbuffers::Offset<android_xr::schemas::MorphTargetAttributeInfo>>
      attribute_offsets(attributes.size());
  absl::c_transform(
      attributes, attribute_offsets.data(),
      [&builder](const Attribute& attribute) {
        return android_xr::schemas::CreateMorphTargetAttributeInfo(
            builder, attribute.positions, attribute.tangents);
      });

  return android_xr::schemas::CreateMorphTargetBuffer(
      builder, SplitEngineSerializer::GetId(morph_target_buffer_),
      android_xr::schemas::CreateMorphTargetBufferInfo(
          builder, builder.CreateVector(attribute_offsets), vertex_count_));
}

void SplitEngineMorphTargetBufferBuilder::ContributeBufferSize(
    FlatbufferSizeCalculator& calculator) noexcept {
  for (size_t i = 0; i < attribute_count_; ++i) {
    calculator.AddVector(positions_[i].size, sizeof(uint8_t));
    calculator.AddVector(tangents_[i].size, sizeof(uint8_t));
    calculator.AddMorphTargetAttributeInfo();
  }
  calculator.AddReferenceVector(attribute_count_);
  calculator.AddMorphTargetBufferInfo();
  calculator.AddMorphTargetBuffer();
}

void SplitEngineMorphTargetBufferBuilder::Finalize(
    filament::MorphTargetBuffer* morph_target_buffer) noexcept {
  morph_target_buffer_ = morph_target_buffer;
}

SplitEngineMeshBuilder::SplitEngineMeshBuilder(
    SplitEngineSerializer& serializer, filament::Engine& engine) noexcept
    : serializer_(serializer), engine_(engine) {}

BaseVertexBufferBuilder&
SplitEngineMeshBuilder::CreateVertexBufferBuilder() noexcept {
  vertex_buffers_.push_back(
      absl::WrapUnique(new SplitEngineVertexBufferBuilder()));
  return *vertex_buffers_.back();
}
BaseIndexBufferBuilder&
SplitEngineMeshBuilder::CreateIndexBufferBuilder() noexcept {
  index_buffers_.push_back(
      absl::WrapUnique(new SplitEngineIndexBufferBuilder()));
  return *index_buffers_.back();
}
BaseMorphTargetBufferBuilder&
SplitEngineMeshBuilder::CreateMorphTargetBufferBuilder() noexcept {
  morph_target_buffers_.push_back(
      absl::WrapUnique(new SplitEngineMorphTargetBufferBuilder()));
  return *morph_target_buffers_.back();
}

flatbuffers::Offset<
    flatbuffers::Vector<flatbuffers::Offset<android_xr::schemas::VertexBuffer>>>
SplitEngineMeshBuilder::SerializeVertexBuffers(
    flatbuffers::FlatBufferBuilder& builder) noexcept {
  std::vector<flatbuffers::Offset<android_xr::schemas::VertexBuffer>>
      vertex_buffers;
  for (std::unique_ptr<imp::split_engine::SplitEngineVertexBufferBuilder>&
           vertex_buffer : vertex_buffers_) {
    flatbuffers::Offset<android_xr::schemas::VertexBuffer> offset =
        vertex_buffer->SerializeVertexBuffer(builder);
    vertex_buffers.push_back(offset);
  }
  return builder.CreateVector(vertex_buffers);
}

flatbuffers::Offset<flatbuffers::Vector<
    flatbuffers::Offset<android_xr::schemas::MorphTargetBuffer>>>
SplitEngineMeshBuilder::SerializeMorphTargetBuffers(
    flatbuffers::FlatBufferBuilder& builder) noexcept {
  std::vector<flatbuffers::Offset<android_xr::schemas::MorphTargetBuffer>>
      morph_target_buffers;
  for (std::unique_ptr<imp::split_engine::SplitEngineMorphTargetBufferBuilder>&
           morph_target_buffer : morph_target_buffers_) {
    flatbuffers::Offset<android_xr::schemas::MorphTargetBuffer> offset =
        morph_target_buffer->SerializeMorphTargetBuffer(builder);
    morph_target_buffers.push_back(offset);
  }
  return builder.CreateVector(morph_target_buffers);
}

void SplitEngineMeshBuilder::ContributeIndexBufferSizes(
    FlatbufferSizeCalculator& calculator) noexcept {
  for (std::unique_ptr<imp::split_engine::SplitEngineIndexBufferBuilder>&
           index_buffer : index_buffers_) {
    index_buffer->ContributeBufferSize(calculator);
  }
  calculator.AddReferenceVector(index_buffers_.size());
}

void SplitEngineMeshBuilder::ContributeVertexBufferSizes(
    FlatbufferSizeCalculator& calculator) noexcept {
  for (std::unique_ptr<imp::split_engine::SplitEngineVertexBufferBuilder>&
           vertex_buffer : vertex_buffers_) {
    vertex_buffer->ContributeBufferSize(calculator);
  }
}

void SplitEngineMeshBuilder::ContributeMorphTargetBufferSizes(
    FlatbufferSizeCalculator& calculator) noexcept {
  for (std::unique_ptr<imp::split_engine::SplitEngineMorphTargetBufferBuilder>&
           morph_target_buffer : morph_target_buffers_) {
    morph_target_buffer->ContributeBufferSize(calculator);
  }
}

void SplitEngineMeshBuilder::Finalize() noexcept {
  serializer_.SerializeMesh(*this);
}

}  // namespace imp::split_engine
