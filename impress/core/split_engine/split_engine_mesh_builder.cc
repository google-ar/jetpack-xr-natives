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
#include "absl/log/check.h"
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
#include "core/split_engine/split_engine_mesh_serializer.h"
#include "core/split_engine/split_engine_serializer.h"
#include "split_engine/schemas/split_engine_data_generated.h"

namespace imp::split_engine {

namespace {
class SplitEngineMeshSerializerImpl : public SplitEngineMeshSerializer {
 public:
  struct Serializers {
    Serializers() = default;
    Serializers(Serializers&& other) noexcept = default;
    Serializers& operator=(Serializers&& other) noexcept = default;
    Serializers(const Serializers&) = delete;
    Serializers& operator=(const Serializers&) = delete;

    std::vector<std::unique_ptr<const SplitEngineIndexBufferSerializer>>
        index_buffers;
    std::vector<std::unique_ptr<const SplitEngineVertexBufferSerializer>>
        vertex_buffers;
    std::vector<std::unique_ptr<const SplitEngineMorphTargetBufferSerializer>>
        morph_target_buffers;
  };

  explicit SplitEngineMeshSerializerImpl(Serializers serializers)
      : serializers_(std::move(serializers)) {}
  ~SplitEngineMeshSerializerImpl() override = default;

  IndexBufferVector SerializeIndexBuffers(
      flatbuffers::FlatBufferBuilder& builder) const noexcept override {
    std::vector<flatbuffers::Offset<android_xr::schemas::IndexBuffer>>
        index_buffers;
    for (const std::unique_ptr<const SplitEngineIndexBufferSerializer>&
             serializer : serializers_.index_buffers) {
      flatbuffers::Offset<android_xr::schemas::IndexBuffer> offset =
          serializer->SerializeIndexBuffer(builder);
      index_buffers.push_back(offset);
    }
    return builder.CreateVector(index_buffers);
  }

  VertexBufferVector SerializeVertexBuffers(
      flatbuffers::FlatBufferBuilder& builder) const noexcept override {
    std::vector<flatbuffers::Offset<android_xr::schemas::VertexBuffer>>
        vertex_buffers;
    for (const std::unique_ptr<const SplitEngineVertexBufferSerializer>&
             serializer : serializers_.vertex_buffers) {
      flatbuffers::Offset<android_xr::schemas::VertexBuffer> offset =
          serializer->SerializeVertexBuffer(builder);
      vertex_buffers.push_back(offset);
    }
    return builder.CreateVector(vertex_buffers);
  }

  MorphTargetBufferVector SerializeMorphTargetBuffers(
      flatbuffers::FlatBufferBuilder& builder) const noexcept override {
    std::vector<flatbuffers::Offset<android_xr::schemas::MorphTargetBuffer>>
        morph_target_buffers;
    for (const std::unique_ptr<const SplitEngineMorphTargetBufferSerializer>&
             serializer : serializers_.morph_target_buffers) {
      flatbuffers::Offset<android_xr::schemas::MorphTargetBuffer> offset =
          serializer->SerializeMorphTargetBuffer(builder);
      morph_target_buffers.push_back(offset);
    }
    return builder.CreateVector(morph_target_buffers);
  }

  // Contribute*BufferSizes: given a FlatbufferSizeCalculator, add the
  // estimated contribution of the internal buffers to serialization size.
  void ContributeIndexBufferSizes(
      FlatbufferSizeCalculator& calculator) const noexcept override {
    for (const std::unique_ptr<const SplitEngineIndexBufferSerializer>&
             serializer : serializers_.index_buffers) {
      serializer->ContributeBufferSize(calculator);
    }
    calculator.AddReferenceVector(serializers_.index_buffers.size());
  }

  void ContributeVertexBufferSizes(
      FlatbufferSizeCalculator& calculator) const noexcept override {
    for (const std::unique_ptr<const SplitEngineVertexBufferSerializer>&
             serializer : serializers_.vertex_buffers) {
      serializer->ContributeBufferSize(calculator);
    }
  }

  void ContributeMorphTargetBufferSizes(
      FlatbufferSizeCalculator& calculator) const noexcept override {
    for (const std::unique_ptr<const SplitEngineMorphTargetBufferSerializer>&
             serializer : serializers_.morph_target_buffers) {
      serializer->ContributeBufferSize(calculator);
    }
  }

 private:
  const Serializers serializers_;
};

class SplitEngineVertexBufferSerializerImpl
    : public SplitEngineVertexBufferSerializer {
 public:
  explicit SplitEngineVertexBufferSerializerImpl(
      SplitEngineVertexBufferBuilder::State state)
      : state_(std::move(state)) {
    
  }
  ~SplitEngineVertexBufferSerializerImpl() override = default;

  void ContributeBufferSize(
      FlatbufferSizeCalculator& calculator) const noexcept override;

  flatbuffers::Offset<android_xr::schemas::VertexBuffer> SerializeVertexBuffer(
      flatbuffers::FlatBufferBuilder& builder) const noexcept override;

 private:
  const SplitEngineVertexBufferBuilder::State state_;
};

class SplitEngineIndexBufferSerializerImpl
    : public SplitEngineIndexBufferSerializer {
 public:
  explicit SplitEngineIndexBufferSerializerImpl(
      SplitEngineIndexBufferBuilder::State state)
      : state_(std::move(state)) {
    
  }
  ~SplitEngineIndexBufferSerializerImpl() override = default;

  void ContributeBufferSize(
      FlatbufferSizeCalculator& calculator) const noexcept override;

  flatbuffers::Offset<android_xr::schemas::IndexBuffer> SerializeIndexBuffer(
      flatbuffers::FlatBufferBuilder& builder) const noexcept override;

 private:
  const SplitEngineIndexBufferBuilder::State state_;
};

class SplitEngineMorphTargetBufferSerializerImpl
    : public SplitEngineMorphTargetBufferSerializer {
 public:
  explicit SplitEngineMorphTargetBufferSerializerImpl(
      SplitEngineMorphTargetBufferBuilder::State state)
      : state_(std::move(state)) {
    
  }
  ~SplitEngineMorphTargetBufferSerializerImpl() override = default;

  void ContributeBufferSize(
      FlatbufferSizeCalculator& calculator) const noexcept override;

  flatbuffers::Offset<android_xr::schemas::MorphTargetBuffer>
  SerializeMorphTargetBuffer(
      flatbuffers::FlatBufferBuilder& builder) const noexcept override;

 private:
  const SplitEngineMorphTargetBufferBuilder::State state_;
};

}  // namespace

SplitEngineVertexBufferBuilder::SplitEngineVertexBufferBuilder() noexcept {}

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
  
  state_.enable_buffer_objects_ = enabled;
  return *this;
}

SplitEngineVertexBufferBuilder&
SplitEngineVertexBufferBuilder::AdvancedSkinningInternal(
    bool enabled) noexcept {
  
  state_.advanced_skinning_ = enabled;
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
  
  if (bufferIndex >= state_.attributes_.size()) {
    // We require an in-order build.
    assert(bufferIndex == state_.attributes_.size());
    state_.attributes_.emplace_back();
    state_.strides_.emplace_back(byteStride);
  }
  // All strides must match.
  assert(state_.strides_[bufferIndex] == byteStride);

  state_.attributes_[bufferIndex].emplace_back(
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

  state_.buffer_descriptors_[bufferIndex] = std::move(buffer);
  return *this;
}

SplitEngineVertexBufferBuilder&
SplitEngineVertexBufferBuilder::VertexAccessFlags(
    uint8_t vertexAccessFlags) noexcept {
  
  state_.vertex_access_flags_ = vertexAccessFlags;

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

void SplitEngineVertexBufferSerializerImpl::ContributeBufferSize(
    FlatbufferSizeCalculator& calculator) const noexcept {
  for (const std::pair<const unsigned char,
                       filament::backend::BufferDescriptor>& buffer_descriptor :
       state_.buffer_descriptors_) {
    int bufferIndex = buffer_descriptor.first;
    calculator.AddAttributeVector(state_.attributes_[bufferIndex].size());
    calculator.AddVector(buffer_descriptor.second.size, sizeof(uint8_t));
    calculator.AddVertexBlockInfo();
  }
  calculator.AddReferenceVector(state_.buffer_descriptors_.size());
  calculator.AddVertexBufferInfo();
  calculator.AddVertexBuffer();
}

flatbuffers::Offset<android_xr::schemas::VertexBuffer>
SplitEngineVertexBufferSerializerImpl::SerializeVertexBuffer(
    flatbuffers::FlatBufferBuilder& builder) const noexcept {
  std::vector<flatbuffers::Offset<android_xr::schemas::VertexBlockInfo>>
      vertex_blocks;
  vertex_blocks.resize(state_.attributes_.size());

  for (const std::pair<const unsigned char,
                       filament::backend::BufferDescriptor>& buffer_descriptor :
       state_.buffer_descriptors_) {
    int bufferIndex = buffer_descriptor.first;
    const filament::backend::BufferDescriptor& buffer =
        buffer_descriptor.second;

    vertex_blocks[bufferIndex] = android_xr::schemas::CreateVertexBlockInfo(
        builder, builder.CreateVectorOfStructs(state_.attributes_[bufferIndex]),
        builder.CreateVector(reinterpret_cast<uint8_t*>(buffer.buffer),
                             buffer.size),
        state_.strides_[bufferIndex]);
  }

  return android_xr::schemas::CreateVertexBuffer(
      builder, state_.vertex_buffer_id,
      android_xr::schemas::CreateVertexBufferInfo(
          builder, builder.CreateVector(vertex_blocks), state_.vertex_count,
          state_.advanced_skinning_),
      static_cast<android_xr::schemas::VertexAccessFlags>(
          state_.vertex_access_flags_));
}

void SplitEngineVertexBufferBuilder::Finalize(
    filament::VertexBuffer* vertex_buffer) noexcept {
  
  state_.finalized = true;
  state_.vertex_buffer_id = SplitEngineSerializer::GetId(vertex_buffer);
  state_.vertex_count = vertex_buffer->getVertexCount();
}

SplitEngineIndexBufferBuilder::SplitEngineIndexBufferBuilder() noexcept {}

SplitEngineIndexBufferBuilder& SplitEngineIndexBufferBuilder::IndexCount(
    uint32_t indexCount) noexcept {
  
  return *this;
}
SplitEngineIndexBufferBuilder& SplitEngineIndexBufferBuilder::BufferType(
    filament::IndexBuffer::IndexType indexType) noexcept {
  
  state_.index_type_ = indexType;
  return *this;
}
SplitEngineIndexBufferBuilder& SplitEngineIndexBufferBuilder::BufferInternal(
    filament::Engine& engine, filament::IndexBuffer::BufferDescriptor&& buffer,
    uint32_t byteOffset) noexcept {
  
  state_.buffer_ = std::move(buffer);

  return *this;
}

SplitEngineIndexBufferBuilder& SplitEngineIndexBufferBuilder::StoreIndexData(
    bool store_index_data) noexcept {
  
  state_.store_index_data_ = store_index_data;
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
  
  state_.finalized = true;
  state_.index_buffer_id = SplitEngineSerializer::GetId(index_buffer);
  state_.index_count = index_buffer->getIndexCount();
}

flatbuffers::Offset<android_xr::schemas::IndexBuffer>
SplitEngineIndexBufferSerializerImpl::SerializeIndexBuffer(
    flatbuffers::FlatBufferBuilder& builder) const noexcept {
  android_xr::schemas::IndexType schema_index_type;
  switch (state_.index_type_) {
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
          builder.CreateVector(
              reinterpret_cast<uint8_t*>(state_.buffer_.buffer),
              state_.buffer_.size));

  return android_xr::schemas::CreateIndexBuffer(builder, state_.index_buffer_id,
                                                serialized_ibinfo,
                                                state_.store_index_data_);
}

void SplitEngineIndexBufferSerializerImpl::ContributeBufferSize(
    FlatbufferSizeCalculator& calculator) const noexcept {
  calculator.AddVector(state_.buffer_.size, sizeof(uint8_t));
  calculator.AddIndexBufferInfo();
  calculator.AddIndexBuffer();
}

SplitEngineMorphTargetBufferBuilder::
    SplitEngineMorphTargetBufferBuilder() noexcept {}

SplitEngineMorphTargetBufferBuilder&
SplitEngineMorphTargetBufferBuilder::VertexCount(size_t vertexCount) noexcept {
  
  state_.vertex_count_ = vertexCount;
  return *this;
}

SplitEngineMorphTargetBufferBuilder& SplitEngineMorphTargetBufferBuilder::Count(
    size_t count) noexcept {
  
  state_.attribute_count_ = count;
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

  state_.positions_.emplace_back(target_index, positions_data, positions_size);

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

  state_.tangents_.emplace_back(target_index, tangents_data, tangents_size);

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
SplitEngineMorphTargetBufferSerializerImpl::SerializeMorphTargetBuffer(
    flatbuffers::FlatBufferBuilder& builder) const noexcept {
  struct Attribute {
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> positions;
    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> tangents;
  };
  std::vector<Attribute> attributes;
  attributes.resize(state_.attribute_count_);

  using AttributeData = SplitEngineMorphTargetBufferBuilder::AttributeData;

  for (const AttributeData& position : state_.positions_) {
    attributes[position.index].positions =
        builder.CreateVector(position.data, position.size);
  }
  for (const AttributeData& tangent : state_.tangents_) {
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
      builder, state_.morph_target_buffer_id,
      android_xr::schemas::CreateMorphTargetBufferInfo(
          builder, builder.CreateVector(attribute_offsets),
          state_.vertex_count_));
}

void SplitEngineMorphTargetBufferSerializerImpl::ContributeBufferSize(
    FlatbufferSizeCalculator& calculator) const noexcept {
  for (size_t i = 0; i < state_.attribute_count_; ++i) {
    calculator.AddVector(state_.positions_[i].size, sizeof(uint8_t));
    calculator.AddVector(state_.tangents_[i].size, sizeof(uint8_t));
    calculator.AddMorphTargetAttributeInfo();
  }
  calculator.AddReferenceVector(state_.attribute_count_);
  calculator.AddMorphTargetBufferInfo();
  calculator.AddMorphTargetBuffer();
}

void SplitEngineMorphTargetBufferBuilder::Finalize(
    filament::MorphTargetBuffer* morph_target_buffer) noexcept {
  
  state_.finalized = true;
  state_.morph_target_buffer_id =
      SplitEngineSerializer::GetId(morph_target_buffer);
}

SplitEngineMeshBuilder::SplitEngineMeshBuilder(
    SplitEngineSerializer& serializer, filament::Engine& engine) noexcept
    : serializer_(serializer), engine_(engine) {}

BaseVertexBufferBuilder&
SplitEngineMeshBuilder::CreateVertexBufferBuilder() noexcept {
  
  vertex_buffers.push_back(
      absl::WrapUnique(new SplitEngineVertexBufferBuilder()));
  return *vertex_buffers.back();
}
BaseIndexBufferBuilder&
SplitEngineMeshBuilder::CreateIndexBufferBuilder() noexcept {
  
  index_buffers.push_back(
      absl::WrapUnique(new SplitEngineIndexBufferBuilder()));
  return *index_buffers.back();
}
BaseMorphTargetBufferBuilder&
SplitEngineMeshBuilder::CreateMorphTargetBufferBuilder() noexcept {
  
  morph_target_buffers.push_back(
      absl::WrapUnique(new SplitEngineMorphTargetBufferBuilder()));
  return *morph_target_buffers.back();
}

void SplitEngineMeshBuilder::Finalize() noexcept {
  
  is_finalized_ = true;

  SplitEngineMeshSerializerImpl::Serializers serializers;

  serializers.vertex_buffers.reserve(vertex_buffers.size());
  for (const auto& vertex_buffer : vertex_buffers) {
    serializers.vertex_buffers.push_back(vertex_buffer->CreateSerializer());
  }

  serializers.index_buffers.reserve(index_buffers.size());
  for (const auto& index_buffer : index_buffers) {
    serializers.index_buffers.push_back(index_buffer->CreateSerializer());
  }

  serializers.morph_target_buffers.reserve(morph_target_buffers.size());
  for (const auto& morph_target_buffer : morph_target_buffers) {
    serializers.morph_target_buffers.push_back(
        morph_target_buffer->CreateSerializer());
  }

  serializer_.SerializeMesh(
      std::make_unique<const SplitEngineMeshSerializerImpl>(
          std::move(serializers)));
}

std::unique_ptr<const SplitEngineVertexBufferSerializer>
SplitEngineVertexBufferBuilder::CreateSerializer() noexcept {
  
  return std::make_unique<const SplitEngineVertexBufferSerializerImpl>(
      std::move(state_));
}

std::unique_ptr<const SplitEngineIndexBufferSerializer>
SplitEngineIndexBufferBuilder::CreateSerializer() noexcept {
  
  return std::make_unique<const SplitEngineIndexBufferSerializerImpl>(
      std::move(state_));
}

std::unique_ptr<const SplitEngineMorphTargetBufferSerializer>
SplitEngineMorphTargetBufferBuilder::CreateSerializer() noexcept {
  
  return std::make_unique<const SplitEngineMorphTargetBufferSerializerImpl>(
      std::move(state_));
}

}  // namespace imp::split_engine
