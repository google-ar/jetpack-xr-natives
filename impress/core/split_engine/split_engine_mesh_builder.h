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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_BUILDER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_BUILDER_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/BufferDescriptor.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "core/loader/loader_options.h"
#include "core/math/vec.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/split_engine/split_engine_mesh_serializer.h"
#include "core/split_engine/split_engine_serializer.h"
#include "split_engine/schemas/split_engine_data_generated.h"

namespace imp::split_engine {

// Builder for VertexBuffers that serializes the buffer to SplitEngine.
class SplitEngineVertexBufferBuilder : public BaseVertexBufferBuilder {
 public:
  struct State {
    State() = default;
    State(State&& other) noexcept = default;
    State& operator=(State&& other) noexcept = default;
    State(const State&) = delete;
    State& operator=(const State&) = delete;

    uint64_t vertex_buffer_id = 0;
    uint32_t vertex_count = 0;
    std::map<uint8_t, filament::backend::BufferDescriptor> buffer_descriptors_;

    bool enable_buffer_objects_ = false;
    bool advanced_skinning_ = false;
    uint8_t vertex_access_flags_ =
        loader::LoaderOptions::VertexAccessFlags::kDefault;
    std::vector<uint8_t> strides_;
    std::vector<std::vector<android_xr::schemas::VertexAttributeInfo>>
        attributes_;

    bool finalized = false;
  };

  SplitEngineVertexBufferBuilder& BufferCount(
      uint8_t bufferCount) noexcept override;
  SplitEngineVertexBufferBuilder& VertexCount(
      uint32_t vertexCount) noexcept override;

  SplitEngineVertexBufferBuilder& VertexAccessFlags(
      uint8_t vertexAccessFlags) noexcept override;

  SplitEngineVertexBufferBuilder& Name(
      absl::string_view name) noexcept override;

  void Finalize(filament::VertexBuffer* vertex_buffer) noexcept override;

  std::unique_ptr<const SplitEngineVertexBufferSerializer>
  CreateSerializer() noexcept;

 protected:
  SplitEngineVertexBufferBuilder& EnableBufferObjectsInternal(
      bool enabled) noexcept override;
  SplitEngineVertexBufferBuilder& AdvancedSkinningInternal(
      bool enabled) noexcept override;
  SplitEngineVertexBufferBuilder& AttributeInternal(
      filament::VertexAttribute attribute, uint8_t bufferIndex,
      filament::VertexBuffer::AttributeType attributeType, uint32_t byteOffset,
      uint8_t byteStride, bool normalized) noexcept override;
  SplitEngineVertexBufferBuilder& BufferAtInternal(
      filament::Engine& engine, uint8_t bufferIndex,
      filament::VertexBuffer::BufferDescriptor&& buffer,
      uint32_t byteOffset) noexcept override;

 private:
  SplitEngineVertexBufferBuilder() noexcept;
  SplitEngineVertexBufferBuilder(
      SplitEngineVertexBufferBuilder const& rhs) noexcept = delete;
  SplitEngineVertexBufferBuilder(
      SplitEngineVertexBufferBuilder&& rhs) noexcept = delete;
  SplitEngineVertexBufferBuilder& operator=(
      SplitEngineVertexBufferBuilder const& rhs) noexcept = delete;
  SplitEngineVertexBufferBuilder& operator=(
      SplitEngineVertexBufferBuilder&& rhs) noexcept = delete;

  friend class SplitEngineMeshBuilder;

  filament::VertexBuffer* Build(filament::Engine& engine) noexcept override;

  State state_;
};

// Builder for IndexBuffers that serializes the buffer to SplitEngine.
class SplitEngineIndexBufferBuilder : public BaseIndexBufferBuilder {
 public:
  struct State {
    State() = default;
    State(State&& other) noexcept = default;
    State& operator=(State&& other) noexcept = default;
    State(const State&) = delete;
    State& operator=(const State&) = delete;

    uint64_t index_buffer_id = 0;
    uint32_t index_count = 0;
    filament::IndexBuffer::IndexType index_type_;
    bool store_index_data_ = false;
    filament::IndexBuffer::BufferDescriptor buffer_;

    bool finalized = false;
  };

  SplitEngineIndexBufferBuilder& IndexCount(
      uint32_t indexCount) noexcept override;
  SplitEngineIndexBufferBuilder& BufferType(
      filament::IndexBuffer::IndexType indexType) noexcept override;

  SplitEngineIndexBufferBuilder& StoreIndexData(
      bool store_index_data) noexcept override;

  SplitEngineIndexBufferBuilder& Name(absl::string_view name) noexcept override;

  void Finalize(filament::IndexBuffer* index_buffer) noexcept override;

  std::unique_ptr<const SplitEngineIndexBufferSerializer>
  CreateSerializer() noexcept;

 protected:
  SplitEngineIndexBufferBuilder& BufferInternal(
      filament::Engine& engine,
      filament::IndexBuffer::BufferDescriptor&& buffer,
      uint32_t byteOffset) noexcept override;

 private:
  SplitEngineIndexBufferBuilder() noexcept;

  SplitEngineIndexBufferBuilder(
      SplitEngineIndexBufferBuilder const& rhs) noexcept = delete;
  SplitEngineIndexBufferBuilder(SplitEngineIndexBufferBuilder&& rhs) noexcept =
      delete;
  SplitEngineIndexBufferBuilder& operator=(
      SplitEngineIndexBufferBuilder const& rhs) noexcept = delete;
  SplitEngineIndexBufferBuilder& operator=(
      SplitEngineIndexBufferBuilder&& rhs) noexcept = delete;

  friend class SplitEngineMeshBuilder;

  filament::IndexBuffer* Build(filament::Engine& engine) noexcept override;

  State state_;
};

class SplitEngineMorphTargetBufferBuilder
    : public BaseMorphTargetBufferBuilder {
 public:
  struct AttributeData {
    AttributeData(const size_t index, const uint8_t* data, const size_t size)
        : index(index), data(data), size(size) {}

    const size_t index;
    const uint8_t* data;
    const size_t size;
  };

  struct State {
    State() = default;
    State(State&& other) noexcept = default;
    State& operator=(State&& other) noexcept = default;
    State(const State&) = delete;
    State& operator=(const State&) = delete;

    uint64_t morph_target_buffer_id = 0;
    size_t attribute_count_ = 0;
    size_t vertex_count_ = 0;
    std::vector<AttributeData> positions_;
    std::vector<AttributeData> tangents_;

    bool enable_custom_morphing_ = false;
    bool finalized = false;
  };

  SplitEngineMorphTargetBufferBuilder& VertexCount(
      size_t vertexCount) noexcept override;
  SplitEngineMorphTargetBufferBuilder& Count(size_t count) noexcept override;
  SplitEngineMorphTargetBufferBuilder& EnableCustomMorphing(
      bool enabled) noexcept override;
  SplitEngineMorphTargetBufferBuilder& PositionsAt(
      size_t target_index, const float3* positions, size_t count,
      size_t offset = 0) noexcept override;
  SplitEngineMorphTargetBufferBuilder& TangentsAt(
      size_t target_index, const short4* tangents, size_t count,
      size_t offset = 0) noexcept override;

  void Finalize(
      filament::MorphTargetBuffer* morph_target_buffer) noexcept override;

  std::unique_ptr<const SplitEngineMorphTargetBufferSerializer>
  CreateSerializer() noexcept;

 private:
  SplitEngineMorphTargetBufferBuilder() noexcept;

  SplitEngineMorphTargetBufferBuilder(
      SplitEngineMorphTargetBufferBuilder const& rhs) noexcept = delete;
  SplitEngineMorphTargetBufferBuilder(
      SplitEngineMorphTargetBufferBuilder&& rhs) noexcept = delete;
  SplitEngineMorphTargetBufferBuilder& operator=(
      SplitEngineMorphTargetBufferBuilder const& rhs) noexcept = delete;
  SplitEngineMorphTargetBufferBuilder& operator=(
      SplitEngineMorphTargetBufferBuilder&& rhs) noexcept = delete;

  friend class SplitEngineMeshBuilder;

  filament::MorphTargetBuffer* Build() noexcept override;

  State state_;
};

class SplitEngineMeshBuilder : public BaseMeshBuilder {
 public:
  explicit SplitEngineMeshBuilder(SplitEngineSerializer& serializer,
                                  filament::Engine& engine) noexcept;

  SplitEngineMeshBuilder(SplitEngineMeshBuilder const& rhs) noexcept = delete;
  SplitEngineMeshBuilder(SplitEngineMeshBuilder&& rhs) noexcept = delete;
  SplitEngineMeshBuilder& operator=(
      SplitEngineMeshBuilder const& rhs) noexcept = delete;
  SplitEngineMeshBuilder& operator=(SplitEngineMeshBuilder&& rhs) noexcept =
      delete;

  BaseVertexBufferBuilder& CreateVertexBufferBuilder() noexcept override;
  BaseIndexBufferBuilder& CreateIndexBufferBuilder() noexcept override;
  BaseMorphTargetBufferBuilder& CreateMorphTargetBufferBuilder() noexcept
      override;

  void Finalize() noexcept override;

 private:
  SplitEngineSerializer& serializer_;
  filament::Engine& engine_;

  std::vector<std::unique_ptr<SplitEngineVertexBufferBuilder>> vertex_buffers;
  std::vector<std::unique_ptr<SplitEngineIndexBufferBuilder>> index_buffers;
  std::vector<std::unique_ptr<SplitEngineMorphTargetBufferBuilder>>
      morph_target_buffers;

  bool is_finalized_ = false;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_BUILDER_H_
