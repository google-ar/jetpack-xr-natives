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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_BUILDER_BASE_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_BUILDER_BASE_H_

#include <cstddef>
#include <cstdint>
#include <utility>

#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "core/math/vec.h"

namespace imp {

class BaseVertexBufferBuilder {
 public:
  virtual ~BaseVertexBufferBuilder() = default;
  virtual BaseVertexBufferBuilder& BufferCount(
      uint8_t bufferCount) noexcept = 0;
  virtual BaseVertexBufferBuilder& VertexCount(
      uint32_t vertexCount) noexcept = 0;
  BaseVertexBufferBuilder& EnableBufferObjects(bool enabled) noexcept {
    return EnableBufferObjectsInternal(enabled);
  }
  BaseVertexBufferBuilder& AdvancedSkinning(bool enabled) noexcept {
    return AdvancedSkinningInternal(enabled);
  }
  BaseVertexBufferBuilder& Attribute(
      filament::VertexAttribute attribute, uint8_t bufferIndex,
      filament::VertexBuffer::AttributeType attributeType,
      uint32_t byteOffset = 0, uint8_t byteStride = 0,
      bool normalized = false) noexcept {
    return AttributeInternal(attribute, bufferIndex, attributeType, byteOffset,
                             byteStride, normalized);
  }
  BaseVertexBufferBuilder& BufferAt(
      filament::Engine& engine, uint8_t bufferIndex,
      filament::VertexBuffer::BufferDescriptor&& buffer,
      uint32_t byteOffset = 0) noexcept {
    return BufferAtInternal(engine, bufferIndex, std::move(buffer), byteOffset);
  }
  virtual BaseVertexBufferBuilder& VertexAccessFlags(
      uint8_t vertexAccessFlags) noexcept = 0;
  virtual BaseVertexBufferBuilder& Name(absl::string_view name) noexcept = 0;
  virtual void Finalize(filament::VertexBuffer* vertex_buffer) noexcept = 0;

 protected:
  virtual filament::VertexBuffer* Build(filament::Engine& engine) noexcept = 0;
  virtual BaseVertexBufferBuilder& EnableBufferObjectsInternal(
      bool enabled) noexcept = 0;
  virtual BaseVertexBufferBuilder& AdvancedSkinningInternal(
      bool enabled) noexcept = 0;
  virtual BaseVertexBufferBuilder& AttributeInternal(
      filament::VertexAttribute attribute, uint8_t bufferIndex,
      filament::VertexBuffer::AttributeType attributeType, uint32_t byteOffset,
      uint8_t byteStride, bool normalized) noexcept = 0;
  virtual BaseVertexBufferBuilder& BufferAtInternal(
      filament::Engine& engine, uint8_t bufferIndex,
      filament::VertexBuffer::BufferDescriptor&& buffer,
      uint32_t byteOffset) noexcept = 0;
};

class BaseIndexBufferBuilder {
 public:
  virtual ~BaseIndexBufferBuilder() = default;
  virtual BaseIndexBufferBuilder& IndexCount(uint32_t indexCount) noexcept = 0;
  virtual BaseIndexBufferBuilder& BufferType(
      filament::IndexBuffer::IndexType indexType) noexcept = 0;
  BaseIndexBufferBuilder& Buffer(
      filament::Engine& engine,
      filament::IndexBuffer::BufferDescriptor&& buffer,
      uint32_t byteOffset = 0) noexcept {
    return BufferInternal(engine, std::move(buffer), byteOffset);
  }
  virtual BaseIndexBufferBuilder& StoreIndexData(
      bool storeIndexData) noexcept = 0;
  virtual BaseIndexBufferBuilder& Name(absl::string_view name) noexcept = 0;
  virtual void Finalize(filament::IndexBuffer* index_buffer) noexcept = 0;

 protected:
  virtual filament::IndexBuffer* Build(filament::Engine& engine) noexcept = 0;
  virtual BaseIndexBufferBuilder& BufferInternal(
      filament::Engine& engine,
      filament::IndexBuffer::BufferDescriptor&& buffer,
      uint32_t byteOffset) noexcept = 0;
};

class BaseMorphTargetBufferBuilder {
 public:
  virtual ~BaseMorphTargetBufferBuilder() = default;

  virtual BaseMorphTargetBufferBuilder& VertexCount(
      size_t vertexCount) noexcept = 0;
  virtual BaseMorphTargetBufferBuilder& Count(size_t count) noexcept = 0;
  virtual BaseMorphTargetBufferBuilder& PositionsAt(
      size_t target_index, const float3* positions, size_t count,
      size_t offset = 0) noexcept = 0;
  virtual BaseMorphTargetBufferBuilder& TangentsAt(
      size_t target_index, const short4* tangents, size_t count,
      size_t offset = 0) noexcept = 0;
  virtual void Finalize(
      filament::MorphTargetBuffer* morph_target_buffer) noexcept = 0;

 protected:
  virtual filament::MorphTargetBuffer* Build() noexcept = 0;
};

class BaseMeshBuilder {
 public:
  virtual ~BaseMeshBuilder() = default;

  // Create*BufferBuilder: Create a new buffer builder. The builder will be
  // owned by the MeshBuilder. Access to the final mesh state is determined by
  // the derivied MeshBuilder type.
  virtual BaseVertexBufferBuilder& CreateVertexBufferBuilder() noexcept = 0;
  virtual BaseIndexBufferBuilder& CreateIndexBufferBuilder() noexcept = 0;
  virtual BaseMorphTargetBufferBuilder&
  CreateMorphTargetBufferBuilder() noexcept = 0;
  virtual void Finalize() noexcept = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_BUILDER_BASE_H_
