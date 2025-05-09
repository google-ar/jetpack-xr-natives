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

#ifndef THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_BUILDER_H_
#define THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_BUILDER_H_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Renderer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "core/common/typed_vector.h"
#include "core/math/vec.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/view/base_view.h"

namespace imp {

// Forward-declare SplitEngineMeshBuilder so it can construct private types.
namespace split_engine {
class SplitEngineMeshBuilder;
}  // namespace split_engine

class VertexBufferBuilder : public BaseVertexBufferBuilder {
 public:
  void Finalize(filament::VertexBuffer* vertex_buffer) noexcept override;
  VertexBufferBuilder& VertexAccessFlags(
      uint8_t vertexAccessFlags) noexcept override;

 protected:
  VertexBufferBuilder& EnableBufferObjectsInternal(
      bool enabled) noexcept override;
  VertexBufferBuilder& AdvancedSkinningInternal(bool enabled) noexcept override;
  VertexBufferBuilder& AttributeInternal(
      filament::VertexAttribute attribute, uint8_t bufferIndex,
      filament::VertexBuffer::AttributeType attributeType, uint32_t byteOffset,
      uint8_t byteStride, bool normalized) noexcept override;
  VertexBufferBuilder& BufferAtInternal(
      filament::Engine& engine, uint8_t bufferIndex,
      filament::VertexBuffer::BufferDescriptor&& buffer,
      uint32_t byteOffset) noexcept override;

 private:
  explicit VertexBufferBuilder(BaseVertexBufferBuilder* spy = nullptr) noexcept;

  VertexBufferBuilder(VertexBufferBuilder const& rhs) noexcept = delete;
  VertexBufferBuilder(VertexBufferBuilder&& rhs) noexcept = delete;
  VertexBufferBuilder& operator=(VertexBufferBuilder const& rhs) noexcept =
      delete;
  VertexBufferBuilder& operator=(VertexBufferBuilder&& rhs) noexcept = delete;

  VertexBufferBuilder& BufferCount(uint8_t bufferCount) noexcept override;
  VertexBufferBuilder& VertexCount(uint32_t vertexCount) noexcept override;
  VertexBufferBuilder& Name(absl::string_view name) noexcept override;
  filament::VertexBuffer* Build(filament::Engine& engine) noexcept override;

  friend class MeshBuilder;
  friend class split_engine::SplitEngineMeshBuilder;

  BaseVertexBufferBuilder* spy_;
  filament::VertexBuffer::Builder builder_;
  filament::VertexBuffer* vertex_buffer_;

  std::string name_;
};

class IndexBufferBuilder : public BaseIndexBufferBuilder {
 public:
  IndexBufferBuilder& IndexCount(uint32_t indexCount) noexcept override;
  IndexBufferBuilder& BufferType(
      filament::IndexBuffer::IndexType indexType) noexcept override;

  IndexBufferBuilder& StoreIndexData(bool storeIndexData) noexcept override;

  IndexBufferBuilder& Name(absl::string_view name) noexcept override;

  filament::IndexBuffer* Build(filament::Engine& engine) noexcept override;
  void Finalize(filament::IndexBuffer* index_buffer) noexcept override;

 protected:
  IndexBufferBuilder& BufferInternal(
      filament::Engine& engine,
      filament::IndexBuffer::BufferDescriptor&& buffer,
      uint32_t byteOffset) noexcept override;

 private:
  explicit IndexBufferBuilder(BaseIndexBufferBuilder* spy = nullptr) noexcept;

  IndexBufferBuilder(IndexBufferBuilder const& rhs) noexcept = delete;
  IndexBufferBuilder(IndexBufferBuilder&& rhs) noexcept = delete;
  IndexBufferBuilder& operator=(IndexBufferBuilder const& rhs) noexcept =
      delete;
  IndexBufferBuilder& operator=(IndexBufferBuilder&& rhs) noexcept = delete;

  friend class MeshBuilder;
  friend class split_engine::SplitEngineMeshBuilder;

  BaseIndexBufferBuilder* spy_;
  filament::IndexBuffer::Builder builder_;
  filament::IndexBuffer* index_buffer_;

  std::string name_;
};

class MorphTargetBufferBuilder : public BaseMorphTargetBufferBuilder {
 public:
  MorphTargetBufferBuilder& VertexCount(size_t vertexCount) noexcept override;
  MorphTargetBufferBuilder& Count(size_t count) noexcept override;
  MorphTargetBufferBuilder& PositionsAt(size_t target_index,
                                        const float3* positions, size_t count,
                                        size_t offset = 0) noexcept override;
  MorphTargetBufferBuilder& TangentsAt(size_t target_index,
                                       const short4* tangents, size_t count,
                                       size_t offset = 0) noexcept override;
  filament::MorphTargetBuffer* Build() noexcept override;
  void Finalize(
      filament::MorphTargetBuffer* morph_target_buffer) noexcept override;

 private:
  explicit MorphTargetBufferBuilder(
      filament::Engine* engine,
      BaseMorphTargetBufferBuilder* spy = nullptr) noexcept;
  MorphTargetBufferBuilder(MorphTargetBufferBuilder const& rhs) noexcept =
      delete;
  MorphTargetBufferBuilder(MorphTargetBufferBuilder&& rhs) noexcept = delete;
  MorphTargetBufferBuilder& operator=(
      MorphTargetBufferBuilder const& rhs) noexcept = delete;
  MorphTargetBufferBuilder& operator=(MorphTargetBufferBuilder&& rhs) noexcept =
      delete;

  friend class MeshBuilder;
  friend class split_engine::SplitEngineMeshBuilder;

  filament::Engine* engine_;
  BaseMorphTargetBufferBuilder* spy_;
  filament::MorphTargetBuffer::Builder builder_;
  filament::MorphTargetBuffer* morph_target_buffer_;
};

class MeshBuilder : public BaseMeshBuilder {
 public:
  explicit MeshBuilder(BaseView& view) noexcept;

  MeshBuilder(MeshBuilder const& rhs) noexcept = delete;
  MeshBuilder(MeshBuilder&& rhs) noexcept = delete;
  MeshBuilder& operator=(MeshBuilder const& rhs) noexcept = delete;
  MeshBuilder& operator=(MeshBuilder&& rhs) noexcept = delete;

  BaseVertexBufferBuilder& CreateVertexBufferBuilder() noexcept override;
  BaseIndexBufferBuilder& CreateIndexBufferBuilder() noexcept override;
  BaseMorphTargetBufferBuilder& CreateMorphTargetBufferBuilder() noexcept
      override;

  // Build and return buffers based on the current state of the MeshBuilder, as
  // generated via the *BufferBuilder operations.
  void Build(TypedVector<filament::VertexBuffer*>* out_vertex_buffers,
             TypedVector<filament::IndexBuffer*>* out_index_buffers,
             TypedVector<filament::MorphTargetBuffer*>*
                 out_morph_target_buffers) noexcept;
  void Build(TypedVector<filament::VertexBuffer*>* out_vertex_buffers) noexcept;
  void Build(TypedVector<filament::IndexBuffer*>* out_index_buffers) noexcept;
  void Build(TypedVector<filament::VertexBuffer*>* out_vertex_buffers,
             TypedVector<filament::IndexBuffer*>* out_index_buffers) noexcept;
  void Build(TypedVector<filament::MorphTargetBuffer*>*
                 out_morph_target_buffers) noexcept;
  void Finalize() noexcept override;

 private:
  BaseView& view_;
  std::unique_ptr<BaseMeshBuilder> spy_;

  std::vector<std::unique_ptr<VertexBufferBuilder>> vertex_buffers_;
  std::vector<std::unique_ptr<IndexBufferBuilder>> index_buffers_;
  std::vector<std::unique_ptr<MorphTargetBufferBuilder>> morph_target_buffers_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MODEL_MESH_MESH_BUILDER_H_
