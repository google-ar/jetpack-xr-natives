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

#include "core/model/mesh/mesh_builder.h"

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/BufferDescriptor.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "core/common/typed_vector.h"
#include "core/math/vec.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"

namespace imp {

VertexBufferBuilder::VertexBufferBuilder(BaseVertexBufferBuilder* spy) noexcept
    : spy_(std::move(spy)),
      builder_(filament::VertexBuffer::Builder()),
      vertex_buffer_(nullptr) {}

VertexBufferBuilder& VertexBufferBuilder::BufferCount(
    uint8_t bufferCount) noexcept {
  assert(!vertex_buffer_);
  if (spy_) {
    spy_->BufferCount(bufferCount);
  }
  builder_ = builder_.bufferCount(bufferCount);
  return *this;
}

VertexBufferBuilder& VertexBufferBuilder::VertexCount(
    uint32_t vertexCount) noexcept {
  assert(!vertex_buffer_);
  if (spy_) {
    spy_->VertexCount(vertexCount);
  }
  builder_ = builder_.vertexCount(vertexCount);
  return *this;
}

VertexBufferBuilder& VertexBufferBuilder::EnableBufferObjectsInternal(
    bool enabled) noexcept {
  assert(!vertex_buffer_);
  if (spy_) {
    spy_->EnableBufferObjects(enabled);
  }
  builder_ = builder_.enableBufferObjects(enabled);
  return *this;
}

VertexBufferBuilder& VertexBufferBuilder::AdvancedSkinningInternal(
    bool enabled) noexcept {
  assert(!vertex_buffer_);
  if (spy_) {
    spy_->AdvancedSkinning(enabled);
  }
  builder_ = builder_.advancedSkinning(enabled);
  return *this;
}

VertexBufferBuilder& VertexBufferBuilder::VertexAccessFlags(
    uint8_t vertexAccessFlags) noexcept {
  if (spy_) {
    spy_->VertexAccessFlags(vertexAccessFlags);
  }
  return *this;
}

// TODO: this requires things being built in the right order, i.e.
// have to do all the Attributes for a buffer and then the buffer itself before
// moving to the next bufferIndex.
VertexBufferBuilder& VertexBufferBuilder::AttributeInternal(
    filament::VertexAttribute attribute, uint8_t bufferIndex,
    filament::VertexBuffer::AttributeType attributeType, uint32_t byteOffset,
    uint8_t byteStride, bool normalized) noexcept {
  assert(!vertex_buffer_);
  if (spy_) {
    spy_->Attribute(attribute, bufferIndex, attributeType, byteOffset,
                    byteStride, normalized);
  }
  builder_ = builder_.attribute(attribute, bufferIndex, attributeType,
                                byteOffset, byteStride);
  if (normalized) {
    builder_.normalized(attribute);
  }
  return *this;
}

VertexBufferBuilder& VertexBufferBuilder::BufferAtInternal(
    filament::Engine& engine, uint8_t bufferIndex,
    filament::backend::BufferDescriptor&& buffer,
    uint32_t byteOffset) noexcept {
  // TODO: VertexBufferInfo doesn't have an offset field.
  assert(byteOffset == 0);

  if (spy_) {
    spy_->BufferAt(engine, bufferIndex, std::move(buffer), byteOffset);
    return *this;
  }
  if (!vertex_buffer_) {
    vertex_buffer_ = builder_.build(engine);
  }
  vertex_buffer_->setBufferAt(engine, bufferIndex, std::move(buffer),
                              byteOffset);
  return *this;
}

VertexBufferBuilder& VertexBufferBuilder::Name(
    absl::string_view name) noexcept {
  if (spy_) {
    spy_->Name(name);
  }

  name_ = name;
  builder_ = builder_.name(name_.data(), name_.length());

  return *this;
}

filament::VertexBuffer* VertexBufferBuilder::Build(
    filament::Engine& engine) noexcept {
  if (!vertex_buffer_) {
    vertex_buffer_ = builder_.build(engine);
  }

  if (spy_) {
    spy_->Finalize(vertex_buffer_);
  }
  return vertex_buffer_;
}

void VertexBufferBuilder::Finalize(
    filament::VertexBuffer* vertex_buffer) noexcept {
  // Do nothing. This is designed to be overridden by
  // SplitEngineVertexBufferBuilder.
}

IndexBufferBuilder::IndexBufferBuilder(BaseIndexBufferBuilder* spy) noexcept
    : spy_(std::move(spy)),
      builder_(filament::IndexBuffer::Builder()),
      index_buffer_(nullptr) {}

IndexBufferBuilder& IndexBufferBuilder::IndexCount(
    uint32_t indexCount) noexcept {
  if (spy_) {
    spy_->IndexCount(indexCount);
  }
  builder_ = builder_.indexCount(indexCount);
  return *this;
}
IndexBufferBuilder& IndexBufferBuilder::BufferType(
    filament::IndexBuffer::IndexType indexType) noexcept {
  if (spy_) {
    spy_->BufferType(indexType);
  }
  builder_ = builder_.bufferType(indexType);
  return *this;
}
IndexBufferBuilder& IndexBufferBuilder::BufferInternal(
    filament::Engine& engine, filament::IndexBuffer::BufferDescriptor&& buffer,
    uint32_t byteOffset) noexcept {
  if (spy_) {
    spy_->Buffer(engine, std::move(buffer), byteOffset);
    return *this;
  }
  if (!index_buffer_) {
    index_buffer_ = builder_.build(engine);
  }
  index_buffer_->setBuffer(engine, std::move(buffer), byteOffset);
  return *this;
}

IndexBufferBuilder& IndexBufferBuilder::StoreIndexData(
    bool storeIndexData) noexcept {
  if (spy_) {
    spy_->StoreIndexData(storeIndexData);
  }
  return *this;
}

IndexBufferBuilder& IndexBufferBuilder::Name(absl::string_view name) noexcept {
  if (spy_) {
    spy_->Name(name);
  }

  name_ = name;
  builder_ = builder_.name(name_.data(), name_.length());

  return *this;
}

filament::IndexBuffer* IndexBufferBuilder::Build(
    filament::Engine& engine) noexcept {
  if (!index_buffer_) {
    index_buffer_ = builder_.build(engine);
  }

  if (spy_) {
    spy_->Finalize(index_buffer_);
  }
  return index_buffer_;
}

void IndexBufferBuilder::Finalize(
    filament::IndexBuffer* index_buffer) noexcept {
  // Do nothing. This is designed to be overridden by
  // SplitEngineIndexBufferBuilder.
}

MorphTargetBufferBuilder::MorphTargetBufferBuilder(
    filament::Engine* engine, BaseMorphTargetBufferBuilder* spy) noexcept
    : engine_(engine),
      spy_(std::move(spy)),
      builder_(filament::MorphTargetBuffer::Builder()),
      morph_target_buffer_(nullptr) {}

MorphTargetBufferBuilder& MorphTargetBufferBuilder::VertexCount(
    size_t vertexCount) noexcept {
  if (spy_) {
    spy_->VertexCount(vertexCount);
  }
  builder_ = builder_.vertexCount(vertexCount);
  return *this;
}

MorphTargetBufferBuilder& MorphTargetBufferBuilder::Count(
    size_t count) noexcept {
  if (spy_) {
    spy_->Count(count);
  }
  builder_ = builder_.count(count);
  return *this;
}

MorphTargetBufferBuilder& MorphTargetBufferBuilder::EnableCustomMorphing(
    bool enabled) noexcept {
  if (spy_) {
    spy_->EnableCustomMorphing(enabled);
  }
  builder_ = builder_.enableCustomMorphing(enabled);
  return *this;
}

MorphTargetBufferBuilder& MorphTargetBufferBuilder::PositionsAt(
    size_t target_index, const float3* positions, size_t count,
    size_t offset) noexcept {
  if (spy_) {
    spy_->PositionsAt(target_index, positions, count, offset);
  }

  if (!morph_target_buffer_) {
    morph_target_buffer_ = builder_.build(*engine_);
  }

  morph_target_buffer_->setPositionsAt(*engine_, target_index, positions, count,
                                       offset);
  return *this;
}
MorphTargetBufferBuilder& MorphTargetBufferBuilder::TangentsAt(
    size_t target_index, const short4* tangents, size_t count,
    size_t offset) noexcept {
  if (spy_) {
    spy_->TangentsAt(target_index, tangents, count, offset);
  }

  if (!morph_target_buffer_) {
    morph_target_buffer_ = builder_.build(*engine_);
  }

  morph_target_buffer_->setTangentsAt(*engine_, target_index, tangents, count,
                                      offset);
  return *this;
}

filament::MorphTargetBuffer* MorphTargetBufferBuilder::Build() noexcept {
  if (!morph_target_buffer_) {
    morph_target_buffer_ = builder_.build(*engine_);
  }

  if (spy_) {
    spy_->Finalize(morph_target_buffer_);
  }
  return morph_target_buffer_;
}

void MorphTargetBufferBuilder::Finalize(
    filament::MorphTargetBuffer* morph_target_buffer) noexcept {
  // Do nothing. This is designed to be overridden by
  // SplitEngineMorphTargetBufferBuilder.
}

MeshBuilder::MeshBuilder(BaseView& view) noexcept : view_(view), spy_(nullptr) {
  if (split_engine::SplitEngineSerializer* serializer =
          view_.GetSplitEngineSerializer()) {
    spy_ = serializer->CreateMeshBuilder();
  }
}

void MeshBuilder::Build(
    TypedVector<filament::VertexBuffer*>* out_vertex_buffers,
    TypedVector<filament::IndexBuffer*>* out_index_buffers,
    TypedVector<filament::MorphTargetBuffer*>*
        out_morph_target_buffers) noexcept {
  bool has_failure = false;

  if (out_vertex_buffers != nullptr) {
    for (auto& vertex_buffer : vertex_buffers_) {
      filament::VertexBuffer* built_vertex_buffer =
          vertex_buffer->Build(*view_.GetSharedEngine());
      if (!built_vertex_buffer) {
        IMP_LOG(imp::ERROR) << "Failed to build vertex buffer.";
        has_failure = true;
        break;
      }
      out_vertex_buffers->push_back(built_vertex_buffer);
    }
  }

  if (out_index_buffers != nullptr && !has_failure) {
    for (auto& index_buffer : index_buffers_) {
      filament::IndexBuffer* built_index_buffer =
          index_buffer->Build(*view_.GetSharedEngine());
      if (!built_index_buffer) {
        IMP_LOG(imp::ERROR) << "Failed to build vertex buffer.";
        has_failure = true;
        break;
      }
      out_index_buffers->push_back(built_index_buffer);
    }
  }

  if (out_morph_target_buffers != nullptr && !has_failure) {
    for (auto& morph_target_buffer : morph_target_buffers_) {
      filament::MorphTargetBuffer* built_morph_target_buffer =
          morph_target_buffer->Build();
      if (!built_morph_target_buffer) {
        IMP_LOG(imp::ERROR) << "Failed to build vertex buffer.";
        has_failure = true;
        break;
      }
      out_morph_target_buffers->push_back(built_morph_target_buffer);
    }
  }

  if (has_failure) {
    // Clean up any buffers that were built.
    filament::Engine* engine = view_.GetSharedEngine();

    if (out_vertex_buffers != nullptr) {
      for (auto& vertex_buffer : *out_vertex_buffers) {
        engine->destroy(vertex_buffer);
      }
      out_vertex_buffers->clear();
    }

    if (out_index_buffers != nullptr) {
      for (auto& index_buffer : *out_index_buffers) {
        engine->destroy(index_buffer);
      }
      out_index_buffers->clear();
    }

    if (out_morph_target_buffers != nullptr) {
      for (auto& morph_target_buffer : *out_morph_target_buffers) {
        engine->destroy(morph_target_buffer);
      }
      out_morph_target_buffers->clear();
    }
  }

  if (spy_) {
    spy_->Finalize();
  }
}

void MeshBuilder::Build(
    TypedVector<filament::VertexBuffer*>* out_vertex_buffers) noexcept {
  Build(out_vertex_buffers, nullptr, nullptr);
}

void MeshBuilder::Build(
    TypedVector<filament::IndexBuffer*>* out_index_buffers) noexcept {
  Build(nullptr, out_index_buffers, nullptr);
}

void MeshBuilder::Build(TypedVector<filament::MorphTargetBuffer*>*
                            out_morph_target_buffers) noexcept {
  Build(nullptr, nullptr, out_morph_target_buffers);
}

void MeshBuilder::Build(
    TypedVector<filament::VertexBuffer*>* out_vertex_buffers,
    TypedVector<filament::IndexBuffer*>* out_index_buffers) noexcept {
  Build(out_vertex_buffers, out_index_buffers, nullptr);
}
BaseVertexBufferBuilder& MeshBuilder::CreateVertexBufferBuilder() noexcept {
  if (spy_) {
    BaseVertexBufferBuilder& builder_spy = spy_->CreateVertexBufferBuilder();
    vertex_buffers_.push_back(
        absl::WrapUnique(new VertexBufferBuilder(&builder_spy)));
    return *vertex_buffers_.back();
  }
  vertex_buffers_.push_back(absl::WrapUnique(new VertexBufferBuilder()));
  return *vertex_buffers_.back();
}

BaseIndexBufferBuilder& MeshBuilder::CreateIndexBufferBuilder() noexcept {
  if (spy_) {
    BaseIndexBufferBuilder& builder_spy = spy_->CreateIndexBufferBuilder();
    index_buffers_.push_back(
        absl::WrapUnique(new IndexBufferBuilder(&builder_spy)));
    return *index_buffers_.back();
  }
  index_buffers_.push_back(absl::WrapUnique(new IndexBufferBuilder()));
  return *index_buffers_.back();
}

BaseMorphTargetBufferBuilder&
MeshBuilder::CreateMorphTargetBufferBuilder() noexcept {
  if (spy_) {
    BaseMorphTargetBufferBuilder& builder_spy =
        spy_->CreateMorphTargetBufferBuilder();
    morph_target_buffers_.push_back(absl::WrapUnique(
        new MorphTargetBufferBuilder(view_.GetSharedEngine(), &builder_spy)));
    return *morph_target_buffers_.back();
  }
  morph_target_buffers_.push_back(
      absl::WrapUnique(new MorphTargetBufferBuilder(view_.GetSharedEngine())));
  return *morph_target_buffers_.back();
}

void MeshBuilder::Finalize() noexcept {
  // Do nothing. This is designed to be overridden by SplitEngineMeshBuilder.
}

}  // namespace imp
