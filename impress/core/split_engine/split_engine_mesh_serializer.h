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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_SERIALIZER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_SERIALIZER_H_

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "flatbuffers/vector.h"
#include "core/split_engine/flatbuffer_size_calculator.h"
#include "split_engine/schemas/split_engine_data_generated.h"

namespace imp::split_engine {

class SplitEngineIndexBufferSerializer {
 public:
  virtual ~SplitEngineIndexBufferSerializer() = default;
  virtual void ContributeBufferSize(
      FlatbufferSizeCalculator& calculator) const noexcept = 0;

  virtual flatbuffers::Offset<android_xr::schemas::IndexBuffer>
  SerializeIndexBuffer(
      flatbuffers::FlatBufferBuilder& builder) const noexcept = 0;
};

class SplitEngineVertexBufferSerializer {
 public:
  virtual ~SplitEngineVertexBufferSerializer() = default;
  virtual void ContributeBufferSize(
      FlatbufferSizeCalculator& calculator) const noexcept = 0;

  virtual flatbuffers::Offset<android_xr::schemas::VertexBuffer>
  SerializeVertexBuffer(
      flatbuffers::FlatBufferBuilder& builder) const noexcept = 0;
};

class SplitEngineMorphTargetBufferSerializer {
 public:
  virtual ~SplitEngineMorphTargetBufferSerializer() = default;
  virtual void ContributeBufferSize(
      FlatbufferSizeCalculator& calculator) const noexcept = 0;

  virtual flatbuffers::Offset<android_xr::schemas::MorphTargetBuffer>
  SerializeMorphTargetBuffer(
      flatbuffers::FlatBufferBuilder& builder) const noexcept = 0;
};

// An interface for SplitEngine to use to serialize a mesh to a provided
// flatbuffer builder. This separates the flatbuffer allocation and
// serialization logic.
class SplitEngineMeshSerializer {
 public:
  virtual ~SplitEngineMeshSerializer() = default;

  using IndexBufferVector = flatbuffers::Offset<flatbuffers::Vector<
      flatbuffers::Offset<android_xr::schemas::IndexBuffer>>>;
  using VertexBufferVector = flatbuffers::Offset<flatbuffers::Vector<
      flatbuffers::Offset<android_xr::schemas::VertexBuffer>>>;
  using MorphTargetBufferVector = flatbuffers::Offset<flatbuffers::Vector<
      flatbuffers::Offset<android_xr::schemas::MorphTargetBuffer>>>;

  // Serialize*Buffers: serializes the internal buffers to the provided
  // flatbuffer builder, and returns the offsets to the serialized data.
  virtual IndexBufferVector SerializeIndexBuffers(
      flatbuffers::FlatBufferBuilder& builder) const noexcept = 0;
  virtual VertexBufferVector SerializeVertexBuffers(
      flatbuffers::FlatBufferBuilder& builder) const noexcept = 0;
  virtual MorphTargetBufferVector SerializeMorphTargetBuffers(
      flatbuffers::FlatBufferBuilder& builder) const noexcept = 0;

  // Contribute*BufferSizes: given a FlatbufferSizeCalculator, add the estimated
  // contribution of the internal buffers to serialization size.
  virtual void ContributeIndexBufferSizes(
      FlatbufferSizeCalculator& calculator) const noexcept = 0;
  virtual void ContributeVertexBufferSizes(
      FlatbufferSizeCalculator& calculator) const noexcept = 0;
  virtual void ContributeMorphTargetBufferSizes(
      FlatbufferSizeCalculator& calculator) const noexcept = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_MESH_SERIALIZER_H_
