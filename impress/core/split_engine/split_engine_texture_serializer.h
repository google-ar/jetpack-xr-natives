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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEXTURE_SERIALIZER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEXTURE_SERIALIZER_H_

#include <cstddef>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "split_engine/schemas/split_engine_data_generated.h"

namespace imp::split_engine {

// An interface for SplitEngine to use to serialize a texture to a provided
// flatbuffer builder. This separates the flatbuffer allocation and
// serialization logic.
class SplitEngineTextureSerializer {
 public:
  virtual ~SplitEngineTextureSerializer() = default;

  // Serializes underlying texture to the provided flatbuffer builder, and
  // returns the offset of the serialized texture in the buffer.
  virtual flatbuffers::Offset<android_xr::schemas::Texture> SerializeTexture(
      flatbuffers::FlatBufferBuilder& builder) const noexcept = 0;

  // Returns the size of the texture and all of its dependent data, including
  // image data, name, etc.
  virtual size_t GetSerializedSize() const noexcept = 0;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_TEXTURE_SERIALIZER_H_
