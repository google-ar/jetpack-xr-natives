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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_TEXTURE_PARAMETER_CREATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_TEXTURE_PARAMETER_CREATOR_H_

#include <cstdint>
#include <optional>
#include <vector>

#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/small_source_location.h"
#include "core/render/texture.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// Helper class for creating BuiltInTextureParameters. This is introduced to
// support both local and remote mode of split engine.
class BuiltInTextureParameterCreator {
 public:
  explicit BuiltInTextureParameterCreator(bool local_mode)
      : local_mode_(local_mode) {}
  // Creates a BuiltInTextureParameter for the given texture. In local mode,
  // the texture is stored in this class and allow borrowing via texture id.
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter> Create(
      flatbuffers::FlatBufferBuilder& fbb, imp::BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> texture_sampler = std::nullopt);

  // Borrows the texture with the given id. This is only used in local mode.
  BorrowedTexturePtr Borrow(
      uint64_t texture_id,
      SmallSourceLocation loc = SmallSourceLocation::Current()) const;

 private:
  std::vector<BorrowedTexturePtr> borrowed_textures_;
  bool local_mode_;
};

};  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_TEXTURE_PARAMETER_CREATOR_H_
