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

#include "core/split_engine/materials/builtin_texture_parameter_creator.h"

#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/small_source_location.h"
#include "core/render/texture.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
BuiltInTextureParameterCreator::Create(
    flatbuffers::FlatBufferBuilder& fbb, imp::BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> texture_sampler) {
  std::optional<uint64_t> texture_id = std::nullopt;

  if (local_mode_) {
    // When local mode, we use the index as the texture id.
    texture_id = borrowed_textures_.size();
    borrowed_textures_.push_back(std::move(texture));
    return CreateBuiltInTextureParameter(fbb, borrowed_textures_.back(),
                                         texture_id, texture_sampler);
  }
  return CreateBuiltInTextureParameter(fbb, texture, texture_id,
                                       texture_sampler);
}

BorrowedTexturePtr BuiltInTextureParameterCreator::Borrow(
    uint64_t texture_id, SmallSourceLocation loc) const {
  if (texture_id >= borrowed_textures_.size()) {
    IMP_LOG(imp::ERROR) << "Texture not found: " << texture_id;
    return BorrowedTexturePtr();
  }
  return borrowed_textures_.at(texture_id).WithNewLocation(loc);
}

}  // namespace imp::split_engine
