/*
 * Copyright 2025 Google LLC
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
#include "core/split_engine/split_engine_external_texture_color_space_store.h"

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/media/media_color_space.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

absl::StatusOr<MediaColorSpace>
SplitEngineExternalTextureColorSpaceStore::GetTextureColorSpace(
    BridgeId bridge_id, TextureId texture_id) const {
  auto it = color_spaces_.find({bridge_id, texture_id});
  if (it != color_spaces_.end()) {
    return it->second;
  }
  return absl::NotFoundError("Color space not found for texture");
}

void SplitEngineExternalTextureColorSpaceStore::SetTextureColorSpace(
    BridgeId bridge_id, TextureId texture_id,
    const MediaColorSpace& color_space) {
  color_spaces_[{bridge_id, texture_id}] = color_space;
}

void SplitEngineExternalTextureColorSpaceStore::RemoveTextureColorSpace(
    BridgeId bridge_id, TextureId texture_id) {
  color_spaces_.erase({bridge_id, texture_id});
}

}  // namespace imp::split_engine
