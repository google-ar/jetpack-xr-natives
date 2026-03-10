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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_EXTERNAL_TEXTURE_COLOR_SPACE_STORE_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_EXTERNAL_TEXTURE_COLOR_SPACE_STORE_H_

#include <functional>
#include <utility>

#include "absl/container/flat_hash_map.h"
#include "absl/status/statusor.h"
#include "core/media/media_color_space.h"
#include "core/split_engine/shared/split_engine_defines.h"

namespace imp::split_engine {

// Stores the color space of textures for a given bridge id and texture id.
class SplitEngineExternalTextureColorSpaceStore {
 public:
  // Returns the color space of the texture for a given bridge id and texture
  // id. Returns a not found error if the color space is not found.
  absl::StatusOr<MediaColorSpace> GetTextureColorSpace(
      BridgeId bridge_id, TextureId texture_id) const;

  // Sets the color space of the texture for a given bridge id and texture id.
  void SetTextureColorSpace(
      BridgeId bridge_id, TextureId texture_id,
      std::function<MediaColorSpace()> get_source_color_space_fn);

  // Removes the color space of the texture for a given bridge id and texture
  // id.
  void RemoveTextureColorSpace(BridgeId bridge_id, TextureId texture_id);

 private:
  absl::flat_hash_map<std::pair<BridgeId, TextureId>,
                      std::function<MediaColorSpace()>>
      color_spaces_;
};
}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_SPLIT_ENGINE_EXTERNAL_TEXTURE_COLOR_SPACE_STORE_H_
