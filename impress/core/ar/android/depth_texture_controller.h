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

#ifndef THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_DEPTH_TEXTURE_CONTROLLER_H_
#define THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_DEPTH_TEXTURE_CONTROLLER_H_

#include <optional>

#include "absl/container/flat_hash_map.h"
#include "absl/time/time.h"
#include "core/ar/android/ar_core_ptrs.h"
#include "core/render/texture.h"
#include "filament/filament/include/filament/Engine.h"

namespace imp {
namespace ar {

class DepthTextureController {
 public:
  // DepthTextureController should not outlive ar_core_adapter or engine.
  DepthTextureController(filament::Engine* engine,
                         TextureFactory* texture_factory);

  DepthTextureController(DepthTextureController const&) = delete;
  void operator=(DepthTextureController const&) = delete;

  void SetTextureCreatedHandler(std::function<void(imp::Texture const&)>);

  void Update(ArSession_* session, ArFrame_* frame);

 private:
  filament::Engine& engine_;
  TextureFactory* texture_factory_;
  std::function<void(imp::Texture const&)> texture_created_handler_;
  imp::OwnedTexturePtr texture_;
  absl::optional<int64_t> last_acquire_image_timestamp_ns_;
  absl::Time timeout_end_ = absl::InfinitePast();

  // Uses the image data in image to update the internal filament texture.
  void UpdateFromArImage(ArSession_* session, UniqueArImage image);
};

}  // namespace ar
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_AR_ANDROID_DEPTH_TEXTURE_CONTROLLER_H_
