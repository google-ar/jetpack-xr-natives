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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_ASSET_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_ASSET_H_

#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp::media {

// A data container for holding audio or video data and all relevant
// information for media playback.
class MediaAsset {
 public:
  static Future<std::unique_ptr<MediaAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future);

  explicit MediaAsset(imp::resources::Resource resource);

  MediaAsset(const MediaAsset&) = delete;
  MediaAsset& operator=(const MediaAsset&) = delete;

  MediaAsset(MediaAsset&&) = default;
  MediaAsset& operator=(MediaAsset&&) = default;

  // Returns the size, in bytes, of the audio file/data buffer attached
  // to this object.
  size_t GetSize() const;
  const uint8_t* GetData() const;

 private:
  // Holding onto the Resource object to ensure that the data is only freed if
  // MediaAsset is destroyed.
  imp::resources::Resource resource_;
};
}  // namespace imp::media

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_MEDIA_ASSET_H_
