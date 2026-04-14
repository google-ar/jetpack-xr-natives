/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_THUMBNAIL_GENERATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_THUMBNAIL_GENERATOR_H_

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"

namespace imp::editor {

// Interface for generating thumbnails for assets.
class AssetThumbnailGenerator {
 public:
  virtual ~AssetThumbnailGenerator() = default;

  // returns true if the generator supports generating a thumbnail for the
  // resource.
  virtual bool SupportsGeneration(absl::string_view resource) const = 0;

  // Requests generation of a thumbnail for the resource. Later, use
  // GetThumbnail(...) to check if the thumbnail is ready and fetch it.
  virtual void GenerateThumbnail(absl::string_view resource) = 0;

  // Tries to get the thumbnail. Returns status and optionally the texture. The
  // status is okay if the thumbnail is being loaded or is ready.
  virtual absl::StatusOr<filament::Texture*> GetThumbnail(
      absl::string_view resource) = 0;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_WIDGETS_ASSET_THUMBNAIL_GENERATOR_H_
