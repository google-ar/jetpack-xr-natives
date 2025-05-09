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

#ifndef THIRD_PARTY_IMPRESS_CORE_RENDER_IMAGE_ASSET_H_
#define THIRD_PARTY_IMPRESS_CORE_RENDER_IMAGE_ASSET_H_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/image/image_contents.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp {

class ImageAsset {
 public:
  static Future<std::unique_ptr<ImageAsset>> Load(
      BaseView* view, absl::string_view asset_url,
      Future<resources::Resource> resource_future);

  ImageAsset(const Context& context, absl::string_view image_name,
             resources::Resource encoded,
             std::unique_ptr<image::ImageContents> content);

  absl::string_view GetName() const { return image_name_; }
  bool HasAlpha() const { return image_content_->HasAlpha(); }
  uint32_t GetWidth() const { return image_content_->GetWidth(); }
  uint32_t GetHeight() const { return image_content_->GetHeight(); }

  // Moves images into a new array of PixelBufferDescriptor's.
  std::vector<filament::backend::PixelBufferDescriptor> GetLevelDescriptors()
      const;
  filament::backend::TextureFormat GetTextureFormat() const {
    return GetImageContents().GetTextureFormat();
  }

  // TODO: this should return a const ImageContents& because it is
  // unsafe to move data out of the contents without resetting image_content_.
  // "Shared constant data (i.e. shared_ptr<const T>) is generally safer than
  // shared mutable data, since making copies of a shared_ptr<const T> is
  // essentially equivalent to making deep copies of the underlying T, only more
  // efficient. Consequently, shared pointers to constant data are permitted as
  // an alternative to making expensive deep copies of the underlying data."
  // There is a bug here, though. It seems like GetImageContents should be
  // private or should return a const ImageContents. It's not necessarily bad
  // for an Asset to internally have mutable state internally as long as from a
  // "user" perspective it's not mutating, which this method violates.
  image::ImageContents& GetImageContents() const;

 private:
  const Context* context_;
  std::string image_name_;
  resources::Resource encoded_resource_;
  mutable std::unique_ptr<image::ImageContents> image_content_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RENDER_IMAGE_ASSET_H_
