// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/render/image_asset.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/PixelBufferDescriptor.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/context.h"
#include "core/image/decode_image.h"
#include "core/image/image_contents.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp {

Future<std::unique_ptr<ImageAsset>> ImageAsset::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future) {
  return resource_future.Then(
      [context = view->GetContext(), asset_url_copy = std::string{asset_url}](
          resources::Resource resource) -> Future<std::unique_ptr<ImageAsset>> {
        return image::DecodeImage(context, asset_url_copy, resource)
            .Then([context, asset_url_copy,
                   resource](std::unique_ptr<image::ImageContents> image)
                      -> std::unique_ptr<ImageAsset> {
              return std::make_unique<ImageAsset>(context, asset_url_copy,
                                                  std::move(resource),
                                                  std::move(image));
            });
      },
      Executor::Type::kBackground);
}

ImageAsset::ImageAsset(const Context& context, absl::string_view image_name,
                       resources::Resource encoded,
                       std::unique_ptr<image::ImageContents> content)
    : context_(&context),
      image_name_(image_name),
      encoded_resource_(std::move(encoded)),
      image_content_(content.release()) {}

image::ImageContents& ImageAsset::GetImageContents() const {
  return *image_content_;
}

std::vector<filament::backend::PixelBufferDescriptor>
ImageAsset::GetLevelDescriptors() const {
  std::vector<filament::backend::PixelBufferDescriptor> descriptor =
      image_content_->CreatePixelBufferDescriptorLevels(
          /* callback=*/{}, /*is_r11_g11_b10=*/false);
  return descriptor;
}

}  // namespace imp
