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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_LOAD_IMAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_LOAD_IMAGE_H_

#include <functional>
#include <memory>
#include <variant>

#include "core/async/future.h"
#include "core/common/context.h"
#include "core/image/image_contents.h"
#include "core/loader/provider/schemas/loaded_model_generated.h"
#include "core/render/texture_asset.h"
#include "core/view/base_view.h"

namespace imp::loader::details {

// Returns the image contents from the data in image_info.
//
// Note that image_info must be kept alive until the image has finished
// decoding, and may possibly be required to stay alive until the ImageContents
// is no longer needed.
Future<std::variant<std::unique_ptr<image::ImageContents>,
                    std::unique_ptr<TextureAsset>>>
LoadImage(BaseView* view, const imp::Context& context,
          const schemas::TextureInfo* texture_info,
          schemas::ImageInfo image_info_type, const void* image_info,
          bool enable_use_texture_asset_api, std::function<void()> callback);

}  // namespace imp::loader::details

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_CREATOR_LOAD_IMAGE_H_
