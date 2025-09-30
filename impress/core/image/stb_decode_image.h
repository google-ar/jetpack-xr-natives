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

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_STB_DECODE_IMAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_STB_DECODE_IMAGE_H_

#include <memory>

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/image/image_contents.h"
#include "core/resources/resource_manager.h"

namespace imp::image::details {

// Universal image loading via stb.  Mobile platforms use system services, this
// method is for desktop/wasm/test platforms.
absl::StatusOr<std::unique_ptr<ImageContents>> StbDecodeImage(
    absl::string_view name, resources::Resource resource,
    bool fatal_on_pink_texture = false);

}  // namespace imp::image::details

#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_STB_DECODE_IMAGE_H_
