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

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_WASM_DECODE_IMAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_WASM_DECODE_IMAGE_H_

#include <memory>

#include "absl/strings/string_view.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "core/async/future.h"
#include "core/image/image_contents.h"
#include "core/image/wasm_texture_contents.h"
#include "core/resources/resource_manager.h"

namespace imp::image::details {

// Decodes an image using available web apis.
Future<std::unique_ptr<ImageContents>> WasmDecodeImage(
    absl::string_view name, resources::Resource resource);

// Decodes an image to a texture using available web apis.
Future<WasmTextureContents> WasmDecodeImageToTexture(
    absl::string_view name, resources::Resource resource,
    filament::backend::TextureFormat format =
        filament::backend::TextureFormat::SRGB8_A8,
    uint8_t requested_levels = 1);

// Sets whether label prep profile logging is enabled.
void SetLabelPrepProfileLogging(bool enabled);

}  // namespace imp::image::details

#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_WASM_DECODE_IMAGE_H_
