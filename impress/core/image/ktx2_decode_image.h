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

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_KTX2_DECODE_IMAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_KTX2_DECODE_IMAGE_H_

#include "absl/status/statusor.h"
#include "core/image/image_contents.h"
#include "core/loader/loader_options.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"

namespace imp::image {

// Decodes a KTX2 image made for glTF 2 content. Does not support features
// unused by the glTF2 KHR_texture_basisu extension, and will return an error if
// it does not conform to the specification.
// https://github.khronos.org/KTX-Specification
// https://github.com/KhronosGroup/glTF/tree/master/extensions/2.0/Khronos/KHR_texture_basisu
// Prefer to query available compression types rather than using
// `TextureTranscodeCompressionType::Unknown`, which will output to a much
// larger uncompressed format.
//
// TODO: Presently only decodes BasisLZ supercompressed formats
// into ETC pixel formats. Other formats will return an unimplemented error.
absl::StatusOr<CompressedImageContents> Ktx2DecodeImage(
    loader::LoaderOptions::TextureTranscodeCompressionType compression_type,
    absl::Span<const uint8_t> file_bytes);

}  // namespace imp::image

#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_KTX2_DECODE_IMAGE_H_
