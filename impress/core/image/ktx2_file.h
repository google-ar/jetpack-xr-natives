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

// Methods to read in Ktx2 related data structures. See
// https://github.khronos.org/KTX-Specification for the full specification.

#ifndef THIRD_PARTY_IMPRESS_CORE_IMAGE_KTX2_FILE_H_
#define THIRD_PARTY_IMPRESS_CORE_IMAGE_KTX2_FILE_H_

#include "absl/status/statusor.h"

namespace imp::image::ktx2 {

// https://github.khronos.org/KTX-Specification/#_file_structure
struct FileHeader {
  uint8_t identifier[12];
  uint32_t vulkan_format;
  uint32_t type_size;
  uint32_t pixel_width;
  uint32_t pixel_height;
  uint32_t pixel_depth;
  uint32_t layer_count;
  uint32_t face_count;
  uint32_t level_count;
  uint32_t supercompression_scheme;
  uint32_t data_format_descriptor_byte_offset;
  uint32_t data_format_descriptor_byte_length;
  uint32_t key_value_data_byte_offset;
  uint32_t key_value_data_byte_length;
  uint64_t supercomperssion_global_data_byte_offset;
  uint64_t supercomperssion_global_data_byte_length;
};

struct LevelInfo {
  uint64_t byte_offset;
  uint64_t byte_length;
  uint64_t uncompressed_byte_length;
};

// https://www.khronos.org/registry/DataFormat/specs/1.3/dataformat.1.3.html#basicdescriptor
struct BasicDataFormatDescriptorHeader {
  uint8_t color_model;
  uint8_t color_primaries;
  uint8_t transfer_function;
  uint8_t flags;
  uint8_t texel_block_dimensions[4];
  uint8_t bytes_planes[8];
};

// https://www.khronos.org/registry/DataFormat/specs/1.3/dataformat.1.3.html#SampleOverview
struct BasicDataFormatDescriptorSample {
  uint16_t bit_offset;
  uint8_t bit_length;
  uint8_t channel_type : 4;
  bool is_float : 1;
  bool is_signed : 1;
  bool is_exponent : 1;
  bool is_linear : 1;
  uint8_t sample_positions[4];
  uint32_t sample_lower;
  uint32_t sample_upper;
};

struct BasicDataFormatDescriptorBlock {
  const BasicDataFormatDescriptorHeader& header;
  absl::Span<const BasicDataFormatDescriptorSample> samples;
};

struct BasisLzGlobalDataHeader {
  uint16_t endpoint_count;
  uint16_t selector_count;
  uint32_t endpoints_byte_length;
  uint32_t selectors_byte_length;
  uint32_t tables_byte_length;
  uint32_t extended_byte_length;
};

struct BasisLzImageDesc {
  uint32_t image_flags;
  uint32_t rgb_slice_byte_offset;
  uint32_t rgb_slice_byte_length;
  uint32_t alpha_slice_byte_offset;
  uint32_t alpha_slice_byte_length;
};

// https://github.khronos.org/KTX-Specification/#basislz_gd
struct BasisLzGlobalData {
  const BasisLzGlobalDataHeader& header;
  absl::Span<const BasisLzImageDesc> image_descriptions;
  absl::Span<const uint8_t> endpoints;
  absl::Span<const uint8_t> selectors;
  absl::Span<const uint8_t> tables;
  absl::Span<const uint8_t> extended;
};

// Gets a pointer to the KTX2 file header data in the provided image file bytes.
// Returns nullptr if the file header is not recognized as a KTX2 file.
const FileHeader* TryGetFileHeader(absl::Span<const uint8_t> file_bytes);

absl::StatusOr<BasicDataFormatDescriptorBlock>
GetBasicDataFormatDescriptorBlock(const FileHeader& file_header,
                                  absl::Span<const uint8_t> file_bytes);

absl::StatusOr<absl::Span<const LevelInfo>> GetLevelInfo(
    const FileHeader& file_header, absl::Span<const uint8_t> file_bytes);

absl::StatusOr<BasisLzGlobalData> GetBasisLzGlobalData(
    const FileHeader& file_header, absl::Span<const uint8_t> file_bytes);

}  // namespace imp::image::ktx2

#endif  // THIRD_PARTY_IMPRESS_CORE_IMAGE_KTX2_FILE_H_
