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

#include "core/image/ktx2_file.h"

#include "absl/types/span.h"

#if defined(__linux__) || defined(__GNU__)
#include <endian.h>
#elif !defined(BYTE_ORDER) && !defined(_MSC_VER)
#include <machine/endian.h>
#endif

#if defined(BYTE_ORDER) && defined(BIG_ENDIAN) && BYTE_ORDER == BIG_ENDIAN
static_assert("Ktx2 decoding on big endian systems not implemented");
#endif

namespace imp::image::ktx2 {

namespace {

// Byte sequence used to identify a file as a KTX2 file.
// https://github.khronos.org/KTX-Specification/#_identifier
constexpr uint8_t kFileIdentifier[]{u'«', 'K',  'T',  'X',  ' ',    '2',
                                    '0',  u'»', '\r', '\n', '\x1A', '\n'};

constexpr uint16_t kDescriptorTypeBasic = 0;
constexpr uint16_t kVendorIdKhronos = 0;
constexpr uint16_t kVersionIdBasic = 2;

// Computes the total image count from a Ktx2 file header.
// https://github.khronos.org/KTX-Specification/#basislz_gd
uint32_t GetImageCount(const FileHeader& file_header) {
  uint32_t layer_pixel_depth = std::max(file_header.pixel_depth, 1u);
  for (int i = 1; i < file_header.level_count; i++)
    layer_pixel_depth += std::max(file_header.pixel_depth >> i, 1u);

  return std::max(file_header.layer_count, 1u) * file_header.face_count *
         layer_pixel_depth;
}

}  // namespace

const FileHeader* TryGetFileHeader(absl::Span<const uint8_t> file_bytes) {
  // KTX2 file bytes must be 4 byte aligned.
  if (reinterpret_cast<std::uintptr_t>(file_bytes.data()) % 4 != 0) {
    return nullptr;
  }

  if (file_bytes.size() < sizeof(FileHeader)) {
    return nullptr;
  }

  auto file_header = reinterpret_cast<const FileHeader*>(file_bytes.data());
  if (memcmp(file_header->identifier, kFileIdentifier,
             std::size(kFileIdentifier))) {
    return nullptr;
  }

  return file_header;
}

absl::StatusOr<absl::Span<const LevelInfo>> GetLevelInfo(
    const FileHeader& file_header, absl::Span<const uint8_t> file_bytes) {
  uint32_t level_count = std::max(file_header.level_count, 1u);
  if (file_bytes.size() <
      sizeof(FileHeader) + level_count * sizeof(LevelInfo)) {
    return absl::InternalError("Missing image levels");
  }

  return absl::MakeConstSpan(reinterpret_cast<const LevelInfo*>(
                                 file_bytes.data() + sizeof(FileHeader)),
                             level_count);
}

absl::StatusOr<BasicDataFormatDescriptorBlock>
GetBasicDataFormatDescriptorBlock(const FileHeader& file_header,
                                  absl::Span<const uint8_t> file_bytes) {
  if (file_bytes.size() < file_header.data_format_descriptor_byte_offset +
                              file_header.data_format_descriptor_byte_length) {
    return absl::InternalError("Missing data format descriptor");
  }

  // Struct to consume the Ktx2 basic data format descriptor block, which must
  // be the first descriptor block available. The descriptor block metadata is
  // not needed by the caller, so this struct is for convenience.
  // https://www.khronos.org/registry/DataFormat/specs/1.3/dataformat.1.3.html#DescriptorPrefix
  struct DataFormatDescriptorBlock {
    uint32_t data_format_descriptor_total_size;
    uint16_t vendor_id;
    uint16_t descriptor_type;
    uint16_t version_number;
    uint16_t descriptor_block_size;
    BasicDataFormatDescriptorHeader basic_data_format_descriptor_header;
  };

  if (file_header.data_format_descriptor_byte_length <
      sizeof(DataFormatDescriptorBlock)) {
    return absl::InternalError("Missing basic data format descriptor");
  }

  auto data_format_descriptor_bytes =
      file_bytes.subspan(file_header.data_format_descriptor_byte_offset,
                         file_header.data_format_descriptor_byte_length);

  auto data_format_descriptor_block =
      reinterpret_cast<const DataFormatDescriptorBlock*>(
          data_format_descriptor_bytes.data());
  data_format_descriptor_bytes.remove_prefix(sizeof(DataFormatDescriptorBlock));

  // https://www.khronos.org/registry/DataFormat/specs/1.3/dataformat.1.3.html#basicdescriptor
  if (data_format_descriptor_block->descriptor_type != kDescriptorTypeBasic ||
      data_format_descriptor_block->vendor_id != kVendorIdKhronos ||
      data_format_descriptor_block->version_number != kVersionIdBasic) {
    return absl::InternalError(
        "First data format descriptor block must be the basic data format "
        "descriptor");
  }

  // Sample information comes after the header, and its size must be computed.
  if (data_format_descriptor_bytes.empty() ||
      (data_format_descriptor_bytes.size() %
           sizeof(BasicDataFormatDescriptorSample) !=
       0)) {
    return absl::InternalError(
        "Missing basic data format descriptor sample information");
  }
  auto sample_count = data_format_descriptor_bytes.size() /
                      sizeof(BasicDataFormatDescriptorSample);
  auto samples = absl::MakeConstSpan(
      reinterpret_cast<const BasicDataFormatDescriptorSample*>(
          data_format_descriptor_bytes.data()),
      sample_count);

  return BasicDataFormatDescriptorBlock{
      .header =
          data_format_descriptor_block->basic_data_format_descriptor_header,
      .samples = samples,
  };
}

absl::StatusOr<BasisLzGlobalData> GetBasisLzGlobalData(
    const FileHeader& file_header, absl::Span<const uint8_t> file_bytes) {
  if (file_bytes.size() <
      file_header.supercomperssion_global_data_byte_offset +
          file_header.supercomperssion_global_data_byte_length) {
    return absl::InternalError("Missing supercompression global data");
  }

  if (file_header.supercomperssion_global_data_byte_length <
      sizeof(BasisLzGlobalDataHeader)) {
    return absl::InternalError("Missing BasisLZ global data");
  }

  auto supercomperssion_global_data_bytes =
      file_bytes.subspan(file_header.supercomperssion_global_data_byte_offset,
                         file_header.supercomperssion_global_data_byte_length);

  BasisLzGlobalData basis_lz_global_data{
      .header = *reinterpret_cast<BasisLzGlobalDataHeader const*>(
          supercomperssion_global_data_bytes.data())};
  supercomperssion_global_data_bytes.remove_prefix(
      sizeof(BasisLzGlobalDataHeader));

  auto image_count = GetImageCount(file_header);
  auto required_supercomperssion_global_data_byte_length =
      image_count * sizeof(BasisLzImageDesc) +
      basis_lz_global_data.header.endpoints_byte_length +
      basis_lz_global_data.header.selectors_byte_length +
      basis_lz_global_data.header.tables_byte_length;
  if (supercomperssion_global_data_bytes.size() <
      required_supercomperssion_global_data_byte_length) {
    return absl::InternalError("Missing BasisLZ global data");
  }

  basis_lz_global_data.image_descriptions =
      absl::MakeConstSpan(reinterpret_cast<BasisLzImageDesc const*>(
                              supercomperssion_global_data_bytes.data()),
                          image_count);
  supercomperssion_global_data_bytes.remove_prefix(image_count *
                                                   sizeof(BasisLzImageDesc));

  basis_lz_global_data.endpoints = supercomperssion_global_data_bytes.subspan(
      0, basis_lz_global_data.header.endpoints_byte_length);
  supercomperssion_global_data_bytes.remove_prefix(
      basis_lz_global_data.header.endpoints_byte_length);

  basis_lz_global_data.selectors = supercomperssion_global_data_bytes.subspan(
      0, basis_lz_global_data.header.selectors_byte_length);
  supercomperssion_global_data_bytes.remove_prefix(
      basis_lz_global_data.header.selectors_byte_length);

  basis_lz_global_data.tables = supercomperssion_global_data_bytes.subspan(
      0, basis_lz_global_data.header.tables_byte_length);
  supercomperssion_global_data_bytes.remove_prefix(
      basis_lz_global_data.header.tables_byte_length);

  return basis_lz_global_data;
}

}  // namespace imp::image::ktx2
