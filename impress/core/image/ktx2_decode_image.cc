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

#include "core/image/ktx2_decode_image.h"

#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/base/const_init.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/synchronization/mutex.h"
#include "absl/types/optional.h"
#include "absl/types/span.h"
#include "absl/types/variant.h"
#include "basis_universal/transcoder/basisu.h"
#include "basis_universal/transcoder/basisu_transcoder.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "core/image/image_contents.h"
#include "core/image/ktx2_file.h"
#include "core/loader/loader_options.h"
#include "stdlib/lib/zstd.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::image {

namespace {

constexpr uint32_t kSupercompressionSchemeBasisLz = 1;
constexpr uint32_t kSupercompressionSchemeZstandard = 2;
constexpr uint32_t kDescriptorFormatColorModelEtc1S = 163;
constexpr uint32_t kDescriptorFormatColorModelUastc = 166;
constexpr uint32_t kDescriptorFormatChannelTypeEtc1sRgb = 0;
constexpr uint32_t kDescriptorFormatChannelTypeEtc1sRrr = 3;
constexpr uint32_t kDescriptorFormatChannelTypeEtc1sAaa = 15;

void InitializeBasisTranscoder() {
  ABSL_CONST_INIT static absl::Mutex init_mutex(absl::kConstInit);
  static bool is_transcoder_initialized = false;
  absl::MutexLock lock(init_mutex);
  if (!is_transcoder_initialized) {
    // Should be called only once globally. Basisu may generate some logging
    // warnings when this is called multiple times but subsequent calls of this
    // functions are going to be ignored.
    basist::basisu_transcoder_init();
    is_transcoder_initialized = true;
  }
}

// Maps basisu enums to filament enums between compatible formats.
template <typename T>
T ToCompressedFilamentEnum(basisu::texture_format format) {
  // Important note regarding handling of sRGB formats:
  //
  // It is possible to detect if the ktx texture is sRGB or linear by checking
  // the transfer function like this:
  //
  // bool is_srgb = false;
  // if (basic_data_format_descriptor_block.header.transfer_function ==
  //   basist::KTX2_KHR_DF_TRANSFER_SRGB) {
  //   is_srgb = true;
  // }
  //
  // We could use this to detect if the texture should be interpreted as sRGB
  // and then pick an appropriate texture format.
  //
  // However, this code instead just automatically assumes that the texture is
  // sRGB. This is intentional. This is because different graphics drivers treat
  // textures that aren't tagged as sRGB differently. Many automatically assume
  // the texture is sRGB and convert it to linear even if it isn't tagged as
  // sRGB, but some do not. This means that if we used a non-sRGB texture
  // format, then there is inconsistent behavior across devices.
  //
  // Instead, we assume that the texture is sRGB, if a linear texture is truly
  // needed then conversion can be done in the shader. This isn't ideal, but is
  // the unfortunate reality due to inconsistent graphics drivers.

  switch (format) {
    case basisu::texture_format::cETC1:   // ETC1
    case basisu::texture_format::cETC1S:  // ETC1 (subset: diff colors only, no
                                          // subblocks)
    case basisu::texture_format::cETC2_RGB:  // ETC2 color block (basisu doesn't
                                             // support ETC2 planar/T/H modes -
                                             // just basic ETC1)
      return T::ETC2_SRGB8;
    case basisu::texture_format::cETC2_RGBA:  // ETC2 EAC alpha block followed
                                              // by ETC2 color block
      return T::ETC2_EAC_SRGBA8;
    case basisu::texture_format::cBC1:  // DXT1
    case basisu::texture_format::cBC1_NV:
    case basisu::texture_format::cBC1_AMD:
      return T::DXT1_RGB;
    case basisu::texture_format::cBC3:  // DXT5 (BC4/DXT5A block followed by a
                                        // BC1/DXT1 block)
      return T::DXT5_RGBA;
    case basisu::texture_format::cASTC_LDR_4x4:  // LDR only
      return T::SRGB8_ALPHA8_ASTC_4x4;
    case basisu::texture_format::cASTC_LDR_5x4:
      return T::SRGB8_ALPHA8_ASTC_5x4;
    case basisu::texture_format::cASTC_LDR_5x5:
      return T::SRGB8_ALPHA8_ASTC_5x5;
    case basisu::texture_format::cASTC_LDR_6x5:
      return T::SRGB8_ALPHA8_ASTC_6x5;
    case basisu::texture_format::cASTC_LDR_6x6:
      return T::SRGB8_ALPHA8_ASTC_6x6;
    case basisu::texture_format::cASTC_LDR_8x5:
      return T::SRGB8_ALPHA8_ASTC_8x5;
    case basisu::texture_format::cASTC_LDR_8x6:
      return T::SRGB8_ALPHA8_ASTC_8x6;
    case basisu::texture_format::cASTC_LDR_8x8:
      return T::SRGB8_ALPHA8_ASTC_8x8;
    case basisu::texture_format::cASTC_LDR_10x5:
      return T::SRGB8_ALPHA8_ASTC_10x5;
    case basisu::texture_format::cASTC_LDR_10x6:
      return T::SRGB8_ALPHA8_ASTC_10x6;
    case basisu::texture_format::cASTC_LDR_10x8:
      return T::SRGB8_ALPHA8_ASTC_10x8;
    case basisu::texture_format::cASTC_LDR_10x10:
      return T::SRGB8_ALPHA8_ASTC_10x10;
    case basisu::texture_format::cASTC_LDR_12x10:
      return T::SRGB8_ALPHA8_ASTC_12x10;
    case basisu::texture_format::cASTC_LDR_12x12:
      return T::SRGB8_ALPHA8_ASTC_12x12;
    case basisu::texture_format::cETC2_R11_EAC:
      return T::EAC_R11;
    case basisu::texture_format::cETC2_RG11_EAC:
      return T::EAC_RG11;
    case basisu::texture_format::cBC6HSigned:
      return T::RGB_BPTC_SIGNED_FLOAT;
    case basisu::texture_format::cBC6HUnsigned:
      return T::RGB_BPTC_UNSIGNED_FLOAT;

      // Unhandled formats:
      // These may be unimplemented or not mapped to a useful filament format.
    case basisu::texture_format::cASTC_HDR_4x4:
    case basisu::texture_format::cASTC_HDR_6x6:
    case basisu::texture_format::cUASTC_HDR_4x4:
    case basisu::texture_format::cRGBA_HALF:
    case basisu::texture_format::cRGB_HALF:
    case basisu::texture_format::cRGB_9E5:
    case basisu::texture_format::cETC2_ALPHA:  // ETC2 EAC alpha block
    case basisu::texture_format::cBC4:         // DXT5A
    case basisu::texture_format::cBC5:         // 3DC/DXN (two BC4/DXT5A blocks)
    case basisu::texture_format::cBC7:
    case basisu::texture_format::cPVRTC1_4_RGB:
    case basisu::texture_format::cPVRTC1_4_RGBA:
    case basisu::texture_format::cATC_RGB:
    case basisu::texture_format::cATC_RGBA_INTERPOLATED_ALPHA:
    case basisu::texture_format::cFXT1_RGB:
    case basisu::texture_format::cPVRTC2_4_RGBA:
    case basisu::texture_format::cUASTC4x4:

      // Uncompressed/raw pixels
    case basisu::texture_format::cRGBA32:
    case basisu::texture_format::cRGB565:
    case basisu::texture_format::cBGR565:
    case basisu::texture_format::cRGBA4444:
    case basisu::texture_format::cABGR4444:
    case basisu::texture_format::cInvalidTextureFormat:
      return static_cast<T>(0xffff);
  }
}

filament::backend::TextureFormat ToTextureFormat(
    basisu::texture_format basis_format) {
  return ToCompressedFilamentEnum<filament::backend::TextureFormat>(
      basis_format);
}

absl::Status ValidateAgainstGltfExtensionSpecification(
    const ktx2::FileHeader& file_header,
    const ktx2::BasicDataFormatDescriptorBlock&
        basic_data_format_descriptor_block) {
  // https://github.com/KhronosGroup/glTF/blob/master/extensions/2.0/Khronos/KHR_texture_basisu/README.md#using-ktx-v2-images-with-basis-universal-supercompression-for-material-textures
  // https://github.khronos.org/KTX-Specification/#_texture_type
  if (!(file_header.pixel_width > 0 && file_header.pixel_height > 0 &&
        file_header.pixel_depth == 0 && file_header.layer_count == 0 &&
        file_header.face_count == 1)) {
    return absl::InternalError("KTX v2 image MUST be of 2D type");
  }

  // https://github.com/KhronosGroup/glTF/blob/master/extensions/2.0/Khronos/KHR_texture_basisu/README.md#ktx-v2-images-with-basis-universal-supercompression
  if (file_header.supercompression_scheme == kSupercompressionSchemeZstandard) {
    if (basic_data_format_descriptor_block.header.color_model !=
        kDescriptorFormatColorModelUastc) {
      return absl::InternalError("DFD colorModel MUST be KHR_DF_MODEL_UASTC");
    }
  } else if (file_header.supercompression_scheme ==
             kSupercompressionSchemeBasisLz) {
    if (basic_data_format_descriptor_block.header.color_model !=
        kDescriptorFormatColorModelEtc1S) {
      return absl::InternalError("DFD colorModel MUST be KHR_DF_MODEL_ETC1S");
    }

    if (basic_data_format_descriptor_block.samples.size() != 1 &&
        basic_data_format_descriptor_block.samples.size() != 2) {
      return absl::InternalError(
          "Only one or two channels allowed with BasisLZ");
    }

    auto sample_0_channel_type =
        basic_data_format_descriptor_block.samples[0].channel_type;
    // Technically RRR is invalid in the glTF extension spec, but the toktx tool
    // will sometimes output it.
    if (sample_0_channel_type != kDescriptorFormatChannelTypeEtc1sRgb &&
        sample_0_channel_type != kDescriptorFormatChannelTypeEtc1sRrr) {
      return absl::InternalError(
          "First BasisLZ channel must be KHR_DF_CHANNEL_ETC1S_RGB");
    }
    if (basic_data_format_descriptor_block.samples.size() == 2 &&
        basic_data_format_descriptor_block.samples[1].channel_type !=
            kDescriptorFormatChannelTypeEtc1sAaa) {
      return absl::InternalError(
          "Second present BasisLZ channel must be KHR_DF_CHANNEL_ETC1S_AAA");
    }
  }

  return absl::OkStatus();
}

}  // namespace

absl::StatusOr<CompressedImageContents> Ktx2DecodeImage(
    loader::LoaderOptions::TextureTranscodeCompressionType compression_type,
    absl::Span<const uint8_t> file_bytes) {
  InitializeBasisTranscoder();

  using TextureTranscodeCompressionType =
      loader::LoaderOptions::TextureTranscodeCompressionType;

  const ktx2::FileHeader* file_header = ktx2::TryGetFileHeader(file_bytes);
  if (!file_header) {
    return absl::InvalidArgumentError("File not recognized as KTX2");
  }

  MP_ASSIGN_OR_RETURN(auto basic_data_format_descriptor_block,
                   GetBasicDataFormatDescriptorBlock(*file_header, file_bytes));

  MP_RETURN_IF_ERROR(ValidateAgainstGltfExtensionSpecification(
      *file_header, basic_data_format_descriptor_block));

  MP_ASSIGN_OR_RETURN(absl::Span<const ktx2::LevelInfo> level_infos,
                   GetLevelInfo(*file_header, file_bytes));

  bool has_alpha = basic_data_format_descriptor_block.samples.size() == 2;

  // RGBA32 is the fallback if a compressed format is unavailable.
  auto transcoder_texture_format = basist::transcoder_texture_format::cTFRGBA32;

  // Only used for etc1s.
  absl::optional<ktx2::BasisLzGlobalData> basis_lz_global_data;

  // Initializes the transcoder according to color model.
  absl::variant<basist::basisu_lowlevel_uastc_ldr_4x4_transcoder,
                basist::basisu_lowlevel_etc1s_transcoder>
      transcoder;
  if (basic_data_format_descriptor_block.header.color_model ==
      kDescriptorFormatColorModelEtc1S) {
    if (compression_type == TextureTranscodeCompressionType::AstcAndEtc ||
        compression_type == TextureTranscodeCompressionType::EtcOnly) {
      transcoder_texture_format =
          has_alpha ? basist::transcoder_texture_format::cTFETC2_RGBA
                    : basist::transcoder_texture_format::cTFETC1_RGB;
    }

    auto basis_lz_global_data_statusor =
        GetBasisLzGlobalData(*file_header, file_bytes);
    if (!basis_lz_global_data_statusor.ok()) {
      return absl::InternalError("Error getting BasisLzGlobalData");
    }
    basis_lz_global_data.emplace(
        std::move(basis_lz_global_data_statusor).value());

    auto& etc1s_transcoder =
        transcoder.emplace<basist::basisu_lowlevel_etc1s_transcoder>();

    if (!etc1s_transcoder.decode_palettes(
            basis_lz_global_data->header.endpoint_count,
            basis_lz_global_data->endpoints.data(),
            basis_lz_global_data->endpoints.size(),
            basis_lz_global_data->header.selector_count,
            basis_lz_global_data->selectors.data(),
            basis_lz_global_data->selectors.size())) {
      return absl::InternalError("Error decoding palettes");
    }

    if (!etc1s_transcoder.decode_tables(basis_lz_global_data->tables.data(),
                                        basis_lz_global_data->tables.size())) {
      return absl::InternalError("Error decoding tables");
    }

    if (level_infos.size() != basis_lz_global_data->image_descriptions.size()) {
      return absl::InternalError(
          "Inconsistent number of BasisLZ image descriptions");
    }
  } else if (basic_data_format_descriptor_block.header.color_model ==
             kDescriptorFormatColorModelUastc) {
    transcoder
        .template emplace<basist::basisu_lowlevel_uastc_ldr_4x4_transcoder>();

    if (compression_type == TextureTranscodeCompressionType::AstcAndEtc) {
      transcoder_texture_format =
          basist::transcoder_texture_format::cTFASTC_4x4;
    } else if (compression_type == TextureTranscodeCompressionType::EtcOnly) {
      transcoder_texture_format =
          has_alpha ? basist::transcoder_texture_format::cTFETC2_RGBA
                    : basist::transcoder_texture_format::cTFETC1_RGB;
    }
  } else {
    return absl::UnimplementedError("Unsupported Ktx2 format");
  }

  // Computes format information now that the target format has been selected.
  basisu::texture_format basis_texture_format =
      basist::basis_get_basisu_texture_format(transcoder_texture_format);
  uint32_t transcoder_bytes_per_block =
      basist::basis_get_bytes_per_block_or_pixel(transcoder_texture_format);
  uint32_t transcoder_block_size_x =
      basist::basis_get_block_width(transcoder_texture_format);
  uint32_t transcoder_block_size_y =
      basist::basis_get_block_height(transcoder_texture_format);
  filament::backend::TextureFormat filament_texture_format =
      ToTextureFormat(basis_texture_format);
  bool is_uncompressed = basist::basis_transcoder_format_is_uncompressed(
      transcoder_texture_format);
  if (is_uncompressed) {
    transcoder_block_size_x = 1;
    transcoder_block_size_y = 1;
    filament_texture_format = filament::backend::TextureFormat::SRGB8_A8;
  }

  // A full mip chain has 1/3 more bytes than its top-most level.
  size_t estimated_buffer_size =
      (file_header->pixel_width / transcoder_block_size_x) *
      (file_header->pixel_height / transcoder_block_size_y) *
      transcoder_bytes_per_block * 4 / 3;

  // Buffer to store all transcoded bytes.
  std::vector<uint8_t> buffer;
  buffer.reserve(estimated_buffer_size);

  uint32_t block_size_x =
      basic_data_format_descriptor_block.header.texel_block_dimensions[0] + 1;
  uint32_t block_size_y =
      basic_data_format_descriptor_block.header.texel_block_dimensions[1] + 1;

  // Temporary storage space if Zstandard decompression is needed.
  std::vector<uint8_t> decompressed_buffer;

  // Transcodes each mip level in the image.
  for (uint32_t level_index = 0u; level_index < level_infos.size();
       ++level_index) {
    uint32_t level_width = file_header->pixel_width >> level_index;
    uint32_t level_height = file_header->pixel_height >> level_index;

    // TODO: Ends the mip chain before a dimension gets
    // below 4.
    if (level_width < 4 || level_height < 4) break;

    const ktx2::LevelInfo& level_info = level_infos[level_index];

    if (file_bytes.size() < level_info.byte_offset + level_info.byte_length) {
      return absl::InternalError("Missing level data");
    }
    absl::Span<const uint8_t> level_data_bytes =
        file_bytes.subspan(level_info.byte_offset, level_info.byte_length);

    if (file_header->supercompression_scheme ==
        kSupercompressionSchemeZstandard) {
      decompressed_buffer.resize(level_info.uncompressed_byte_length);
      if (level_info.uncompressed_byte_length !=
          ZSTD_decompress(decompressed_buffer.data(),
                          decompressed_buffer.size(), level_data_bytes.data(),
                          level_data_bytes.size())) {
        return absl::InternalError("Error decompressing zstd data");
      }
      level_data_bytes = decompressed_buffer;
    }

    uint32_t block_count_x = (level_width + block_size_x - 1) / block_size_x;
    uint32_t block_count_y = (level_height + block_size_y - 1) / block_size_y;

    uint32_t transcoder_block_count = (level_width / transcoder_block_size_x) *
                                      (level_height / transcoder_block_size_y);
    uint32_t transcoded_byte_count =
        transcoder_block_count * transcoder_bytes_per_block;

    size_t buffer_offset = buffer.size();
    buffer.resize(buffer.size() + transcoded_byte_count);

    // Performs the transcode for this level according to the type the
    // transcoder variant was initialized with.
    if (auto etc1s_transcoder =
            absl::get_if<basist::basisu_lowlevel_etc1s_transcoder>(
                &transcoder)) {
      const ktx2::BasisLzImageDesc& basis_lz_image_desc =
          basis_lz_global_data->image_descriptions[level_index];
      if (level_data_bytes.size() <
          basis_lz_image_desc.rgb_slice_byte_offset +
              basis_lz_image_desc.rgb_slice_byte_length) {
        return absl::InternalError("Missing rgb slice bytes");
      }

      if (!etc1s_transcoder->transcode_image(
              transcoder_texture_format, &buffer[buffer_offset],
              transcoder_block_count, level_data_bytes.data(),
              level_data_bytes.size(), block_count_x, block_count_y,
              level_width, level_height, 0,
              basis_lz_image_desc.rgb_slice_byte_offset,
              basis_lz_image_desc.rgb_slice_byte_length,
              basis_lz_image_desc.alpha_slice_byte_offset,
              basis_lz_image_desc.alpha_slice_byte_length, 0, has_alpha)) {
        return absl::InternalError("Error transcoding BasisLZ rgb slice");
      }
    } else if (auto uastc_transcoder = absl::get_if<
                   basist::basisu_lowlevel_uastc_ldr_4x4_transcoder>(
                   &transcoder)) {
      if (!uastc_transcoder->transcode_image(
              transcoder_texture_format, &buffer[buffer_offset],
              transcoder_block_count, level_data_bytes.data(),
              level_data_bytes.size(), block_count_x, block_count_y,
              level_width, level_height, 0, 0, level_data_bytes.size(), 0,
              has_alpha)) {
        return absl::InternalError("Error transcoding uastc");
      }
    } else {
      return absl::InternalError("Unrecognized transcoder type");
    }
  }

  return CompressedImageContents{.width = file_header->pixel_width,
                                 .height = file_header->pixel_height,
                                 .format = filament_texture_format,
                                 .buffer = std::move(buffer)};
}

}  // namespace imp::image
