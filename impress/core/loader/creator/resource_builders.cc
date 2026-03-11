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

#include "core/loader/creator/resource_builders.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/IndexBuffer.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/filament/include/filament/VertexBuffer.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/math/include/math/mat3.h"
#include "filament/libs/math/include/math/mathfwd.h"
#include "core/common/buffer_access.h"
#include "core/common/enum_flags.h"
#include "core/common/schemas/render_generated.h"
#include "core/image/image_contents.h"
#include "core/loader/creator/inflight_creation.h"
#include "core/loader/details/bundle_resource_helpers.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/math/vec.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/model/mesh/mesh_builder.h"
#include "core/render/texture_builder.h"
#include "core/view/base_view.h"

namespace imp::loader::details {

using ::filament::Engine;
using ::filament::IndexBuffer;
using ::filament::LinearColorA;
using ::filament::Material;
using ::filament::MorphTargetBuffer;
using ::filament::Texture;
using ::filament::TextureSampler;
using ::filament::VertexBuffer;
using ::filament::math::bool2;
using ::filament::math::bool3;
using ::filament::math::bool4;
using ::filament::math::float2;
using ::filament::math::float3;
using ::filament::math::float4;
using ::filament::math::int2;
using ::filament::math::int3;
using ::filament::math::int4;
using ::filament::math::mat3f;
using ::filament::math::quatf;

namespace {

static constexpr const size_t kMaxAttributeCount =
    static_cast<size_t>(schemas::AttributeType::MAX);

static constexpr const size_t kMaxTargetsCount =
    static_cast<size_t>(filament::MAX_MORPH_TARGETS);

static constexpr size_t kMaxTextureSize = 8192;

#define VERTEX_ATTRIBUTE_ASSERT(x)                                    \
  static_assert(static_cast<size_t>(filament::VertexAttribute::x) ==  \
                    static_cast<size_t>(schemas::VertexAttribute::x), \
                "Enum mismatch")

VERTEX_ATTRIBUTE_ASSERT(POSITION);
VERTEX_ATTRIBUTE_ASSERT(TANGENTS);
VERTEX_ATTRIBUTE_ASSERT(COLOR);
VERTEX_ATTRIBUTE_ASSERT(UV0);
VERTEX_ATTRIBUTE_ASSERT(UV1);
VERTEX_ATTRIBUTE_ASSERT(BONE_INDICES);
VERTEX_ATTRIBUTE_ASSERT(BONE_WEIGHTS);
VERTEX_ATTRIBUTE_ASSERT(MORPH_POSITION_0);
VERTEX_ATTRIBUTE_ASSERT(MORPH_POSITION_1);
VERTEX_ATTRIBUTE_ASSERT(MORPH_POSITION_2);
VERTEX_ATTRIBUTE_ASSERT(MORPH_POSITION_3);
VERTEX_ATTRIBUTE_ASSERT(MORPH_TANGENTS_0);
VERTEX_ATTRIBUTE_ASSERT(MORPH_TANGENTS_1);
VERTEX_ATTRIBUTE_ASSERT(MORPH_TANGENTS_2);
VERTEX_ATTRIBUTE_ASSERT(MORPH_TANGENTS_3);
static_assert(schemas::VertexAttribute::MAX ==
                  schemas::VertexAttribute::MORPH_TANGENTS_3,
              "New fields added but assert not updated");

filament::VertexAttribute ToFilament(schemas::VertexAttribute attribute) {
  return static_cast<filament::VertexAttribute>(attribute);
}

#define ATTRIBUTE_TYPE_ASSERT(x)                                       \
  static_assert(                                                       \
      static_cast<size_t>(filament::VertexBuffer::AttributeType::x) == \
          static_cast<size_t>(schemas::AttributeType::x),              \
      "Enum mismatch")

ATTRIBUTE_TYPE_ASSERT(BYTE);
ATTRIBUTE_TYPE_ASSERT(BYTE2);
ATTRIBUTE_TYPE_ASSERT(BYTE3);
ATTRIBUTE_TYPE_ASSERT(BYTE4);
ATTRIBUTE_TYPE_ASSERT(UBYTE);
ATTRIBUTE_TYPE_ASSERT(UBYTE2);
ATTRIBUTE_TYPE_ASSERT(UBYTE3);
ATTRIBUTE_TYPE_ASSERT(UBYTE4);
ATTRIBUTE_TYPE_ASSERT(SHORT);
ATTRIBUTE_TYPE_ASSERT(SHORT2);
ATTRIBUTE_TYPE_ASSERT(SHORT3);
ATTRIBUTE_TYPE_ASSERT(SHORT4);
ATTRIBUTE_TYPE_ASSERT(USHORT);
ATTRIBUTE_TYPE_ASSERT(USHORT2);
ATTRIBUTE_TYPE_ASSERT(USHORT3);
ATTRIBUTE_TYPE_ASSERT(USHORT4);
ATTRIBUTE_TYPE_ASSERT(INT);
ATTRIBUTE_TYPE_ASSERT(UINT);
ATTRIBUTE_TYPE_ASSERT(FLOAT);
ATTRIBUTE_TYPE_ASSERT(FLOAT2);
ATTRIBUTE_TYPE_ASSERT(FLOAT3);
ATTRIBUTE_TYPE_ASSERT(FLOAT4);
ATTRIBUTE_TYPE_ASSERT(HALF);
ATTRIBUTE_TYPE_ASSERT(HALF2);
ATTRIBUTE_TYPE_ASSERT(HALF3);
ATTRIBUTE_TYPE_ASSERT(HALF4);
static_assert(schemas::AttributeType::MAX == schemas::AttributeType::HALF4,
              "New fields added but assert not updated");

filament::VertexBuffer::AttributeType ToFilament(schemas::AttributeType type) {
  return static_cast<filament::VertexBuffer::AttributeType>(type);
}

filament::IndexBuffer::IndexType ToFilament(schemas::IndexType type) {
  static_assert(schemas::IndexType::MAX == schemas::IndexType::UINT,
                "New fields added but converter not updated");

  switch (type) {
    default:
    case schemas::IndexType::USHORT:
      return filament::IndexBuffer::IndexType::USHORT;
    case schemas::IndexType::UINT:
      return filament::IndexBuffer::IndexType::UINT;
  }
}

////

#define WRAP_MODE_ASSERT(x)                                                   \
  static_assert(static_cast<size_t>(filament::TextureSampler::WrapMode::x) == \
                    static_cast<size_t>(schemas::WrapMode::x),                \
                "Enum mismatch")

WRAP_MODE_ASSERT(CLAMP_TO_EDGE);
WRAP_MODE_ASSERT(REPEAT);
WRAP_MODE_ASSERT(MIRRORED_REPEAT);
static_assert(schemas::WrapMode::MAX == schemas::WrapMode::MIRRORED_REPEAT,
              "New fields added but assert not updated");

filament::TextureSampler::WrapMode ToFilament(schemas::WrapMode wrap_mode) {
  return static_cast<filament::TextureSampler::WrapMode>(wrap_mode);
}

#define MIN_FILTER_ASSERT(x)                                                   \
  static_assert(static_cast<size_t>(filament::TextureSampler::MinFilter::x) == \
                    static_cast<size_t>(schemas::MinFilter::x),                \
                "Enum mismatch")

MIN_FILTER_ASSERT(NEAREST);
MIN_FILTER_ASSERT(LINEAR);
MIN_FILTER_ASSERT(NEAREST_MIPMAP_NEAREST);
MIN_FILTER_ASSERT(LINEAR_MIPMAP_NEAREST);
MIN_FILTER_ASSERT(NEAREST_MIPMAP_LINEAR);
MIN_FILTER_ASSERT(LINEAR_MIPMAP_LINEAR);

static_assert(schemas::MinFilter::MAX ==
                  schemas::MinFilter::LINEAR_MIPMAP_LINEAR,
              "New fields added but assert not updated");

filament::TextureSampler::MinFilter ToFilament(schemas::MinFilter min_filter) {
  return static_cast<filament::TextureSampler::MinFilter>(min_filter);
}

#define MAG_FILTER_ASSERT(x)                                                   \
  static_assert(static_cast<size_t>(filament::TextureSampler::MagFilter::x) == \
                    static_cast<size_t>(schemas::MagFilter::x),                \
                "Enum mismatch")

MAG_FILTER_ASSERT(NEAREST);
MAG_FILTER_ASSERT(LINEAR);

static_assert(schemas::MagFilter::MAX == schemas::MagFilter::LINEAR,
              "New fields added but assert not updated");

filament::TextureSampler::MagFilter ToFilament(schemas::MagFilter mag_filter) {
  return static_cast<filament::TextureSampler::MagFilter>(mag_filter);
}

#define COMPARE_MODE_ASSERT(x)                                         \
  static_assert(                                                       \
      static_cast<size_t>(filament::TextureSampler::CompareMode::x) == \
          static_cast<size_t>(schemas::CompareMode::x),                \
      "Enum mismatch")

COMPARE_MODE_ASSERT(NONE);
COMPARE_MODE_ASSERT(COMPARE_TO_TEXTURE);

static_assert(schemas::CompareMode::MAX ==
                  schemas::CompareMode::COMPARE_TO_TEXTURE,
              "New fields added but assert not updated");

filament::TextureSampler::CompareMode ToFilament(
    schemas::CompareMode compare_mode) {
  return static_cast<filament::TextureSampler::CompareMode>(compare_mode);
}

#define COMPARE_FUNC_ASSERT(x)                                         \
  static_assert(                                                       \
      static_cast<size_t>(filament::TextureSampler::CompareFunc::x) == \
          static_cast<size_t>(schemas::CompareFunc::x),                \
      "Enum mismatch")
COMPARE_FUNC_ASSERT(LE);
COMPARE_FUNC_ASSERT(GE);
COMPARE_FUNC_ASSERT(L);
COMPARE_FUNC_ASSERT(G);
COMPARE_FUNC_ASSERT(E);
COMPARE_FUNC_ASSERT(NE);
COMPARE_FUNC_ASSERT(A);
COMPARE_FUNC_ASSERT(N);

static_assert(schemas::CompareFunc::MAX == schemas::CompareFunc::N,
              "New fields added but assert not updated");

filament::TextureSampler::CompareFunc ToFilament(
    schemas::CompareFunc compare_func) {
  return static_cast<filament::TextureSampler::CompareFunc>(compare_func);
}

bool SupportsGenerateMipmaps(Flags<schemas::TextureInfoFlags> flags,
                             bool hasAlpha) {
  // Filament can't generate mipmaps for sRGB without alpha.
  // See OpenGLDriver::isRenderTargetFormatSupported, SRGB8 is desktop-only.
  return !flags.Test(schemas::TextureInfoFlags::IsSrgb) || hasAlpha;
}

}  // namespace

absl::Status FillVertexBuffer(BaseView& view, Engine* engine,
                              BaseVertexBufferBuilder& builder,
                              const schemas::VertexBufferInfo* info,
                              InflightCreation* inflight_creation,
                              std::optional<uint8_t> vertex_access_flags,
                              std::optional<absl::string_view> name) {
  const size_t block_count = info->blocks()->size();
  if (!block_count || block_count > kMaxAttributeCount) {
    // We need at least one block, and at most one block per attribute, so the
    // max block count is the same as the max attribute count.
    return absl::InternalError(
        "Failed to create Vertex Buffer - block_count out of range");
  }

  const uint64_t vertex_count = info->vertex_count();
  if (!vertex_count || vertex_count > std::numeric_limits<uint32_t>::max()) {
    return absl::InternalError(
        "Failed to create Vertex Buffer - invalid vertex count");
  }

  std::array<bool, kMaxAttributeCount> found_attributes;
  absl::c_fill(found_attributes, false);

  builder.VertexCount(vertex_count)
      .BufferCount(block_count)
      .AdvancedSkinning(info->advanced_skinning());
  if (vertex_access_flags.has_value()) {
    builder.VertexAccessFlags(vertex_access_flags.value());
  }
  for (size_t block_index = 0; block_index < block_count; block_index++) {
    const schemas::VertexBlockInfo* block_info =
        info->blocks()->Get(block_index);
    const size_t stride = block_info->stride();
    const size_t buffer_size = block_info->buffer()->size();

    const auto kFilamentMaxStride = std::numeric_limits<uint8_t>::max();
    if (stride > buffer_size || stride > kFilamentMaxStride) {
      return absl::InternalError(
          "Failed to create Vertex Buffer - invalid buffer stride: " +
          std::to_string(stride) +
          " , buffer size: " + std::to_string(buffer_size));
    }

    // Each block must have at least one attribute.
    if (!block_info->attributes()->size()) {
      return absl::InternalError(
          "Failed to create Vertex Buffer - Attributes out of range");
    }

    for (const schemas::VertexAttributeInfo* attribute_info :
         *block_info->attributes()) {
      if (!VerifyEnum(attribute_info->attribute()) ||
          !VerifyEnum(attribute_info->type())) {
        return absl::InternalError(
            "Failed to create Vertex Buffer - Invalid enum");
      }

      // Each attribute type can be used exactly once, otherwise the second one
      // will clobber the first.
      const uint8_t attrib_int =
          static_cast<uint8_t>(attribute_info->attribute());
      if (found_attributes[attrib_int]) {
        return absl::InternalError(
            "Failed to create Vertex Buffer - Duplicate attribute");
      }
      found_attributes[attrib_int] = true;

      // The stride must be larger than one element; other bounds of the stride
      // have been validated above.
      const size_t attribute_type_size =
          GetAttributeTypeSize(ToFilament(attribute_info->type()));
      const size_t actual_stride = stride == 0 ? attribute_type_size : stride;
      if (actual_stride < attribute_type_size) {
        return absl::InternalError(
            "Failed to create Vertex Buffer - Invalid stride");
      }

      // While offset is technically unbounded, a reasonable buffer shouldn't
      // have an offset larger than the actual data.
      if (attribute_info->offset() > buffer_size) {
        return absl::InternalError(
            "Failed to create Vertex Buffer - Offset out of range");
      }

      if (buffer_size <
          (actual_stride * (vertex_count - 1)) + attribute_type_size) {
        IMP_LOG(imp::ERROR) << "Invalid buffer size, buffer_size=" << buffer_size
                   << ", vertex_count=" << vertex_count
                   << ", actual_stride=" << actual_stride
                   << ", attribute_type_size=" << attribute_type_size;
        return absl::InternalError(
            "Failed to create Vertex Buffer - Invalid buffer size");
      }

      builder.Attribute(ToFilament(attribute_info->attribute()), block_index,
                        ToFilament(attribute_info->type()),
                        attribute_info->offset(), stride,
                        attribute_info->normalized());
    }
  }
  for (size_t block_index = 0; block_index < block_count; block_index++) {
    const schemas::VertexBlockInfo* block_info =
        info->blocks()->Get(block_index);
    builder.BufferAt(
        *engine, block_index,
        inflight_creation->MakeDescriptor(block_info->buffer()->data(),
                                          block_info->buffer()->size()));
  }

  if (name.has_value()) {
    builder.Name(absl::StrFormat("%s_vb", *name));
  }

  return absl::OkStatus();
}

absl::Status FillMorphTargetBuffer(BaseView& view, Engine* engine,
                                   BaseMorphTargetBufferBuilder& builder,
                                   const schemas::MorphTargetBufferInfo* info) {
  const size_t targets_count = info->targets()->size();

  if (!targets_count || targets_count > kMaxTargetsCount) {
    return absl::InternalError(
        "Failed to create Morph Target Buffer - Target count exceeds max "
        "target limit of " +
        std::to_string(kMaxTargetsCount));
  }

  const uint64_t vertex_count = info->vertex_count();
  if (!vertex_count || vertex_count > std::numeric_limits<uint32_t>::max()) {
    return absl::InternalError(
        "Failed to create Morph Target Buffer - Invalid vertex count");
  }

  builder.Count(targets_count).VertexCount(vertex_count);
  for (int index = 0; index < info->targets()->size(); index++) {
    const schemas::MorphTargetAttributeInfo* target =
        info->targets()->Get(index);
    if (target->positions()->size() < vertex_count * sizeof(float3)) {
      return absl::InternalError(
          "Failed to create Morph Target Buffer - Invalid positions size");
    }
    builder.PositionsAt(
        index, reinterpret_cast<const float3*>(target->positions()->Data()),
        vertex_count);

    if (target->tangents()->size() < vertex_count * sizeof(short4)) {
      return absl::InternalError(
          "Failed to create Morph Target Buffer - Invalid tangents size");
    }
    builder.TangentsAt(
        index, reinterpret_cast<const short4*>(target->tangents()->Data()),
        vertex_count);
  }
  return absl::OkStatus();
}

absl::Status FillIndexBuffer(BaseView& view, Engine* engine,
                             BaseIndexBufferBuilder& builder,
                             const schemas::IndexBufferInfo* info,
                             InflightCreation* inflight_creation,
                             std::optional<bool> store_index_data,
                             std::optional<absl::string_view> name) {
  if (!VerifyEnum(info->type())) {
    return absl::InternalError("Failed to create Index Buffer - Invalid enum");
  }

  const size_t index_size = GetIndexElementSize(info);
  const size_t index_count = GetIndexCount(info);
  if (!index_count || index_count > std::numeric_limits<uint32_t>::max() ||
      index_count * index_size != info->buffer()->size()) {
    return absl::InternalError(
        "Failed to create Index Buffer - Invalid index count");
  }

  builder.BufferType(ToFilament(info->type()))
      .IndexCount(index_count)
      .Buffer(*engine, inflight_creation->MakeDescriptor(
                           info->buffer()->data(), info->buffer()->size()));
  if (store_index_data.has_value()) {
    builder.StoreIndexData(store_index_data.value());
  }

  if (name.has_value()) {
    builder.Name(absl::StrFormat("%s_ib", *name));
  }

  return absl::OkStatus();
}

Texture* BuildAndFillTexture(BaseView& view, Engine* engine,
                             const schemas::TextureInfo* info,
                             std::unique_ptr<image::ImageContents> image,
                             InflightCreation* out_inflight_creation,
                             std::optional<absl::string_view> name) {
  const Flags<schemas::TextureInfoFlags> flags(info->flags());
  const uint8_t levels = (flags.Test(schemas::TextureInfoFlags::IsLookup))
                             ? 0x1
                             : image->GetLevelCount();

  if (image->GetWidth() == 0 || image->GetHeight() == 0 ||
      image->GetWidth() > kMaxTextureSize ||
      image->GetHeight() > kMaxTextureSize) {
    IMP_LOG(imp::ERROR) << "Invalid dimensions";
    return nullptr;
  }

  bool hasAlpha = image->HasAlpha();

  // Finds a suitable Filament texture format based on the image's format and
  // the texture flags.
  auto format = image->GetTextureFormat();
  if (flags.Test(schemas::TextureInfoFlags::IsR11G11B10)) {
    format = Texture::InternalFormat::R11F_G11F_B10F;
  } else if (flags.Test(schemas::TextureInfoFlags::IsSrgb)) {
    if (format == Texture::InternalFormat::RGBA8) {
      format = Texture::InternalFormat::SRGB8_A8;
    } else if (format == Texture::InternalFormat::RGB8) {
      format = Texture::InternalFormat::SRGB8;
    }
  } else {
    // TODO: It shouldn't be necessary to remove sRGB, but present
    // code expects GetTextureFormat to return an sRGB format.
    if (format == Texture::InternalFormat::SRGB8_A8) {
      format = Texture::InternalFormat::RGBA8;
    } else if (format == Texture::InternalFormat::SRGB8) {
      format = Texture::InternalFormat::RGB8;
    } else if (format == Texture::InternalFormat::ETC2_EAC_SRGBA8) {
      format = Texture::InternalFormat::ETC2_EAC_RGBA8;
    } else if (format == Texture::InternalFormat::ETC2_SRGB8) {
      format = Texture::InternalFormat::ETC2_RGB8;
    } else if (format == Texture::InternalFormat::SRGB8_ALPHA8_ASTC_4x4) {
      format = Texture::InternalFormat::RGBA_ASTC_4x4;
    }
  }

  // TODO: (broken link) - Call TextureFactory::CreateTexture() instead of
  // building the texture manually and return an OwnedTexturePtr.
  int32_t image_levels = 1;
  TextureBuilder texture_builder(view);
  if (name.has_value()) {
    std::string texture_name =
        info->name() ? absl::StrFormat("_%s", info->name()->c_str())
                     : "unnamed";
    texture_builder.Name(absl::StrFormat("%s%s_tex", *name, texture_name));
  }
  texture_builder.Width(image->GetWidth())
      .Height(image->GetHeight())
      .Levels(levels)
      .Format(format)
      .Sampler(Texture::Sampler::SAMPLER_2D)
      .Image(*engine, *image,
             // TODO: Does CreateImageCallback() need to be exposed
             // as a public method? It seems like it should be an implementation
             // detail of how this code works - the image lifetime is most clear
             // right here.
             out_inflight_creation->CreateImageCallback(), &image_levels);
  if (image_levels == 1 &&
      flags.Test(schemas::TextureInfoFlags::GenerateMips) &&
      SupportsGenerateMipmaps(flags, hasAlpha)) {
    texture_builder.GenerateMipmaps(*engine);
  }

  absl::StatusOr<filament::Texture*> texture = texture_builder.Build(*engine);
  if (!texture.ok()) {
    IMP_LOG(imp::ERROR) << "Could not create texture.";
    return nullptr;
  }

  return *texture;
}

Material* BuildMaterial(Engine* engine, const BufferAccess& compiled_material) {
  return Material::Builder()
      .package(compiled_material.Data(), compiled_material.Size())
      .build(*engine);
}

filament::TextureSampler BuildTextureSampler(
    const schemas::TextureSampler& sampler) {
  TextureSampler sampler_state;

  sampler_state.setMinFilter(ToFilament(sampler.min_filter()));
  sampler_state.setMagFilter(ToFilament(sampler.mag_filter()));
  sampler_state.setWrapModeR(ToFilament(sampler.wrap_mode_r()));
  sampler_state.setWrapModeS(ToFilament(sampler.wrap_mode_s()));
  sampler_state.setWrapModeT(ToFilament(sampler.wrap_mode_t()));
  sampler_state.setAnisotropy(1u << sampler.anisotropy_log2());
  sampler_state.setCompareMode(ToFilament(sampler.compare_mode()),
                               ToFilament(sampler.compare_func()));
  return sampler_state;
}

}  // namespace imp::loader::details
