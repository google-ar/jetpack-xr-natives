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

#include "core/loader/provider/gltf/gltf_texture.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "absl/types/variant.h"
#include "filament/filament/include/filament/Texture.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"
#include "core/common/robin_map.h"
#include "core/image/image_contents.h"
#include "core/loader/loader_options.h"
#include "core/loader/provider/details/loaded_model_builder.h"
#include "core/loader/provider/extensions/gltf_extension_basis.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/loader/provider/gltf/gltf_attribute.h"
#include "core/loader/provider/gltf/gltf_geometry.h"
#include "core/loader/provider/gltf/gltf_helpers.h"
#include "core/material_library/generic_material_constants.h"
#include "core/material_library/material_param_value.h"
#include "core/material_library/schemas/generic_material_generated.h"
#include "core/model/entity_data.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_gltf {
namespace {

using ::imp::gltf::imp_proto::Image;
using ::imp::gltf::imp_proto::Material;
using ::imp::gltf::imp_proto::Primitive;
using ::imp::gltf::imp_proto::Texture;
using MinFilter = schemas::MinFilter;
using MagFilter = schemas::MagFilter;
using WrapMode = schemas::WrapMode;
using TextureInfoFlags = schemas::TextureInfoFlags;
using PathOrIndex = absl::variant<std::string, int>;
using InternalFormat = ::filament::Texture::InternalFormat;

constexpr float kDefaultIndexOfRefraction = 1.5f;

std::string GetTextureNameHelper(const Image& image, int index) {
  if (!image.uri.empty()) {
    return std::string(image.uri);
  } else if (!image.name.empty()) {
    return std::string(image.name);
  } else {
    return absl::StrFormat("%d", index);
  }
}

// Helper method to determine if material should use lighting.
schemas::GenericMaterialLightingModel GetLightingModel(
    const Material& material) {
  if (material.extensions.unlit) {
    return schemas::GenericMaterialLightingModel::Unlit;
  } else {
    return schemas::GenericMaterialLightingModel::Lit;
  }
}

// Creates a texture.
template <typename TextureInfo>
absl::Status AddTexture(
    LoadedModelBuilder& builder, const GltfModel& gltf_model,
    absl::string_view index_parameter_name, const TextureInfo* tex_info,
    LoaderOptions::TextureTranscodeCompressionType compression_type,
    TextureInfoFlags flags = TextureInfoFlags::NONE) {
  // There is no texture specified in the TextureInfo. Assign fallback sample
  // to the index and return early.
  if (!tex_info || !tex_info->index) {
    return absl::OkStatus();
  }

  // Get the gltf Texture.
  int texture_index = *tex_info->index;
  if (texture_index < 0 || texture_index >= gltf_model.GetTextureCount()) {
    return absl::NotFoundError(absl::StrFormat(
        "Gltf requested an invalid texture index: %d", texture_index));
  }
  const Texture& texture = gltf_model.GetTexture(texture_index);

  // Get the gltf Image that contains the pixel data for the texture.
  MP_ASSIGN_OR_RETURN(uint16_t image_index, GetTextureLookupIndex(texture));
  if (image_index < 0 || image_index >= gltf_model.GetImageCount()) {
    return absl::NotFoundError(absl::StrFormat(
        "Gltf requested an invalid image index: %d", image_index));
  }
  const Image& gltf_image = gltf_model.GetImage(image_index);

  // Get the image data as a BufferAccess so it can be added to the builder.
  image::EncodedImageContents encoded_image_contents;
  if (gltf_image.buffer_view) {
    const auto& gltf = gltf_model.Root();
    const uint32_t buffer_view_index = *gltf_image.buffer_view;
    if (buffer_view_index >= gltf.buffer_views.size())
      return Error("Invalid Image");
    const auto& buffer_view = gltf.buffer_views[buffer_view_index];
    if (!buffer_view.buffer) return Error("Invalid Image");
    const uint32_t buffer_index = *buffer_view.buffer;
    if (buffer_index >= gltf.buffers.size()) return Error("Invalid Image");
    const auto& buffer = gltf.buffers[buffer_index];
    size_t buffer_size =
        std::min(buffer.access.size(), static_cast<size_t>(buffer.byte_length));
    size_t view_offset = buffer_view.byte_offset;
    size_t view_length = buffer_view.byte_length;
    if (view_offset + view_length > buffer_size) return Error("Invalid Image");

    encoded_image_contents = BufferAccess::Wrap(
        reinterpret_cast<const uint8_t*>(buffer.access.data()) + view_offset,
        view_length);
  } else {
    if (gltf_image.access.empty()) {
      return Error("Image contents are empty");
    }
    encoded_image_contents = BufferAccess::Wrap(
        reinterpret_cast<const uint8_t*>(gltf_image.access.data()),
        gltf_image.access.size());
  }
  std::string image_name = GetTextureNameHelper(gltf_image, image_index);

  if (encoded_image_contents.Empty()) {
    return Error("Failed to load image");
  }

  LoadedModelBuilder::ImageData texture_data;
  // Uses the basisu extension if available.
  bool is_basis_compressed = texture.extensions.basisu != nullptr;
  if (is_basis_compressed) {
    if ((reinterpret_cast<std::uintptr_t>(encoded_image_contents.Data()) % 4) !=
        0) {
      // Creating a copy of KTX2 file bytes because its memory was not 4 byte
      // aligned.
      encoded_image_contents = image::EncodedImageContents::Clone(
          encoded_image_contents.Data(), encoded_image_contents.Size());
    }

    MP_ASSIGN_OR_RETURN(texture_data,
                     extensions::Ktx2DecodeImage(
                         compression_type, encoded_image_contents.View()));
  } else {
    texture_data = std::move(encoded_image_contents);
  }

  if (!is_basis_compressed && !(flags & TextureInfoFlags::IsR11G11B10)) {
    flags |= TextureInfoFlags::GenerateMips;
  }

  // Record this texture using image_index as the key, which is the index of the
  // actual image pixels in the glTF. This is used to look up the texture when
  // creating materials. The image index is used instead of the texture index
  // because different texture entries in the glTF can refer to the same image
  // but use a different sampler.
  builder.AddTexture(image_index, image_name, flags, std::move(texture_data));

  return absl::OkStatus();
}

}  // namespace

absl::Status ProcessTextureInfoFromNode(
    LoadedModelBuilder& builder, const GltfModel& model,
    const std::vector<imp::gltf::imp_proto::Primitive>& primitives,
    const GltfPrimitiveVector<ProcessedPrimitive>& processed_primitives,
    const Gltf2AttributeMask& mask,
    LoaderOptions::TextureTranscodeCompressionType compression_type,
    bool use_lite_materials) {
  if (primitives.empty()) {
    return Error("Mesh contains no geometry.");
  }

  RobinMap<int, model::TextureId> lookup_index_to_texture_id;

  // Parameter look up keys.
  const std::string kColorsKey = "COLOR_0";

  for (auto& primitive : primitives) {
    size_t primitive_index = &primitive - &primitives.front();
    const ProcessedPrimitive& processed_primitive =
        processed_primitives[primitive_index];
    int feature_id_texture_index = 0;
    for (const auto& texture_index : processed_primitive.feature_id_textures) {
      MP_RETURN_IF_ERROR(AddTexture(
          builder, model, kFeatureIdTextureNames[feature_id_texture_index],
          &texture_index, compression_type, TextureInfoFlags::IsLookup));
      feature_id_texture_index++;
      if (feature_id_texture_index >= kFeatureIdTextureNames.size()) {
        IMP_LOG(imp::WARNING) << "glTF contains "
                     << processed_primitive.feature_id_textures.size()
                     << " feature id textures, which is more than the "
                        "currently max supported "
                     << kFeatureIdTextureNames.size()
                     << " feature id textures.";
        break;
      }
    }
    for (auto material_index : processed_primitive.required_materials) {
      LoadedModelBuilder::MaterialId material =
          builder.GetMaterial(material_index);
      if (material) {
        // Material processing isn't cheap, and we already did it.
        continue;
      }

      // -1 is the Default material. Access the default material by passing
      // absl::nullopt into GetMaterial.
      const Material& m = model.GetMaterial(
          material_index == -1 ? absl::nullopt
                               : absl::optional<uint32_t>(material_index));

      if (m.extensions.mask) {
        continue;
      }

      // Base Color Texture.
      auto& base_color_texture =
          m.extensions.pbr_specular_glossiness
              ? m.extensions.pbr_specular_glossiness->diffuse_texture
              : m.pbr_metallic_roughness.base_color_texture;
      MP_RETURN_IF_ERROR(AddTexture(builder, model, kBaseColorIndex,
                                 &base_color_texture, compression_type,
                                 TextureInfoFlags::IsSrgb));

      schemas::GenericMaterialLightingModel lighting_model =
          GetLightingModel(m);
      if (lighting_model == schemas::GenericMaterialLightingModel::Lit) {
        // Metallic Roughness Texture.
        MP_RETURN_IF_ERROR(
            AddTexture(builder, model, kMetallicRoughnessIndex,
                       &m.pbr_metallic_roughness.metallic_roughness_texture,
                       compression_type));

        // Normal Texture.
        MP_RETURN_IF_ERROR(AddTexture(builder, model, kNormalIndex,
                                   &m.normal_texture, compression_type));

        // Occlusion Texture.
        MP_RETURN_IF_ERROR(AddTexture(builder, model, kAoIndex,
                                   &m.occlusion_texture, compression_type));

        // Emissive Texture.
        MP_RETURN_IF_ERROR(AddTexture(builder, model, kEmissiveIndex,
                                   &m.emissive_texture, compression_type,
                                   TextureInfoFlags::IsSrgb));

        // KHR_materials_clearcoat
        if (lighting_model == schemas::GenericMaterialLightingModel::Lit) {
          if (m.extensions.clearcoat) {
            auto& clearcoat = *m.extensions.clearcoat;

            MP_RETURN_IF_ERROR(AddTexture(builder, model, kClearcoatIndex,
                                       &clearcoat.clearcoat_texture,
                                       compression_type));

            MP_RETURN_IF_ERROR(AddTexture(builder, model, kClearcoatRoughnessIndex,
                                       &clearcoat.clearcoat_roughness_texture,
                                       compression_type));

            MP_RETURN_IF_ERROR(AddTexture(builder, model, kClearcoatNormalIndex,
                                       &clearcoat.clearcoat_normal_texture,
                                       compression_type));
          }
        }

        // KHR_materials_sheen
        if (m.extensions.sheen) {
          const std::unique_ptr<gltf::imp_proto::MaterialSheen>& sheen_info =
              m.extensions.sheen;

          MP_RETURN_IF_ERROR(AddTexture(builder, model, kSheenColorIndex,
                                     &sheen_info->sheen_color_texture,
                                     compression_type,
                                     TextureInfoFlags::IsSrgb));

          MP_RETURN_IF_ERROR(AddTexture(builder, model, kSheenRoughnessIndex,
                                     &sheen_info->sheen_roughness_texture,
                                     compression_type));
        }

        // KHR_materials_ior
        if (!use_lite_materials) {
          // KHR_materials_transmission
          if (m.extensions.transmission) {
            // Transmission Texture.
            MP_RETURN_IF_ERROR(
                AddTexture(builder, model, kTransmissionIndex,
                           &m.extensions.transmission->transmission_texture,
                           compression_type));
          }
        }
      }
    }
  }
  return absl::OkStatus();
}

absl::StatusOr<uint16_t> GetTextureLookupIndex(
    const gltf::imp_proto::Texture& texture) {
  if (texture.extensions.basisu != nullptr) {
    return *texture.extensions.basisu->source;
  } else if (texture.extensions.webp != nullptr) {
    return *texture.extensions.webp->source;
  } else if (texture.source.has_value()) {
    return *texture.source;
  }
  return absl::FailedPreconditionError("Texture has no valid image index.");
}

}  // namespace imp::loader::details::provider_gltf
