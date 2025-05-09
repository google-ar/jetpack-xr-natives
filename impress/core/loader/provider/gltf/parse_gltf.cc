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

#include "core/loader/provider/gltf/parse_gltf.h"

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <utility>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"
#include "core/common/schemas/render_generated.h"
#include "core/loader/provider/extensions/gltf_extension_behavior.h"
#include "core/loader/provider/extensions/gltf_extension_interactivity.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "core/proto/json_message_visitor.h"
#include "core/proto/json_reader.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::loader::details::provider_gltf {
namespace {
using ::imp::gltf::Gltf;
using MinFilter = schemas::MinFilter;
using MagFilter = schemas::MagFilter;
using WrapMode = schemas::WrapMode;

constexpr MinFilter ConvertMinFilter(int filter) {
  switch (filter) {
    case imp::gltf::Sampler::NEAREST:
      return MinFilter::NEAREST;
    case imp::gltf::Sampler::LINEAR:
      return MinFilter::LINEAR;
    case imp::gltf::Sampler::NEAREST_MIPMAP_NEAREST:
      return MinFilter::NEAREST_MIPMAP_NEAREST;
    case imp::gltf::Sampler::LINEAR_MIPMAP_NEAREST:
      return MinFilter::LINEAR_MIPMAP_NEAREST;
    case imp::gltf::Sampler::NEAREST_MIPMAP_LINEAR:
      return MinFilter::NEAREST_MIPMAP_NEAREST;
    default:
    case imp::gltf::Sampler::LINEAR_MIPMAP_LINEAR:
      return MinFilter::LINEAR_MIPMAP_LINEAR;
      break;
  }
}

constexpr MagFilter ConvertMagFilter(int filter) {
  switch (filter) {
    case imp::gltf::Sampler::NEAREST:
      return MagFilter::NEAREST;
    default:
    case imp::gltf::Sampler::LINEAR:
      return MagFilter::LINEAR;
  }
}

constexpr WrapMode ConvertWrapMode(int wrap_mode) {
  switch (wrap_mode) {
    case imp::gltf::Sampler::CLAMP_TO_EDGE:
      return WrapMode::CLAMP_TO_EDGE;
    case imp::gltf::Sampler::REPEAT:
    default:
      return WrapMode::REPEAT;
    case imp::gltf::Sampler::MIRRORED_REPEAT:
      return WrapMode::MIRRORED_REPEAT;
  }
}

OptionalError ParseGltf(const BufferAccess& access,
                        absl::optional<imp::gltf::Gltf>& out_gltf) {
  absl::string_view data(reinterpret_cast<const char*>(access.Data()),
                         access.Size());
  imp::gltf::Gltf parse_result;

  proto::JsonMessageVisitor visitor;
  std::unique_ptr<loader::extensions::Behavior> behavior =
      loader::extensions::CreateBehaviorGltfExtension();
  behavior->AddHooks(visitor);

  std::unique_ptr<loader::extensions::Interactivity> interactivity =
      loader::extensions::CreateInteractivityGltfExtension();
  interactivity->AddHooks(visitor);

  MP_RETURN_IF_ERROR(imp::proto::ParseJson(data, &parse_result, &visitor));
  out_gltf.emplace(std::move(parse_result));
  imp::gltf::Gltf* gltf = &out_gltf.value();

  // Validate data.
  auto check_optional = [](auto id, const auto& container) {
    return !id || (*id >= 0 && *id < container.size());
  };
  auto check_required = [](auto id, const auto& container) {
    return id && *id >= 0 && *id < container.size();
  };

  for (const auto& view : gltf->buffer_views) {
    if (!check_required(view.buffer, gltf->buffers)) {
      return Error("invalid buffer id in buffer view");
    }
  }

  for (const auto& accessor : gltf->accessors) {
    if (!check_optional(accessor.buffer_view, gltf->buffer_views)) {
      return Error("invalid buffer view id in accessor");
    }
  }
  for (const auto& image : gltf->images) {
    if (!check_optional(image.buffer_view, gltf->buffer_views)) {
      return Error("invalid buffer view id in image");
    }
  }
  for (const auto& texture : gltf->textures) {
    if (!check_optional(texture.sampler, gltf->samplers)) {
      return Error("invalid sampler id in texture");
    }
    if (!check_optional(texture.source, gltf->images)) {
      return Error("invalid image id in texture");
    }
    if (texture.extensions.basisu &&
        !check_required(texture.extensions.basisu->source, gltf->images)) {
      return Error("invalid KTX2 image id in texture");
    }
    if (texture.extensions.webp &&
        !check_required(texture.extensions.webp->source, gltf->images)) {
      return Error("invalid webp image id in texture");
    }
  }
  for (const auto& material : gltf->materials) {
    if (!check_optional(
            material.pbr_metallic_roughness.base_color_texture.index,
            gltf->textures)) {
      return Error(
          "invalid texture id in material.pbr_metallic_roughness.base");
    }
    if (!check_optional(
            material.pbr_metallic_roughness.metallic_roughness_texture.index,
            gltf->textures)) {
      return Error(
          "invalid texture id in material.pbr_metallic_roughness.metallic");
    }
    if (!check_optional(material.normal_texture.index, gltf->textures)) {
      return Error("invalid texture id in material.normal_texture");
    }
    if (!check_optional(material.occlusion_texture.index, gltf->textures)) {
      return Error("invalid texture id in material.occlusion_texture");
    }
    if (!check_optional(material.emissive_texture.index, gltf->textures)) {
      return Error("invalid texture id in material.emissive_texture");
    }
    if (material.extensions.pbr_specular_glossiness) {
      const auto& pbr = *material.extensions.pbr_specular_glossiness;
      if (!check_optional(pbr.diffuse_texture.index, gltf->textures)) {
        return Error(
            "invalid texture id in material.pbr_specular_glossiness.diffuse");
      }
      if (!check_optional(pbr.specular_glossiness_texture.index,
                          gltf->textures)) {
        return Error(
            "invalid texture id in "
            "material.pbr_specular_glossiness.specular");
      }
    }
  }
  for (const auto& mesh : gltf->meshes) {
    for (const auto& primitive : mesh.primitives) {
      for (const auto& [name, id] : primitive.attributes) {
        if (id >= gltf->accessors.size()) {
          return Error("invalid accessor id in primitive attributes");
        }
      }
      if (!check_optional(primitive.indices, gltf->accessors)) {
        return Error("invalid accessor id in primitive.indices");
      }
      if (!check_optional(primitive.material, gltf->materials)) {
        return Error("invalid material id in primitive");
      }
    }
  }
  for (const auto& skin : gltf->skins) {
    for (const auto& joint : skin.joints) {
      if (joint >= gltf->nodes.size()) {
        return Error("invalid node id in skin.joints");
      }
    }
    if (!check_optional(skin.inverse_bind_matrices, gltf->accessors)) {
      return Error("invalid accessor id in skin.inverse_bind_matrices");
    }
    if (!check_optional(skin.skeleton, gltf->nodes)) {
      return Error("invalid node id in skin.skeleton");
    }
  }
  for (const auto& node : gltf->nodes) {
    if (node.extensions.lights_punctual) {
      if (!gltf->extensions.lights_punctual ||
          !check_required(node.extensions.lights_punctual->light,
                          gltf->extensions.lights_punctual->lights)) {
        return Error("invalid punctual light id in node");
      }
    }
    if (!check_optional(node.mesh, gltf->meshes)) {
      return Error("invalid mesh id in node");
    }
    if (node.skin && !node.mesh) {
      return Error("node.mesh required if node.skin is specified");
    }
    if (!check_optional(node.skin, gltf->skins)) {
      return Error("invalid node id in node.skin");
    }
    for (const auto& child : node.children) {
      if (child >= gltf->nodes.size()) {
        return Error("invalid node id in node.children");
      }
    }
  }
  for (const auto& scene : gltf->scenes) {
    for (const auto& node : scene.nodes) {
      if (node >= gltf->nodes.size()) {
        return Error("invalid node id in scene.nodes");
      }
    }
  }
  for (const auto& anim : gltf->animations) {
    for (const auto& channel : anim.channels) {
      if (!check_required(channel.sampler, anim.samplers)) {
        return Error("invalid sampler id in animation channel");
      }
      if (!check_optional(channel.target.node, gltf->nodes)) {
        return Error("invalid target node id in animation channel");
      }
    }
    for (const auto& sampler : anim.samplers) {
      if (!check_required(sampler.input, gltf->accessors)) {
        return Error("invalid accessor id in animation sampler.input");
      }
      if (!check_required(sampler.output, gltf->accessors)) {
        return Error("invalid accessor id in animation sampler.output");
      }
    }
  }
  if (gltf->scene >= gltf->scenes.size()) {
    return Error("invalid scene id at top level");
  }
  return NoError();
}

constexpr const size_t kGlbHeaderSize = 12;
constexpr const char* kErrInvalidGltf = "invalid glTF";
constexpr const char* kErrDataTooShort = "data too short";
constexpr const char* kErrBadVersion = "bad version";

OptionalError ParseGlb(const BufferAccess& access,
                       absl::optional<imp::gltf::Gltf>& out_gltf) {
  constexpr const size_t kGlbChunkHeaderSize = 8;
  constexpr const uint32_t kGlbVersion = 2;
  constexpr const uint32_t kGlbMagicJsonChunk = 0x4E4F534A;  // 'JSON'
  constexpr const uint32_t kGlbMagicBinChunk = 0x004E4942;   // 'BIN'

  uint32_t version;
  memcpy(&version, access.Data() + 4, sizeof(version));
  if (version != kGlbVersion) {
    return Error(kErrBadVersion);
  }
  uint32_t size;
  memcpy(&size, access.Data() + 8, sizeof(size));
  if (kGlbHeaderSize + kGlbChunkHeaderSize > access.Size()) {
    return Error(kErrDataTooShort);
  }

  const auto* json_chunk = access.Data() + kGlbHeaderSize;
  uint32_t json_size;
  memcpy(&json_size, json_chunk, sizeof(json_size));
  if (kGlbHeaderSize + kGlbChunkHeaderSize + json_size > access.Size()) {
    return Error(kErrDataTooShort);
  }

  uint32_t magic;
  memcpy(&magic, json_chunk + 4, sizeof(magic));
  if (magic != kGlbMagicJsonChunk) {
    return Error(kErrInvalidGltf);
  }
  json_chunk += kGlbChunkHeaderSize;

  const uint8_t* bin = nullptr;
  uint32_t bin_size = 0;
  MP_RETURN_IF_ERROR(
      ParseGltf(BufferAccess::Wrap(json_chunk, json_size), out_gltf));
  imp::gltf::Gltf* gltf = &out_gltf.value();

  if (size > access.Size()) {
    return Error(kErrDataTooShort);
  }

  if (kGlbHeaderSize + 2 * kGlbChunkHeaderSize + json_size <= access.Size()) {
    bin = json_chunk + json_size;
    memcpy(&bin_size, bin, sizeof(bin_size));
    if (kGlbHeaderSize + 2 * kGlbChunkHeaderSize + json_size + bin_size >
        access.Size()) {
      return Error(kErrDataTooShort);
    }
    memcpy(&magic, bin + 4, sizeof(magic));
    if (magic != kGlbMagicBinChunk) {
      return Error(kErrInvalidGltf);
    }
    bin += kGlbChunkHeaderSize;
  }

  if (bin != nullptr && bin_size > 0) {
    if (gltf->buffers.empty()) {
      // Declaring binary data but not having a buffer entry for it is invalid.
      return Error(kErrInvalidGltf);
    }
    if (!gltf->buffers[0].uri.empty()) {
      // If the first buffer has a URI, that indicates the binary payload is
      // included twice; prefer the URI version.
    } else {
      gltf->buffers[0].access =
          absl::string_view(reinterpret_cast<const char*>(bin), bin_size);
      gltf->buffers[0].byte_length = bin_size;
    }
  }

  return NoError();
}

}  // namespace

OptionalError TryParseGltf(const imp::BufferAccess& primary_resource,
                           absl::optional<imp::gltf::Gltf>& out_gltf) {
  constexpr const uint32_t kGlbMagic = 0x46546C67;  // 'glTF'
  // Check whether the buffer is a .glb file.
  if (primary_resource.Size() < kGlbHeaderSize) {
    return Error(kErrDataTooShort);
  }

  uint32_t magic;
  memcpy(&magic, primary_resource.Data(), sizeof(magic));
  if (magic == kGlbMagic) {
    return ParseGlb(primary_resource, out_gltf);
  }
  return ParseGltf(primary_resource, out_gltf);
}

}  // namespace imp::loader::details::provider_gltf
