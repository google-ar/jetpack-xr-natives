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

#include "core/loader/provider/extensions/gltf_extension_draco.h"

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

// Declaring status macros first to prevent Draco from declaring its own
// clang-format off
#include "mediapipe/framework/port/status_macros.h"
// clang-format on

#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "draco/attributes/geometry_indices.h"
#include "draco/attributes/point_attribute.h"
#include "draco/compression/decode.h"
#include "draco/core/decoder_buffer.h"
#include "draco/mesh/mesh.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"

namespace imp {
namespace loader {
namespace extensions {

size_t GetComponentSize(imp::gltf::ComponentType component_type) {
  switch (component_type) {
    case imp::gltf::BYTE:
    case imp::gltf::UNSIGNED_BYTE:
      return 1;
    case imp::gltf::SHORT:
    case imp::gltf::UNSIGNED_SHORT:
      return 2;
    case imp::gltf::UNSIGNED_INT:
    case imp::gltf::FLOAT:
      return 4;
    default:
      return 0;
  }
}

OptionalError ResolveDracoIndices(imp::gltf::Gltf* gltf,
                                  const draco::Mesh& mesh,
                                  const imp::gltf::Primitive& prim,
                                  std::vector<BufferAccess>& decoded_buffers) {
  auto& accessor = gltf->accessors[*prim.indices];
  int component_size = GetComponentSize(accessor.component_type);
  if (!component_size) {
    return Error("invalid component type in primitive indices");
  }
  int decoded_size = mesh.num_faces() * 3 * component_size;
  auto* decoded = reinterpret_cast<char*>(
      BufferAccess::Create(decoded_size, &decoded_buffers.emplace_back()));
  auto stride = component_size * 3;
  if (component_size == 4) {
    for (draco::FaceIndex f(0); f < mesh.num_faces(); ++f) {
      const auto& face = mesh.face(f);
      uint32_t indices[3] = {static_cast<uint32_t>(face[0].value()),
                             static_cast<uint32_t>(face[1].value()),
                             static_cast<uint32_t>(face[2].value())};
      memcpy(decoded + f.value() * stride, indices, stride);
    }
  } else if (component_size == 2) {
    for (draco::FaceIndex f(0); f < mesh.num_faces(); ++f) {
      const auto& face = mesh.face(f);
      uint16_t indices[3] = {static_cast<uint16_t>(face[0].value()),
                             static_cast<uint16_t>(face[1].value()),
                             static_cast<uint16_t>(face[2].value())};
      memcpy(decoded + f.value() * stride, indices, stride);
    }
  } else {
    assert(component_size == 1);
    for (draco::FaceIndex f(0); f < mesh.num_faces(); ++f) {
      const auto& face = mesh.face(f);
      uint8_t indices[3] = {static_cast<uint8_t>(face[0].value()),
                            static_cast<uint8_t>(face[1].value()),
                            static_cast<uint8_t>(face[2].value())};
      memcpy(decoded + f.value() * stride, indices, stride);
    }
  }
  gltf->buffers.emplace_back();
  gltf->buffers.back().access = absl::string_view(decoded, decoded_size);
  gltf->buffers.back().byte_length = decoded_size;

  gltf->buffer_views.emplace_back();
  auto& decoded_view = gltf->buffer_views.back();
  decoded_view.buffer = gltf->buffers.size() - 1;
  decoded_view.byte_length = mesh.num_faces() * 3 * component_size;
  decoded_view.target = imp::gltf::BufferView::ARRAY_BUFFER_TARGET;

  accessor.buffer_view = gltf->buffer_views.size() - 1;
  accessor.count = mesh.num_faces() * 3;

  return NoError();
}

template <typename T>
OptionalError CopyDracoAttribute(const draco::Mesh& mesh,
                                 const draco::PointAttribute& attr,
                                 char* output) {
  assert(attr.num_components() <= 4);
  size_t offset = 0;
  T values[4] = {0, 0, 0, 0};
  for (draco::PointIndex i(0); i < mesh.num_points(); ++i) {
    const auto vi = attr.mapped_index(i);
    if (!attr.ConvertValue<T>(vi, attr.num_components(), values)) {
      return Error("couldn't convert draco value");
    }
    memcpy(output + offset, values, sizeof(T) * attr.num_components());
    offset += sizeof(T) * attr.num_components();
  }
  return NoError();
}

OptionalError ResolveDracoAttribute(
    imp::gltf::Gltf* gltf, const draco::Mesh& mesh, const std::string& attr,
    uint32_t id, const imp::gltf::Primitive& prim,
    std::vector<BufferAccess>& decoded_buffers) {
  const auto* draco_attr = mesh.GetAttributeByUniqueId(id);
  if (!draco_attr) {
    return Error("draco mesh is missing expected attribute %s", attr.c_str());
  }
  const auto prim_attr = prim.attributes.find(attr);
  if (prim_attr == prim.attributes.end()) {
    return Error("draco attribute missing corresponding primitive attribute");
  }
  const auto component_type = gltf->accessors[prim_attr->second].component_type;
  int decoded_size = mesh.num_points() * draco_attr->num_components() *
                     GetComponentSize(component_type);
  auto* decoded = reinterpret_cast<char*>(
      BufferAccess::Create(decoded_size, &decoded_buffers.emplace_back()));
  auto copy = [](imp::gltf::ComponentType component_type,
                 const draco::Mesh& mesh, const draco::PointAttribute& attr,
                 char* output) -> OptionalError {
    switch (component_type) {
      case imp::gltf::BYTE:
        return CopyDracoAttribute<int8_t>(mesh, attr, output);
      case imp::gltf::UNSIGNED_BYTE:
        return CopyDracoAttribute<uint8_t>(mesh, attr, output);
      case imp::gltf::SHORT:
        return CopyDracoAttribute<int16_t>(mesh, attr, output);
      case imp::gltf::UNSIGNED_SHORT:
        return CopyDracoAttribute<uint16_t>(mesh, attr, output);
      case imp::gltf::FLOAT:
        return CopyDracoAttribute<float>(mesh, attr, output);
      case imp::gltf::UNSIGNED_INT:
        return CopyDracoAttribute<uint32_t>(mesh, attr, output);
      default:
        return Error("invalid component type");
    }
  };
  MP_RETURN_IF_ERROR(copy(component_type, mesh, *draco_attr, decoded));

  gltf->buffers.emplace_back();
  auto& buffer = gltf->buffers.back();
  buffer.access = absl::string_view(decoded, decoded_size);
  buffer.byte_length = decoded_size;

  gltf->buffer_views.emplace_back();
  auto& view = gltf->buffer_views.back();
  view.buffer = gltf->buffers.size() - 1;
  view.byte_length = decoded_size;
  view.byte_offset = draco_attr->byte_offset();
  view.byte_stride = draco_attr->byte_stride();
  view.target = prim.indices
                    ? imp::gltf::BufferView::ELEMENT_ARRAY_BUFFER_TARGET
                    : imp::gltf::BufferView::ARRAY_BUFFER_TARGET;

  gltf->accessors[prim_attr->second].buffer_view =
      gltf->buffer_views.size() - 1;
  gltf->accessors[prim_attr->second].count = mesh.num_points();

  return NoError();
}

absl::StatusOr<std::vector<BufferAccess>> ResolveDraco(imp::gltf::Gltf* gltf) {
  std::vector<BufferAccess> decoded_buffers;
  // Stores whether a buffer_view has already been draco-decoded.
  std::vector<bool> draco_buffer_view_decoded(gltf->buffer_views.size(), false);
  for (auto& mesh : gltf->meshes) {
    for (auto& prim : mesh.primitives) {
      if (!prim.extensions.draco) continue;

      const auto& draco = *prim.extensions.draco;
      if (!draco.buffer_view) {
        return Error("draco extension must specify a buffer view");
      }
      uint32_t buffer_view_index = *draco.buffer_view;
      if (buffer_view_index >= gltf->buffer_views.size()) {
        return Error("Invalid buffer view");
      }
      auto& buffer_view = gltf->buffer_views[buffer_view_index];
      if (!buffer_view.buffer) {
        return Error("draco buffer_view must specify a buffer");
      }
      uint32_t buffer_index = *buffer_view.buffer;
      if (buffer_index >= gltf->buffers.size()) {
        return Error("Invalid buffer");
      }

      const auto& buffer = gltf->buffers[buffer_index];
      size_t buffer_size = std::min(buffer.access.size(),
                                    static_cast<size_t>(buffer.byte_length));

      // Avoid re-decoding shared draco buffers between primitives.
      if (draco_buffer_view_decoded[buffer_view_index]) continue;
      draco_buffer_view_decoded[buffer_view_index] = true;

      size_t view_offset = buffer_view.byte_offset;
      size_t view_length = buffer_view.byte_length;

      if (view_offset + view_length > buffer_size) {
        return Error("invalid buffer view");
      }

      draco::DecoderBuffer decoder_buf;
      decoder_buf.Init(buffer.access.data() + view_offset, view_length);
      draco::Decoder decoder;
      auto decode_result = decoder.DecodeMeshFromBuffer(&decoder_buf);
      if (!decode_result.ok()) {
        return Error(decode_result.status().error_msg());
      }
      const auto& draco_mesh = decode_result.value();
      if (prim.indices) {
        // Create new buffer view for indices.
        MP_RETURN_IF_ERROR(
            ResolveDracoIndices(gltf, *draco_mesh, prim, decoded_buffers));
      }
      for (const auto& [attr, id] : draco.attributes) {
        MP_RETURN_IF_ERROR(ResolveDracoAttribute(gltf, *draco_mesh, attr, id, prim,
                                              decoded_buffers));
      }
    }
  }
  return decoded_buffers;
}

}  // namespace extensions
}  // namespace loader
}  // namespace imp
