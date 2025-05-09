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

#include "core/loader/provider/extensions/gltf_extension_meshopt.h"

#include <string>

#include "core/common/platform_helpers.h"
#include "meshoptimizer/src/meshoptimizer.h"

namespace imp::loader::extensions {

absl::StatusOr<BufferAccess> ResolveMeshOpt(imp::gltf::Gltf* gltf) {
  std::string output_storage = "";
  for (auto& buffer_view : gltf->buffer_views) {
    imp::gltf::MeshoptCompression* compression =
        buffer_view.extensions.meshopt_compression.get();
    if (!compression) continue;
    if (compression->buffer >= gltf->buffers.size())
      return absl::InternalError("Invalid compressed buffer");

    auto& input_storage = gltf->buffers[compression->buffer].access;
    size_t dest_offset = output_storage.size();
    size_t output_size = compression->count * compression->byteStride;
    if (!output_size) return absl::InternalError("Invalid compressed buffer");
    output_storage.resize(dest_offset + output_size);
    int decode_result = -1;

    if (compression->byteOffset + compression->byteLength >=
        input_storage.size())
      return absl::InternalError("Invalid compressed buffer");

    auto* destination = static_cast<void*>(&output_storage.at(dest_offset));
    auto* buffer = reinterpret_cast<const unsigned char*>(
        &input_storage.at(compression->byteOffset));

    if (compression->mode == "ATTRIBUTES") {
      decode_result = meshopt_decodeVertexBuffer(
          destination, compression->count, compression->byteStride, buffer,
          compression->byteLength);
    } else if (compression->mode == "TRIANGLES") {
      decode_result = meshopt_decodeIndexBuffer(destination, compression->count,
                                                compression->byteStride, buffer,
                                                compression->byteLength);
    } else if (compression->mode == "INDICES") {
      decode_result = meshopt_decodeIndexSequence(
          destination, compression->count, compression->byteStride, buffer,
          compression->byteLength);
    }

    if (decode_result != 0) {
      return absl::InternalError("Failed to decode buffer view");
    }

    if (compression->mode == "ATTRIBUTES" && compression->filter != "NONE") {
      if (compression->filter == "OCTAHEDRAL") {
        meshopt_decodeFilterOct(destination, compression->count,
                                compression->byteStride);
      } else if (compression->filter == "QUATERNION") {
        meshopt_decodeFilterQuat(destination, compression->count,
                                 compression->byteStride);
      } else if (compression->filter == "EXPONENTIAL") {
        meshopt_decodeFilterExp(destination, compression->count,
                                compression->byteStride);
      }
    }

    // Update the buffer view to point to the new data
    buffer_view.buffer = gltf->buffers.size();
    buffer_view.byte_offset = dest_offset;
  }

  auto output_buffer = BufferAccess{};
  if (!output_storage.empty()) {
    output_buffer = BufferAccess::Clone(
        reinterpret_cast<const uint8_t*>(output_storage.data()),
        output_storage.size());
    gltf->buffers.push_back({});
    gltf->buffers.back().access = output_buffer.StringView();
    gltf->buffers.back().byte_length = output_buffer.Size();
  }

  return output_buffer;
}

}  // namespace imp::loader::extensions
