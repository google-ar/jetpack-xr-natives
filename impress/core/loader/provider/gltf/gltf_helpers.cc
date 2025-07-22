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

#include "core/loader/provider/gltf/gltf_helpers.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/types/optional.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"

namespace imp::loader::details::provider_gltf {
namespace {
using ::filament::math::float3;
using ::imp::gltf::imp_proto::Buffer;
using ::imp::gltf::imp_proto::BufferView;
using ::imp::gltf::imp_proto::Gltf;
using ::imp::gltf::imp_proto::Material;

}  // namespace

GltfModel::GltfModel(Gltf* gltf_root) : gltf_root_(gltf_root) {
  empty_material_.name = "default_material";
}

const Material& GltfModel::GetMaterial(absl::optional<uint32_t> index) const {
  return index ? gltf_root_->materials[*index] : empty_material_;
}

const imp::gltf::imp_proto::Accessor& GltfModel::GetAccessor(int index) const {
  return gltf_root_->accessors[index];
}

const imp::gltf::imp_proto::Texture& GltfModel::GetTexture(int index) const {
  return gltf_root_->textures[index];
}

const imp::gltf::imp_proto::Sampler& GltfModel::GetSampler(int index) const {
  return gltf_root_->samplers[index];
}

const imp::gltf::imp_proto::Image& GltfModel::GetImage(int index) const {
  return gltf_root_->images[index];
}

int GltfModel::GetTextureCount() const { return gltf_root_->textures.size(); }
int GltfModel::GetImageCount() const { return gltf_root_->images.size(); }
int GltfModel::GetSamplerCount() const { return gltf_root_->samplers.size(); }

absl::StatusOr<BufferAccess> BufferAccessFromBufferView(
    const Gltf& gltf, uint32_t buffer_view_index, uint32_t byte_offset) {
  if (buffer_view_index >= gltf.buffer_views.size()) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Invalid BufferView index %d", buffer_view_index));
  }

  const BufferView& buffer_view = gltf.buffer_views[buffer_view_index];
  if (!buffer_view.buffer.has_value() ||
      (*buffer_view.buffer >= gltf.buffers.size()))
    return absl::InternalError("BufferView had invalid Buffer");
  const Buffer& buffer = gltf.buffers[*buffer_view.buffer];
  size_t base_offset = static_cast<size_t>(buffer_view.byte_offset) +
                       static_cast<size_t>(byte_offset);
  size_t buffer_size =
      std::min(buffer.access.size(), static_cast<size_t>(buffer.byte_length));

  if (base_offset >= buffer_size) {
    return Error("Offset is larger than the buffer size");
  }

  const uint8_t* base =
      reinterpret_cast<const uint8_t*>(&buffer.access.at(base_offset));

  if (base == nullptr) {
    return absl::InternalError("Buffer access has no data");
  }

  return BufferAccess::Wrap(base, buffer_view.byte_length);
}
}  // namespace imp::loader::details::provider_gltf
