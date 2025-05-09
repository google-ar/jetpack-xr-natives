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

#ifndef THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_HELPERS_H_

#include <cstddef>
#include <cstdint>

#include "absl/container/inlined_vector.h"
#include "absl/status/statusor.h"
#include "absl/types/optional.h"
#include "core/common/buffer_access.h"
#include "core/common/typed_id.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"

namespace imp::loader::details::provider_gltf {

using NodeId = TypedId<const imp::gltf::Node, int>;
using MaterialId = TypedId<const imp::gltf::Material, int>;
using LightPunctualId = TypedId<const imp::gltf::LightPunctual, int>;
using ChannelId = TypedId<const imp::gltf::AnimationChannel, int>;
using AnimationId = TypedId<const imp::gltf::Animation, int>;
using ImageId = TypedId<const imp::gltf::Image, int>;

// Since most meshes have 1-4 primitives, pick a watermark that eliminates most
// primitive-related vector allocations.
constexpr size_t kGltfPrimitiveWatermark = 8;
template <typename T>
using GltfPrimitiveVector = absl::InlinedVector<T, kGltfPrimitiveWatermark>;

// Simple wrapper for the tinygltf root object.
class GltfModel {
 public:
  explicit GltfModel(imp::gltf::Gltf* gltf_root);

  const imp::gltf::Material& GetMaterial(absl::optional<uint32_t> index) const;
  const imp::gltf::Accessor& GetAccessor(int index) const;
  const imp::gltf::Texture& GetTexture(int index) const;
  const imp::gltf::Sampler& GetSampler(int index) const;
  const imp::gltf::Image& GetImage(int index) const;

  const imp::gltf::Gltf& Root() const { return *gltf_root_; }

  int GetTextureCount() const;
  int GetImageCount() const;
  int GetSamplerCount() const;

 private:
  imp::gltf::Gltf* gltf_root_;

  imp::gltf::Material empty_material_;
  imp::gltf::Sampler empty_sampler_;
};

absl::StatusOr<BufferAccess> BufferAccessFromBufferView(
    const imp::gltf::Gltf& gltf, uint32_t buffer_view_index,
    uint32_t byte_offset = 0);

}  // namespace imp::loader::details::provider_gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_LOADER_PROVIDER_GLTF_GLTF_HELPERS_H_
