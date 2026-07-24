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

#include "core/split_engine/skinning_helpers.h"

#include <sys/types.h>

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "absl/base/optimization.h"
#include "absl/types/span.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"

namespace imp::split_engine {

namespace {

template <typename BoneIndexType>
void UpdateSkinningTyped(MeshVertexData& unskinned_primitive_vertex_data,
                         MeshVertexData& skinned_primitive_vertex_data,
                         VertexFormat::AttributeKey bone_indices_key,
                         VertexFormat::AttributeKey bone_weights_key,
                         VertexFormat::AttributeKey position_key,
                         absl::Span<const mat4f> bone_transforms) {
  const int vertex_count =
      unskinned_primitive_vertex_data.GetDescription().vertex_count;
  if (vertex_count <= 0) {
    return;
  }

  const size_t num_transforms = bone_transforms.size();
  const mat4f* __restrict const bone_transforms_data = bone_transforms.data();

  const VertexFormat& unskinned_format =
      unskinned_primitive_vertex_data.GetDescription().vertex_format;
  const VertexFormat& skinned_format =
      skinned_primitive_vertex_data.GetDescription().vertex_format;

  const size_t bone_indices_stride =
      unskinned_format.GetVertexSize(bone_indices_key.group_index);
  const size_t bone_weights_stride =
      unskinned_format.GetVertexSize(bone_weights_key.group_index);
  const size_t unskinned_position_stride =
      unskinned_format.GetVertexSize(position_key.group_index);
  const size_t skinned_position_stride =
      skinned_format.GetVertexSize(position_key.group_index);

  const uint8_t* __restrict bone_indices_ptr = reinterpret_cast<const uint8_t*>(
      &unskinned_primitive_vertex_data.VertexAttributeAt<BoneIndexType>(
          0, bone_indices_key.attribute_offset, bone_indices_key.group_index));
  const uint8_t* __restrict bone_weights_ptr = reinterpret_cast<const uint8_t*>(
      &unskinned_primitive_vertex_data.VertexAttributeAt<float4>(
          0, bone_weights_key.attribute_offset, bone_weights_key.group_index));
  const uint8_t* __restrict unskinned_position_ptr =
      reinterpret_cast<const uint8_t*>(
          &unskinned_primitive_vertex_data.VertexAttributeAt<float3>(
              0, position_key.attribute_offset, position_key.group_index));
  uint8_t* __restrict skinned_position_ptr = reinterpret_cast<uint8_t*>(
      &skinned_primitive_vertex_data.VertexAttributeAt<float3>(
          0, position_key.attribute_offset, position_key.group_index));

  for (int vertex_index = 0; vertex_index < vertex_count; ++vertex_index) {
    const ushort4 bone_indices = static_cast<ushort4>(
        *reinterpret_cast<const BoneIndexType*>(bone_indices_ptr));

    const uint32_t max_bone_index = std::max(
        {bone_indices.x, bone_indices.y, bone_indices.z, bone_indices.w});

    if (ABSL_PREDICT_TRUE(max_bone_index < num_transforms)) {
      const float3& unskinned_pos =
          *reinterpret_cast<const float3*>(unskinned_position_ptr);
      const float4& bone_weights =
          *reinterpret_cast<const float4*>(bone_weights_ptr);

      const float4 v(unskinned_pos, 1.0f);

      const mat4f transform =
          bone_weights.x * bone_transforms_data[bone_indices.x] +
          bone_weights.y * bone_transforms_data[bone_indices.y] +
          bone_weights.z * bone_transforms_data[bone_indices.z] +
          bone_weights.w * bone_transforms_data[bone_indices.w];
      const float4 new_posi = transform * v;

      *reinterpret_cast<float3*>(skinned_position_ptr) = new_posi.xyz;
    }

    bone_indices_ptr += bone_indices_stride;
    bone_weights_ptr += bone_weights_stride;
    unskinned_position_ptr += unskinned_position_stride;
    skinned_position_ptr += skinned_position_stride;
  }
}

}  // namespace

void UpdateSkinning(const MeshVertexAndIndexData& unskinned_primitive_mesh_data,
                    MeshVertexData& skinned_primitive_vertex_data,
                    absl::Span<const mat4f> bone_transforms) {
  MeshVertexData& unskinned_primitive_vertex_data =
      *unskinned_primitive_mesh_data.vertex_data;

  const VertexFormat& vertex_format =
      unskinned_primitive_vertex_data.GetDescription().vertex_format;
  const std::optional<VertexFormat::AttributeKey> bone_indices_key =
      vertex_format.GetKeyForAttribute(
          VertexFormat::VertexAttribute::BONE_INDICES);
  const std::optional<VertexFormat::AttributeKey> bone_weights_key =
      vertex_format.GetKeyForAttribute(
          VertexFormat::VertexAttribute::BONE_WEIGHTS);
  const std::optional<VertexFormat::AttributeKey> position_key =
      vertex_format.GetKeyForAttribute(VertexFormat::VertexAttribute::POSITION);
  if (!bone_indices_key.has_value() || !bone_weights_key.has_value() ||
      !position_key.has_value()) {
    return;
  }

  const std::optional<size_t> bone_indices_attr_id =
      vertex_format.GetIndexForAttribute(
          VertexFormat::VertexAttribute::BONE_INDICES);
  const VertexFormat::AttributeType bone_indices_type =
      vertex_format.GetAttributeAt(bone_indices_attr_id.value()).type;

  if (bone_indices_type == VertexFormat::AttributeType::USHORT4) {
    UpdateSkinningTyped<ushort4>(
        unskinned_primitive_vertex_data, skinned_primitive_vertex_data,
        *bone_indices_key, *bone_weights_key, *position_key, bone_transforms);
  } else {
    UpdateSkinningTyped<ubyte4>(
        unskinned_primitive_vertex_data, skinned_primitive_vertex_data,
        *bone_indices_key, *bone_weights_key, *position_key, bone_transforms);
  }
}

}  // namespace imp::split_engine
