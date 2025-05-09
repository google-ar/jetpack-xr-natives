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

#include <cassert>
#include <cstddef>
#include <optional>
#include <vector>

#include "absl/log/check.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/model/mesh/mesh_vertex_and_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"

namespace imp::split_engine {

namespace {

// Transforms a single vertex using the given bone indices and weights.
float3 TransformVertexUsingBones(const float3& vertex_position,
                                 const ushort4 bone_indices,
                                 const float4& bone_weights,
                                 const std::vector<mat4f>& bone_transforms) {
  const mat4f transform = bone_weights.x * bone_transforms[bone_indices.x] +
                          bone_weights.y * bone_transforms[bone_indices.y] +
                          bone_weights.z * bone_transforms[bone_indices.z] +
                          bone_weights.w * bone_transforms[bone_indices.w];
  return (transform * float4(vertex_position, 1.0f)).xyz;
}

// Returns the bone indices of a vertex at index vertex_index. If the
// bone indices are ubyte4, they are cast to ushort4.
ushort4 GetBoneIndices(MeshVertexData& unskinned_primitive_vertex_data,
                       VertexFormat::AttributeType bone_indices_type,
                       size_t bone_indices_attribute_offset, int vertex_index) {
  if (bone_indices_type == VertexFormat::AttributeType::USHORT4) {
    return unskinned_primitive_vertex_data.VertexAttributeAt<ushort4>(
        vertex_index, bone_indices_attribute_offset);
  } else {
    return static_cast<ushort4>(
        unskinned_primitive_vertex_data.VertexAttributeAt<ubyte4>(
            vertex_index, bone_indices_attribute_offset));
  }
}

}  // namespace

void UpdateSkinning(const MeshVertexAndIndexData& unskinned_primitive_mesh_data,
                    MeshVertexData& skinned_primitive_vertex_data,
                    const std::vector<mat4f>& bone_transforms) {
  MeshVertexData& unskinned_primitive_vertex_data =
      *unskinned_primitive_mesh_data.vertex_data;

  const VertexFormat& vertex_format =
      unskinned_primitive_vertex_data.GetDescription().vertex_format;
  const std::optional<size_t> bone_indices_attr_id =
      vertex_format.GetIndexForAttribute(
          VertexFormat::VertexAttribute::BONE_INDICES);
  // If there are no bone indices, skip skinning.
  if (!bone_indices_attr_id.has_value()) {
    return;
  }

  VertexFormat::AttributeType bone_indices_type =
      vertex_format.GetAttributeAt(bone_indices_attr_id.value()).type;
  size_t bone_indicies_attribute_offset =
      vertex_format.GetAttributeOffsetAt(bone_indices_attr_id.value());
  size_t bone_weights_attribute_offset = vertex_format.GetAttributeOffsetAt(
      vertex_format
          .GetIndexForAttribute(VertexFormat::VertexAttribute::BONE_WEIGHTS)
          .value());
  size_t position_attribute_offset = vertex_format.GetAttributeOffsetAt(
      vertex_format
          .GetIndexForAttribute(VertexFormat::VertexAttribute::POSITION)
          .value());

  const int vertex_count =
      unskinned_primitive_vertex_data.GetDescription().vertex_count;
  for (int vertex_index = 0; vertex_index < vertex_count; ++vertex_index) {
    const ushort4 bone_indices =
        GetBoneIndices(unskinned_primitive_vertex_data, bone_indices_type,
                       bone_indicies_attribute_offset, vertex_index);
    if (bone_indices.x >= bone_transforms.size() ||
        bone_indices.y >= bone_transforms.size() ||
        bone_indices.z >= bone_transforms.size() ||
        bone_indices.w >= bone_transforms.size()) {
      return;
    }
    const float3& unskinned_vertex_position =
        unskinned_primitive_vertex_data.VertexAttributeAt<float3>(
            vertex_index, position_attribute_offset);
    const float4& bone_weights =
        unskinned_primitive_vertex_data.VertexAttributeAt<float4>(
            vertex_index, bone_weights_attribute_offset);
    skinned_primitive_vertex_data.VertexAttributeAt<float3>(
        vertex_index, position_attribute_offset) =
        TransformVertexUsingBones(unskinned_vertex_position, bone_indices,
                                  bone_weights, bone_transforms);
  }
}

}  // namespace imp::split_engine
