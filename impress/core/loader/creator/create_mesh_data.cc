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

#include "core/loader/creator/create_mesh_data.h"

#include <cstddef>
#include <cstdint>
#include <memory>

#include "core/common/schemas/render_generated.h"
#include "core/loader/creator/model_creator_helper.h"
#include "core/loader/details/bundle_resource_helpers.h"
#include "core/loader/loader_options.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"

namespace imp::loader::details {

static constexpr MeshDescription::IndexType kIndexTypes[] = {
    MeshDescription::IndexType::USHORT, MeshDescription::IndexType::UINT};

static constexpr MeshDescription::IndexType kDefaultIndexType =
    MeshDescription::IndexType::USHORT;

MeshIndexDataPtr CreateMeshIndexData(
    const schemas::IndexBufferInfo& index_buffer) {
  // Collect metadata for creating MeshIndexData object.
  MeshDescription::IndexType index_type =
      kIndexTypes[static_cast<uint8_t>(index_buffer.type())];

  size_t index_type_size = GetIndexElementSize(&index_buffer);
  size_t index_count = index_buffer.buffer()->size() / index_type_size;

  const MeshDescription kMeshDescription = {.vertex_format = {},
                                            .index_type = index_type,
                                            .vertex_count = 0,
                                            .index_count = index_count};
  MeshIndexDataPtr index_data =
      std::make_unique<MeshIndexData>(kMeshDescription);

  // Copy index data.
  if (index_type == MeshDescription::IndexType::USHORT) {
    for (size_t j = 0; j < index_count; ++j) {
      index_data->IndexAt<uint16_t>(j) = *reinterpret_cast<const uint16_t*>(
          index_buffer.buffer()->data() + (index_type_size * j));
    }
  } else {
    for (size_t j = 0; j < index_count; ++j) {
      index_data->IndexAt<uint32_t>(j) = *reinterpret_cast<const uint32_t*>(
          index_buffer.buffer()->data() + (index_type_size * j));
    }
  }

  return index_data;
}

MeshVertexDataPtr CreateMeshVertexData(
    const schemas::VertexBufferInfo& vertex_buffer,
    uint8_t vertex_access_flags) {
  using VertexAttribute = VertexFormat::VertexAttribute;
  using AttributeType = VertexFormat::AttributeType;

  VertexFormat vertex_format;
  const size_t block_count = vertex_buffer.blocks()->size();
  // The following for loop is based on the assumption that only the last
  // attribute in a block has data.
  for (size_t block_index = 0; block_index < block_count; block_index++) {
    const schemas::VertexAttributeInfo* attribute_info =
        *(vertex_buffer.blocks()->Get(block_index))->attributes()->rbegin();
    VertexAttribute vertex_attr =
        static_cast<VertexAttribute>(attribute_info->attribute());
    AttributeType attribute_type =
        static_cast<AttributeType>(attribute_info->type());
    vertex_format.AppendAttribute({vertex_attr, attribute_type});
  }

  // If no required data is available, create only empty MeshData to save
  // memory usage. Also, this is a stakeholder keep stored data align with
  // part info.
  if (!((vertex_access_flags & LoaderOptions::VertexAccessFlags::kPosition &&
         vertex_format.GetIndexForAttribute(VertexAttribute::POSITION)
             .has_value()) ||
        (vertex_access_flags & LoaderOptions::VertexAccessFlags::kTangent &&
         vertex_format.GetIndexForAttribute(VertexAttribute::TANGENTS)
             .has_value()))) {
    const MeshDescription kMeshDescription = {{}, kDefaultIndexType, 0, 0};
    return std::make_unique<MeshVertexData>(kMeshDescription);
  }

  size_t vertex_count = vertex_buffer.vertex_count();

  const MeshDescription kMeshDescription = {vertex_format, kDefaultIndexType,
                                            vertex_count, 0};
  MeshVertexDataPtr vertex_data =
      std::make_unique<MeshVertexData>(kMeshDescription);

  // Copy vertex data.
  for (size_t block_index = 0; block_index < block_count; block_index++) {
    const schemas::VertexBlockInfo* block_info =
        vertex_buffer.blocks()->Get(block_index);

    // Get attributes data.
    const schemas::VertexAttributeInfo* attribute_info =
        *(vertex_buffer.blocks()->Get(block_index))->attributes()->rbegin();
    VertexAttribute vertex_attr =
        static_cast<VertexAttribute>(attribute_info->attribute());

    bool vertex_attr_required =
        vertex_attr == VertexAttribute::POSITION &&
        vertex_access_flags & LoaderOptions::VertexAccessFlags::kPosition;
    vertex_attr_required |=
        vertex_attr == VertexAttribute::TANGENTS &&
        vertex_access_flags & LoaderOptions::VertexAccessFlags::kTangent;
    // If skin data is available, it must be loaded for get correct positions
    // of vertices.
    vertex_attr_required |=
        vertex_attr == VertexAttribute::BONE_INDICES &&
        vertex_access_flags & LoaderOptions::VertexAccessFlags::kPosition;
    vertex_attr_required |=
        vertex_attr == VertexAttribute::BONE_WEIGHTS &&
        vertex_access_flags & LoaderOptions::VertexAccessFlags::kPosition;

    if (vertex_attr_required) {
      AttributeType attribute_type =
          static_cast<AttributeType>(attribute_info->type());
      const size_t attribute_type_size = GetAttributeTypeSize(attribute_type);

      for (size_t j = 0; j < vertex_count; j++) {
        ApplyVertexAttribute(attribute_type, j, vertex_attr, vertex_data.get(),
                             block_info->buffer()->data() +
                                 (attribute_type_size * j) +
                                 attribute_info->offset());
      }
    }
  }
  return vertex_data;
}

}  // namespace imp::loader::details
