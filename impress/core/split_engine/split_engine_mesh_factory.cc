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

#include "core/split_engine/split_engine_mesh_factory.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>

#include "absl/algorithm/container.h"
#include "core/common/log.h"
#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MorphTargetBuffer.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "core/common/schemas/render_generated.h"
#include "core/common/type_helpers.h"
#include "core/common/typed_vector.h"
#include "core/loader/creator/inflight_creation.h"
#include "core/loader/creator/model_creator_helper.h"
#include "core/loader/details/bundle_resource_helpers.h"
#include "core/loader/loader_options.h"
#include "core/math/vec.h"
#include "core/model/mesh/base_mesh_builder.h"
#include "core/model/mesh/mesh_builder.h"
#include "core/model/mesh/mesh_description.h"
#include "core/model/mesh/mesh_index_data.h"
#include "core/model/mesh/mesh_vertex_data.h"
#include "core/model/mesh/vertex_format.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_data_generated.h"

namespace imp::split_engine {

namespace {

static constexpr const size_t kMaxAttributeCount =
    static_cast<size_t>(android_xr::schemas::AttributeType::MAX);

static constexpr const size_t kMaxTargetsCount =
    static_cast<size_t>(filament::MAX_MORPH_TARGETS);

template <typename T>
inline bool VerifyEnum(T value) {
  return value >= T::MIN && value <= T::MAX;
}

// Verify android_xr::schemas::VertexAttribute and filament::VertexAttribute
// enums match.
static_assert(DoEnumsMatch(filament::VertexAttribute::POSITION,
                           android_xr::schemas::VertexAttribute::POSITION),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::VertexAttribute::TANGENTS,
                           android_xr::schemas::VertexAttribute::TANGENTS),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::VertexAttribute::COLOR,
                           android_xr::schemas::VertexAttribute::COLOR),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::VertexAttribute::UV0,
                           android_xr::schemas::VertexAttribute::UV0),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::VertexAttribute::UV1,
                           android_xr::schemas::VertexAttribute::UV1),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::VertexAttribute::BONE_INDICES,
                           android_xr::schemas::VertexAttribute::BONE_INDICES),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::VertexAttribute::BONE_WEIGHTS,
                           android_xr::schemas::VertexAttribute::BONE_WEIGHTS),
              "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::VertexAttribute::MORPH_POSITION_0,
                 android_xr::schemas::VertexAttribute::MORPH_POSITION_0),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::VertexAttribute::MORPH_POSITION_1,
                 android_xr::schemas::VertexAttribute::MORPH_POSITION_1),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::VertexAttribute::MORPH_POSITION_2,
                 android_xr::schemas::VertexAttribute::MORPH_POSITION_2),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::VertexAttribute::MORPH_POSITION_3,
                 android_xr::schemas::VertexAttribute::MORPH_POSITION_3),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::VertexAttribute::MORPH_TANGENTS_0,
                 android_xr::schemas::VertexAttribute::MORPH_TANGENTS_0),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::VertexAttribute::MORPH_TANGENTS_1,
                 android_xr::schemas::VertexAttribute::MORPH_TANGENTS_1),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::VertexAttribute::MORPH_TANGENTS_2,
                 android_xr::schemas::VertexAttribute::MORPH_TANGENTS_2),
    "Enum mismatch");
static_assert(
    DoEnumsMatch(filament::VertexAttribute::MORPH_TANGENTS_3,
                 android_xr::schemas::VertexAttribute::MORPH_TANGENTS_3),
    "Enum mismatch");
static_assert(android_xr::schemas::VertexAttribute::MAX ==
                  android_xr::schemas::VertexAttribute::MORPH_TANGENTS_3,
              "New fields added but assert not updated");

// Converts an Android XR vertex attribute to a Filament vertex attribute.
// This cast is guaranteed to be safe because of the above static asserts.
filament::VertexAttribute ToFilament(
    android_xr::schemas::VertexAttribute attribute) {
  return static_cast<filament::VertexAttribute>(attribute);
}

// Verify android_xr::schemas::AttributeType and filament::backend::ElementType
// enums match.
static_assert(DoEnumsMatch(filament::backend::ElementType::BYTE,
                           android_xr::schemas::AttributeType::BYTE),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::BYTE2,
                           android_xr::schemas::AttributeType::BYTE2),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::BYTE3,
                           android_xr::schemas::AttributeType::BYTE3),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::BYTE4,
                           android_xr::schemas::AttributeType::BYTE4),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::UBYTE,
                           android_xr::schemas::AttributeType::UBYTE),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::UBYTE2,
                           android_xr::schemas::AttributeType::UBYTE2),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::UBYTE3,
                           android_xr::schemas::AttributeType::UBYTE3),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::UBYTE4,
                           android_xr::schemas::AttributeType::UBYTE4),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::SHORT,
                           android_xr::schemas::AttributeType::SHORT),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::SHORT2,
                           android_xr::schemas::AttributeType::SHORT2),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::SHORT3,
                           android_xr::schemas::AttributeType::SHORT3),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::SHORT4,
                           android_xr::schemas::AttributeType::SHORT4),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::USHORT,
                           android_xr::schemas::AttributeType::USHORT),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::USHORT2,
                           android_xr::schemas::AttributeType::USHORT2),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::USHORT3,
                           android_xr::schemas::AttributeType::USHORT3),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::USHORT4,
                           android_xr::schemas::AttributeType::USHORT4),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::INT,
                           android_xr::schemas::AttributeType::INT),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::UINT,
                           android_xr::schemas::AttributeType::UINT),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::FLOAT,
                           android_xr::schemas::AttributeType::FLOAT),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::FLOAT2,
                           android_xr::schemas::AttributeType::FLOAT2),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::FLOAT3,
                           android_xr::schemas::AttributeType::FLOAT3),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::FLOAT4,
                           android_xr::schemas::AttributeType::FLOAT4),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::HALF,
                           android_xr::schemas::AttributeType::HALF),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::HALF2,
                           android_xr::schemas::AttributeType::HALF2),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::HALF3,
                           android_xr::schemas::AttributeType::HALF3),
              "Enum mismatch");
static_assert(DoEnumsMatch(filament::backend::ElementType::HALF4,
                           android_xr::schemas::AttributeType::HALF4),
              "Enum mismatch");

static_assert(android_xr::schemas::AttributeType::MAX ==
                  android_xr::schemas::AttributeType::HALF4,
              "New fields added but assert not updated");

// Converts an Android XR attribute type to a Filament attribute type.
// This cast is guaranteed to be safe because of the above static asserts.
filament::VertexBuffer::AttributeType ToFilament(
    android_xr::schemas::AttributeType type) {
  return static_cast<filament::VertexBuffer::AttributeType>(type);
}

// Converts an Android XR index type to a Filament index type.
// Note: this is an explicit switch statement because the filament IndexType
// enum values are plucked from another enum and have odd values.
filament::IndexBuffer::IndexType ToFilament(
    android_xr::schemas::IndexType type) {
  static_assert(schemas::IndexType::MAX == schemas::IndexType::UINT,
                "New fields added but converter not updated");

  switch (type) {
    default:
    case android_xr::schemas::IndexType::USHORT:
      return filament::IndexBuffer::IndexType::USHORT;
    case android_xr::schemas::IndexType::UINT:
      return filament::IndexBuffer::IndexType::UINT;
  }
}

size_t GetIndexElementSize(
    const android_xr::schemas::IndexBufferInfo& index_info) {
  return index_info.type() == android_xr::schemas::IndexType::UINT ? 4ll : 2ll;
}

size_t GetIndexCount(const android_xr::schemas::IndexBufferInfo& index_info) {
  return index_info.buffer()->size() / GetIndexElementSize(index_info);
}

}  // namespace

static constexpr MeshDescription::IndexType kIndexTypes[] = {
    MeshDescription::IndexType::USHORT, MeshDescription::IndexType::UINT};

static constexpr MeshDescription::IndexType kDefaultIndexType =
    MeshDescription::IndexType::USHORT;

filament::VertexBuffer* BuildVertexBuffer(
    BaseView& view, filament::Engine& engine,
    const android_xr::schemas::VertexBufferInfo& info,
    loader::details::InflightCreation* inflight_creation,
    std::optional<uint8_t> vertex_access_flags) {
  if (!info.blocks()) {
    IMP_LOG(imp::ERROR) << "No blocks";
    return nullptr;
  }
  const size_t block_count = info.blocks()->size();
  if (!block_count || block_count > kMaxAttributeCount) {
    // We need at least one block, and at most one block per attribute, so the
    // max block count is the same as the max attribute count.
    IMP_LOG(imp::ERROR) << "block_count out of range";
    return nullptr;
  }

  const uint64_t vertex_count = info.vertex_count();
  if (!vertex_count || vertex_count > std::numeric_limits<uint32_t>::max()) {
    IMP_LOG(imp::ERROR) << "invalid vertex_count";
    return nullptr;
  }

  std::array<bool, kMaxAttributeCount> found_attributes;
  absl::c_fill(found_attributes, false);

  MeshBuilder mesh_builder(view);
  BaseVertexBufferBuilder& builder = mesh_builder.CreateVertexBufferBuilder();
  builder.VertexCount(vertex_count)
      .BufferCount(block_count)
      .AdvancedSkinning(info.advanced_skinning());
  if (vertex_access_flags.has_value()) {
    builder.VertexAccessFlags(vertex_access_flags.value());
  }
  for (size_t block_index = 0; block_index < block_count; block_index++) {
    const android_xr::schemas::VertexBlockInfo* block_info =
        info.blocks()->Get(block_index);
    const size_t stride = block_info->stride();
    const size_t buffer_size = block_info->buffer()->size();

    const auto kFilamentMaxStride = std::numeric_limits<uint8_t>::max();
    if (stride > buffer_size || stride > kFilamentMaxStride) {
      IMP_LOG(imp::ERROR) << "Invalid buffer stride: " << stride
                 << " , buffer size: " << buffer_size;
      return nullptr;
    }
    if ((stride & 0x3u) != 0) {
      IMP_LOG(imp::ERROR) << "Stride is not 4-byte aligned, stride=" << stride;
      return nullptr;
    }

    // Each block must have at least one attribute.
    if (!block_info->attributes() || !block_info->attributes()->size()) {
      IMP_LOG(imp::ERROR) << "Attributes out of range";
      return nullptr;
    }

    for (const android_xr::schemas::VertexAttributeInfo* attribute_info :
         *block_info->attributes()) {
      if (!VerifyEnum(attribute_info->attribute()) ||
          !VerifyEnum(attribute_info->type())) {
        IMP_LOG(imp::ERROR) << "Invalid enum";
        return nullptr;
      }

      // Each attribute type can be used exactly once, otherwise the second one
      // will clobber the first.
      const uint8_t attrib_int =
          static_cast<uint8_t>(attribute_info->attribute());
      if (attrib_int >= kMaxAttributeCount) {
        IMP_LOG(imp::ERROR) << "Attribute out of range";
        return nullptr;
      }

      if (found_attributes[attrib_int]) {
        IMP_LOG(imp::ERROR) << "Duplicate attribute";
        return nullptr;
      }
      found_attributes[attrib_int] = true;

      // The stride must be larger than one element; other bounds of the stride
      // have been validated above.
      const size_t attribute_type_size = loader::details::GetAttributeTypeSize(
          ToFilament(attribute_info->type()));
      const size_t actual_stride = stride == 0 ? attribute_type_size : stride;
      if (actual_stride < attribute_type_size) {
        IMP_LOG(imp::ERROR) << "Invalid stride";
        return nullptr;
      }

      // While offset is technically unbounded, a reasonable buffer shouldn't
      // have an offset larger than the actual data.
      if (attribute_info->offset() > buffer_size) {
        IMP_LOG(imp::ERROR) << "Offset out of range";
        return nullptr;
      }
      // Offset must be 4-byte aligned.
      if ((attribute_info->offset() & 0x3u) != 0) {
        IMP_LOG(imp::ERROR) << "Offset is not 4-byte aligned, offset="
                   << attribute_info->offset();
        return nullptr;
      }

      if (buffer_size <
          (actual_stride * (vertex_count - 1)) + attribute_type_size) {
        IMP_LOG(imp::ERROR) << "Invalid buffer size, buffer_size=" << buffer_size
                   << ", vertex_count=" << vertex_count
                   << ", actual_stride=" << actual_stride
                   << ", attribute_type_size=" << attribute_type_size;
        return nullptr;
      }

      builder.Attribute(ToFilament(attribute_info->attribute()), block_index,
                        ToFilament(attribute_info->type()),
                        attribute_info->offset(), stride,
                        attribute_info->normalized());
    }
  }
  for (size_t block_index = 0; block_index < block_count; block_index++) {
    const android_xr::schemas::VertexBlockInfo* block_info =
        info.blocks()->Get(block_index);
    if (!block_info->buffer()) {
      IMP_LOG(imp::ERROR) << "No buffer";
      return nullptr;
    }
    builder.BufferAt(
        engine, block_index,
        inflight_creation->MakeDescriptor(block_info->buffer()->data(),
                                          block_info->buffer()->size()));
  }

  TypedVector<filament::VertexBuffer*> out_vertex_buffers;
  mesh_builder.Build(&out_vertex_buffers);
  return out_vertex_buffers.front();
}

filament::IndexBuffer* BuildIndexBuffer(
    BaseView& view, filament::Engine& engine,
    const android_xr::schemas::IndexBufferInfo& info,
    loader::details::InflightCreation* inflight_creation,
    std::optional<bool> store_index_data) {
  if (!VerifyEnum(info.type())) {
    IMP_LOG(imp::INFO) << "Invalid enum";
    return nullptr;
  }

  const size_t index_size = GetIndexElementSize(info);
  const size_t index_count = GetIndexCount(info);
  if (!index_count || index_count > std::numeric_limits<uint32_t>::max() ||
      index_count * index_size != info.buffer()->size()) {
    IMP_LOG(imp::INFO) << "Invalid index count";
    return nullptr;
  }

  MeshBuilder mesh_builder(view);
  BaseIndexBufferBuilder& builder = mesh_builder.CreateIndexBufferBuilder();
  builder.BufferType(ToFilament(info.type()))
      .IndexCount(index_count)
      .Buffer(engine, inflight_creation->MakeDescriptor(info.buffer()->data(),
                                                        info.buffer()->size()));
  if (store_index_data.has_value()) {
    builder.StoreIndexData(store_index_data.value());
  }
  TypedVector<filament::IndexBuffer*> out_index_buffers;
  mesh_builder.Build(&out_index_buffers);
  return out_index_buffers.front();
}

MeshVertexDataPtr CreateMeshVertexData(
    const android_xr::schemas::VertexBufferInfo& vertex_buffer,
    uint8_t vertex_access_flags) {
  using VertexAttribute = VertexFormat::VertexAttribute;
  using AttributeType = VertexFormat::AttributeType;

  VertexFormat vertex_format;
  const size_t block_count = vertex_buffer.blocks()->size();
  for (size_t block_index = 0; block_index < block_count; block_index++) {
    for (const android_xr::schemas::VertexAttributeInfo* attribute_info :
         *vertex_buffer.blocks()->Get(block_index)->attributes()) {
      VertexAttribute vertex_attr =
          static_cast<VertexAttribute>(attribute_info->attribute());
      AttributeType attribute_type =
          static_cast<AttributeType>(attribute_info->type());
      vertex_format.AppendAttribute({vertex_attr, attribute_type});
    }
  }

  // If no required data is available, create only empty MeshData to save
  // memory usage. Also, this is a stakeholder keep stored data align with
  // part info.
  if (!((vertex_access_flags &
             loader::LoaderOptions::VertexAccessFlags::kPosition &&
         vertex_format.GetIndexForAttribute(VertexAttribute::POSITION)
             .has_value()) ||
        (vertex_access_flags &
             loader::LoaderOptions::VertexAccessFlags::kTangent &&
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
    const android_xr::schemas::VertexBlockInfo* block_info =
        vertex_buffer.blocks()->Get(block_index);
    const uint8_t* buffer_data = block_info->buffer()->data();
    const size_t stride = block_info->stride();

    for (const android_xr::schemas::VertexAttributeInfo* attribute_info :
         *block_info->attributes()) {
      VertexAttribute vertex_attr =
          static_cast<VertexAttribute>(attribute_info->attribute());

      bool vertex_attr_required =
          vertex_attr == VertexAttribute::POSITION &&
          vertex_access_flags &
              loader::LoaderOptions::VertexAccessFlags::kPosition;
      vertex_attr_required |=
          vertex_attr == VertexAttribute::TANGENTS &&
          vertex_access_flags &
              loader::LoaderOptions::VertexAccessFlags::kTangent;
      // If skin data is available, it must be loaded for get correct positions
      // of vertices.
      vertex_attr_required |=
          vertex_attr == VertexAttribute::BONE_INDICES &&
          vertex_access_flags &
              loader::LoaderOptions::VertexAccessFlags::kPosition;
      vertex_attr_required |=
          vertex_attr == VertexAttribute::BONE_WEIGHTS &&
          vertex_access_flags &
              loader::LoaderOptions::VertexAccessFlags::kPosition;

      if (vertex_attr_required) {
        AttributeType attribute_type =
            static_cast<AttributeType>(attribute_info->type());
        const size_t attribute_type_size =
            loader::details::GetAttributeTypeSize(attribute_type);
        const size_t offset = attribute_info->offset();
        const size_t current_stride =
            stride == 0 ? attribute_type_size : stride;

        for (size_t j = 0; j < vertex_count; j++) {
          loader::details::ApplyVertexAttribute(
              attribute_type, j, vertex_attr, vertex_data.get(),
              buffer_data + (current_stride * j) + offset);
        }
      }
    }
  }
  return vertex_data;
}

MeshIndexDataPtr CreateMeshIndexData(
    const android_xr::schemas::IndexBufferInfo& index_buffer) {
  // Collect metadata for creating MeshIndexData object.
  MeshDescription::IndexType index_type =
      kIndexTypes[static_cast<uint8_t>(index_buffer.type())];

  size_t index_type_size = GetIndexElementSize(index_buffer);
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

filament::MorphTargetBuffer* BuildMorphTargetBuffer(
    BaseView& view, filament::Engine& engine,
    const android_xr::schemas::MorphTargetBufferInfo& info) {
  const size_t targets_count = info.targets()->size();

  if (!targets_count || targets_count > kMaxTargetsCount) {
    IMP_LOG(imp::ERROR) << "Target count " << targets_count
               << " exceeds max target limit of " << kMaxTargetsCount;
    return nullptr;
  }

  const uint64_t vertex_count = info.vertex_count();
  if (!vertex_count || vertex_count > std::numeric_limits<uint32_t>::max()) {
    IMP_LOG(imp::ERROR) << "Invalid vertex count";
    return nullptr;
  }

  MeshBuilder mesh_builder(view);
  BaseMorphTargetBufferBuilder& builder =
      mesh_builder.CreateMorphTargetBufferBuilder();
  builder.Count(targets_count).VertexCount(vertex_count);
  for (int index = 0; index < info.targets()->size(); index++) {
    const android_xr::schemas::MorphTargetAttributeInfo* target =
        info.targets()->Get(index);
    if (target->positions()->size() < vertex_count * sizeof(float3)) {
      IMP_LOG(imp::ERROR) << "Invalid positions size";
      return nullptr;
    }
    builder.PositionsAt(
        index, reinterpret_cast<const float3*>(target->positions()->Data()),
        vertex_count);

    if (target->tangents()->size() < vertex_count * sizeof(short4)) {
      IMP_LOG(imp::ERROR) << "Invalid tangents size";
      return nullptr;
    }
    builder.TangentsAt(
        index, reinterpret_cast<const short4*>(target->tangents()->Data()),
        vertex_count);
  }
  TypedVector<filament::MorphTargetBuffer*> out_morph_target_buffers;
  mesh_builder.Build(&out_morph_target_buffers);
  return out_morph_target_buffers.front();
}

}  // namespace imp::split_engine
