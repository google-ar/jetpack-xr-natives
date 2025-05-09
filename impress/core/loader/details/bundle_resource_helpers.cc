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

#include "core/loader/details/bundle_resource_helpers.h"

#include <cstddef>
#include <cstdint>

#include "filament/filament/backend/include/backend/DriverEnums.h"
#include "filament/libs/filabridge/include/filament/MaterialEnums.h"
#include "filament/libs/math/include/math/half.h"
#include "filament/libs/math/include/math/mathfwd.h"
#include "core/common/context.h"
#include "core/common/platform_helpers.h"
#include "core/common/schemas/render_generated.h"
#include "core/math/flatbuffer_support.h"
#include "robin_map/include/tsl/robin_set.h"

namespace imp::loader::details {

namespace {

static constexpr const size_t kMaxAttributeCount =
    static_cast<size_t>(schemas::AttributeType::MAX);

static constexpr size_t kMaxTextureSize = 8192;

#define VERTEX_ATTRIBUTE_ASSERT(x)                                    \
  static_assert(static_cast<size_t>(filament::VertexAttribute::x) ==  \
                    static_cast<size_t>(schemas::VertexAttribute::x), \
                "Enum mismatch")

VERTEX_ATTRIBUTE_ASSERT(POSITION);
VERTEX_ATTRIBUTE_ASSERT(TANGENTS);
VERTEX_ATTRIBUTE_ASSERT(COLOR);
VERTEX_ATTRIBUTE_ASSERT(UV0);
VERTEX_ATTRIBUTE_ASSERT(UV1);
VERTEX_ATTRIBUTE_ASSERT(BONE_INDICES);
VERTEX_ATTRIBUTE_ASSERT(BONE_WEIGHTS);
VERTEX_ATTRIBUTE_ASSERT(MORPH_POSITION_0);
VERTEX_ATTRIBUTE_ASSERT(MORPH_POSITION_1);
VERTEX_ATTRIBUTE_ASSERT(MORPH_POSITION_2);
VERTEX_ATTRIBUTE_ASSERT(MORPH_POSITION_3);
VERTEX_ATTRIBUTE_ASSERT(MORPH_TANGENTS_0);
VERTEX_ATTRIBUTE_ASSERT(MORPH_TANGENTS_1);
VERTEX_ATTRIBUTE_ASSERT(MORPH_TANGENTS_2);
VERTEX_ATTRIBUTE_ASSERT(MORPH_TANGENTS_3);
static_assert(schemas::VertexAttribute::MAX ==
                  schemas::VertexAttribute::MORPH_TANGENTS_3,
              "New fields added but assert not updated");

filament::VertexAttribute ToFilament(schemas::VertexAttribute attribute) {
  return static_cast<filament::VertexAttribute>(attribute);
}

#define ATTRIBUTE_TYPE_ASSERT(x)                                          \
  static_assert(static_cast<size_t>(filament::backend::ElementType::x) == \
                    static_cast<size_t>(schemas::AttributeType::x),       \
                "Enum mismatch")

ATTRIBUTE_TYPE_ASSERT(BYTE);
ATTRIBUTE_TYPE_ASSERT(BYTE2);
ATTRIBUTE_TYPE_ASSERT(BYTE3);
ATTRIBUTE_TYPE_ASSERT(BYTE4);
ATTRIBUTE_TYPE_ASSERT(UBYTE);
ATTRIBUTE_TYPE_ASSERT(UBYTE2);
ATTRIBUTE_TYPE_ASSERT(UBYTE3);
ATTRIBUTE_TYPE_ASSERT(UBYTE4);
ATTRIBUTE_TYPE_ASSERT(SHORT);
ATTRIBUTE_TYPE_ASSERT(SHORT2);
ATTRIBUTE_TYPE_ASSERT(SHORT3);
ATTRIBUTE_TYPE_ASSERT(SHORT4);
ATTRIBUTE_TYPE_ASSERT(USHORT);
ATTRIBUTE_TYPE_ASSERT(USHORT2);
ATTRIBUTE_TYPE_ASSERT(USHORT3);
ATTRIBUTE_TYPE_ASSERT(USHORT4);
ATTRIBUTE_TYPE_ASSERT(INT);
ATTRIBUTE_TYPE_ASSERT(UINT);
ATTRIBUTE_TYPE_ASSERT(FLOAT);
ATTRIBUTE_TYPE_ASSERT(FLOAT2);
ATTRIBUTE_TYPE_ASSERT(FLOAT3);
ATTRIBUTE_TYPE_ASSERT(FLOAT4);
ATTRIBUTE_TYPE_ASSERT(HALF);
ATTRIBUTE_TYPE_ASSERT(HALF2);
ATTRIBUTE_TYPE_ASSERT(HALF3);
ATTRIBUTE_TYPE_ASSERT(HALF4);
static_assert(schemas::AttributeType::MAX == schemas::AttributeType::HALF4,
              "New fields added but assert not updated");

filament::backend::ElementType ToFilament(schemas::AttributeType type) {
  return static_cast<filament::backend::ElementType>(type);
}

}  // namespace

size_t GetAttributeTypeSize(filament::backend::ElementType type) {
  switch (type) {
    case filament::backend::ElementType::BYTE:
      return sizeof(int8_t);
    case filament::backend::ElementType::BYTE2:
      return sizeof(filament::math::byte2);
    case filament::backend::ElementType::BYTE3:
      return sizeof(filament::math::byte3);
    case filament::backend::ElementType::BYTE4:
      return sizeof(filament::math::byte4);
    case filament::backend::ElementType::UBYTE:
      return sizeof(uint8_t);
    case filament::backend::ElementType::UBYTE2:
      return sizeof(filament::math::ubyte2);
    case filament::backend::ElementType::UBYTE3:
      return sizeof(filament::math::ubyte3);
    case filament::backend::ElementType::UBYTE4:
      return sizeof(filament::math::ubyte4);
    case filament::backend::ElementType::SHORT:
      return sizeof(int16_t);
    case filament::backend::ElementType::SHORT2:
      return sizeof(filament::math::short2);
    case filament::backend::ElementType::SHORT3:
      return sizeof(filament::math::short3);
    case filament::backend::ElementType::SHORT4:
      return sizeof(filament::math::short4);
    case filament::backend::ElementType::USHORT:
      return sizeof(uint16_t);
    case filament::backend::ElementType::USHORT2:
      return sizeof(filament::math::ushort2);
    case filament::backend::ElementType::USHORT3:
      return sizeof(filament::math::ushort3);
    case filament::backend::ElementType::USHORT4:
      return sizeof(filament::math::ushort4);
    case filament::backend::ElementType::INT:
      return sizeof(int32_t);
    case filament::backend::ElementType::UINT:
      return sizeof(uint32_t);
    case filament::backend::ElementType::FLOAT:
      return sizeof(float);
    case filament::backend::ElementType::FLOAT2:
      return sizeof(filament::math::float2);
    case filament::backend::ElementType::FLOAT3:
      return sizeof(filament::math::float3);
    case filament::backend::ElementType::FLOAT4:
      return sizeof(filament::math::float4);
    case filament::backend::ElementType::HALF:
      return sizeof(filament::math::half);
    case filament::backend::ElementType::HALF2:
      return sizeof(filament::math::half2);
    case filament::backend::ElementType::HALF3:
      return sizeof(filament::math::half3);
    case filament::backend::ElementType::HALF4:
      return sizeof(filament::math::half4);
  }
}

size_t GetAttributeTypeSize(schemas::AttributeType type) {
  return GetAttributeTypeSize(ToFilament(type));
}

size_t GetIndexElementSize(const schemas::IndexBufferInfo* index_info) {
  return index_info->type() == schemas::IndexType::UINT ? 4ll : 2ll;
}

size_t GetIndexCount(const schemas::IndexBufferInfo* index_info) {
  return index_info->buffer()->size() / GetIndexElementSize(index_info);
}

}  // namespace imp::loader::details
