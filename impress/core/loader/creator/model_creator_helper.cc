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

#include "core/loader/creator/model_creator_helper.h"

namespace imp::loader::details {

void ApplyVertexAttribute(VertexFormat::AttributeType attr_type, size_t id,
                          VertexFormat::VertexAttribute vertex_attr,
                          MeshVertexData* vertex, const unsigned char* data) {
  switch (attr_type) {
    case VertexFormat::AttributeType::BYTE:
      ApplyTypedVertexAttribute<int8_t>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::BYTE2:
      ApplyTypedVertexAttribute<byte2>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::BYTE3:
      ApplyTypedVertexAttribute<byte3>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::BYTE4:
      ApplyTypedVertexAttribute<byte4>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::UBYTE:
      ApplyTypedVertexAttribute<uint8_t>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::UBYTE2:
      ApplyTypedVertexAttribute<ubyte2>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::UBYTE3:
      ApplyTypedVertexAttribute<ubyte3>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::UBYTE4:
      ApplyTypedVertexAttribute<ubyte4>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::SHORT:
      ApplyTypedVertexAttribute<int16_t>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::SHORT2:
      ApplyTypedVertexAttribute<short2>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::SHORT3:
      ApplyTypedVertexAttribute<short3>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::SHORT4:
      ApplyTypedVertexAttribute<short4>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::USHORT:
      ApplyTypedVertexAttribute<uint16_t>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::USHORT2:
      ApplyTypedVertexAttribute<ushort2>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::USHORT3:
      ApplyTypedVertexAttribute<ushort3>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::USHORT4:
      ApplyTypedVertexAttribute<ushort4>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::INT:
      ApplyTypedVertexAttribute<int32_t>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::UINT:
      ApplyTypedVertexAttribute<uint32_t>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::FLOAT:
      ApplyTypedVertexAttribute<float>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::FLOAT2:
      ApplyTypedVertexAttribute<float2>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::FLOAT3:
      ApplyTypedVertexAttribute<float3>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::FLOAT4:
      ApplyTypedVertexAttribute<float4>(id, vertex_attr, vertex, data);
      break;
    case VertexFormat::AttributeType::HALF:
      ApplyTypedVertexAttribute<filament::math::half>(id, vertex_attr, vertex,
                                                      data);
      break;
    case VertexFormat::AttributeType::HALF2:
      ApplyTypedVertexAttribute<filament::math::half2>(id, vertex_attr, vertex,
                                                       data);
      break;
    case VertexFormat::AttributeType::HALF3:
      ApplyTypedVertexAttribute<filament::math::half3>(id, vertex_attr, vertex,
                                                       data);
      break;
    case VertexFormat::AttributeType::HALF4:
      ApplyTypedVertexAttribute<filament::math::half4>(id, vertex_attr, vertex,
                                                       data);
  }
}

}  // namespace imp::loader::details
