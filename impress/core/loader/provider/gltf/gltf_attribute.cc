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

#include "core/loader/provider/gltf/gltf_attribute.h"

#include <string>

#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/optional_error.h"
#include "core/common/schemas/render_generated.h"
#include "core/loader/provider/gltf/gltf.proto.imp.h"
#include "robin_map/include/tsl/robin_map.h"

namespace imp::loader::details::provider_gltf {
namespace {
const tsl::robin_map<std::string, Gltf2Attribute>&
GetGlTF2VertexAttributeMap() {
  static auto* map = new tsl::robin_map<std::string, Gltf2Attribute>{
      {"POSITION", Gltf2Attribute::POSITION},
      {"NORMAL", Gltf2Attribute::NORMAL},
      {"TANGENT", Gltf2Attribute::TANGENT},
      {"TEXCOORD_0", Gltf2Attribute::TEXCOORD_0},
      {"TEXCOORD_1", Gltf2Attribute::TEXCOORD_1},
      {"COLOR_0", Gltf2Attribute::COLOR_0},
      {"JOINTS_0", Gltf2Attribute::JOINTS_0},
      {"WEIGHTS_0", Gltf2Attribute::WEIGHTS_0},
      {"MORPH_POSITION_0", Gltf2Attribute::MORPH_POSITION_0},
      {"MORPH_POSITION_1", Gltf2Attribute::MORPH_POSITION_1},
      {"MORPH_POSITION_2", Gltf2Attribute::MORPH_POSITION_2},
      {"MORPH_POSITION_3", Gltf2Attribute::MORPH_POSITION_3},
      {"MORPH_TANGENT_0", Gltf2Attribute::MORPH_TANGENT_0},
      {"MORPH_TANGENT_1", Gltf2Attribute::MORPH_TANGENT_1},
      {"MORPH_TANGENT_2", Gltf2Attribute::MORPH_TANGENT_2},
      {"MORPH_TANGENT_3", Gltf2Attribute::MORPH_TANGENT_3},
      {"MORPH_NORMAL_0", Gltf2Attribute::MORPH_NORMAL_0},
      {"MORPH_NORMAL_1", Gltf2Attribute::MORPH_NORMAL_1},
      {"MORPH_NORMAL_2", Gltf2Attribute::MORPH_NORMAL_2},
      {"MORPH_NORMAL_3", Gltf2Attribute::MORPH_NORMAL_3},
  };
  return *map;
}
}  // namespace

OptionalError GetAttributeType(absl::string_view type, const int component_type,
                               schemas::AttributeType* out_type) {
  using ET = schemas::AttributeType;
  if (type == "VEC2") {
    switch (component_type) {
      case imp::gltf::imp_proto::ComponentType::BYTE:
        *out_type = ET::BYTE2;
        break;
      case imp::gltf::imp_proto::ComponentType::UNSIGNED_BYTE:
        *out_type = ET::UBYTE2;
        break;
      case imp::gltf::imp_proto::ComponentType::SHORT:
        *out_type = ET::SHORT2;
        break;
      case imp::gltf::imp_proto::ComponentType::UNSIGNED_SHORT:
        *out_type = ET::USHORT2;
        break;
      case imp::gltf::imp_proto::ComponentType::FLOAT:
        *out_type = ET::FLOAT2;
        break;
      default:
        return Error("invalid attribute");
    }
  } else if (type == "VEC3") {
    switch (component_type) {
      case imp::gltf::imp_proto::ComponentType::BYTE:
        *out_type = ET::BYTE3;
        break;
      case imp::gltf::imp_proto::ComponentType::UNSIGNED_BYTE:
        *out_type = ET::UBYTE3;
        break;
      case imp::gltf::imp_proto::ComponentType::SHORT:
        *out_type = ET::SHORT3;
        break;
      case imp::gltf::imp_proto::ComponentType::UNSIGNED_SHORT:
        *out_type = ET::USHORT3;
        break;
      case imp::gltf::imp_proto::ComponentType::FLOAT:
        *out_type = ET::FLOAT3;
        break;
      default:
        return Error("invalid attribute");
    }
  } else if (type == "VEC4") {
    switch (component_type) {
      case imp::gltf::imp_proto::ComponentType::BYTE:
        *out_type = ET::BYTE4;
        break;
      case imp::gltf::imp_proto::ComponentType::UNSIGNED_BYTE:
        *out_type = ET::UBYTE4;
        break;
      case imp::gltf::imp_proto::ComponentType::SHORT:
        *out_type = ET::SHORT4;
        break;
      case imp::gltf::imp_proto::ComponentType::UNSIGNED_SHORT:
        *out_type = ET::USHORT4;
        break;
      case imp::gltf::imp_proto::ComponentType::FLOAT:
        *out_type = ET::FLOAT4;
        break;
      default:
        return Error("invalid attribute");
    }
  } else if (type == "SCALAR") {
    switch (component_type) {
      case imp::gltf::imp_proto::ComponentType::BYTE:
        *out_type = ET::BYTE;
        break;
      case imp::gltf::imp_proto::ComponentType::UNSIGNED_BYTE:
        *out_type = ET::UBYTE;
        break;
      case imp::gltf::imp_proto::ComponentType::SHORT:
        *out_type = ET::SHORT;
        break;
      case imp::gltf::imp_proto::ComponentType::UNSIGNED_SHORT:
        *out_type = ET::USHORT;
        break;
      case imp::gltf::imp_proto::ComponentType::UNSIGNED_INT:
        *out_type = ET::UINT;
        break;
      case imp::gltf::imp_proto::ComponentType::FLOAT:
        *out_type = ET::FLOAT;
        break;
      default:
        return Error("invalid attribute");
    }
  } else {
    return Error("Invalid attribute");
  }
  return NoError();
}

const char* GetAttributeName(Gltf2Attribute attribute) {
  switch (attribute) {
    case Gltf2Attribute::POSITION:
      return "POSITION";
    case Gltf2Attribute::NORMAL:
      return "NORMAL";
    case Gltf2Attribute::TANGENT:
      return "TANGENT";
    case Gltf2Attribute::TEXCOORD_0:
      return "TEXCOORD_0";
    case Gltf2Attribute::TEXCOORD_1:
      return "TEXCOORD_1";
    case Gltf2Attribute::COLOR_0:
      return "COLOR_0";
    case Gltf2Attribute::JOINTS_0:
      return "JOINTS_0";
    case Gltf2Attribute::WEIGHTS_0:
      return "WEIGHTS_0";
    case Gltf2Attribute::MORPH_POSITION_0:
      return "MORPH_POSITION_0";
    case Gltf2Attribute::MORPH_POSITION_1:
      return "MORPH_POSITION_1";
    case Gltf2Attribute::MORPH_POSITION_2:
      return "MORPH_POSITION_2";
    case Gltf2Attribute::MORPH_POSITION_3:
      return "MORPH_POSITION_3";
    case Gltf2Attribute::MORPH_TANGENT_0:
      return "MORPH_TANGENT_0";
    case Gltf2Attribute::MORPH_TANGENT_1:
      return "MORPH_TANGENT_1";
    case Gltf2Attribute::MORPH_TANGENT_2:
      return "MORPH_TANGENT_2";
    case Gltf2Attribute::MORPH_TANGENT_3:
      return "MORPH_TANGENT_3";
    case Gltf2Attribute::MORPH_NORMAL_0:
      return "MORPH_NORMAL_0";
    case Gltf2Attribute::MORPH_NORMAL_1:
      return "MORPH_NORMAL_1";
    case Gltf2Attribute::MORPH_NORMAL_2:
      return "MORPH_NORMAL_2";
    case Gltf2Attribute::MORPH_NORMAL_3:
      return "MORPH_NORMAL_3";
    default:
      return "INVALID";
  }
}

absl::optional<Gltf2Attribute> GetGlTF2VertexAttribute(
    const std::string& attribute_name) {
  auto map = GetGlTF2VertexAttributeMap();
  auto it = map.find(attribute_name);
  return (it == map.end()) ? absl::optional<Gltf2Attribute>{}
                           : absl::optional<Gltf2Attribute>{it->second};
}

schemas::VertexAttribute GetVertexAttribute(Gltf2Attribute gltf2_attr) {
  switch (gltf2_attr) {
    default:
    case Gltf2Attribute::POSITION:
      return schemas::VertexAttribute::POSITION;
    case Gltf2Attribute::NORMAL:
      return schemas::VertexAttribute::TANGENTS;
    case Gltf2Attribute::TANGENT:
      return schemas::VertexAttribute::TANGENTS;
    case Gltf2Attribute::COLOR_0:
      return schemas::VertexAttribute::COLOR;
    case Gltf2Attribute::TEXCOORD_0:
      return schemas::VertexAttribute::UV0;
    case Gltf2Attribute::TEXCOORD_1:
      return schemas::VertexAttribute::UV1;
    case Gltf2Attribute::JOINTS_0:
      return schemas::VertexAttribute::BONE_INDICES;
    case Gltf2Attribute::WEIGHTS_0:
      return schemas::VertexAttribute::BONE_WEIGHTS;
    case Gltf2Attribute::MORPH_POSITION_0:
      return schemas::VertexAttribute::MORPH_POSITION_0;
    case Gltf2Attribute::MORPH_POSITION_1:
      return schemas::VertexAttribute::MORPH_POSITION_1;
    case Gltf2Attribute::MORPH_POSITION_2:
      return schemas::VertexAttribute::MORPH_POSITION_2;
    case Gltf2Attribute::MORPH_POSITION_3:
      return schemas::VertexAttribute::MORPH_POSITION_3;
    case Gltf2Attribute::MORPH_NORMAL_0:
    case Gltf2Attribute::MORPH_TANGENT_0:
      return schemas::VertexAttribute::MORPH_TANGENTS_0;
    case Gltf2Attribute::MORPH_NORMAL_1:
    case Gltf2Attribute::MORPH_TANGENT_1:
      return schemas::VertexAttribute::MORPH_TANGENTS_1;
    case Gltf2Attribute::MORPH_NORMAL_2:
    case Gltf2Attribute::MORPH_TANGENT_2:
      return schemas::VertexAttribute::MORPH_TANGENTS_2;
    case Gltf2Attribute::MORPH_NORMAL_3:
    case Gltf2Attribute::MORPH_TANGENT_3:
      return schemas::VertexAttribute::MORPH_TANGENTS_3;
  }
}

}  // namespace imp::loader::details::provider_gltf
