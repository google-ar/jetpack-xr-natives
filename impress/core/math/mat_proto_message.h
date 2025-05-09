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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATH_MAT_PROTO_MESSAGE_H_
#define THIRD_PARTY_IMPRESS_CORE_MATH_MAT_PROTO_MESSAGE_H_

#include <cstddef>

#include "absl/strings/string_view.h"
#include "core/common/hash.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/proto/proto_common.h"

// Visitors for ::imp::mat3f|mat3|mat4f|mat4
// Custom code to serialize/deserialize mat3f|mat3|mat4f|mat4 directly to/from
// a proto. Due to the way the matrices store data, codegen cannot
// auto-generate this code, so instead it's written manually.
namespace imp {

template <>
struct proto::ProtoMessage<mat3f> : public mat3f {
  static constexpr std::size_t kFieldsCount = 3;
  static constexpr int kFieldIds[] = {1, 2, 3};
  static constexpr absl::string_view kFieldEditorControlTypes[] = {"{}", "{}",
                                                                   "{}"};
  static constexpr absl::string_view kFieldNames[] = {"m0", "m1", "m2"};
  static constexpr imp::HashValue kFieldNameHashes[] = {
      imp::Hash(kFieldNames[0]),
      imp::Hash(kFieldNames[1]),
      imp::Hash(kFieldNames[2]),
  };
  static constexpr absl::string_view kFieldJsonNames[] = {"m0", "m1", "m2"};
  static constexpr imp::HashValue kFieldJsonNameHashes[] = {
      imp::Hash(kFieldJsonNames[0]),
      imp::Hash(kFieldJsonNames[1]),
      imp::Hash(kFieldJsonNames[2]),
  };

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor cursor, mat3f* other, Args... args) {
    auto& self = *this;
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 1, &self[0],
        other ? &(*other)[0] : static_cast<float3*>(nullptr),
        std::forward<Args>(args)...);
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 2, &self[1],
        other ? &(*other)[1] : static_cast<float3*>(nullptr),
        std::forward<Args>(args)...);
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 3, &self[2],
        other ? &(*other)[2] : static_cast<float3*>(nullptr),
        std::forward<Args>(args)...);
    return cursor;
  }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor cursor, mat3f* other,
                    Args... args) {
    auto& self = *this;

    switch (field_id) {
      case 1:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 1, &self[0],
            other ? &(*other)[0] : static_cast<float3*>(nullptr),
            std::forward<Args>(args)...);
      case 2:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 2, &self[1],
            other ? &(*other)[1] : static_cast<float3*>(nullptr),
            std::forward<Args>(args)...);
      case 3:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 3, &self[2],
            other ? &(*other)[2] : static_cast<float3*>(nullptr),
            std::forward<Args>(args)...);
        break;
      default:
        return v.Unknown(cursor, field_id, std::forward<Args>(args)...);
        break;
    }
  }
};

template <>
struct proto::ProtoMessage<mat3> : public mat3 {
  static constexpr std::size_t kFieldsCount = 3;
  static constexpr int kFieldIds[] = {1, 2, 3};
  static constexpr absl::string_view kFieldEditorControlTypes[] = {"{}", "{}",
                                                                   "{}"};
  static constexpr absl::string_view kFieldNames[] = {"m0", "m1", "m2"};
  static constexpr imp::HashValue kFieldNameHashes[] = {
      imp::Hash(kFieldNames[0]),
      imp::Hash(kFieldNames[1]),
      imp::Hash(kFieldNames[2]),
  };
  static constexpr absl::string_view kFieldJsonNames[] = {"m0", "m1", "m2"};
  static constexpr imp::HashValue kFieldJsonNameHashes[] = {
      imp::Hash(kFieldJsonNames[0]),
      imp::Hash(kFieldJsonNames[1]),
      imp::Hash(kFieldJsonNames[2]),
  };

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor cursor, mat3* other, Args... args) {
    auto& self = *this;
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 1, &self[0],
        other ? &(*other)[0] : static_cast<double3*>(nullptr),
        std::forward<Args>(args)...);
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 2, &self[1],
        other ? &(*other)[1] : static_cast<double3*>(nullptr),
        std::forward<Args>(args)...);
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 3, &self[2],
        other ? &(*other)[2] : static_cast<double3*>(nullptr),
        std::forward<Args>(args)...);
    return cursor;
  }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor cursor, mat3* other,
                    Args... args) {
    auto& self = *this;

    switch (field_id) {
      case 1:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 1, &self[0],
            other ? &(*other)[0] : static_cast<double3*>(nullptr),
            std::forward<Args>(args)...);
      case 2:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 2, &self[1],
            other ? &(*other)[1] : static_cast<double3*>(nullptr),
            std::forward<Args>(args)...);
      case 3:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 3, &self[2],
            other ? &(*other)[2] : static_cast<double3*>(nullptr),
            std::forward<Args>(args)...);
        break;
      default:
        return v.Unknown(cursor, field_id, std::forward<Args>(args)...);
        break;
    }
  }
};

template <>
struct proto::ProtoMessage<mat4f> : public mat4f {
  static constexpr std::size_t kFieldsCount = 4;
  static constexpr int kFieldIds[] = {1, 2, 3, 4};
  static constexpr absl::string_view kFieldEditorControlTypes[] = {"{}", "{}",
                                                                   "{}", "{}"};
  static constexpr absl::string_view kFieldNames[] = {"m0", "m1", "m2", "m3"};
  static constexpr imp::HashValue kFieldNameHashes[] = {
      imp::Hash(kFieldNames[0]),
      imp::Hash(kFieldNames[1]),
      imp::Hash(kFieldNames[2]),
      imp::Hash(kFieldNames[3]),
  };
  static constexpr absl::string_view kFieldJsonNames[] = {"m0", "m1", "m2",
                                                          "m3"};
  static constexpr imp::HashValue kFieldJsonNameHashes[] = {
      imp::Hash(kFieldJsonNames[0]),
      imp::Hash(kFieldJsonNames[1]),
      imp::Hash(kFieldJsonNames[2]),
      imp::Hash(kFieldJsonNames[3]),
  };

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor cursor, mat4f* other, Args... args) {
    auto& self = *this;
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 1, &self[0],
        other ? &(*other)[0] : static_cast<float4*>(nullptr),
        std::forward<Args>(args)...);
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 2, &self[1],
        other ? &(*other)[1] : static_cast<float4*>(nullptr),
        std::forward<Args>(args)...);
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 3, &self[2],
        other ? &(*other)[2] : static_cast<float4*>(nullptr),
        std::forward<Args>(args)...);
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 4, &self[3],
        other ? &(*other)[3] : static_cast<float4*>(nullptr),
        std::forward<Args>(args)...);
    return cursor;
  }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor cursor, mat4f* other,
                    Args... args) {
    auto& self = *this;

    switch (field_id) {
      case 1:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 1, &self[0],
            other ? &(*other)[0] : static_cast<float4*>(nullptr),
            std::forward<Args>(args)...);
      case 2:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 2, &self[1],
            other ? &(*other)[1] : static_cast<float4*>(nullptr),
            std::forward<Args>(args)...);
      case 3:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 3, &self[2],
            other ? &(*other)[2] : static_cast<float4*>(nullptr),
            std::forward<Args>(args)...);
        break;
      case 4:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 4, &self[3],
            other ? &(*other)[3] : static_cast<float4*>(nullptr),
            std::forward<Args>(args)...);
        break;
      default:
        return v.Unknown(cursor, field_id, std::forward<Args>(args)...);
        break;
    }
  }
};

template <>
struct proto::ProtoMessage<mat4> : public mat4 {
  static constexpr std::size_t kFieldsCount = 4;
  static constexpr int kFieldIds[] = {1, 2, 3, 4};
  static constexpr absl::string_view kFieldEditorControlTypes[] = {"{}", "{}",
                                                                   "{}", "{}"};
  static constexpr absl::string_view kFieldNames[] = {"m0", "m1", "m2", "m3"};
  static constexpr imp::HashValue kFieldNameHashes[] = {
      imp::Hash(kFieldNames[0]),
      imp::Hash(kFieldNames[1]),
      imp::Hash(kFieldNames[2]),
      imp::Hash(kFieldNames[3]),
  };
  static constexpr absl::string_view kFieldJsonNames[] = {"m0", "m1", "m2",
                                                          "m3"};
  static constexpr imp::HashValue kFieldJsonNameHashes[] = {
      imp::Hash(kFieldJsonNames[0]),
      imp::Hash(kFieldJsonNames[1]),
      imp::Hash(kFieldJsonNames[2]),
      imp::Hash(kFieldJsonNames[3]),
  };

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& v, Cursor cursor, mat4* other, Args... args) {
    auto& self = *this;
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 1, &self[0],
        other ? &(*other)[0] : static_cast<double4*>(nullptr),
        std::forward<Args>(args)...);
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 2, &self[1],
        other ? &(*other)[1] : static_cast<double4*>(nullptr),
        std::forward<Args>(args)...);
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 3, &self[2],
        other ? &(*other)[2] : static_cast<double4*>(nullptr),
        std::forward<Args>(args)...);
    cursor = v.template Visit<TYPE_MESSAGE>(
        cursor, 4, &self[3],
        other ? &(*other)[3] : static_cast<double4*>(nullptr),
        std::forward<Args>(args)...);
    return cursor;
  }

  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& v, Cursor cursor, mat4* other,
                    Args... args) {
    auto& self = *this;

    switch (field_id) {
      case 1:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 1, &self[0],
            other ? &(*other)[0] : static_cast<double4*>(nullptr),
            std::forward<Args>(args)...);
      case 2:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 2, &self[1],
            other ? &(*other)[1] : static_cast<double4*>(nullptr),
            std::forward<Args>(args)...);
      case 3:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 3, &self[2],
            other ? &(*other)[2] : static_cast<double4*>(nullptr),
            std::forward<Args>(args)...);
      case 4:
        return v.template Visit<TYPE_MESSAGE>(
            cursor, 4, &self[3],
            other ? &(*other)[3] : static_cast<double4*>(nullptr),
            std::forward<Args>(args)...);
      default:
        return v.Unknown(cursor, field_id, std::forward<Args>(args)...);
    }
  }
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATH_MAT_PROTO_MESSAGE_H_
