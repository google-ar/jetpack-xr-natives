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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_COMMON_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_COMMON_H_

#include <cmath>
#include <cstdint>
#include <string>
#include <type_traits>
#include <utility>

#include "absl/strings/string_view.h"
#include "core/common/copyable_ptr.h"  // IWYU pragma: keep
#include "core/common/hash.h"
#include "core/common/type_traits.h"
#include "core/proto/proto_traits.h"

namespace imp {
namespace proto {

enum FieldType {
  // This should match FieldDescriptor::Type in descriptor.h
  TYPE_DOUBLE = 1,
  TYPE_FLOAT = 2,
  TYPE_INT64 = 3,
  TYPE_UINT64 = 4,
  TYPE_INT32 = 5,
  TYPE_FIXED64 = 6,
  TYPE_FIXED32 = 7,
  TYPE_BOOL = 8,
  TYPE_STRING = 9,
  TYPE_GROUP_DEPRECATED = 10,
  TYPE_MESSAGE = 11,
  TYPE_BYTES = 12,
  TYPE_UINT32 = 13,
  TYPE_ENUM = 14,
  TYPE_SFIXED32 = 15,
  TYPE_SFIXED64 = 16,
  TYPE_SINT32 = 17,
  TYPE_SINT64 = 18,
};

enum WireFormat {
  WIRE_VARINT = 0,
  WIRE_FIXED64 = 1,
  WIRE_DELIMITED = 2,
  WIRE_FIXED32 = 5,
};

// See imp.proto for details.
enum class RepeatedMergeStrategy : uint8_t {
  kPerElement = 0,
  kOverwrite = 1,
  kConcat = 2,
};

template <int field_type>
constexpr int WireType() {
  if constexpr (field_type == TYPE_MESSAGE || field_type == TYPE_STRING ||
                field_type == TYPE_BYTES) {
    return WIRE_DELIMITED;
  } else if constexpr (field_type == TYPE_FIXED64 ||
                       field_type == TYPE_SFIXED64 ||
                       field_type == TYPE_DOUBLE) {
    return WIRE_FIXED64;
  } else if constexpr (field_type == TYPE_FIXED32 ||
                       field_type == TYPE_SFIXED32 ||
                       field_type == TYPE_FLOAT) {
    return WIRE_FIXED32;
  } else {
    return WIRE_VARINT;
  }
}

template <typename T>
constexpr proto::FieldType GetFieldType();

template <>
constexpr proto::FieldType GetFieldType<bool>() {
  return proto::TYPE_BOOL;
}

template <>
constexpr proto::FieldType GetFieldType<int>() {
  return proto::TYPE_SINT32;
}

template <>
constexpr proto::FieldType GetFieldType<uint32_t>() {
  return proto::TYPE_UINT32;
}

template <>
constexpr proto::FieldType GetFieldType<float>() {
  return proto::TYPE_FLOAT;
}

template <>
constexpr proto::FieldType GetFieldType<absl::string_view>() {
  return proto::TYPE_STRING;
}

template <>
constexpr proto::FieldType GetFieldType<std::string>() {
  return proto::TYPE_STRING;
}

// Helper for getting the kTypeUrlHash defined for a type if it exists, and if
// it doesn't falling back on type_traits::kTypeHash<T>.
template <typename T, typename = int>
struct ProtoTypeUrlHash {
  static constexpr HashValue value = type_traits::kTypeHash<T>;
};

template <typename T>
struct ProtoTypeUrlHash<T, decltype((void)T::kTypeUrlHash, 0)> {
  static constexpr HashValue value = T::kTypeUrlHash;
};

constexpr bool IsPacked(int field_type) {
  return (field_type != TYPE_MESSAGE && field_type != TYPE_STRING &&
          field_type != TYPE_GROUP_DEPRECATED && field_type != TYPE_BYTES);
}

// Function to reset a Message.
template <typename Message>
void Reset(Message* m) {
  // invoke the destructor then constructor to reset the message.
  m->~Message();
  new (m) Message();
}

// If providing custom visitors, override this class.
template <typename Message>
struct ProtoMessage : public Message {
  template <typename Visitor, typename Cursor, typename... Args>
  Cursor Visit(Visitor& visitor, Cursor cursor, Message* other,
               Args&&... args) {
    return visitor.Unknown(cursor, 0, std::forward<Args>(args)...);
  }
  template <typename Visitor, typename Cursor, typename... Args>
  Cursor VisitField(int field_id, Visitor& visitor, Cursor cursor,
                    const Message* other, Args... args) const {
    return visitor.Unknown(cursor, field_id, std::forward<Args>(args)...);
  }
};

// Visit all of the fields of the given message including a base message.
// The given Visitor's Visit(Cursor c, T* field, const T* base) methods
// will be invoked for every field on the message, recursively.
// If the base param is non-null, that Message will be "plumbed-in-parallel",
// i.e. every Visit(...) method will be called with both the field from the
// main message and the matching field of the base message, all the way through
// the message structure.
template <typename Message, typename Visitor, typename Cursor, typename... Args>
Cursor VisitPaired(Message* message, Visitor* visitor, Cursor cursor,
                   Message* other, Args&&... args) {
  if constexpr (proto_traits::kHasVisitFunction<Message, Visitor, Cursor>) {
    // Specialization for Messages that implement Visit.
    return message->Visit(*visitor, cursor, other, std::forward<Args>(args)...);
  } else {
    // Specialization for cases where custom visitors have been authored.
    return static_cast<ProtoMessage<Message>*>(
               const_cast<std::decay_t<Message>*>(message))
        ->Visit(*visitor, cursor, other, std::forward<Args>(args)...);
  }
}

// Visit a specific field of the given message including a base message.
// The given Visitor's Visit(Cursor c, T* field, const T* base, ...) method
// will be invoked the field specified by field_id message with the same field
// on the base message, if non-null. The given Args will be forwarded.
template <typename Message, typename Visitor, typename Cursor, typename... Args>
Cursor VisitFieldPaired(Message* message, int field_id, Visitor* visitor,
                        Cursor cursor, Message* other, Args... args) {
  if constexpr (proto_traits::kHasVisitFieldFunction<Message, Visitor,
                                                     Cursor>) {
    // Specialization for Messages that implement Visit.
    return message->VisitField(field_id, *visitor, cursor, other,
                               std::forward<Args>(args)...);
  } else {
    // Specialization for cases where custom visitors have been authored.
    return static_cast<ProtoMessage<Message>*>(message)->VisitField(
        field_id, *visitor, cursor, other, std::forward<Args>(args)...);
  }
}

// Visit all of the fields of the given message including a base message.
// The given Visitor's Visit(Cursor c, T* field, const T* base) methods
// will be invoked for every field on the message, recursively. The base param
// will always be null when calling this version (see VisitWithBase above).
template <typename Message, typename Visitor, typename Cursor, typename... Args>
Cursor Visit(Message* message, Visitor* visitor, Cursor cursor, Args... args) {
  return VisitPaired<Message, Visitor, Cursor, Args...>(
      message, visitor, cursor, static_cast<Message*>(nullptr),
      std::forward<Args>(args)...);
}

// Visit a specific field of the given message including a base message.
// The given Visitor's Visit(Cursor c, T* field, const T* base, ...) method
// will be invoked the field specified by field_id message with a nullptr base
// param (see VisitFieldWithBase above), if non-null. The given Args will be
// forwarded.
template <typename Message, typename Visitor, typename Cursor, typename... Args>
Cursor VisitField(Message* message, int field_id, Visitor* visitor,
                  Cursor cursor, Args... args) {
  return VisitFieldPaired(message, field_id, visitor, cursor,
                          static_cast<Message*>(nullptr),
                          std::forward<Args>(args)...);
}

template <typename T, typename = int>
struct HasFields : std::false_type {};

template <typename T>
struct HasFields<T, decltype(T::kFieldsCount, 0)> : std::true_type {};

// Finds the type that contains the fields information for a given proto T.
// If the type was generated, then it's just T. If it's a proto for a native
// type, it's ProtoMessage<T>.
template <typename T>
using FieldsT = std::conditional_t<HasFields<T>::value, T, ProtoMessage<T>>;

template <typename T>
int GetFieldIndex(int field_id) {
  using Fields = FieldsT<T>;
  for (int i = 0; i < Fields::kFieldsCount; i++) {
    if (Fields::kFieldIds[i] == field_id) {
      return i;
    }
  }
  return -1;
}

template <typename T>
absl::string_view GetFieldJsonName(int field_id) {
  using Fields = FieldsT<T>;
  int field_index = GetFieldIndex<T>(field_id);
  if (field_index >= 0) {
    return Fields::kFieldJsonNames[field_index];
  }
  return "";
}

template <typename T>
absl::string_view GetFieldName(int field_id) {
  using Fields = FieldsT<T>;
  int field_index = GetFieldIndex<T>(field_id);
  if (field_index >= 0) {
    return Fields::kFieldNames[field_index];
  }
  return "";
}

// Declaration for EnumMetaData template.
// Generated impress protos create template specializations of this type for
// each enum in the proto to provide information about the enum.
template <typename E>
struct EnumMetaData;

// Default version of AbslStringifyProto that just prints the type url. This is
// used by any.proto to provide basic output. This is needed because
// proto_stringify transitively depends on any.proto.
//
// This is an implementation detail of the generated code and should not be used
// directly. Instead just use AbslStringify (via LOG, absl::StrCat, etc).
template <typename Sink, typename T, typename... Ts>
void AbslStringifyProto(Sink& sink, const T& message, Ts...) {
  sink.Append(T::kTypeUrl);
}

}  // namespace proto
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_COMMON_H_
