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

#include "core/proto/proto_reader.h"

#include <cstdint>
#include <cstring>
#include <string>

#include "absl/base/optimization.h"
#include "absl/strings/string_view.h"
#include "core/proto/proto_common.h"
#include "boost_beast/varint.hpp"

namespace imp {
namespace proto {

const char* ProtoReader::Unknown(const char* ptr, int field_id,
                                 int wire_type) const {
  uint64_t scratch = 0;
  switch (wire_type) {
    case WIRE_VARINT:
      return ReadVarint(ptr, &scratch);
    case WIRE_FIXED64:
      if (!ptr || (ptr + 8) > end_) return nullptr;
      return ptr + 8;
    case WIRE_DELIMITED:
      ptr = ReadVarint(ptr, &scratch);
      if (!ptr || scratch > (end_ - ptr)) return nullptr;
      return ptr += scratch;
    case WIRE_FIXED32:
      if (!ptr || (ptr + 4) > end_) return nullptr;
      return ptr + 4;
    default:
      break;
  }
  return nullptr;
}

const char* ProtoReader::ReadVarint(const char* ptr, uint32_t* varint) const {
  if (ABSL_PREDICT_FALSE(!ptr)) return nullptr;
  // We don't need to parse with limit if the buffer ends with a byte that
  // terminates a varint.
  if (ABSL_PREDICT_TRUE(ptr < end_) && !(end_[-1] & 0x80)) {
    *varint = boost::beast::detail::varint_read(ptr); return ptr;
  }
  *varint = boost::beast::detail::varint_read(ptr); return ptr;
}

const char* ProtoReader::ReadVarint(const char* ptr, uint64_t* varint) const {
  if (ABSL_PREDICT_FALSE(!ptr)) return nullptr;
  // We don't need to parse with limit if the buffer ends with a byte that
  // terminates a varint.
  if (ABSL_PREDICT_TRUE(ptr < end_) && !(end_[-1] & 0x80)) {
    *varint = boost::beast::detail::varint_read(ptr); return ptr;
  }
  *varint = boost::beast::detail::varint_read(ptr); return ptr;
}

const char* ProtoReader::ReadZigzag(const char* ptr, int32_t* zigzag) const {
  uint32_t varint = 0;
  ptr = ReadVarint(ptr, &varint);
  *zigzag = static_cast<int32_t>((varint >> 1) ^ (~(varint & 1) + 1));
  return ptr;
}

const char* ProtoReader::ReadZigzag(const char* ptr, int64_t* zigzag) const {
  uint64_t varint = 0;
  ptr = ReadVarint(ptr, &varint);
  *zigzag = static_cast<int64_t>((varint >> 1) ^ (~(varint & 1) + 1));
  return ptr;
}

const char* ProtoReader::Read32(const char* ptr, void* v32) const {
  if (ABSL_PREDICT_FALSE(!ptr || (ptr + 4) > end_)) return nullptr;
  memcpy(v32, ptr, 4);
  return ptr + 4;
}

const char* ProtoReader::Read64(const char* ptr, void* v64) const {
  if (ABSL_PREDICT_FALSE(!ptr || (ptr + 8) > end_)) return nullptr;
  memcpy(v64, ptr, 8);
  return ptr + 8;
}

template <int field_type, typename T>
const char* ProtoReader::ReadInt(const char* ptr, int field_id, T* field,
                                 int wire_type) {
  // We could just fail if wire_type != WireType<field_type>(), but doing so
  // means we can't ever change a field from one integer type to another,
  // even if they have the same runtime representation.  So to allow more
  // flexible schema evolution, we allow for some mismatch.
  if (wire_type == WireType<field_type>()) {
    static_assert(field_type == TYPE_INT32 || field_type == TYPE_UINT32 ||
                  field_type == TYPE_INT64 || field_type == TYPE_UINT64 ||
                  field_type == TYPE_SINT32 || field_type == TYPE_SINT64 ||
                  field_type == TYPE_FIXED32 || field_type == TYPE_SFIXED32 ||
                  field_type == TYPE_FIXED64 || field_type == TYPE_SFIXED64);

    if constexpr (field_type == TYPE_INT32 || field_type == TYPE_UINT32 ||
                  field_type == TYPE_INT64 || field_type == TYPE_UINT64) {
      uint64_t varint = 0;
      ptr = ReadVarint(ptr, &varint);
      *field = static_cast<T>(varint);
    } else if constexpr (field_type == TYPE_SINT32 ||
                         field_type == TYPE_SINT64) {
      ptr = ReadZigzag(ptr, field);
    } else if constexpr (field_type == TYPE_FIXED32 ||
                         field_type == TYPE_SFIXED32) {
      ptr = Read32(ptr, field);
    } else if constexpr (field_type == TYPE_FIXED64 ||
                         field_type == TYPE_SFIXED64) {
      ptr = Read64(ptr, field);
    }
  } else {
    ptr = Unknown(ptr, field_id, wire_type);
  }
  return ptr;
}

template <int field_type, typename T>
const char* ProtoReader::ReadFloat(const char* ptr, int field_id, T* field,
                                   int wire_type) {
  if (wire_type != WireType<field_type>()) {
    return Unknown(ptr, field_id, wire_type);
  }
  if constexpr (WireType<field_type>() == WIRE_FIXED64) {
    double d = 0.0;
    ptr = Read64(ptr, &d);
    *field = static_cast<T>(d);
  } else {
    static_assert(WireType<field_type>() == WIRE_FIXED32);
    float f = 0.0f;
    ptr = Read32(ptr, &f);
    *field = static_cast<T>(f);
  }
  return ptr;
}

size_t ProtoReader::RepeatedSize(const char* ptr, int field_id) const {
  uint64_t this_tag = field_id_ << 3 | wire_type_;
  size_t count = 0;
  while (true) {
    ++count;
    ptr = Unknown(ptr, field_id, wire_type_);
    uint64_t next_tag = 0;
    ptr = ReadVarint(ptr, &next_tag);
    if (next_tag != this_tag) break;
  }
  return count;
}

template <int field_type>
const char* ProtoReader::Visit(const char* ptr, int field_id, bool* field,
                               bool* other, int wire_type) {
  static_assert(field_type == TYPE_BOOL);
  if (wire_type != 0) {
    ptr_ = nullptr;  // Not a boolean, abort.
    return ptr_;
  }
  uint64_t tmp = 0;
  ptr_ = ReadVarint(ptr_, &tmp);
  *field = tmp != 0;
  return ptr_;
}

template const char* ProtoReader::Visit<TYPE_BOOL>(const char* ptr,
                                                   int field_id, bool* field,
                                                   bool* other, int wire_type);

template <int field_type>
const char* ProtoReader::Visit(const char* ptr, int field_id, int32_t* field,
                               int32_t* other, int wire_type) {
  ptr_ = ReadInt<field_type>(ptr_, field_id, field, wire_type);
  return ptr_;
}
template const char* ProtoReader::Visit<TYPE_INT32>(const char* ptr,
                                                    int field_id,
                                                    int32_t* field,
                                                    int32_t* other,
                                                    int wire_type);
template const char* ProtoReader::Visit<TYPE_SINT32>(const char* ptr,
                                                     int field_id,
                                                     int32_t* field,
                                                     int32_t* other,
                                                     int wire_type);
template const char* ProtoReader::Visit<TYPE_SFIXED32>(const char* ptr,
                                                       int field_id,
                                                       int32_t* field,
                                                       int32_t* other,
                                                       int wire_type);
template const char* ProtoReader::Visit<TYPE_UINT64>(const char* ptr,
                                                     int field_id,
                                                     int32_t* field,
                                                     int32_t* other,
                                                     int wire_type);

template <int field_type>
const char* ProtoReader::Visit(const char* ptr, int field_id, uint32_t* field,
                               uint32_t* other, int wire_type) {
  ptr_ = ReadInt<field_type>(ptr_, field_id, field, wire_type);
  return ptr_;
}
template const char* ProtoReader::Visit<TYPE_UINT32>(const char* ptr,
                                                     int field_id,
                                                     uint32_t* field,
                                                     uint32_t* other,
                                                     int wire_type);
template const char* ProtoReader::Visit<TYPE_FIXED32>(const char* ptr,
                                                      int field_id,
                                                      uint32_t* field,
                                                      uint32_t* other,
                                                      int wire_type);

template <int field_type>
const char* ProtoReader::Visit(const char* ptr, int field_id, int64_t* field,
                               int64_t* other, int wire_type) {
  ptr_ = ReadInt<field_type>(ptr_, field_id, field, wire_type);
  return ptr_;
}
template const char* ProtoReader::Visit<TYPE_INT64>(const char* ptr,
                                                    int field_id,
                                                    int64_t* field,
                                                    int64_t* other,
                                                    int wire_type);
template const char* ProtoReader::Visit<TYPE_SINT64>(const char* ptr,
                                                     int field_id,
                                                     int64_t* field,
                                                     int64_t* other,
                                                     int wire_type);
template const char* ProtoReader::Visit<TYPE_SFIXED64>(const char* ptr,
                                                       int field_id,
                                                       int64_t* field,
                                                       int64_t* other,
                                                       int wire_type);

template <int field_type>
const char* ProtoReader::Visit(const char* ptr, int field_id, uint64_t* field,
                               uint64_t* other, int wire_type) {
  ptr_ = ReadInt<field_type>(ptr_, field_id, field, wire_type);
  return ptr_;
}
template const char* ProtoReader::Visit<TYPE_UINT64>(const char* ptr,
                                                     int field_id,
                                                     uint64_t* field,
                                                     uint64_t* other,
                                                     int wire_type);
template const char* ProtoReader::Visit<TYPE_FIXED64>(const char* ptr,
                                                      int field_id,
                                                      uint64_t* field,
                                                      uint64_t* other,
                                                      int wire_type);

template <int field_type>
const char* ProtoReader::Visit(const char* ptr, int field_id, float* field,
                               float* other, int wire_type) {
  ptr_ = ReadFloat<field_type>(ptr_, field_id, field, wire_type);
  return ptr_;
}
template const char* ProtoReader::Visit<TYPE_FLOAT>(const char* ptr,
                                                    int field_id, float* field,
                                                    float* other,
                                                    int wire_type);

template <int field_type>
const char* ProtoReader::Visit(const char* ptr, int field_id, double* field,
                               double* other, int wire_type) {
  ptr_ = ReadFloat<field_type>(ptr_, field_id, field, wire_type);
  return ptr_;
}
template const char* ProtoReader::Visit<TYPE_DOUBLE>(
    const char* ptr, int field_id, double* field, double* other, int wire_type);

template <int field_type>
const char* ProtoReader::Visit(const char* ptr, int field_id,
                               std::string* field, std::string* other,
                               int wire_type) {
  if (wire_type != WireType<field_type>()) {
    ptr_ = nullptr;
    return ptr_;
  }
  uint64_t size = 0;
  ptr_ = ReadVarint(ptr_, &size);
  if (!ptr_ || size > end_ - ptr_) {
    ptr_ = nullptr;
    return ptr_;
  }
  *field = std::string(ptr_, size);
  ptr_ += size;
  return ptr_;
}
template const char* ProtoReader::Visit<TYPE_STRING>(const char* ptr,
                                                     int field_id,
                                                     std::string* field,
                                                     std::string* other,
                                                     int wire_type);
template const char* ProtoReader::Visit<TYPE_BYTES>(const char* ptr,
                                                    int field_id,
                                                    std::string* field,
                                                    std::string* other,
                                                    int wire_type);

template <int field_type>
const char* ProtoReader::Visit(const char* ptr, int field_id,
                               absl::string_view* field,
                               absl::string_view* other, int wire_type) {
  if (wire_type != WireType<field_type>()) {
    ptr_ = nullptr;
    return ptr_;
  }
  uint64_t size = 0;
  ptr_ = ReadVarint(ptr_, &size);
  if (!ptr_ || size > end_ - ptr_) {
    ptr_ = nullptr;
    return ptr_;
  }
  *field = absl::string_view(ptr_, size);
  ptr_ += size;
  return ptr_;
}
template const char* ProtoReader::Visit<TYPE_STRING>(const char* ptr,
                                                     int field_id,
                                                     absl::string_view* field,
                                                     absl::string_view* other,
                                                     int wire_type);
template const char* ProtoReader::Visit<TYPE_BYTES>(const char* ptr,
                                                    int field_id,
                                                    absl::string_view* field,
                                                    absl::string_view* other,
                                                    int wire_type);

}  // namespace proto
}  // namespace imp
