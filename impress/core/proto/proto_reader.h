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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_READER_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_READER_H_

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/copyable_ptr.h"
#include "core/common/platform_helpers.h"
#include "core/common/robin_map.h"
#include "core/common/type_traits.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/parse_message_visitor.h"
#include "core/proto/proto_common.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp {

namespace proto {

class ProtoReader {
 public:
  explicit ProtoReader(absl::string_view data,
                       ParseMessageVisitor* parse_message_visitor = nullptr)
      : ptr_(data.data()),
        end_(data.data() + data.size()),
        parse_message_visitor_(parse_message_visitor) {}

  ProtoReader(const ProtoReader& parent, const char* const end)
      : ptr_(parent.ptr_),
        end_(end),
        parse_message_visitor_(parent.parse_message_visitor_) {}

  bool Done() const { return !ptr_ || ptr_ >= end_; }

  std::pair<int, int> ReadTag() {
    uint64_t varint = 0;
    ptr_ = ReadVarint(ptr_, &varint);
    return std::pair<int, int>(varint >> 3, varint & 7);
  }

  const char* Unknown(const char* ptr, int field_id, int wire_type) const;

  template <typename T>
  const char* ParseMsg(T* msg);

  template <int field_type, typename T>
  const char* Visit(const char* ptr, int field_id, T* field, T* other,
                    int wire_type);

  template <int field_type, typename T>
  const char* Visit(const char* ptr, int field_id, absl::optional<T>* field,
                    absl::optional<T>* other, int wire_type);

  template <int field_type, typename T>
  const char* Visit(const char* ptr, int field_id, CopyablePtr<T>* field,
                    CopyablePtr<T>* other, int wire_type);

  template <int field_type, RepeatedMergeStrategy merge_type, typename T>
  const char* Visit(const char* ptr, int field_id, std::vector<T>* field,
                    std::vector<T>* other, int wire_type);

  template <int key_type, int value_type, typename K, typename V>
  const char* Visit(const char* ptr, int field_id, std::map<K, V>* field,
                    std::map<K, V>* other, int wire_type);

  template <int field_type>
  const char* Visit(const char* ptr, int field_id, bool* field, bool* other,
                    int wire_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, int32_t* field,
                    int32_t* other, int wire_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, uint32_t* field,
                    uint32_t* other, int wire_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, int64_t* field,
                    int64_t* other, int wire_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, uint64_t* field,
                    uint64_t* other, int wire_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, float* field, float* other,
                    int wire_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, double* field, double* other,
                    int wire_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, std::string* field,
                    std::string* other, int wire_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, absl::string_view* field,
                    absl::string_view* other, int wire_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, absl::Cord* field,
                    absl::Cord* other, int wire_type);

  template <typename Proto>
  const char* VisitStandardProto(const char* ptr, int field_id, Proto* proto,
                                 Proto* other, int wire_type);

 private:
  const char* ReadVarint(const char* ptr, uint32_t* varint) const;
  const char* ReadVarint(const char* ptr, uint64_t* varint) const;
  const char* ReadZigzag(const char* ptr, int32_t* zigzag) const;
  const char* ReadZigzag(const char* ptr, int64_t* zigzag) const;
  const char* Read32(const char* ptr, void* v32) const;
  const char* Read64(const char* ptr, void* v64) const;

  template <int field_type, typename T>
  const char* ReadInt(const char* ptr, int field_id, T* field, int wire_type);

  template <int field_type, typename T>
  const char* ReadFloat(const char* ptr, int field_id, T* field, int wire_type);

  size_t RepeatedSize(const char* ptr, int field_id) const;

  const char* ptr_;
  const char* const end_;
  int field_id_;
  int wire_type_;
  ParseMessageVisitor* parse_message_visitor_;

  // Tracks information about repeated fields that we've encountered so that
  // they can be merged correctly.
  RobinMap<int, size_t> repeated_field_ids_to_index_;
};

template <typename T>
const char* ProtoReader::ParseMsg(T* msg) {
  while (!Done()) {
    std::tie(field_id_, wire_type_) = ReadTag();
    // 0 is never a valid field id, but is returned by ReadTag if we read past
    // the end of the stream.  So if we see zero, just return.
    // On an invalid stream, we may see zero before the end of the stream,
    // but it's still OK to abort the parse at that point.
    if (field_id_ == 0) return ptr_;
    ::imp::proto::VisitField(msg, field_id_, this, ptr_, wire_type_);
  }
  return ptr_;
}

template <int field_type, typename T>
const char* ProtoReader::Visit(const char* ptr, int field_id, T* field,
                               T* other, int wire_type) {
  if constexpr (std::is_convertible<T, int32_t>::value) {
    // This is an enum.
    static_assert(field_type == TYPE_ENUM);
    uint32_t value;
    ptr_ = ReadVarint(ptr_, &value);
    if (ptr_) {
      *field = static_cast<T>(value);
    }
  } else {
    static_assert(field_type == TYPE_MESSAGE);
    uint64_t size = 0;
    ptr_ = ReadVarint(ptr_, &size);
    if (!ptr_ || size > static_cast<uint64_t>(end_ - ptr_)) {
      ptr_ = nullptr;
      return ptr_;
    }
    const char* end = ptr_ + size;
    ProtoReader sub(*this, end);
    ptr_ = sub.ParseMsg(field);

    if (parse_message_visitor_ != nullptr) {
      parse_message_visitor_->Accept(ProtoTypeUrlHash<T>::value, field);
    }
  }
  return ptr_;
}

template <int field_type, typename T>
const char* ProtoReader::Visit(const char* ptr, int field_id,
                               absl::optional<T>* field,
                               absl::optional<T>* other, int wire_type) {
  if (!field->has_value()) {
    field->emplace(T());
  }
  return Visit<field_type>(ptr, field_id, &(**field), static_cast<T*>(nullptr),
                           wire_type);
}

template <int field_type, typename T>
const char* ProtoReader::Visit(const char* ptr, int field_id,
                               CopyablePtr<T>* field, CopyablePtr<T>* other,
                               int wire_type) {
  if (!*field) {
    *field = MakeCopyablePtr<T>();
  }
  return Visit<field_type>(ptr, field_id, field->get(),
                           static_cast<T*>(nullptr), wire_type);
}

template <int field_type, RepeatedMergeStrategy merge_type, typename T>
const char* ProtoReader::Visit(const char* ptr, int field_id,
                               std::vector<T>* field, std::vector<T>* other,
                               int wire_type) {
  auto pair = repeated_field_ids_to_index_.try_emplace(field_id, 0);
  size_t& index = pair.first.value();
  if (pair.second) {
    if constexpr (merge_type == RepeatedMergeStrategy::kConcat) {
      index = field->size();
    } else if constexpr (merge_type == RepeatedMergeStrategy::kOverwrite) {
      field->clear();
    }
  }

  // If the field is of a type that *can* be encoded as packed, check to see if
  // it currently is packed. If so, decode it as packed.
  if constexpr (WireType<field_type>() != 2) {
    // per element only supported for messages.
    static_assert(merge_type != RepeatedMergeStrategy::kPerElement);
    if (wire_type == 2) {
      uint64_t size = 0;
      ptr_ = ReadVarint(ptr_, &size);
      if (!ptr_ || size > static_cast<uint64_t>(end_ - ptr_)) {
        ptr_ = nullptr;
        return ptr_;
      }
      const char* end = ptr_ + size;
      ProtoReader packed(*this, end);
      constexpr int kFieldWireType = WireType<field_type>();

      if constexpr (WireType<field_type>() == 1) {
        // packed 8 byte values, reserve space in the vector.
        field->reserve(index + size / 8);
      } else if constexpr (WireType<field_type>() == 5) {
        // packed 4 byte values, reserve space in the vector
        field->reserve(index + size / 4);
      }

      while (!packed.Done()) {
        T* field_element;
        if (index < field->size()) {
          field_element = &(*field)[index];
        } else {
          field_element = &field->emplace_back();
        }
        packed.Visit<field_type>(ptr, 0, field_element,
                                 static_cast<T*>(nullptr), kFieldWireType);
        ++index;
      }
      ptr_ = packed.ptr_;
      if (ptr_ != end) ptr_ = nullptr;
    }
  }

  // Repeated field is not packed, we'll encounter values one at a time.
  if (wire_type != 2 || WireType<field_type>() == 2) {
    if (pair.second) {
      // If this is the first time we're adding to the vector, assume this is
      // a well formed proto and try to reserve exactly the capacity we need.
      field->reserve(index + this->RepeatedSize(ptr_, field_id));
    }
    T* field_element;
    if (index < field->size()) {
      field_element = &(*field)[index];
    } else {
      field_element = &field->emplace_back();
    }
    Visit<field_type>(ptr, 0, field_element, static_cast<T*>(nullptr),
                      wire_type);
    ++index;
  }

  return ptr_;
}

template <int key_type, int value_type, typename K, typename V>
const char* ProtoReader::Visit(const char* ptr, int field_id,
                               std::map<K, V>* field, std::map<K, V>* other,
                               int wire_type) {
  constexpr int kKeyId = 1;
  constexpr int kValueId = 2;
  uint64_t size = 0;
  ptr_ = ReadVarint(ptr_, &size);
  if (!ptr_ || size > static_cast<uint64_t>(end_ - ptr_)) {
    ptr_ = nullptr;
    return ptr_;
  }
  const char* end = ptr_ + size;
  int id;
  K key = K();
  V value = V();
  ProtoReader entry(*this, end);
  while (!entry.Done()) {
    std::tie(id, wire_type) = entry.ReadTag();
    if (id == kKeyId) {
      entry.Visit<key_type>(ptr, 0, &key, static_cast<K*>(nullptr), wire_type);
    } else if (id == kValueId) {
      entry.Visit<value_type>(ptr_, 0, &value, static_cast<V*>(nullptr),
                              wire_type);
    } else {
      ptr_ = entry.Unknown(ptr_, id, wire_type);
    }
  }
  ptr_ = entry.ptr_;
  if (ptr_ <= end) {
    field->insert_or_assign(std::move(key), std::move(value));
  }
  if (ptr_ != end) ptr_ = nullptr;
  return ptr_;
}

// The absl::Cord visitor is left as a template so that we don't expand
// it if it's not required.
template <int field_type>
const char* ProtoReader::Visit(const char* ptr, int field_id, absl::Cord* field,
                               absl::Cord* other, int wire_type) {
  if (wire_type != WireType<field_type>()) {
    ptr_ = nullptr;
    return ptr_;
  }
  uint64_t size = 0;
  ptr_ = ReadVarint(ptr_, &size);
  if (!ptr_ || size > static_cast<uint64_t>(end_ - ptr_)) {
    ptr_ = nullptr;
    return ptr_;
  }
  field->Append(
      absl::MakeCordFromExternal(absl::string_view(ptr_, size), [] {}));
  ptr_ += size;
  return ptr_;
}

template <typename Proto>
const char* ProtoReader::VisitStandardProto(const char* ptr, int field_id,
                                            Proto* proto, Proto* other,
                                            int wire_type) {
  uint64_t size = 0;
  ptr_ = ReadVarint(ptr_, &size);
  if (!ptr_ || size > static_cast<uint64_t>(end_ - ptr_)) {
    ptr_ = nullptr;
    return ptr_;
  }

  if (proto->ParseFromArray(ptr_, size)) {
    ptr_ += size;
  } else {
    ptr_ = nullptr;
    return ptr_;
  }
  return ptr_;
}

template <typename T>
bool ParseMessage(absl::string_view data, T* msg,
                  ParseMessageVisitor* parse_message_visitor = nullptr) {
  ProtoReader in(data, parse_message_visitor);
  return in.ParseMsg(msg) != nullptr;
}

template <typename T>
bool ParseMessage(absl::Cord data, T* msg,
                  ParseMessageVisitor* parse_message_visitor = nullptr) {
  return ParseMessage(data.Flatten(), msg, parse_message_visitor);
}

// Unpacks an any into the out parameter of type T passed in.
template <typename T>
absl::Status UnpackAny(const google::protobuf::imp_proto::Any& any,
                       T* out_message) {
  if (out_message == nullptr) {
    return absl::InvalidArgumentError("out_message is nullptr");
  }

  if (any.type_url != T::kTypeUrl) {
    return absl::InvalidArgumentError(
        absl::StrFormat("Can't unpack any of type %s into message of type %s",
                        any.type_url, T::kTypeUrl));
  }

  if (!ParseMessage(any.value, out_message)) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Unable to unpack any of type %s into message of type %s", any.type_url,
        T::kTypeUrl));
  }

  return absl::OkStatus();
}

// Unpacks an any into proto message of type T.
template <typename T>
absl::StatusOr<T> UnpackAny(const google::protobuf::imp_proto::Any& any) {
  T result;
  MP_RETURN_IF_ERROR(UnpackAny(any, &result));
  return result;
}

}  // namespace proto

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_TOOLS_PROTO_READER_H_
