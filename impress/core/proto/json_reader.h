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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_JSON_READER_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_JSON_READER_H_

#include <stdbool.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "absl/strings/escaping.h"
#include "absl/strings/numbers.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/copyable_ptr.h"
#include "core/common/hash.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/json_message_visitor.h"
#include "core/proto/proto_common.h"
#include "core/proto/proto_writer.h"
#include "core/view/utils/string_map.h"

namespace imp {

namespace proto {

class JsonReader {
 public:
  explicit JsonReader(absl::string_view data,
                      JsonMessageVisitor* json_message_visitor = nullptr)
      : begin_(data.data()),
        ptr_(data.data()),
        end_(data.data() + data.size()),
        status_(absl::OkStatus()),
        line_start_(data.data()),
        json_message_visitor_(json_message_visitor) {
    ptr_ = ExpectToken(ptr_, '{');
  }
  explicit JsonReader(const JsonReader& parent, const char* const end,
                      JsonMessageVisitor* json_message_visitor = nullptr)
      : begin_(parent.begin_),
        ptr_(parent.ptr_),
        end_(end),
        status_(absl::OkStatus()),
        line_number_(parent.line_number_),
        line_start_(parent.line_start_),
        current_dictionary_key_(parent.current_dictionary_key_),
        json_message_visitor_(json_message_visitor) {}

  explicit JsonReader(const JsonReader& parent,
                      JsonMessageVisitor* json_message_visitor = nullptr)
      : begin_(parent.begin_),
        ptr_(parent.ptr_),
        end_(parent.end_),
        status_(absl::OkStatus()),
        line_number_(parent.line_number_),
        line_start_(parent.line_start_),
        json_message_visitor_(json_message_visitor) {}

  bool Done() const { return !ptr_ || ptr_ >= end_; }

  absl::Status GetStatus() const { return status_; }

  template <typename T>
  std::pair<int, int> ReadTag(T* m);

  const char* Unknown(const char*, int field_id, int token_type);

  template <typename T>
  const char* ParseMsg(T* msg);

  template <int field_type, typename T>
  const char* Visit(const char* ptr, int field_id, T* field, T* other,
                    int token_type);

  template <int field_type, typename T>
  const char* Visit(const char* ptr, int field_id, absl::optional<T>* field,
                    absl::optional<T>* other, int token_type);

  template <int field_type, typename T>
  const char* Visit(const char* ptr, int field_id, CopyablePtr<T>* field,
                    CopyablePtr<T>* other, int token_type);

  template <int field_type, RepeatedMergeStrategy merge_type, typename T>
  const char* Visit(const char* ptr, int field_id, std::vector<T>* field,
                    std::vector<T>* other, int token_type);

  template <int key_type, int value_type, typename K, typename V>
  const char* Visit(const char* ptr, int field_id, std::map<K, V>* field,
                    std::map<K, V>* other, int token_type);

  template <int field_type>
  const char* Visit(const char* ptr, int field_id, bool* field, bool* other,
                    int token_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, int32_t* field,
                    int32_t* other, int token_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, uint32_t* field,
                    uint32_t* other, int token_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, int64_t* field,
                    int64_t* other, int token_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, uint64_t* field,
                    uint64_t* other, int token_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, float* field, float* other,
                    int token_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, double* field, double* other,
                    int token_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, std::string* field,
                    std::string* otherm, int token_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, absl::string_view* field,
                    absl::string_view* other, int token_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id, absl::Cord* field,
                    absl::Cord* other, int token_type);
  template <int field_type>
  const char* Visit(const char* ptr, int field_id,
                    ::google::protobuf::imp_proto::Any* field,
                    ::google::protobuf::imp_proto::Any* other, int token_type);

  template <typename Proto>
  const char* VisitStandardProto(const char* ptr, int field_id, Proto* proto,
                                 Proto* other, int token_type);

  JsonMessageVisitor* GetMessageVisitor() const {
    return json_message_visitor_;
  }

  // Registers a protobuf type so it can be deserialized as part of an any.
  template <typename T>
  static void RegisterKnownType();

  // Clears all registered protobuf types from RegisterKnownType<T>().
  static void ClearRegisteredTypes();

  // Returns the current dictionary key, if the reader is currently visiting a
  // dictionary.
  // Returns std::nullopt otherwise.
  //
  // This is useful for message visitors that need to access the key the message
  // is paired with. Example:
  //
  //   JsonReader reader(json_data);
  //   reader.OnVisit([](Message& message, int field_id,
  //                     JsonReader& reader, const char* ptr,
  //                     int token_type) -> absl::StatusOr<bool> {
  //     std::optional<absl::string_view> key =
  //      reader.GetCurrentDictionaryKey();
  //     if (key.has_value()) {
  //       // Do something with the key.
  //     }
  //     ...
  //   });
  //
  std::optional<absl::string_view> GetCurrentDictionaryKey() const {
    return current_dictionary_key_;
  }

 private:
  using VisitRegisteredAnyFn = std::function<void(
      JsonReader* visitor, ::google::protobuf::imp_proto::Any*)>;

  static StringMap<VisitRegisteredAnyFn>* GetVisitRegisteredFnMap();

  static const char* Consume(const char* ptr, const char* end, char end_token);

  const char* SkipWhitespace(const char* ptr);

  const char* ExpectToken(const char* ptr, int expect);
  const char* NextToken(const char* ptr, int* token);
  template <typename T>
  void ReadNumber(T* field);

  void SetNumberError();
  void SetMessageError();
  void SetMessageOrLiteralError();
  void SetTokenError(char expected);
  void SetTokenError(char expected, std::string_view actual);
  void SetStringKeyError();
  void SetNumberKeyError();
  void SetStringError();
  void SetBase64StringError();
  void SetChordError();
  void SetNoHandlerError(const std::string& type_url, int line_number,
                         ptrdiff_t line_pos);
  void SetBooleanError();

  const char* const begin_;
  const char* ptr_;
  const char* const end_;
  absl::string_view token_;
  int field_id_ = 0;
  int token_type_ = 0;
  absl::Status status_;
  int line_number_ = 1;
  const char* line_start_ = nullptr;
  int token_line_number_ = 0;
  ptrdiff_t token_line_pos_ = 0;

  std::optional<std::string> current_dictionary_key_;

  JsonMessageVisitor* json_message_visitor_ = nullptr;
};

template <typename T>
std::pair<int, int> JsonReader::ReadTag(T* m) {
  if (!status_.ok()) {
    return std::pair<int, int>(0, 0);
  }
  using Fields = FieldsT<T>;

  ptr_ = ExpectToken(ptr_, '"');
  if (!ptr_) return std::pair<int, int>(0, 0);
  int index = -1;
  for (int i = 0; i < Fields::kFieldsCount; i++) {
    if (Fields::kFieldJsonNameHashes[i] == imp::Hash(token_)) {
      index = i;
      break;
    }
  }
  ptr_ = ExpectToken(ptr_, ':');
  if (!ptr_) return std::pair<int, int>(0, 0);

  int type;
  ptr_ = NextToken(ptr_, &type);
  return std::pair<int, int>(index != -1 ? Fields::kFieldIds[index] : 0, type);
}

template <typename T>
void JsonReader::ReadNumber(T* field) {
  if (!status_.ok()) {
    return;
  }
  if constexpr (std::is_integral_v<T> || std::is_enum_v<T>) {
    if (!absl::SimpleAtoi(token_, field)) {
      SetNumberError();
      ptr_ = nullptr;
    }
  } else {
    double d;
    if (absl::SimpleAtod(token_, &d)) {
      *field = static_cast<T>(d);
    } else {
      SetNumberError();
      ptr_ = nullptr;
    }
  }
}

template <typename T>
const char* JsonReader::ParseMsg(T* msg) {
  if (json_message_visitor_) {
    const absl::Status status = json_message_visitor_->Poll(
        ProtoTypeUrlHash<T>::value, msg, this, ptr_);
    if (!status.ok()) {
      status_.Update(status);
      return ptr_;
    }
  }

  while (!Done()) {
    if (ptr_ && ptr_ < end_ && *ptr_ == '}') {
      return SkipWhitespace(ptr_ + 1);
    }
    std::tie(field_id_, token_type_) = ReadTag(msg);
    // 0 is never a valid token type, but is returned by ReadTag if we read past
    // the end of the stream.  So if we see zero, just return.
    // On an invalid stream, we may see zero before the end of the stream,
    // but it's still OK to abort the parse at that point.
    if (!status_.ok() || token_type_ == 0) return ptr_;

    bool handled_by_visitor = false;
    if (json_message_visitor_) {
      absl::StatusOr<bool> handle_result = json_message_visitor_->Handle(
          ProtoTypeUrlHash<T>::value, msg, field_id_, this, ptr_, token_type_);
      if (!handle_result.ok()) {
        status_.Update(handle_result.status());
        return ptr_;
      }

      handled_by_visitor = *handle_result;
    }

    if (!handled_by_visitor) {
      ::imp::proto::VisitField(msg, field_id_, this, ptr_, token_type_);
      if (!status_.ok()) {
        return ptr_;
      }
    }

    ptr_ = SkipWhitespace(ptr_);
    if (ptr_ && ptr_ < end_ && *ptr_ == ',') {
      ptr_ = SkipWhitespace(ptr_ + 1);
    }
  }

  if (json_message_visitor_) {
    json_message_visitor_->Accept(ProtoTypeUrlHash<T>::value, msg);
  }

  return ptr_;
}

template <int field_type, typename T>
const char* JsonReader::Visit(const char* ptr, int field_id, T* field, T* other,
                              int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  if constexpr (std::is_convertible<T, int32_t>::value) {
    // This is an enum.
    static_assert(field_type == TYPE_ENUM);
    ReadNumber(field);
  } else {
    static_assert(field_type == TYPE_MESSAGE);
    if (token_type != '{') {
      SetMessageError();
      ptr_ = nullptr;
      return ptr_;
    }

    JsonReader sub(*this, end_, json_message_visitor_);
    ptr_ = sub.ParseMsg(field);
    if (!ptr_) {
      status_.Update(sub.GetStatus());
    }
  }
  return ptr_;
}

template <int field_type, typename T>
const char* JsonReader::Visit(const char* ptr, int field_id,
                              absl::optional<T>* field,
                              absl::optional<T>* other, int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  if (token_type != '{' && token_type != '*') {
    SetMessageOrLiteralError();
    ptr_ = nullptr;
    return ptr_;
  }
  if (!field->has_value()) {
    field->emplace(T());
  }
  Visit<field_type>(ptr_, field_id, &(**field), static_cast<T*>(nullptr),
                    token_type);
  return ptr_;
}

template <int field_type, typename T>
const char* JsonReader::Visit(const char* ptr, int field_id,
                              CopyablePtr<T>* field, CopyablePtr<T>* other,
                              int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  if (token_type != '{' && token_type != '*') {
    SetMessageOrLiteralError();
    ptr_ = nullptr;
    return ptr_;
  }
  if (!*field) {
    *field = MakeCopyablePtr<T>();
  }
  Visit<field_type>(ptr_, field_id, field->get(), static_cast<T*>(nullptr),
                    token_type);
  return ptr_;
}

template <int field_type, RepeatedMergeStrategy merge_type, typename T>
const char* JsonReader::Visit(const char* ptr, int field_id,
                              std::vector<T>* field, std::vector<T>* other,
                              int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  if (token_type != '[') {
    SetTokenError('[');
    ptr_ = nullptr;
    return ptr_;
  }
  while (ptr_ && ptr_ < end_) {
    ptr_ = NextToken(ptr_, &token_type);
    if (token_type == 0 || token_type == ']') break;
    // If we've read an item, consume a comma.
    if (!field->empty()) {
      if (token_type != ',') {
        SetTokenError(',', std::string(1, token_type));
        ptr_ = nullptr;
        return ptr_;
      }
      ptr_ = NextToken(ptr_, &token_type);
    }
    if constexpr (std::is_same_v<T, bool>) {
      // We need to handle bool specially because the &vector<bool> leads to an
      // error for taking the address of a temporary object.
      // See https://en.cppreference.com/w/cpp/container/vector_bool
      bool bool_value = false;
      Visit<field_type>(ptr_, 0, &bool_value, static_cast<T*>(nullptr),
                        token_type);
      field->emplace_back(bool_value);
    } else {
      field->emplace_back();
      Visit<field_type>(ptr_, 0, &field->back(), static_cast<T*>(nullptr),
                        token_type);
    }
  }

  field->shrink_to_fit();

  if (json_message_visitor_) {
    json_message_visitor_->Accept(ProtoTypeUrlHash<std::vector<T>>::value,
                                  field);
  }
  return ptr_;
}

template <int key_type, int value_type, typename K, typename V>
const char* JsonReader::Visit(const char* ptr, int field_id,
                              std::map<K, V>* field, std::map<K, V>* other,
                              int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  if (token_type != '{') {
    SetTokenError('{', std::string(1, token_type));
    ptr_ = nullptr;
    return ptr_;
  }
  while (ptr_ && ptr_ < end_) {
    ptr_ = NextToken(ptr_, &token_type);
    if (token_type == 0 || token_type == '}') break;
    // If we've read a pair, consume a comma.
    if (!field->empty()) {
      if (token_type != ',') {
        SetTokenError(',', std::string(1, token_type));
        ptr_ = nullptr;
        return ptr_;
      }
      ptr_ = NextToken(ptr_, &token_type);
    }
    // JSON keys are always strings.
    if (!ptr_ || token_type != '"' || token_.empty()) {
      SetStringKeyError();
      ptr_ = nullptr;
      return ptr_;
    }

    absl::string_view key_string = token_;
    K key = K();
    if constexpr (std::is_integral_v<K>) {
      if (!absl::SimpleAtoi(token_, &key)) {
        SetNumberKeyError();
        ptr_ = nullptr;
        return ptr_;
      }
    } else {
      key = K(token_);
    }
    ptr_ = ExpectToken(ptr_, ':');
    if (!ptr_) {
      return nullptr;
    }
    ptr_ = NextToken(ptr_, &token_type);
    if (!ptr_ || token_type == 0) {
      ptr_ = nullptr;
      return ptr_;
    }
    V value = V();

    // Populates the key so visitor can access it.
    current_dictionary_key_ = key_string;

    Visit<value_type>(ptr_, 0, &value, static_cast<V*>(nullptr), token_type);

    // Resets the key.
    current_dictionary_key_.reset();

    field->emplace(key, value);
  }
  return ptr_;
}

// The absl::Cord visitor is left as a template so that we don't expand
// it if it's not required.
template <int field_type>
const char* JsonReader::Visit(const char* ptr, int field_id, absl::Cord* field,
                              absl::Cord* other, int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  if constexpr (field_type == TYPE_BYTES) {
    std::string bytes;
    if (absl::Base64Unescape(token_, &bytes)) {
      field->Append(std::move(bytes));
    } else {
      SetBase64StringError();
      ptr_ = nullptr;
    }
  } else {
    // We don't support type CORD, which is unused by our glTF proto spec.
    SetChordError();
  }
  return ptr_;
}

template <int field_type>
const char* JsonReader::Visit(const char* ptr, int field_id,
                              ::google::protobuf::imp_proto::Any* field,
                              ::google::protobuf::imp_proto::Any* other,
                              int token_type) {
  if (!status_.ok()) {
    return ptr_;
  }
  static_assert(field_type == FieldType::TYPE_MESSAGE);
  // Find the type url of the any block.
  while (token_ != "@type") {
    ptr_ = NextToken(ptr_, &token_type);
  }
  // Move two more tokens to get across ':' to the type url string.
  ptr_ = NextToken(ptr_, &token_type);
  ptr_ = NextToken(ptr_, &token_type);
  std::string type_url(token_);

  // Record the line number and position of the type url in case of error.
  const int line_number = token_line_number_;
  const ptrdiff_t line_pos = token_line_pos_;

  // Move one more token to get to the real contents of the any.
  ptr_ = NextToken(ptr_, &token_type);

  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  auto itr = map->find(type_url);
  if (itr != map->end()) {
    itr->second(this, field);
  } else {
    // We cannot read this Any as we don't have a handler for the type url.
    // This makes the json effectively invalid as it won't deserialize properly.
    SetNoHandlerError(type_url, line_number, line_pos);
    ptr_ = nullptr;
  }
  return ptr_;
}

// TODO: Support standard (non-impress) protos.
template <typename Proto>
const char* JsonReader::VisitStandardProto(const char* ptr, int field_id,
                                           Proto* proto, Proto* other,
                                           int token_type) {}

template <typename T>
absl::Status ParseJson(absl::string_view data, T* msg,
                       JsonMessageVisitor* visitor = nullptr) {
  JsonReader in(data, visitor);
  in.ParseMsg(msg);
  return in.GetStatus();
}

template <typename T>
absl::Status ParseJson(absl::Cord data, T* msg,
                       JsonMessageVisitor* visitor = nullptr) {
  return ParseJson(data.Flatten(), msg, visitor);
}

template <typename T>
void JsonReader::RegisterKnownType() {
  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  (*map).emplace(
      std::string(T::kTypeUrl),
      [](JsonReader* visitor, ::google::protobuf::imp_proto::Any* any) {
        if (!visitor->status_.ok()) {
          return;
        }
        // Use a nested JsonReader to parse a message of type T.
        T t;
        // Find the end of the any block for the sub-reader to use as its end.
        const char* end = Consume(visitor->ptr_, visitor->end_, '}') - 1;
        JsonReader sub(*visitor, end, visitor->json_message_visitor_);
        if (!sub.ParseMsg<T>(&t)) {
          // If we fail to parse the message, the json is invalid.
          visitor->ptr_ = nullptr;
          visitor->status_.Update(sub.GetStatus());
          return;
        }
        // Set the any type url.
        any->type_url = T::kTypeUrl;
        // Serialize the message back into binary and set as the any's value.
        std::string serialized;
        imp::proto::SerializeTo(&t, &serialized);
        any->value = serialized;
        // Set the visitor ptr to be on the next character after the any block.
        visitor->ptr_ = end + 1;
      });
}

}  // namespace proto

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_JSON_READER_H_
