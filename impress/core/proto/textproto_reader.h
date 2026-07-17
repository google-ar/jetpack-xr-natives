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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_TEXTPROTO_READER_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_TEXTPROTO_READER_H_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/ascii.h"
#include "absl/strings/cord.h"
#include "absl/strings/escaping.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/copyable_ptr.h"
#include "core/common/hash.h"
#include "core/common/optional_with_default.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_common.h"
#include "core/proto/proto_writer.h"
#include "core/view/utils/string_map.h"

namespace imp {

namespace proto {

class TextprotoReader {
 public:
  explicit TextprotoReader(absl::string_view data)
      : ptr_(data.data()), end_(data.data() + data.size()) {}
  explicit TextprotoReader(const TextprotoReader& parent, const char* const end)
      : ptr_(parent.ptr_), end_(end) {}

  absl::Status GetStatus() const { return status_; }

  bool Done() const { return !ptr_ || ptr_ >= end_; }

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

  template <int field_type, typename T, const auto* DefaultValuePointer>
  const char* Visit(const char* ptr, int field_id,
                    OptionalWithDefault<T, DefaultValuePointer>* field,
                    OptionalWithDefault<T, DefaultValuePointer>* other,
                    int token_type);

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
                    std::string* other, int token_type);
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

  // Registers a protobuf type so it can be deserialized as part of an any.
  template <typename T>
  static void RegisterKnownType();

  // Clears all registered protobuf types from RegisterKnownType<T>().
  static void ClearRegisteredTypes();

 private:
  using VisitRegisteredAnyFn = std::function<void(
      TextprotoReader* visitor, ::google::protobuf::imp_proto::Any*)>;

  static StringMap<VisitRegisteredAnyFn>* GetVisitRegisteredFnMap();

  static const char* Consume(const char* ptr, const char* end, char end_token);

  const char* SkipWhitespace(const char* ptr) {
    while (ptr && ptr < end_) {
      if (absl::ascii_isspace(*ptr)) {
        ++ptr;
      } else if (*ptr == '#') {
        while (ptr < end_ && *ptr != '\n') ++ptr;
      } else {
        break;
      }
    }
    return ptr;
  }

  const char* ExpectToken(const char* ptr, int expect);
  const char* NextToken(const char* ptr, int* token);
  template <typename T>
  void ReadNumber(T* field);

  absl::Status status_ = absl::OkStatus();
  const char* ptr_;
  const char* const end_;
  absl::string_view token_;
  int field_id_ = 0;
  int token_type_ = 0;
};

template <typename T>
std::pair<int, int> TextprotoReader::ReadTag(T* m) {
  using Fields = FieldsT<T>;

  int token_type;
  ptr_ = NextToken(ptr_, &token_type);

  if (!ptr_) return std::pair<int, int>(0, 0);
  int index = -1;
  for (int i = 0; i < Fields::kFieldsCount; i++) {
    if (Fields::kFieldNameHashes[i] == imp::Hash(token_)) {
      index = i;
      break;
    }
  }
  // TODO: We don't distinguish between a field that we can't find
  // and reaching the end of the stream, as a value of 0 just causes a return
  // in ParseMsg().
  ptr_ = ExpectToken(ptr_, ':');
  if (!ptr_) return std::pair<int, int>(0, 0);

  ptr_ = NextToken(ptr_, &token_type);
  return std::pair<int, int>(index != -1 ? Fields::kFieldIds[index] : 0,
                             token_type);
}

template <typename T>
void TextprotoReader::ReadNumber(T* field) {
  if constexpr (std::is_integral_v<T> || std::is_enum_v<T>) {
    if (!absl::SimpleAtoi(token_, field)) {
      status_ = absl::InvalidArgumentError("Failed to parse integer.");
      ptr_ = nullptr;
    }
  } else {
    double d;
    if (absl::SimpleAtod(token_, &d)) {
      *field = static_cast<T>(d);
    } else {
      status_ = absl::InvalidArgumentError("Failed to parse double.");
      ptr_ = nullptr;
    }
  }
}

template <typename T>
const char* TextprotoReader::ParseMsg(T* msg) {
  while (!Done()) {
    if (ptr_ && ptr_ < end_ && *ptr_ == '}') {
      return SkipWhitespace(ptr_ + 1);
    }
    std::tie(field_id_, token_type_) = ReadTag(msg);
    // 0 is never a valid token type, but is returned by ReadTag if we read past
    // the end of the stream.  So if we see zero, just return.
    // On an invalid stream, we may see zero before the end of the stream,
    // but it's still OK to abort the parse at that point.
    if (token_type_ == 0) return ptr_;
    ::imp::proto::VisitField(msg, field_id_, this, ptr_, token_type_);
    ptr_ = SkipWhitespace(ptr_);
  }
  return ptr_;
}

template <int field_type, typename T>
const char* TextprotoReader::Visit(const char* ptr, int field_id, T* field,
                                   T* other, int token_type) {
  if constexpr (std::is_convertible<T, int32_t>::value) {
    // This is an enum.
    static_assert(field_type == TYPE_ENUM);
    std::optional<T> enum_value = EnumMetaData<T>::FromName(token_);
    if (enum_value) {
      *field = *enum_value;
    } else {
      std::underlying_type_t<T> underlying_enum;
      ReadNumber(&underlying_enum);
      if (EnumMetaData<T>::IsValid(underlying_enum)) {
        *field = static_cast<T>(underlying_enum);
      } else {
        status_ = absl::InvalidArgumentError(
            absl::StrFormat("Invalid enum value %i for enum of type %s",
                            underlying_enum, type_traits::kTypeName<T>));
        ptr_ = nullptr;
      }
    }
  } else {
    static_assert(field_type == TYPE_MESSAGE);
    if (token_type != '{') {
      status_ = absl::InvalidArgumentError("Expected '{' for message.");
      ptr_ = nullptr;
      return ptr_;
    }
    TextprotoReader sub(*this, end_);
    ptr_ = sub.ParseMsg(field);
  }
  return ptr_;
}

template <int field_type, typename T>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   absl::optional<T>* field,
                                   absl::optional<T>* other, int token_type) {
  if (!field->has_value()) {
    field->emplace(T());
  }
  return Visit<field_type>(ptr, field_id, &(**field), static_cast<T*>(nullptr),
                           token_type);
}

template <int field_type, typename T, const auto* DefaultValuePointer>
const char* TextprotoReader::Visit(
    const char* ptr, int field_id,
    OptionalWithDefault<T, DefaultValuePointer>* field,
    OptionalWithDefault<T, DefaultValuePointer>* other, int token_type) {
  if (!field->HasValue()) {
    *field = T();
  }

  return Visit<field_type>(ptr, field_id, &field->MutableValue(),
                           static_cast<T*>(nullptr), token_type);
}

template <int field_type, typename T>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   CopyablePtr<T>* field, CopyablePtr<T>* other,
                                   int token_type) {
  if (!*field) {
    *field = MakeCopyablePtr<T>();
  }
  return Visit<field_type>(ptr, field_id, field->get(),
                           static_cast<T*>(nullptr), token_type);
}

template <int field_type, RepeatedMergeStrategy merge_type, typename T>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   std::vector<T>* field, std::vector<T>* other,
                                   int token_type) {
  if (token_type != '[') {
    status_ = absl::InvalidArgumentError("Expected '[' for repeated field.");
    ptr_ = nullptr;
    return ptr_;
  }
  while (ptr_ && ptr_ < end_) {
    ptr_ = NextToken(ptr_, &token_type);
    if (token_type == 0 || token_type == ']') break;
    // If we've read an item, consume a comma.
    if (!field->empty()) {
      if (token_type != ',') {
        status_ =
            absl::InvalidArgumentError("Expected ',' for repeated field.");
        ptr_ = nullptr;
        return ptr_;
      }
      ptr_ = NextToken(ptr_, &token_type);
    }
    field->emplace_back();
    Visit<field_type>(ptr_, 0, &field->back(), static_cast<T*>(nullptr),
                      token_type);
  }
  field->shrink_to_fit();
  return ptr_;
}

template <int key_type, int value_type, typename K, typename V>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   std::map<K, V>* field, std::map<K, V>* other,
                                   int token_type) {
  if (token_type != '[') {
    status_ = absl::InvalidArgumentError("Expected '[' for map field.");
    ptr_ = nullptr;
    return ptr_;
  }
  while (ptr_ && ptr_ < end_) {
    ptr_ = NextToken(ptr_, &token_type);

    // Stop when we hit the closing ']'.
    if (token_type == 0 || token_type == ']') break;

    // If we've read a pair, consume a comma.
    if (!field->empty()) {
      if (token_type != ',') {
        status_ = absl::InvalidArgumentError("Expected ',' for map field.");
        ptr_ = nullptr;
        return ptr_;
      }
      ptr_ = NextToken(ptr_, &token_type);
    }

    // Map entries look like {key: k value: v}.

    // Check for opening brace.
    if (token_type != '{') {
      status_ = absl::InvalidArgumentError("Expected '{' for map field.");
      ptr_ = nullptr;
      return ptr_;
    }

    // Parse the key.
    ptr_ = SkipWhitespace(ptr_);
    if (absl::string_view(ptr_, 4) != "key:") {
      status_ = absl::InvalidArgumentError("Expected 'key: ' for map entry.");
      ptr_ = nullptr;
      return ptr_;
    }
    ptr_ = Consume(ptr_, end_, ':');
    // NextToken() should get us the key in token_.
    ptr_ = NextToken(ptr_, &token_type);

    K key = K();
    if constexpr (std::is_integral_v<K>) {
      if (!absl::SimpleAtoi(token_, &key)) {
        status_ = absl::InvalidArgumentError("Failed to parse integer.");
        ptr_ = nullptr;
        return ptr_;
      }
    } else {
      key = K(token_);
    }

    // Parse the value.
    ptr_ = SkipWhitespace(ptr_);
    if (absl::string_view(ptr_, 6) != "value:") {
      status_ = absl::InvalidArgumentError("Expected 'value: ' for map entry.");
      ptr_ = nullptr;
      return ptr_;
    }
    ptr_ = Consume(ptr_, end_, ':');
    // NextToken() should get us the value in token_.
    ptr_ = NextToken(ptr_, &token_type);

    V value = V();
    Visit<value_type>(ptr_, 0, &value, static_cast<V*>(nullptr), token_type);
    field->emplace(key, value);

    // Each entry ends with }.
    ptr_ = ExpectToken(ptr_, '}');
  }
  return ptr_;
}

// The absl::Cord visitor is left as a template so that we don't expand
// it if it's not required.
template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   absl::Cord* field, absl::Cord* other,
                                   int token_type) {
  if constexpr (field_type == TYPE_BYTES) {
    std::string bytes;
    if (absl::Base64Unescape(token_, &bytes)) {
      field->Append(std::move(bytes));
    } else {
      status_ = absl::InvalidArgumentError("Failed to parse bytes.");
      ptr_ = nullptr;
    }
  } else {
    IMP_LOG(imp::FATAL) << "Unable to process Cord field of type other than TYPE_BYTES";
  }
  return ptr_;
}

template <int field_type>
const char* TextprotoReader::Visit(const char* ptr, int field_id,
                                   ::google::protobuf::imp_proto::Any* field,
                                   ::google::protobuf::imp_proto::Any* other,
                                   int token_type) {
  static_assert(field_type == FieldType::TYPE_MESSAGE);

  // Content should look like this:
  // {
  //   [type.googleapis.com/imp.GltfState] {
  //     asset: "Avocado.glb"
  //   }
  // }
  ptr_ = ExpectToken(ptr_, '[');
  if (ptr_ == nullptr) {
    status_ = absl::InvalidArgumentError("Expected '[' for any.");
    return ptr_;
  }

  // The next token should be the type URL.
  ptr_ = NextToken(ptr_, &token_type);
  std::string type_url(token_);

  ptr_ = ExpectToken(ptr_, ']');
  if (ptr_ == nullptr) {
    status_ = absl::InvalidArgumentError("Expected ']' for any.");
    return ptr_;
  }

  ptr_ = ExpectToken(ptr_, '{');
  if (ptr_ == nullptr) {
    status_ = absl::InvalidArgumentError("Expected '{' for any.");
    return ptr_;
  }

  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  auto itr = map->find(type_url);
  if (itr != map->end()) {
    itr->second(this, field);
    // Move past the closing brace containing the Any block.
    ptr_ = SkipWhitespace(ptr_);
    ptr_ = ExpectToken(ptr_, '}');
    if (ptr_ == nullptr) {
      status_ = absl::InvalidArgumentError("Expected '}' for any.");
      return ptr_;
    }
  } else {
    // We cannot read this Any as we don't have a handler for the type url.
    // This makes the textproto effectively invalid as it won't deserialize.
    status_ = absl::InvalidArgumentError(
        absl::StrCat("No handler for any of type: ", type_url));
    ptr_ = nullptr;
  }
  return ptr_;
}

// TODO: Support standard (non-impress) protos.
template <typename Proto>
const char* TextprotoReader::VisitStandardProto(const char* ptr, int field_id,
                                                Proto* proto, Proto* other,
                                                int token_type) {
  return ptr;
}

template <typename T>
absl::Status ParseTextproto(absl::string_view data, T* msg) {
  TextprotoReader in(data);
  in.ParseMsg(msg);
  return in.GetStatus();
}

template <typename T>
absl::Status ParseTextproto(absl::Cord data, T* msg) {
  return ParseTextproto(data.Flatten(), msg);
}

template <typename T>
void TextprotoReader::RegisterKnownType() {
  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  (*map).emplace(
      std::string(T::kTypeUrl),
      [](TextprotoReader* visitor, ::google::protobuf::imp_proto::Any* any) {
        // Use a nested TextprotoReader to parse a message of type T.
        T t;
        // Find the end of the any block for the sub-reader to use as its end.
        const char* end = Consume(visitor->ptr_, visitor->end_, '}') - 1;
        TextprotoReader sub(*visitor, end);
        if (!sub.ParseMsg<T>(&t)) {
          // If we fail to parse the message, the textproto is invalid.
          visitor->status_.Update(sub.GetStatus());
          visitor->ptr_ = nullptr;
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

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_TEXTPROTO_READER_H_
