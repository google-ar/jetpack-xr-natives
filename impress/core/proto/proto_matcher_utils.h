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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_MATCHER_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_MATCHER_UTILS_H_

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/memory/memory.h"
#include "absl/strings/cord.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_common.h"
#include "core/proto/proto_reader.h"
#include "core/view/utils/string_map.h"
#include "json/reader.h"
#include "json/value.h"
#include "json/writer.h"
#include "google/protobuf/util/json_util.h"

namespace imp {
namespace testing {
namespace proto {
namespace matcher_utils {

using ::imp::proto::FieldType;
using ::imp::proto::GetFieldJsonName;
using ::imp::proto::ParseMessage;

// Writes an impress proto to json specifically for the purpose of debugging
// used in matchers.
//
// This differs from the regular JsonWriter, because instead of simply writing
// the proto to a raw json string, this writes the proto into the json data
// structures from the C++ library JsonCPP. Normally, we don't want to take a
// dependency on JsonCPP, but for our testing use-case, it's fine.
//
// Writing the proto to a json data structure allows us to inspect, compare, and
// print the individual fields of imp protos. This is necessary because imp
// protos don't support comparison or reflection by default.
//
// Using this, we can compare and print out exactly what is different between
// two protos.
// TODO: Any's get printed as their raw binary data. Add the
// ability to print and compare any's in a human readable way.
class DebugJsonWriter {
 public:
  DebugJsonWriter() : current_value_(&root_) {}

  template <int field_type, typename M, typename T>
  M* Visit(M* m, int field_id, T* field, T* other);

  template <int field_type, typename M, typename T>
  M* Visit(M* m, int field_id, absl::optional<T>* field,
           absl::optional<T>* other);

  template <int field_type, typename M, typename T>
  M* Visit(M* m, int field_id, std::unique_ptr<T>* field,
           std::unique_ptr<T>* other);

  template <int field_type, imp::proto::RepeatedMergeStrategy merge_type,
            typename M, typename T>
  M* Visit(M* m, int field_id, std::vector<T>* field, std::vector<T>* other);

  template <int key_type, int value_type, typename M, typename K, typename V>
  M* Visit(M* m, int field_id, std::map<K, V>* field, std::map<K, V>* other);

  template <int field_type, typename M>
  M* Visit(M* m, int field_id, bool* field, bool* other);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, int32_t* field, int32_t* other);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, uint32_t* field, uint32_t* other);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, int64_t* field, int64_t* other);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, uint64_t* field, uint64_t* other);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, float* field, float* other);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, double* field, double* other);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, std::string* field, std::string* other);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, absl::string_view* field,
           absl::string_view* other);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, absl::Cord* field, absl::Cord* other);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, ::google::protobuf::imp_proto::Any* field,
           ::google::protobuf::imp_proto::Any* other);

  template <typename M, typename Proto>
  M* VisitStandardProto(M* m, int field_id, Proto* proto, Proto* other);

  template <typename M>
  M* Unknown(M* m, int field_id) {
    return m;
  }

  const Json::Value& GetJsonValue() const { return root_; }

  template <typename T>
  static void RegisterKnownType();

 private:
  using FieldValueMap = std::map<std::string, std::string>;
  using VisitRegisteredAnyFn =
      std::function<Json::Value(::google::protobuf::imp_proto::Any*)>;

  Json::Value root_;
  Json::Value* current_value_;

  template <int field_type, typename M, typename T>
  M* WriteInt(M* m, int field_id, T& field);
  template <int field_type, typename M, typename T>
  M* WriteFloat(M* m, int field_id, T& field);

  // Set of overloads to convert the given key type to string.
  // This is required since Json must have map keys be strings but protos can
  // have integral type keys as well.
  std::string KeyToString(const std::string& key) const { return key; }
  template <typename Key>
  std::string KeyToString(Key key) const {
    return absl::StrFormat("%d", key);
  }

  static StringMap<VisitRegisteredAnyFn>* GetVisitRegisteredFnMap();
};

template <int field_type, typename M, typename T>
M* DebugJsonWriter::Visit(M* m, int field_id, T* field, T* other) {
  if constexpr (std::is_convertible<T, int32_t>::value) {
    // This is an enum.
    if constexpr (field_type != FieldType::TYPE_ENUM) {
      return false;
    }

    if (field_id) {
      if (!*field) {
        return m;
      }
      (*current_value_)[GetFieldJsonName<M>(field_id)] = *field;
    } else {
      current_value_->append(*field);
    }
  } else {
    static_assert(field_type == FieldType::TYPE_MESSAGE);
    Json::Value* previous_value = current_value_;
    if (field_id) {
      current_value_ = &((*current_value_)[GetFieldJsonName<M>(field_id)]);
    }
    ::imp::proto::Visit(field, this, field);
    current_value_ = previous_value;
  }
  return m;
}

template <int field_type, typename M, typename T>
M* DebugJsonWriter::Visit(M* m, int field_id, absl::optional<T>* field,
                          absl::optional<T>* other) {
  if (!field->has_value()) {
    return m;
  }
  return Visit<field_type>(m, field_id, &(**field), static_cast<T*>(nullptr));
}

template <int field_type, typename M, typename T>
M* DebugJsonWriter::Visit(M* m, int field_id, std::unique_ptr<T>* field,
                          std::unique_ptr<T>* other) {
  if (!*field) {
    return m;
  }
  return Visit<field_type>(m, field_id, field->get(), static_cast<T*>(nullptr));
}

template <int field_type, imp::proto::RepeatedMergeStrategy merge_type,
          typename M, typename T>
M* DebugJsonWriter::Visit(M* m, int field_id, std::vector<T>* field,
                          std::vector<T>* other) {
  if (field->empty()) {
    return m;
  }
  // Since protobuf can't directly represent an array of arrays, field_id
  // should always be set.
  assert(field_id);
  Json::Value* previous_value = current_value_;
  Json::Value* array_value =
      &((*current_value_)[GetFieldJsonName<M>(field_id)]);

  for (int i = 0; i < field->size(); ++i) {
    current_value_ = &(*array_value)[i];
    m = Visit<field_type>(m, 0, &(*field)[i], static_cast<T*>(nullptr));
  }

  current_value_ = previous_value;

  return m;
}

template <int key_type, int value_type, typename M, typename K, typename V>
M* DebugJsonWriter::Visit(M* m, int field_id, std::map<K, V>* field,
                          std::map<K, V>* other) {
  if (field->empty()) {
    return m;
  }
  Json::Value* previous_value = current_value_;
  Json::Value* map_value;
  if (field_id) {
    map_value = &((*current_value_)[GetFieldJsonName<M>(field_id)]);
  } else {
    map_value = &((*current_value_)[current_value_->size()]);
  }
  for (auto& [key, value] : *field) {
    // Proto keys can be integral type, so it has to be converted to string.
    current_value_ = &(*map_value)[KeyToString(key)];
    m = Visit<value_type>(m, 0, const_cast<V*>(&value),
                          static_cast<V*>(nullptr));
  }

  current_value_ = previous_value;
  return m;
}

template <int field_type, typename M, typename T>
M* DebugJsonWriter::WriteInt(M* m, int field_id, T& field) {
  if (field_id) {
    if (!field) {
      return m;
    }

    (*current_value_)[GetFieldJsonName<M>(field_id)] = field;
  } else {
    *current_value_ = field;
  }

  return m;
}

template <int field_type, typename M, typename T>
M* DebugJsonWriter::WriteFloat(M* m, int field_id, T& field) {
  if (field_id) {
    if (field == 0.0f) {
      return m;
    }
    (*current_value_)[GetFieldJsonName<M>(field_id)] = field;
  } else {
    *current_value_ = field;
  }

  return m;
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id, bool* field, bool* other) {
  if (field_id) {
    if (!*field) {
      return m;
    }
    (*current_value_)[GetFieldJsonName<M>(field_id)] = *field;
  }
  return m;
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id, int32_t* field, int32_t* other) {
  return WriteInt<field_type>(m, field_id, *field);
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id, uint32_t* field,
                          uint32_t* other) {
  return WriteInt<field_type>(m, field_id, *field);
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id, int64_t* field, int64_t* other) {
  return WriteInt<field_type>(m, field_id, *field);
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id, uint64_t* field,
                          uint64_t* other) {
  return WriteInt<field_type>(m, field_id, *field);
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id, float* field, float* other) {
  return WriteFloat<field_type>(m, field_id, *field);
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id, double* field, double* other) {
  return WriteFloat<field_type>(m, field_id, *field);
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id, std::string* field,
                          std::string* other) {
  if (field_id) {
    if (field->empty()) {
      return m;
    }
  }
  if constexpr (field_type == FieldType::TYPE_BYTES) {
    (*current_value_)[GetFieldJsonName<M>(field_id)] =
        absl::Base64Escape(*field);
  } else {
    (*current_value_)[GetFieldJsonName<M>(field_id)] = *field;
  }
  return m;
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id, absl::string_view* field,
                          absl::string_view* other) {
  if (field_id) {
    if (field->empty()) {
      return m;
    }
  }
  if constexpr (field_type == FieldType::TYPE_BYTES) {
    (*current_value_)[GetFieldJsonName<M>(field_id)] =
        absl::Base64Escape(*field);
  } else {
    (*current_value_)[GetFieldJsonName<M>(field_id)] = std::string(*field);
  }
  return m;
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id, absl::Cord* field,
                          absl::Cord* other) {
  if (field_id) {
    if (field->empty()) {
      return m;
    }
  }

  absl::Cord copy = *field;
  auto view = copy.Flatten();
  if constexpr (field_type == FieldType::TYPE_BYTES) {
    (*current_value_)[GetFieldJsonName<M>(field_id)] = absl::Base64Escape(view);
  } else {
    (*current_value_)[GetFieldJsonName<M>(field_id)] = view;
  }
  return m;
}

template <int field_type, typename M>
M* DebugJsonWriter::Visit(M* m, int field_id,
                          ::google::protobuf::imp_proto::Any* field,
                          ::google::protobuf::imp_proto::Any* other) {
  static_assert(field_type == FieldType::TYPE_MESSAGE);
  Json::Value* previous_value = current_value_;
  if (field_id) {
    current_value_ = &((*current_value_)[GetFieldJsonName<M>(field_id)]);
  }

  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  auto itr = map->find(field->type_url);
  if (itr != map->end()) {
    *current_value_ = itr->second(field);
  } else {
    ::imp::proto::Visit(field, this, field);
  }
  current_value_ = previous_value;

  return m;
}

template <typename M, typename Proto>
M* DebugJsonWriter::VisitStandardProto(M* m, int field_id, Proto* proto,
                                       Proto* other) {
  std::string msg_json_string;
  proto2::util::MessageToJsonString(*proto, &msg_json_string);

  Json::Value msg_json;
  Json::Reader reader;
  if (!reader.parse(msg_json_string, msg_json)) {
    IMP_LOG(imp::FATAL) << "Failed to parse proto to json: " << msg_json_string;
  }

  (*current_value_)[GetFieldJsonName<M>(field_id)] = std::move(msg_json);

  return m;
}

template <typename T>
Json::Value ToDebugJson(const T& message);

template <typename T>
void DebugJsonWriter::RegisterKnownType() {
  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  (*map).emplace(T::kTypeUrl, [](::google::protobuf::imp_proto::Any* any) {
    auto value = any->value;
    T message;
    ParseMessage(value.Flatten(), &message);
    Json::Value result = ToDebugJson(message);
    // Inserting "@type" based on proto3 json mapping here:
    // (broken link)
    result["@type"] = std::string(T::kTypeUrl);
    return result;
  });
}

template <typename T>
Json::Value ToDebugJson(const T& message) {
  DebugJsonWriter stream;
  // Have to cast to non-const to avoid template errors with mismatched const.
  std::decay_t<T>* message_mutable = const_cast<std::decay_t<T>*>(&message);
  ::imp::proto::Visit(message_mutable, &stream, message_mutable);
  return stream.GetJsonValue();
}

}  // namespace matcher_utils
}  // namespace proto
}  // namespace testing
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_PROTO_MATCHER_UTILS_H_
