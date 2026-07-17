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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_JSON_WRITER_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_JSON_WRITER_H_

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "absl/memory/memory.h"
#include "absl/strings/cord.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/copyable_ptr.h"
#include "core/common/one_of.h"
#include "core/common/optional_with_default.h"
#include "core/common/template_helpers.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_common.h"
#include "core/proto/proto_reader.h"
#include "core/view/utils/string_map.h"

namespace imp {

namespace proto {

class JsonWriter {
 public:
  explicit JsonWriter(std::string* str) : str_(str) {}

  bool Finish() {
    str_->shrink_to_fit();
    return true;
  }

  template <int field_type, typename M, typename T>
  M* Visit(M* m, int field_id, T* field, T* other, bool optional = false);

  template <int field_type, typename M, typename T>
  M* Visit(M* m, int field_id, absl::optional<T>* field,
           absl::optional<T>* other, bool optional = false);

  template <int field_type, typename M, typename T,
            const auto* DefaultValuePointer>
  M* Visit(M* m, int field_id,
           OptionalWithDefault<T, DefaultValuePointer>* field,
           OptionalWithDefault<T, DefaultValuePointer>* other,
           bool optional = false);

  template <int field_type, typename M, typename T>
  M* Visit(M* m, int field_id, CopyablePtr<T>* field, CopyablePtr<T>* other,
           bool optional = false);

  template <int field_type, RepeatedMergeStrategy merge_type, typename M,
            typename T>
  M* Visit(M* m, int field_id, std::vector<T>* field, std::vector<T>* other,
           bool optional = false);

  template <int key_type, int value_type, typename M, typename K, typename V>
  M* Visit(M* m, int field_id, std::map<K, V>* field, std::map<K, V>* other,
           bool optional = false);

  template <int field_type, typename M>
  M* Visit(M* m, int field_id, bool* field, bool* other, bool optional = false);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, int32_t* field, int32_t* other,
           bool optional = false);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, uint32_t* field, uint32_t* other,
           bool optional = false);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, int64_t* field, int64_t* other,
           bool optional = false);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, uint64_t* field, uint64_t* other,
           bool optional = false);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, float* field, float* other,
           bool optional = false);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, double* field, double* other,
           bool optional = false);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, std::string* field, std::string* other,
           bool optional = false);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, absl::string_view* field,
           absl::string_view* other, bool optional = false);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, absl::Cord* field, absl::Cord* other,
           bool optional = false);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, ::google::protobuf::imp_proto::Any* field,
           ::google::protobuf::imp_proto::Any* other);

  template <typename M, typename Proto>
  M* VisitStandardProto(M* m, int field_id, Proto* proto, Proto* other);

  template <typename M, typename... Tags, typename FieldType,
            FieldType... field_types, size_t... I>
  M* VisitOneOf(M* m, imp::OneOf<Tags...>* field, imp::OneOf<Tags...>* other,
                absl::string_view field_name,
                std::integer_sequence<FieldType, field_types...>,
                const std::vector<int>& variant_field_ids,
                std::index_sequence<I...>);

  template <typename M, typename T, typename FieldType,
            FieldType... field_types>
  M* VisitVariant(
      M* m, T* field, T* other, absl::string_view field_name,
      std::integer_sequence<int, field_types...> variant_field_types,
      const std::vector<int>& variant_field_ids);

  template <typename M>
  M* Unknown(M* m) {
    return m;
  }

  // Registers a protobuf type so it can be serialized as part of an any.
  template <typename T>
  static void RegisterKnownType();

  // Clears all registered protobuf types from RegisterKnownType<T>().
  static void ClearRegisteredTypes();

 private:
  using VisitRegisteredAnyFn = std::function<void(
      JsonWriter* visitor, ::google::protobuf::imp_proto::Any*)>;

  static StringMap<VisitRegisteredAnyFn>* GetVisitRegisteredFnMap();

  std::string* str_;

  void StripTrailingComma() {
    if ((*str_)[str_->size() - 2] == ',') {
      str_->resize(str_->size() - 2);
    }
  }

  template <int field_type, typename M, typename T>
  M* WriteInt(M* m, int field_id, T& field, bool optional = false);
  template <int field_type, typename M, typename T>
  M* WriteFloat(M* m, int field_id, T& field, bool optional = false);
};

template <int field_type, typename M, typename T>
M* JsonWriter::Visit(M* m, int field_id, T* field, T* other, bool optional) {
  if constexpr (std::is_convertible<T, int32_t>::value) {
    // This is an enum.
    if constexpr (field_type != TYPE_ENUM) {
      return false;
    }
    if (field_id) {
      if (!*field) {
        return m;
      }
      // Static cast to int32_t to avoid printing the enum as a string via
      // AbslStringify. In the future we may want to print the enum string for
      // better readability, but right now json_reader doesn't support this.
      absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id),
                      "\": ", static_cast<int32_t>(*field), ", ");
    } else {
      absl::StrAppend(str_, *field, ", ");
    }
  } else {
    static_assert(field_type == TYPE_MESSAGE);
    if (field_id) {
      absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id), "\": { ");
    } else {
      absl::StrAppend(str_, "{ ");
    }
    ::imp::proto::Visit(field, this, field);
    StripTrailingComma();
    absl::StrAppend(str_, " }, ");
  }
  return m;
}

template <int field_type, typename M, typename T>
M* JsonWriter::Visit(M* m, int field_id, absl::optional<T>* field,
                     absl::optional<T>* other, bool optional) {
  if (!field->has_value()) {
    return m;
  }
  return Visit<field_type>(m, field_id, &(**field), static_cast<T*>(nullptr),
                           optional);
}

template <int field_type, typename M, typename T,
          const auto* DefaultValuePointer>
M* JsonWriter::Visit(M* m, int field_id,
                     OptionalWithDefault<T, DefaultValuePointer>* field,
                     OptionalWithDefault<T, DefaultValuePointer>* other,
                     bool optional) {
  if (!field->HasValue()) {
    return m;
  }
  return Visit<field_type>(m, field_id, &field->MutableValue(),
                           static_cast<T*>(nullptr), /*optional=*/true);
}

template <int field_type, typename M, typename T>
M* JsonWriter::Visit(M* m, int field_id, CopyablePtr<T>* field,
                     CopyablePtr<T>* other, bool optional) {
  if (!*field) {
    return m;
  }
  return Visit<field_type>(m, field_id, field->get(), static_cast<T*>(nullptr),
                           optional);
}

template <int field_type, RepeatedMergeStrategy merge_type, typename M,
          typename T>
M* JsonWriter::Visit(M* m, int field_id, std::vector<T>* field,
                     std::vector<T>* other, bool optional) {
  if (field->empty()) {
    return m;
  }
  // Since protobuf can't directly represent an array of arrays, field_id
  // should always be set.
  assert(field_id);
  absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id), "\": [ ");

  for (auto& value : *field) {
    m = Visit<field_type>(m, 0, &value, static_cast<T*>(nullptr));
  }
  StripTrailingComma();
  absl::StrAppend(str_, " ], ");
  return m;
}

template <int key_type, int value_type, typename M, typename K, typename V>
M* JsonWriter::Visit(M* m, int field_id, std::map<K, V>* field,
                     std::map<K, V>* other, bool optional) {
  if (field->empty()) {
    return m;
  }
  if (field_id) {
    absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id), "\": { ");
  } else {
    absl::StrAppend(str_, "{ ");
  }
  for (auto& [key, value] : *field) {
    absl::StrAppend(str_, "\"", key, "\": ");
    m = Visit<value_type>(m, 0, const_cast<V*>(&value),
                          static_cast<V*>(nullptr));
  }
  StripTrailingComma();
  absl::StrAppend(str_, " }, ");
  return m;
}

template <int field_type, typename M, typename T>
M* JsonWriter::WriteInt(M* m, int field_id, T& field, bool optional) {
  if (field_id) {
    if (!field && !optional) {
      return m;
    }
    absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id), "\": ");
  }
  absl::StrAppend(str_, field, ", ");
  return m;
}

template <int field_type, typename M, typename T>
M* JsonWriter::WriteFloat(M* m, int field_id, T& field, bool optional) {
  if (field_id) {
    if (field == 0.0f && !optional) {
      return m;
    }
    absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id), "\": ");
  }

  absl::StrAppend(str_, field, ", ");
  return m;
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id, bool* field, bool* other,
                     bool optional) {
  if (field_id) {
    if (!*field && !optional) {
      return m;
    }
    absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id), "\": ");
  }
  absl::StrAppend(str_, *field ? "true" : "false", ", ");
  return m;
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id, int32_t* field, int32_t* other,
                     bool optional) {
  return WriteInt<field_type>(m, field_id, *field, optional);
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id, uint32_t* field, uint32_t* other,
                     bool optional) {
  return WriteInt<field_type>(m, field_id, *field, optional);
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id, int64_t* field, int64_t* other,
                     bool optional) {
  return WriteInt<field_type>(m, field_id, *field, optional);
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id, uint64_t* field, uint64_t* other,
                     bool optional) {
  return WriteInt<field_type>(m, field_id, *field, optional);
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id, float* field, float* other,
                     bool optional) {
  return WriteFloat<field_type>(m, field_id, *field, optional);
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id, double* field, double* other,
                     bool optional) {
  return WriteFloat<field_type>(m, field_id, *field, optional);
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id, std::string* field, std::string* other,
                     bool optional) {
  if (field_id) {
    if (field->empty() && !optional) {
      return m;
    }
    absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id), "\": ");
  }
  if constexpr (field_type == TYPE_BYTES) {
    absl::StrAppend(str_, "\"", absl::Base64Escape(*field), "\", ");
  } else {
    absl::StrAppend(str_, "\"", *field, "\", ");
  }
  return m;
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id, absl::string_view* field,
                     absl::string_view* other, bool optional) {
  if (field_id) {
    if (field->empty() && !optional) {
      return m;
    }
    absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id), "\": ");
  }
  if constexpr (field_type == TYPE_BYTES) {
    absl::StrAppend(str_, "\"", absl::Base64Escape(*field), "\", ");
  } else {
    absl::StrAppend(str_, "\"", *field, "\", ");
  }
  return m;
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id, absl::Cord* field, absl::Cord* other,
                     bool optional) {
  if (field_id) {
    if (field->empty() && !optional) {
      return m;
    }
    absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id), "\": ");
  }

  absl::Cord copy = *field;
  auto view = copy.Flatten();
  if constexpr (field_type == TYPE_BYTES) {
    absl::StrAppend(str_, "\"", absl::Base64Escape(view), "\", ");
  } else {
    absl::StrAppend(str_, "\"", view, "\", ");
  }
  return m;
}

template <int field_type, typename M>
M* JsonWriter::Visit(M* m, int field_id,
                     ::google::protobuf::imp_proto::Any* field,
                     ::google::protobuf::imp_proto::Any* other) {
  static_assert(field_type == FieldType::TYPE_MESSAGE);
  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  auto itr = map->find(field->type_url);
  if (itr != map->end()) {
    if (field_id) {
      absl::StrAppend(str_, "\"", GetFieldJsonName<M>(field_id), "\": ");
    }
    itr->second(this, field);
  } else {
    ::imp::proto::Visit(field, this, field);
  }
  // TODO: Each Visit function shouldn't be responsible for ','s.
  absl::StrAppend(str_, ", ");
  return m;
}

// TODO: Support standard (non-impress) protos.
template <typename M, typename Proto>
M* JsonWriter::VisitStandardProto(M* m, int field_id, Proto* proto,
                                  Proto* other) {
  return m;
}

template <typename T>
void JsonWriter::RegisterKnownType() {
  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  (*map).emplace(T::kTypeUrl, [](JsonWriter* visitor,
                                 ::google::protobuf::imp_proto::Any* any) {
    // Extract the any into the real type T.
    T message;
    auto value = any->value;
    ParseMessage(value, &message);
    // Serialize the T into an any-style message (including @type attr).
    std::string serialized;
    absl::StrAppend(&serialized, "{\n");
    absl::StrAppend(&serialized,
                    absl::StrFormat("\"@type\": %s,\n", T::kTypeUrl));
    JsonWriter sub_writer(&serialized);
    ::imp::proto::Visit(&message, &sub_writer, &message);
    // Append the serialized any to the current string.
    absl::StrAppend(&serialized, "}");
    absl::StrAppend(visitor->str_, serialized);
  });
}

template <typename M, typename... Tags, typename FieldType,
          FieldType... field_types, size_t... I>
M* JsonWriter::VisitOneOf(M* m, imp::OneOf<Tags...>* field,
                          imp::OneOf<Tags...>* other,
                          absl::string_view field_name,
                          std::integer_sequence<FieldType, field_types...>,
                          const std::vector<int>& variant_field_ids,
                          std::index_sequence<I...>) {
  constexpr FieldType types_array[] = {field_types...};
  (
      [&]() {
        if (!field->template Holds<Tags>()) {
          return;
        }

        int variant_field_id = variant_field_ids[I];
        if constexpr (proto_traits::kIsStandardProto<typename Tags::Type>) {
          m = VisitStandardProto(m, variant_field_id,
                                 field->template GetIf<Tags>(),
                                 (other && other->template Holds<Tags>())
                                     ? other->template GetIf<Tags>()
                                     : nullptr);
        } else {
          constexpr FieldType field_type = types_array[I];
          m = Visit<field_type>(m, variant_field_id,
                                field->template GetIf<Tags>(),
                                (other && other->template Holds<Tags>())
                                    ? other->template GetIf<Tags>()
                                    : nullptr,
                                /*optional=*/true);
        }
      }(),
      ...);
  return m;
}

template <typename M, typename T, typename FieldType, FieldType... field_types>
M* JsonWriter::VisitVariant(
    M* m, T* field, T* other, absl::string_view field_name,
    std::integer_sequence<int, field_types...> variant_field_types,
    const std::vector<int>& variant_field_ids) {
  ForConstexpr<0, std::variant_size_v<T>>([&m, &variant_field_ids,
                                           &variant_field_types, field, other,
                                           this](auto i) mutable {
    if (field->index() != i) {
      return;
    }
    if constexpr (i != 0) {
      using VariantAlternativeT = std::variant_alternative_t<i, T>;
      constexpr int field_type = GetAt<i - 1>(variant_field_types);
      int variant_field_id = variant_field_ids[i - 1];

      if constexpr (proto_traits::kIsStandardProto<VariantAlternativeT>) {
        m = this->VisitStandardProto(
            m, variant_field_id, absl::get_if<i>(field),
            (!other || other->index() != i) ? nullptr : absl::get_if<i>(other));
      } else {
        m = this->Visit<field_type>(
            m, variant_field_id, absl::get_if<i>(field),
            (!other || other->index() != i) ? nullptr : absl::get_if<i>(other),
            /*optional=*/true);
      }
    }
  });
  return m;
}

template <typename T>
bool ToJson(T* msg, std::string* str) {
  *str = "{ ";
  JsonWriter stream(str);
  ::imp::proto::Visit(msg, &stream, msg);
  if ((*str)[str->size() - 2] == ',') {
    str->resize(str->size() - 2);
  }
  absl::StrAppend(str, " }");
  return stream.Finish();
}

}  // namespace proto

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_JSON_WRITER_H_
