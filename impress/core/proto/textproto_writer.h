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

#ifndef THIRD_PARTY_IMPRESS_CORE_PROTO_TEXTPROTO_WRITER_H_
#define THIRD_PARTY_IMPRESS_CORE_PROTO_TEXTPROTO_WRITER_H_

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <map>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "absl/strings/cord.h"
#include "absl/strings/escaping.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/common/bit_flag.h"
#include "core/common/copyable_ptr.h"
#include "core/common/optional_with_default.h"
#include "core/common/platform_helpers.h"
#include "core/common/template_helpers.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_common.h"
#include "core/proto/proto_reader.h"
#include "core/view/utils/string_map.h"

namespace imp {

namespace proto {

class TextprotoWriter {
 public:
  enum VisitFlags : BitFlag {
    kNone = 0,
    // If set, indicates that printing a new line should be skipped.
    kSkipEndL = 1 << 0,
    // If set, indicates that the message should be printed even if it is empty.
    kKeepEmptyMessage = 1 << 1,
    // If set, indicates that the field is optional and should be printed even
    // if it is the default value.
    kOptional = 1 << 2,
  };

  explicit TextprotoWriter(std::string* str) : printer_(str) {}

  bool Finish() {
    // If the proto is empty, we still want to print a single new line.
    if (printer_.GetSize() == 0) {
      printer_.PrintEndl();
    }
    return true;
  }

  template <int field_type, typename M, typename T>
  M* Visit(M* m, int field_id, T* field, T* other, BitFlag flags = 0);

  template <typename M, typename T, typename FieldType,
            FieldType... field_types>
  M* VisitVariant(
      M* m, T* field, T* other, absl::string_view field_name,
      std::integer_sequence<int, field_types...> variant_field_types,
      const std::vector<int>& variant_field_ids, BitFlag flags = 0);

  template <int field_type, typename M, typename T>
  M* Visit(M* m, int field_id, absl::optional<T>* field,
           absl::optional<T>* other, BitFlag flags = 0);

  template <int field_type, typename M, typename T,
            const auto* DefaultValuePointer>
  M* Visit(M* m, int field_id,
           OptionalWithDefault<T, DefaultValuePointer>* field,
           OptionalWithDefault<T, DefaultValuePointer>* other,
           BitFlag flags = 0);

  template <int field_type, typename M, typename T>
  M* Visit(M* m, int field_id, CopyablePtr<T>* field, CopyablePtr<T>* other,
           BitFlag flags = 0);

  template <int field_type, RepeatedMergeStrategy merge_type, typename M,
            typename T>
  M* Visit(M* m, int field_id, std::vector<T>* field, std::vector<T>* other,
           BitFlag flags = 0);

  template <int key_type, int value_type, typename M, typename K, typename V>
  M* Visit(M* m, int field_id, std::map<K, V>* field, std::map<K, V>* other,
           BitFlag flags = 0);

  template <int field_type, typename M>
  M* Visit(M* m, int field_id, bool* field, bool* other, BitFlag flags = 0);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, int32_t* field, int32_t* other,
           BitFlag flags = 0);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, uint32_t* field, uint32_t* other,
           BitFlag flags = 0);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, int64_t* field, int64_t* other,
           BitFlag flags = 0);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, uint64_t* field, uint64_t* other,
           BitFlag flags = 0);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, float* field, float* other, BitFlag flags = 0);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, double* field, double* other, BitFlag flags = 0);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, std::string* field, std::string* other,
           BitFlag flags = 0);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, absl::string_view* field,
           absl::string_view* other, BitFlag flags = 0);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, absl::Cord* field, absl::Cord* other,
           BitFlag flags = 0);
  template <int field_type, typename M>
  M* Visit(M* m, int field_id, ::google::protobuf::imp_proto::Any* field,
           ::google::protobuf::imp_proto::Any* other, BitFlag flags = 0);

  template <typename M, typename Proto>
  M* VisitStandardProto(M* m, int field_id, Proto* proto, Proto* other,
                        BitFlag flags = 0);

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
      TextprotoWriter* visitor, ::google::protobuf::imp_proto::Any*)>;

  static StringMap<VisitRegisteredAnyFn>* GetVisitRegisteredFnMap();

  // The goal of this class is to print with predictable/correct "tabs" so all
  // blocks are properly indented.
  class Printer {
   public:
    explicit Printer(std::string* str)
        : str_(str), indent_(0), skip_next_indent_(false) {}

    // Adds the given strings to the buffer at the current indentation level.
    template <typename... Strs>
    void Print(Strs... strs) {
      std::string combined;
      absl::StrAppend(&combined, strs...);
      std::vector<std::string> lines = absl::StrSplit(combined, '\n');
      // absl::StrSplit leaves a single empty line at the end if it ended in \n.
      if (lines.back().empty()) {
        lines.pop_back();
      }
      for (size_t line = 0; line < lines.size(); line++) {
        if (!skip_next_indent_) {
          for (int32_t i = 0; i < indent_; i++) {
            PrintRaw("  ");
          }
        }
        skip_next_indent_ = false;
        PrintRaw(lines[line], line < lines.size() - 1 ? "\n" : "");
      }
    }

    // Prints without any indentation.
    template <typename... Strs>
    void PrintRaw(Strs... strs) {
      absl::StrAppend(str_, strs...);
    }

    // Print a newline character.
    void PrintEndl() { PrintRaw("\n"); }

    // Increases the indentation level for all future Print() calls.
    // Call Outdent() to return to the previous indentation level.
    void Indent() { indent_++; }
    // Returns to the previous indentation level (must match a Indent() call).
    void Outdent() {
      if (indent_ == 0) {
        // We need to use imp::output::Fatal here instead of LOG because IMP_LOG
        // (which gets copybara transformed from LOG) depends on a proto. This
        // leads to cyclic dependnecies.
        imp::output::Fatal("Attempt to outdent beyond initial indent.");
      }
      indent_--;
    }

    // Skips the indentation on the next Print() call.
    // This is to handle the recursive nature of the Visit methods - often a
    // Visit method won't be aware of the context in which it is being
    // invoked. This is to handle a case like an entry in a map entry, where
    // we don't want the current indentation level to apply for the field in a
    // key or value slot.
    void SkipNextIndent() { skip_next_indent_ = true; }
    void ClearSkipNextIndent() { skip_next_indent_ = false; }
    void ShrinkToFit() { str_->shrink_to_fit(); }

    size_t GetSize() const { return str_->size(); }

    void Erase(size_t i) { str_->erase(i); }

   public:
    std::string* str_;
    int32_t indent_;
    bool skip_next_indent_;
  };

  Printer printer_;

  template <int field_type, typename M, typename T>
  M* WriteInt(M* m, int field_id, T& field, BitFlag flags = 0);
  template <int field_type, typename M, typename T>
  M* WriteFloat(M* m, int field_id, T& field, BitFlag flags = 0);
};

template <int field_type, typename M, typename T>
M* TextprotoWriter::Visit(M* m, int field_id, T* field, T* other,
                          BitFlag flags) {
  bool skip_end_l = CheckBit(kSkipEndL, flags);

  if constexpr (std::is_convertible<T, int32_t>::value) {
    // This is an enum.
    if constexpr (field_type != TYPE_ENUM) {
      return false;
    }
    if (field_id) {
      if (!*field && !CheckBit(flags, kOptional)) {
        return m;
      }

      std::underlying_type_t<T> underlying_enum = *field;
      if (EnumMetaData<T>::IsValid(underlying_enum)) {
        printer_.Print(GetFieldName<M>(field_id), ": ",
                       EnumMetaData<T>::GetName(*field));
      } else {
        // TODO: Report this case as an error.
      }
    } else {
      printer_.Print(*field);
    }
  } else {
    static_assert(field_type == TYPE_MESSAGE);
    size_t size_before_message = printer_.GetSize();

    if (field_id) {
      printer_.Print(GetFieldName<M>(field_id), ": {");
    } else {
      printer_.Print("{");
    }
    printer_.PrintEndl();

    size_t size_before_message_contents = printer_.GetSize();

    printer_.Indent();
    ::imp::proto::Visit(field, this, field);
    printer_.Outdent();

    if (size_before_message_contents == printer_.GetSize() &&
        !CheckBit(flags, kKeepEmptyMessage | kOptional)) {
      // The message was empty, erase the containing block.
      printer_.Erase(size_before_message);
      skip_end_l = true;
    } else {
      if (size_before_message_contents == printer_.GetSize()) {
        // The message was empty but we still need to print it.
        // In this case, erase extraneous whitespace.
        printer_.Erase(printer_.GetSize() - 1);
        printer_.SkipNextIndent();
      }

      // Close out the containing block.
      printer_.Print("}");
    }
  }
  if (!skip_end_l) {
    printer_.PrintEndl();
  }

  return m;
}

template <typename M, typename T, typename FieldType, FieldType... field_types>
M* TextprotoWriter::VisitVariant(
    M* m, T* field, T* other, absl::string_view field_name,
    std::integer_sequence<int, field_types...> variant_field_types,
    const std::vector<int>& variant_field_ids, BitFlag flags) {
  ForConstexpr<0, std::variant_size_v<T>>([this, &m, &variant_field_ids,
                                           &variant_field_types, field, other,
                                           flags](auto i) mutable {
    if (field->index() != i) {
      return;
    }

    if constexpr (i != 0) {
      using VariantAlternativeT = std::variant_alternative_t<i, T>;

      // TODO: Support standard (non-impress) protos.
      if constexpr (!proto_traits::kIsStandardProto<VariantAlternativeT>) {
        constexpr int field_type = GetAt<i - 1>(variant_field_types);
        int variant_field_id = variant_field_ids[i - 1];

        // We need a special VisitVariant method so we can set this flag
        // ensuring that empty messages are printed when they are part of a
        // variant.
        flags = SetBit(kKeepEmptyMessage, flags);
        m = Visit<field_type>(
            m, variant_field_id, absl::get_if<i>(field),
            (!other || other->index() != i) ? nullptr : absl::get_if<i>(other),
            flags);
      }
    }
  });

  return m;
}

template <int field_type, typename M, typename T>
M* TextprotoWriter::Visit(M* m, int field_id, absl::optional<T>* field,
                          absl::optional<T>* other, BitFlag flags) {
  if (!field->has_value()) {
    return m;
  }
  return Visit<field_type>(m, field_id, &(**field), static_cast<T*>(nullptr),
                           flags | kOptional);
}

template <int field_type, typename M, typename T,
          const auto* DefaultValuePointer>
M* TextprotoWriter::Visit(M* m, int field_id,
                          OptionalWithDefault<T, DefaultValuePointer>* field,
                          OptionalWithDefault<T, DefaultValuePointer>* other,
                          BitFlag flags) {
  if (!field->HasValue()) {
    return m;
  }

  return Visit<field_type>(m, field_id, &field->MutableValue(),
                           static_cast<T*>(nullptr), flags | kOptional);
}

template <int field_type, typename M, typename T>
M* TextprotoWriter::Visit(M* m, int field_id, CopyablePtr<T>* field,
                          CopyablePtr<T>* other, BitFlag flags) {
  if (!*field) {
    return m;
  }
  return Visit<field_type>(m, field_id, field->get(), static_cast<T*>(nullptr),
                           flags | kOptional);
}

template <int field_type, RepeatedMergeStrategy merge_type, typename M,
          typename T>
M* TextprotoWriter::Visit(M* m, int field_id, std::vector<T>* field,
                          std::vector<T>* other, BitFlag flags) {
  if (field->empty()) {
    return m;
  }
  // Since protobuf can't directly represent an array of arrays, field_id
  // should always be set.
  assert(field_id);
  printer_.Print(GetFieldName<M>(field_id), ": [");
  printer_.Indent();
  printer_.PrintEndl();

  for (size_t i = 0; i < field->size(); i++) {
    auto& value = field->at(i);
    BitFlag elem_flags = kSkipEndL | kKeepEmptyMessage;
    m = Visit<field_type>(m, 0, &value, static_cast<T*>(nullptr), elem_flags);
    if (i < field->size() - 1) {
      printer_.PrintRaw(",");
    }
    printer_.PrintEndl();
  }
  printer_.Outdent();
  printer_.Print("]");
  if (!CheckBit(kSkipEndL, flags)) {
    printer_.PrintEndl();
  }

  return m;
}

template <int key_type, int value_type, typename M, typename K, typename V>
M* TextprotoWriter::Visit(M* m, int field_id, std::map<K, V>* field,
                          std::map<K, V>* other, BitFlag flags) {
  if (field->empty()) {
    return m;
  }
  if (field_id) {
    printer_.Print(GetFieldName<M>(field_id), ": [");
  } else {
    printer_.Print("[");
  }
  printer_.Indent();
  printer_.PrintEndl();

  // The printout should look like this:
  // my_map: [
  //   {
  //     key: "foo"
  //     value: 10
  //   },
  //   {
  //     key: "bar"
  //     value: 11
  //   },
  // ]
  size_t i = 0;
  for (const auto& [key, value] : *field) {
    printer_.Print("{");
    printer_.PrintEndl();
    printer_.Indent();
    printer_.Print("key: ");
    // If we don't tell printer to skip the indentation for key/value fields,
    // the current indent level would get inserted improperly and look like:
    // `key:   "foo"` instead of `key: "foo"`.
    printer_.SkipNextIndent();
    BitFlag elem_flags = kSkipEndL | kKeepEmptyMessage;
    m = Visit<key_type>(m, 0, const_cast<K*>(&key), static_cast<K*>(nullptr),
                        elem_flags);
    printer_.ClearSkipNextIndent();
    printer_.PrintEndl();
    printer_.Print("value: ");
    printer_.SkipNextIndent();
    m = Visit<value_type>(m, 0, const_cast<V*>(&value),
                          static_cast<V*>(nullptr), elem_flags);
    printer_.ClearSkipNextIndent();
    printer_.PrintEndl();
    printer_.Outdent();
    printer_.Print("}");
    if (i < field->size() - 1) {
      printer_.PrintRaw(",");
    }
    printer_.PrintEndl();
    i++;
  }
  printer_.Outdent();
  printer_.Print("]");
  if (!CheckBit(kSkipEndL, flags)) {
    printer_.PrintEndl();
  }
  return m;
}

template <int field_type, typename M, typename T>
M* TextprotoWriter::WriteInt(M* m, int field_id, T& field, BitFlag flags) {
  std::string prefix = "";
  if (field_id) {
    if (!field && !CheckBit(flags, kOptional)) {
      return m;
    }
    absl::StrAppend(&prefix, GetFieldName<M>(field_id), ": ");
  }
  printer_.Print(prefix, field);
  if (!CheckBit(kSkipEndL, flags)) {
    printer_.PrintEndl();
  }
  return m;
}

template <int field_type, typename M, typename T>
M* TextprotoWriter::WriteFloat(M* m, int field_id, T& field, BitFlag flags) {
  std::string prefix = "";
  if (field_id) {
    if (field == 0.0f && !CheckBit(flags, kOptional)) {
      return m;
    }
    absl::StrAppend(&prefix, GetFieldName<M>(field_id), ": ");
  }

  printer_.Print(prefix, field);
  if (!CheckBit(kSkipEndL, flags)) {
    printer_.PrintEndl();
  }
  return m;
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id, bool* field, bool* other,
                          BitFlag flags) {
  std::string prefix = "";
  if (field_id) {
    if (!*field && !CheckBit(flags, kOptional)) {
      return m;
    }
    absl::StrAppend(&prefix, GetFieldName<M>(field_id), ": ");
  }
  printer_.Print(prefix, *field ? "true" : "false");
  if (!CheckBit(kSkipEndL, flags)) {
    printer_.PrintEndl();
  }
  return m;
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id, int32_t* field, int32_t* other,
                          BitFlag flags) {
  return WriteInt<field_type>(m, field_id, *field, flags);
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id, uint32_t* field, uint32_t* other,
                          BitFlag flags) {
  return WriteInt<field_type>(m, field_id, *field, flags);
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id, int64_t* field, int64_t* other,
                          BitFlag flags) {
  return WriteInt<field_type>(m, field_id, *field, flags);
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id, uint64_t* field, uint64_t* other,
                          BitFlag flags) {
  return WriteInt<field_type>(m, field_id, *field, flags);
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id, float* field, float* other,
                          BitFlag flags) {
  return WriteFloat<field_type>(m, field_id, *field, flags);
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id, double* field, double* other,
                          BitFlag flags) {
  return WriteFloat<field_type>(m, field_id, *field, flags);
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id, std::string* field,
                          std::string* other, BitFlag flags) {
  std::string prefix = "";
  if (field_id) {
    if (field->empty() && !CheckBit(flags, kOptional)) {
      return m;
    }
    absl::StrAppend(&prefix, GetFieldName<M>(field_id), ": ");
  }

  if constexpr (field_type == TYPE_BYTES) {
    printer_.Print(prefix, "\"", absl::Base64Escape(*field), "\"");
  } else {
    printer_.Print(prefix, "\"", *field, "\"");
  }
  if (!CheckBit(kSkipEndL, flags)) {
    printer_.PrintEndl();
  }
  return m;
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id, absl::string_view* field,
                          absl::string_view* other, BitFlag flags) {
  std::string prefix = "";
  if (field_id) {
    if (field->empty() && !CheckBit(flags, kOptional)) {
      return m;
    }
    absl::StrAppend(&prefix, GetFieldName<M>(field_id), ": ");
  }
  if constexpr (field_type == TYPE_BYTES) {
    printer_.Print(prefix, "\"", absl::Base64Escape(*field), "\"");
  } else {
    printer_.Print(prefix, "\"", *field, "\"");
  }
  if (!CheckBit(kSkipEndL, flags)) {
    printer_.PrintEndl();
  }
  return m;
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id, absl::Cord* field,
                          absl::Cord* other, BitFlag flags) {
  std::string prefix = "";
  if (field_id) {
    if (field->empty() && !CheckBit(flags, kOptional)) {
      return m;
    }
    absl::StrAppend(&prefix, GetFieldName<M>(field_id), ": ");
  }

  absl::Cord copy = *field;
  absl::optional<absl::string_view> view = copy.TryFlat();
  if constexpr (field_type == TYPE_BYTES) {
    printer_.Print(prefix, "\"", absl::Base64Escape(*view), "\"");
  } else {
    printer_.Print(prefix, "\"", *view, "\"");
  }
  if (!CheckBit(kSkipEndL, flags)) {
    printer_.PrintEndl();
  }
  return m;
}

template <int field_type, typename M>
M* TextprotoWriter::Visit(M* m, int field_id,
                          ::google::protobuf::imp_proto::Any* field,
                          ::google::protobuf::imp_proto::Any* other,
                          BitFlag flags) {
  static_assert(field_type == FieldType::TYPE_MESSAGE);

  if (field_id) {
    printer_.Print(GetFieldName<M>(field_id), ": ");
  }
  printer_.Print("{");
  printer_.PrintEndl();
  printer_.Indent();

  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  auto itr = map->find(field->type_url);
  if (itr != map->end()) {
    itr->second(this, field);
  } else {
    ::imp::proto::Visit(field, this, field);
  }

  printer_.Outdent();
  printer_.PrintEndl();
  printer_.Print("}");

  if (!CheckBit(kSkipEndL, flags)) {
    printer_.PrintEndl();
  }
  return m;
}

// TODO: Support standard (non-impress) protos.
template <typename M, typename Proto>
M* TextprotoWriter::VisitStandardProto(M* m, int field_id, Proto* proto,
                                       Proto* other, BitFlag flags) {
  return m;
}

template <typename T>
void TextprotoWriter::RegisterKnownType() {
  StringMap<VisitRegisteredAnyFn>* map = GetVisitRegisteredFnMap();
  (*map).emplace(
      T::kTypeUrl, [](TextprotoWriter* visitor,
                      ::google::protobuf::imp_proto::Any* any) mutable {
        // Extract the any into the real type T.
        T message;
        auto value = any->value;
        ParseMessage(value, &message);
        // Serialize the T into an any-style message (including @type attr).
        visitor->printer_.Print(absl::StrFormat("[%s] {", T::kTypeUrl));
        visitor->printer_.PrintEndl();
        visitor->printer_.Indent();
        std::string serialized;
        TextprotoWriter sub_writer(&serialized);
        ::imp::proto::Visit(&message, &sub_writer, &message);
        // Append the serialized any to the current string.
        visitor->printer_.Print(serialized);
        visitor->printer_.PrintEndl();
        visitor->printer_.Outdent();
        visitor->printer_.Print("}");
      });
}

// TODO: This should return an absl::Status.
// Right now, the bool also always return true, there is no actual mechanism
// for reporting errors.
template <typename T>
bool ToTextproto(T* msg, std::string* str) {
  TextprotoWriter stream(str);
  ::imp::proto::Visit(msg, &stream, msg);
  return stream.Finish();
}

}  // namespace proto

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_PROTO_TEXTPROTO_WRITER_H_
