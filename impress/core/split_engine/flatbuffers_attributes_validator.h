/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_FLATBUFFERS_API_LEVEL_VALIDATOR_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_FLATBUFFERS_API_LEVEL_VALIDATOR_H_

#include <cstdint>
#include <optional>
#include <stack>
#include <string>
#include <string_view>
#include <type_traits>
#include <variant>
#include <vector>

#include "absl/base/nullability.h"
#include "absl/container/flat_hash_set.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/base.h"
#include "flatbuffers/reflection_generated.h"
#include "flatbuffers/struct.h"
#include "flatbuffers/table.h"
#include "flatbuffers/vector.h"
#include "core/common/invocable.h"

namespace imp::split_engine {

template <typename, typename = void>
inline constexpr bool is_flatbuffers_table_v = false;

// Flatbuffers tables generated structs do not have a public base class, so we
// check for a few specific methods:
//  - T is a struct.
//  - T has a `Builder` type.
//  - T has a `BinarySchema` type with a static method `data()` (only available
//    when passing `--bfbs-gen-embed` to the flatbuffer compiler).
//  - T has a static method called `GetFullyQualifiedName` (only available when
//    passing `--gen-name-strings` to the flatbuffer compiler).
template <typename T>
inline constexpr bool is_flatbuffers_table_v<
    T, std::void_t<typename T::Builder, decltype(T::BinarySchema::data()),
                   decltype(T::GetFullyQualifiedName())>> = true;

// Used to validate the flatbuffers objects against the type and fields
// attributes defined in the flatbuffer schema.
// It relies on the flatbuffer reflection API to get the type and fields api
// level attributes. It requires the flatbuffer schema to be embedded in the
// binary and to have their fully qualified names (added by passing
// `--bfbs-gen-embed` and
// `--gen-name-strings` to the flatbuffer compiler).
class FlatbuffersAttributesValidator {
 public:
  struct Options {
    std::optional<int32_t> max_api_level = std::nullopt;
  };

  explicit FlatbuffersAttributesValidator(Options options);

  template <typename T>
  absl::Status Validate(const T* table);

 private:
  struct PathSegment {
    std::string_view name;
    std::string_view type;
    bool is_vector = false;
    std::optional<flatbuffers::uoffset_t> index = std::nullopt;
    bool is_union = false;
  };
  using Path = std::vector<PathSegment>;

  template <typename Sink>
  friend void AbslStringify(Sink& sink, const Path& path);

  using ObjectPtr =
      std::variant<const flatbuffers::Table*, const flatbuffers::Struct*>;

  struct StackEntry {
    const ObjectPtr object_ptr;
    const reflection::Object* object_def;
    const reflection::Schema* schema;
    Path parent_path;
  };

  struct EnumFieldVisitor {
    const FlatbuffersAttributesValidator& self;
    const reflection::Schema* /*absl_nonnull*/ schema;
    const reflection::Object* /*absl_nonnull*/ object_def;
    const std::variant<const flatbuffers::Table*, const flatbuffers::Struct*>
        object_ptr;

    absl::Status& status;

    template <typename T>
    void Visit(const reflection::Field* /*absl_nonnull*/ field) const;
  };

  absl::Status ValidateAttributes(
      const flatbuffers::Vector<
          flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/ attributes)
      const;

  std::vector<Invocable<absl::Status(
      const flatbuffers::Vector<flatbuffers::Offset<
          reflection::KeyValue>>* /*absl_nullable*/ attributes)>>
      attribute_validators_;
  std::stack<StackEntry> stack_;
  absl::flat_hash_set<ObjectPtr> visited_;

  absl::Status ValidateImpl();

  void Cleanup();

  absl::Status ValidateObject(const StackEntry& entry);
};

template <typename T>
absl::Status FlatbuffersAttributesValidator::Validate(const T* table) {
  static_assert(is_flatbuffers_table_v<T>,
                "Validate only supports flatbuffers tables.");
  if (attribute_validators_.empty()) {
    return absl::OkStatus();
  }

  const reflection::Schema* schema =
      reflection::GetSchema(T::BinarySchema::data());
  if (schema == nullptr) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Schema for ", table->GetFullyQualifiedName(), " is null."));
  }
  const reflection::Object* table_def =
      schema->objects()->LookupByKey(table->GetFullyQualifiedName());
  if (table_def == nullptr) {
    return absl::InvalidArgumentError(
        absl::StrCat("Table ", table->GetFullyQualifiedName(),
                     " is not found in the schema."));
  }

  stack_.push(StackEntry{
      .object_ptr = reinterpret_cast<const flatbuffers::Table*>(table),
      .object_def = table_def,
      .schema = schema,
      .parent_path = {{.name = "<ROOT>",
                       .type = table_def->name()->string_view()}},
  });
  auto status = ValidateImpl();
  Cleanup();
  return status;
}

template <typename Sink>
void AbslStringify(Sink& sink,
                   const FlatbuffersAttributesValidator::Path& path) {
  bool first = true;
  for (const auto& segment : path) {
    if (segment.index.has_value()) {
      sink.Append(absl::StrCat("[", segment.index.value(), "]"));
      continue;
    }

    if (!first) {
      sink.Append(".");
    }
    if (!segment.name.empty()) {
      sink.Append(segment.name);
    }
    if (!segment.type.empty()) {
      sink.Append("(");
      if (segment.is_vector) {
        sink.Append("Vector<");
      } else if (segment.is_union) {
        sink.Append("Union<");
      }
      // Strip the package name from the type name if present.
      auto last_dot_pos = segment.type.find_last_of('.');
      if (last_dot_pos != std::string::npos) {
        sink.Append(segment.type.substr(last_dot_pos + 1));
      } else {
        sink.Append(segment.type);
      }
      if (segment.is_vector || segment.is_union) {
        sink.Append(">");
      }
      sink.Append(")");
    }
    first = false;
  }
}
}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_FLATBUFFERS_API_LEVEL_VALIDATOR_H_
