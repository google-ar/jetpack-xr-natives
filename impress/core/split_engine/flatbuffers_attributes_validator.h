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
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/base.h"
#include "flatbuffers/reflection.h"
#include "flatbuffers/reflection_generated.h"
#include "flatbuffers/struct.h"
#include "flatbuffers/table.h"
#include "flatbuffers/vector.h"
#include "flatbuffers/verifier.h"
#include "core/common/invocable.h"
#include "mediapipe/framework/port/status_macros.h"

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

// Basic schema provider that returns the embedded schema.
struct EmbeddedSchemaProvider {
  template <typename T>
  absl::StatusOr<const reflection::Schema*> GetSchema() const {
    flatbuffers::Verifier verifier(T::BinarySchema::data(),
                                   T::BinarySchema::size());
    if (!reflection::VerifySchemaBuffer(verifier)) {
      return absl::InvalidArgumentError("Schema is not valid.");
    }
    return reflection::GetSchema(T::BinarySchema::data());
  }
};

// A schema provider that caches the embedded schemas in memory aligned buffers.
//
// It uses the embedded schema buffer pointers as cache key since they can be
// retrieved from the generated table types and are guaranteed to be unique and
// stable since they are static buffers embedded in the binary.
// Multiple generated types can point to the same schema.
class MemoryAlignedCachingSchemaProvider {
 public:
  template <typename T>
  absl::StatusOr<const reflection::Schema*> GetSchema() const {
    if (auto it = cache_.find(T::BinarySchema::data()); it != cache_.end()) {
      return reflection::GetSchema(it->second.data());
    }

    flatbuffers::Verifier verifier(T::BinarySchema::data(),
                                   T::BinarySchema::size());
    if (!reflection::VerifySchemaBuffer(verifier)) {
      return absl::InvalidArgumentError("Schema is not valid.");
    }

    cache_[T::BinarySchema::data()] =
        std::vector<uint8_t>(T::BinarySchema::data(),
                             T::BinarySchema::data() + T::BinarySchema::size());

    return reflection::GetSchema(cache_[T::BinarySchema::data()].data());
  }

 private:
  mutable absl::flat_hash_map<const uint8_t*, std::vector<uint8_t>> cache_;
};

// A validator that checks that the `requires_api` attribute is less than or
// equal to the maximum API level.
class ApiLevelValidator {
 public:
  explicit ApiLevelValidator(int32_t max_api_level)
      : max_api_level_(max_api_level) {}

  absl::Status operator()(
      const flatbuffers::Vector<
          flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/  attributes)
      const;

 private:
  static constexpr std::string_view kRequiresApiAttrName = "requires_api";

  int32_t max_api_level_;
};

// Dispatches to the visitor based on the integer type of the field.
// The visitor is expected to be a class with a template Visit method that
// takes a single template parameter that is the integer type and is overloaded
// for each integer type.
template <typename Visitor>
void VisitIntegerField(const reflection::Field* field, Visitor visitor) {
  switch (field->type()->base_type()) {
    case reflection::BaseType::Bool:  // Bool is stored as int8_t.
    case reflection::BaseType::Byte:
      visitor.template Visit<int8_t>(field);
      break;
    case reflection::BaseType::UType:  // UType is stored as uint8_t.
    case reflection::BaseType::UByte:
      visitor.template Visit<uint8_t>(field);
      break;
    case reflection::BaseType::Short:
      visitor.template Visit<int16_t>(field);
      break;
    case reflection::BaseType::UShort:
      visitor.template Visit<uint16_t>(field);
      break;
    case reflection::BaseType::Int:
      visitor.template Visit<int32_t>(field);
      break;
    case reflection::BaseType::UInt:
      visitor.template Visit<uint32_t>(field);
      break;
    case reflection::BaseType::Long:
      visitor.template Visit<int64_t>(field);
      break;
    case reflection::BaseType::ULong:
      visitor.template Visit<uint64_t>(field);
      break;
    default:
      IMP_LOG(imp::FATAL) << "VisitIntegerField called on a non-integer field: "
                 << field->name()->string_view() << ":"
                 << reflection::EnumNameBaseType(field->type()->base_type());
      break;
  }
}

// Dispatches to the visitor based on the real (float or double) type of the
// field. The visitor is expected to be a class with a template Visit method
// that takes a single template parameter that is the integer type and is
// overloaded for each integer type.
template <typename Visitor>
void VisitRealField(const reflection::Field* field, Visitor visitor) {
  switch (field->type()->base_type()) {
    case reflection::BaseType::Float:
      visitor.template Visit<float>(field);
      break;
    case reflection::BaseType::Double:
      visitor.template Visit<double>(field);
      break;
    default:
      IMP_LOG(imp::FATAL) << "VisitRealField called on a non-real field: "
                 << field->name()->string_view() << ":"
                 << reflection::EnumNameBaseType(field->type()->base_type());
      break;
  }
}

// Dispatches to the visitor based on the real (float or double) type of the
// field. The visitor is expected to be a class with a template Visit method
// that takes a single template parameter that is the integer type and is
// overloaded for each integer type.
template <typename Visitor>
void VisitScalarField(const reflection::Field* field, Visitor visitor) {
  if (flatbuffers::IsInteger(field->type()->base_type())) {
    return VisitIntegerField(field, visitor);
  } else if (flatbuffers::IsFloat(field->type()->base_type())) {
    return VisitRealField(field, visitor);
  } else {
    IMP_LOG(imp::FATAL) << "VisitScalarField called on a non-scalar field: "
               << field->name()->string_view() << ":"
               << reflection::EnumNameBaseType(field->type()->base_type());
  }
}

// Returns the object definition of the child object of a union field.
// If the field is not a union or if the union type is not set, returns
// std::nullopt.
absl::StatusOr<std::optional<const reflection::Object*>> GetUnionChildObjectDef(
    const reflection::Schema* /*absl_nonnull*/  schema,
    const reflection::Field* /*absl_nonnull*/  field,
    const flatbuffers::Table* /*absl_nonnull*/  parent_table,
    const reflection::Object* /*absl_nonnull*/  parent_table_def);

// Returns the value of the attribute with the given name, parsed as the given
// type. If the attribute is not found or if the value cannot be parsed, returns
// an error.
template <typename T>
absl::StatusOr<T> GetAttributeValue(
    std::string_view attribute_name,
    const flatbuffers::Vector<
        flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/  attributes) {
  static_assert(std::is_same_v<T, std::string_view> ||
                    std::is_same_v<T, std::string> || std::is_integral_v<T>,
                "Unsupported type.");

  if (attributes == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Attribute '", attribute_name, "' not found."));
  }

  const reflection::KeyValue* attribute =
      attributes->LookupByKey(attribute_name.data());
  if (attribute == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Attribute '", attribute_name, "' not found."));
  }

  std::string_view string_value = attribute->value()->string_view();

  if constexpr (std::is_same_v<T, std::string>) {
    return std::string(string_value);
  } else if constexpr (std::is_integral_v<T>) {
    T value;
    if (!absl::SimpleAtoi(string_value, &value)) {
      return absl::InvalidArgumentError(
          absl::StrCat("Attribute '", attribute_name,
                       "' has invalid value: ", string_value));
    }
    return value;
  } else {
    return string_value;
  }
}

// Options for the `FlatbuffersAttributesValidator`.
struct FlatbuffersAttributesValidatorOptions {
  std::optional<int32_t> max_api_level;
};

// Used to validate the flatbuffers objects against the type and fields
// attributes defined in the flatbuffer schema.
// It relies on the flatbuffer reflection API to get the type and fields api
// level attributes. It requires the flatbuffer schema to be embedded in the
// binary and to have their fully qualified names (added by passing
// `--bfbs-gen-embed` and
// `--gen-name-strings` to the flatbuffer compiler).
template <typename SchemaProvider>
class FlatbuffersAttributesValidator {
 public:
  explicit FlatbuffersAttributesValidator(
      FlatbuffersAttributesValidatorOptions options,
      const SchemaProvider& schema_provider)
      : schema_provider_(schema_provider) {
    if (options.max_api_level.has_value()) {
      attribute_validators_.push_back(
          ApiLevelValidator{*options.max_api_level});
    }
  }

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
  friend void AbslStringify(Sink& sink, const Path& path) {
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
    const reflection::Schema* /*absl_nonnull*/  schema;
    const reflection::Object* /*absl_nonnull*/  object_def;
    const std::variant<const flatbuffers::Table*, const flatbuffers::Struct*>
        object_ptr;

    absl::Status& status;

    template <typename T>
    void Visit(const reflection::Field* /*absl_nonnull*/  field) const;
  };

  struct HasDefaultValueVisitor {
    const flatbuffers::Table* table;
    bool& has_default_value;

    template <typename T>
    void Visit(const reflection::Field* /*absl_nonnull*/  field) const;
  };

  std::vector<Invocable<absl::Status(
      const flatbuffers::Vector<flatbuffers::Offset<
          reflection::KeyValue>>* /*absl_nullable*/  attributes)>>
      attribute_validators_;
  std::stack<StackEntry> stack_;
  absl::flat_hash_set<ObjectPtr> visited_;
  const SchemaProvider& schema_provider_;

  absl::Status ValidateAttributes(
      const flatbuffers::Vector<
          flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/  attributes)
      const;

  void Cleanup();

  absl::Status ValidateObject(const StackEntry& entry);
};

// Deduction guide for `FlatbuffersAttributesValidator`.
template <typename SchemaProvider>
FlatbuffersAttributesValidator(FlatbuffersAttributesValidatorOptions,
                               const SchemaProvider&)
    -> FlatbuffersAttributesValidator<SchemaProvider>;

template <typename SchemaProvider>
template <typename T>
absl::Status FlatbuffersAttributesValidator<SchemaProvider>::Validate(
    const T* table) {
  static_assert(is_flatbuffers_table_v<T>,
                "Validate only supports flatbuffers tables.");
  if (attribute_validators_.empty()) {
    return absl::OkStatus();
  }

  MP_ASSIGN_OR_RETURN(const reflection::Schema* schema,
                   schema_provider_.template GetSchema<T>());
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

  absl::Status status;
  while (!stack_.empty()) {
    StackEntry entry = stack_.top();
    stack_.pop();
    if (visited_.contains(entry.object_ptr)) {
      continue;
    }
    status = ValidateObject(entry);
    if (!status.ok()) {
      break;
    }
    visited_.insert(entry.object_ptr);
  }
  Cleanup();
  return status;
}

template <typename SchemaProvider>
void FlatbuffersAttributesValidator<SchemaProvider>::Cleanup() {
  visited_.clear();
  while (!stack_.empty()) {
    stack_.pop();
  }
}

template <typename SchemaProvider>
absl::Status FlatbuffersAttributesValidator<SchemaProvider>::ValidateObject(
    const StackEntry& entry) {
  // Validate the object attributes.
  MP_RETURN_IF_ERROR(ValidateAttributes(entry.object_def->attributes()))
      << absl::StrCat(" (", entry.parent_path, ")");

  // Validate there are no unknown fields in the table.
  if (!entry.object_def->is_struct()) {
    auto table = std::get<const flatbuffers::Table*>(entry.object_ptr);
    auto vtable = table->GetVTable();
    auto vtable_byte_size =
        flatbuffers::ReadScalar<flatbuffers::voffset_t>(vtable);
    // The vtable starts with two voffset_t before the fields offsets:
    // the size of the vtable and the size of the table object itself.
    // We substract those to get the number of fields in the vtable.
    auto vtable_field_count =
        (vtable_byte_size / sizeof(flatbuffers::voffset_t)) - 2;
    if (vtable_field_count > entry.object_def->fields()->size()) {
      // Get the highest field offset to ensure that all fields are known.
      flatbuffers::voffset_t max_field_offset = 0;
      for (const auto* field : *entry.object_def->fields()) {
        max_field_offset = std::max(max_field_offset, field->offset());
      }
      // Check if there any unknown fields are set
      for (flatbuffers::voffset_t unknown_field_offset =
               max_field_offset + sizeof(flatbuffers::voffset_t);
           unknown_field_offset < vtable_byte_size;
           unknown_field_offset += sizeof(flatbuffers::voffset_t)) {
        if (table->CheckField(unknown_field_offset)) {
          return absl::InvalidArgumentError(absl::StrCat(
              "Table '", entry.object_def->name()->string_view(),
              "' has unknown fields at offset ", unknown_field_offset));
        }
      }
    }
  }

  // Validate the field attributes.
  for (const auto* field : *entry.object_def->fields()) {
    const reflection::BaseType field_base_type = field->type()->base_type();
    const std::string_view field_name = field->name()->string_view();

    // Skip optional fields in tables that are not set.
    if (!entry.object_def->is_struct() && field->optional()) {
      auto table = std::get<const flatbuffers::Table*>(entry.object_ptr);
      if (!table->CheckField(field->offset())) {
        continue;
      }
    }

    // Skip scalar fields that have the default value (tables only). Struct
    // fields are never skipped because they must all have the same attribute
    // value as the struct, regardless if they are set or not.
    if (!entry.object_def->is_struct() &&
        flatbuffers::IsScalar(field_base_type)) {
      bool has_default_value = false;
      VisitScalarField(
          field,
          HasDefaultValueVisitor{
              .table = std::get<const flatbuffers::Table*>(entry.object_ptr),
              .has_default_value = has_default_value});
      if (has_default_value) {
        continue;
      }
    }

    Path field_path = entry.parent_path;
    field_path.push_back(PathSegment{
        .name = field_name,
        .type = reflection::EnumNameBaseType(field->type()->base_type())});

    // Validate the field attributes.
    // Union type fields do not support attributes.
    if (field_base_type != reflection::BaseType::UType &&
        !(field_base_type == reflection::BaseType::Vector &&
          field->type()->element() == reflection::BaseType::UType)) {
      MP_RETURN_IF_ERROR(ValidateAttributes(field->attributes()))
          << absl::StrCat(" (", field_path, ")");
    }

    const bool is_enum =
        flatbuffers::IsInteger(field_base_type) && field->type()->index() > -1;
    if (is_enum) {
      absl::Status enum_validation_status = absl::OkStatus();
      VisitIntegerField(field,
                        EnumFieldVisitor{.self = *this,
                                         .schema = entry.schema,
                                         .object_def = entry.object_def,
                                         .object_ptr = entry.object_ptr,
                                         .status = enum_validation_status});
      MP_RETURN_IF_ERROR(enum_validation_status)
          << absl::StrCat(" (", field_path, ")");
    }

    // Handle field types that are objects, vectors/arrays of objects, or
    // unions of objects.
    if (field_base_type == reflection::BaseType::Obj) {
      auto child_obj_def = entry.schema->objects()->Get(field->type()->index());
      field_path.back().type = child_obj_def->name()->string_view();
      if (entry.object_def->is_struct()) {
        auto struct_object =
            std::get<const flatbuffers::Struct*>(entry.object_ptr);
        stack_.push(StackEntry{
            .object_ptr =
                struct_object->template GetStruct<const flatbuffers::Struct*>(
                    field->offset()),
            .object_def = child_obj_def,
            .schema = entry.schema,
            .parent_path = field_path});
      } else {
        auto table_object =
            std::get<const flatbuffers::Table*>(entry.object_ptr);
        if (child_obj_def->is_struct()) {
          stack_.push(StackEntry{
              .object_ptr =
                  table_object->template GetStruct<const flatbuffers::Struct*>(
                      field->offset()),
              .object_def = child_obj_def,
              .schema = entry.schema,
              .parent_path = field_path});
        } else {
          stack_.push(StackEntry{
              .object_ptr =
                  table_object->template GetPointer<const flatbuffers::Table*>(
                      field->offset()),
              .object_def = child_obj_def,
              .schema = entry.schema,
              .parent_path = field_path});
        }
      }
    } else if (field_base_type == reflection::BaseType::Union) {
      auto table_object = std::get<const flatbuffers::Table*>(entry.object_ptr);
      MP_ASSIGN_OR_RETURN(auto child_obj_def,
                       GetUnionChildObjectDef(entry.schema, field, table_object,
                                              entry.object_def),
                       _ << absl::StrCat(" (", field_path, ")"));
      if (child_obj_def.has_value()) {
        field_path.back().type = child_obj_def.value()->name()->string_view();
        ObjectPtr child_obj_ptr;
        if (child_obj_def.value()->is_struct()) {
          child_obj_ptr =
              table_object->template GetStruct<const flatbuffers::Struct*>(
                  field->offset());
        } else {
          child_obj_ptr =
              table_object->template GetPointer<const flatbuffers::Table*>(
                  field->offset());
        }
        stack_.push(StackEntry{.object_ptr = child_obj_ptr,
                               .object_def = child_obj_def.value(),
                               .schema = entry.schema,
                               .parent_path = field_path});
      }
    } else if (field_base_type == reflection::BaseType::Array &&
               field->type()->element() == reflection::BaseType::Obj &&
               entry.object_def->is_struct()) {
      auto child_obj_def = entry.schema->objects()->Get(field->type()->index());
      field_path.back().type = child_obj_def->name()->string_view();
      auto struct_object =
          std::get<const flatbuffers::Struct*>(entry.object_ptr);
      for (int i = 0; i < field->type()->fixed_length(); ++i) {
        stack_.push(StackEntry{
            .object_ptr =
                struct_object->template GetStruct<const flatbuffers::Struct*>(
                    field->offset() + i * child_obj_def->bytesize()),
            .object_def = child_obj_def,
            .schema = entry.schema,
            .parent_path = field_path});
      }
    } else if (field_base_type == reflection::BaseType::Vector) {
      field_path.back().is_vector = true;
      if (field->type()->element() == reflection::BaseType::Obj) {
        auto child_obj_def =
            entry.schema->objects()->Get(field->type()->index());
        field_path.back().type = child_obj_def->name()->string_view();
        auto table_object =
            std::get<const flatbuffers::Table*>(entry.object_ptr);
        if (child_obj_def->is_struct()) {
          // Structs are stored inline in the vector.
          auto vec =
              table_object->template GetPointer<const flatbuffers::Vector<
                  flatbuffers::Offset<flatbuffers::Struct>>*>(field->offset());
          for (decltype(vec->size()) i = 0; i < vec->size(); ++i) {
            Path entry_path = field_path;
            PathSegment segment;
            segment.index = i;
            entry_path.push_back(segment);

            stack_.push(StackEntry{
                .object_ptr = reinterpret_cast<const flatbuffers::Struct*>(
                    vec->Data() + i * child_obj_def->bytesize()),
                .object_def = child_obj_def,
                .schema = entry.schema,
                .parent_path = entry_path});
          }
        } else {
          // Tables are stored as offsets in the vector.
          auto vec =
              table_object->template GetPointer<const flatbuffers::Vector<
                  flatbuffers::Offset<flatbuffers::Table>>*>(field->offset());
          for (decltype(vec->size()) i = 0; i < vec->size(); ++i) {
            Path entry_path = field_path;
            PathSegment segment;
            segment.index = i;
            entry_path.push_back(segment);
            stack_.push(StackEntry{.object_ptr = vec->Get(i),
                                   .object_def = child_obj_def,
                                   .schema = entry.schema,
                                   .parent_path = entry_path});
          }
        }
      } else if (field->type()->element() == reflection::BaseType::Union) {
        field_path.back().type = "Union";

        constexpr std::string_view kUnionTypeFieldSuffix = "_type";
        const std::string type_field_name =
            absl::StrCat(field->name()->string_view(), kUnionTypeFieldSuffix);
        auto type_field =
            entry.object_def->fields()->LookupByKey(type_field_name.c_str());
        auto union_def =
            entry.schema->enums()->Get(type_field->type()->index());

        auto table_object =
            std::get<const flatbuffers::Table*>(entry.object_ptr);
        auto union_type_vec =
            table_object
                ->template GetPointer<const flatbuffers::Vector<uint8_t>*>(
                    type_field->offset());
        auto union_vec = table_object->template GetPointer<
            const flatbuffers::Vector<flatbuffers::Offset<>>*>(field->offset());
        for (flatbuffers::uoffset_t i = 0; i < union_type_vec->size(); ++i) {
          Path entry_path = field_path;
          PathSegment segment;
          segment.index = i;
          entry_path.push_back(segment);

          auto type_enumval =
              union_def->values()->LookupByKey(union_type_vec->Get(i));
          if (type_enumval == nullptr) {
            return absl::NotFoundError(absl::StrCat(
                "Definition for enum value ", union_type_vec->Get(i),
                " is not found in the schema. (", entry_path, ")"));
          }
          entry_path.push_back({.name = type_enumval->name()->string_view(),
                                .type = "",
                                .is_union = true});

          const reflection::Object* child_obj_def =
              entry.schema->objects()->Get(type_enumval->union_type()->index());
          entry_path.back().type = child_obj_def->name()->string_view();
          if (child_obj_def->is_struct()) {
            // Structs are stored inline in the vector.
            stack_.push(StackEntry{
                .object_ptr = reinterpret_cast<const flatbuffers::Struct*>(
                    union_vec->Data() + i * child_obj_def->bytesize()),
                .object_def = child_obj_def,
                .schema = entry.schema,
                .parent_path = entry_path});
          } else {
            // Tables are stored as offsets in the vector.
            auto vec =
                table_object->template GetPointer<const flatbuffers::Vector<
                    flatbuffers::Offset<flatbuffers::Table>>*>(field->offset());
            stack_.push(StackEntry{.object_ptr = vec->Get(i),
                                   .object_def = child_obj_def,
                                   .schema = entry.schema,
                                   .parent_path = entry_path});
          }
        }
      }
    }
  }
  return absl::OkStatus();
}

// Validates the attributes using the given validators.
// The validators are called in the order they are provided for each
// attribute.
template <typename SchemaProvider>
absl::Status FlatbuffersAttributesValidator<SchemaProvider>::ValidateAttributes(
    const flatbuffers::Vector<
        flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/  attributes)
    const {
  for (const auto& validators : attribute_validators_) {
    MP_RETURN_IF_ERROR(validators(attributes));
  }
  return absl::OkStatus();
}

template <typename SchemaProvider>
template <typename T>
void FlatbuffersAttributesValidator<SchemaProvider>::EnumFieldVisitor::Visit(
    const reflection::Field* /*absl_nonnull*/  field) const {
  const reflection::Enum* enum_def =
      schema->enums()->Get(field->type()->index());
  T enum_value;
  if (object_def->is_struct()) {
    auto struct_ = std::get<const flatbuffers::Struct*>(object_ptr);
    enum_value = struct_->template GetField<T>(field->offset());
  } else {
    auto table = std::get<const flatbuffers::Table*>(object_ptr);
    enum_value =
        table->template GetField<T>(field->offset(), field->default_integer());
  }
  const reflection::EnumVal* enum_value_def =
      enum_def->values()->LookupByKey(enum_value);
  if (enum_value_def == nullptr) {
    status = absl::NotFoundError(absl::StrCat("Definition for enum value ",
                                              enum_value,
                                              " is not found in the schema."));
    return;
  }
  status = self.ValidateAttributes(enum_value_def->attributes());
}

template <typename SchemaProvider>
template <typename T>
void FlatbuffersAttributesValidator<SchemaProvider>::HasDefaultValueVisitor::
    Visit(const reflection::Field* /*absl_nonnull*/  field) const {
  static_assert(std::is_arithmetic_v<T>, "Unsupported type");
  T default_value;
  if constexpr (std::is_integral_v<T>) {
    default_value = static_cast<T>(field->default_integer());
  } else {
    default_value = static_cast<T>(field->default_real());
  }
  if (field->optional()) {
    has_default_value = false;
    return;
  }
  has_default_value =
      table->GetField<T>(field->offset(), default_value) == default_value;
}

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_FLATBUFFERS_API_LEVEL_VALIDATOR_H_
