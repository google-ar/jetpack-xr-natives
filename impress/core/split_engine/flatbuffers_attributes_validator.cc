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

#include "core/split_engine/flatbuffers_attributes_validator.h"

#include <algorithm>
#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <vector>

#include "flatbuffers/base.h"
#include "absl/base/nullability.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/reflection.h"
#include "flatbuffers/reflection_generated.h"
#include "flatbuffers/struct.h"
#include "flatbuffers/table.h"
#include "flatbuffers/vector.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::split_engine {

// Returns the value of the attribute with the given name, parsed as the given
// type. If the attribute is not found or if the value cannot be parsed, returns
// an error.
template <typename T>
absl::StatusOr<T> GetAttributeValue(
    std::string_view attribute_name,
    const flatbuffers::Vector<
        flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/ attributes) {
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

// Specialization for bool attributes to handle flag behavior.
// If the attribute does not have a value it is handled as a flag: true if the
// attribute is present, false otherwise.
// If the attribute has a value, it is parsed as a boolean.
template <>
absl::StatusOr<bool> GetAttributeValue<bool>(
    std::string_view attribute_name,
    const flatbuffers::Vector<
        flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/ attributes) {
  if (attributes == nullptr) {
    return false;
  }
  const reflection::KeyValue* attribute =
      attributes->LookupByKey(attribute_name.data());
  if (attribute == nullptr) {
    return false;
  }

  std::string_view string_value = attribute->value()->string_view();
  // `0` is the default value for attributes when the value is not specified.
  if (string_value == "0") {
    return true;
  }
  bool value;
  if (!absl::SimpleAtob(string_value, &value)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Attribute '", attribute_name, "' has invalid value: ", string_value));
  }
  return value;
}

// Validates that the given API level is less than or equal to the maximum API
// level.
struct ApiLevelValidator {
  static constexpr std::string_view kRequiresApiAttrName = "requires_api";

  int32_t max_api_level;

  absl::Status operator()(
      const flatbuffers::Vector<
          flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/ attributes)
      const {
    MP_ASSIGN_OR_RETURN(int32_t api_level, GetAttributeValue<int32_t>(
                                            kRequiresApiAttrName, attributes));
    if (api_level > max_api_level) {
      return absl::PermissionDeniedError(
          absl::StrCat("Requires API level ", api_level,
                       " but the maximum API level is ", max_api_level));
    }
    return absl::OkStatus();
  }
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
    
  }
}

absl::StatusOr<std::optional<const reflection::Object*>> GetUnionChildObjectDef(
    const reflection::Schema* /*absl_nonnull*/ schema,
    const reflection::Field* /*absl_nonnull*/ field,
    const flatbuffers::Table* /*absl_nonnull*/ parent_table,
    const reflection::Object* /*absl_nonnull*/ parent_table_def) {
  

  constexpr std::string_view kUnionTypeFieldSuffix = "_type";
  const std::string type_field_name =
      absl::StrCat(field->name()->string_view(), kUnionTypeFieldSuffix);
  auto type_field =
      parent_table_def->fields()->LookupByKey(type_field_name.c_str());
  auto type_value =
      parent_table->GetField<uint8_t>(type_field->offset(),
                                      /*defaultval=*/0 /* NONE */);
  if (type_value == 0 /* NONE */) {
    return std::nullopt;
  }
  auto enumdef = schema->enums()->Get(field->type()->index());
  auto enumval = enumdef->values()->LookupByKey(type_value);
  if (enumval == nullptr) {
    return absl::NotFoundError(absl::StrCat("Definition for union type ",
                                            type_value,
                                            " is not found in the schema."));
  }
  return schema->objects()->Get(enumval->union_type()->index());
}

struct HasDefaultValueVisitor {
  const flatbuffers::Table* table;
  bool& has_default_value;

  template <typename T>
  void Visit(const reflection::Field* /*absl_nonnull*/ field) const {
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
};

FlatbuffersAttributesValidator::FlatbuffersAttributesValidator(
    Options options) {
  if (options.max_api_level.has_value()) {
    attribute_validators_.push_back(ApiLevelValidator{*options.max_api_level});
  }
}

absl::Status FlatbuffersAttributesValidator::ValidateImpl() {
  while (!stack_.empty()) {
    StackEntry entry = stack_.top();
    stack_.pop();
    if (visited_.contains(entry.object_ptr)) {
      continue;
    }
    auto status = ValidateObject(entry);
    if (!status.ok()) {
      return status;
    }
    visited_.insert(entry.object_ptr);
  }
  return absl::OkStatus();
}

void FlatbuffersAttributesValidator::Cleanup() {
  visited_.clear();
  while (!stack_.empty()) {
    stack_.pop();
  }
}

absl::Status FlatbuffersAttributesValidator::ValidateObject(
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
            .object_ptr = struct_object->GetStruct<const flatbuffers::Struct*>(
                field->offset()),
            .object_def = child_obj_def,
            .schema = entry.schema,
            .parent_path = field_path});
      } else {
        auto table_object =
            std::get<const flatbuffers::Table*>(entry.object_ptr);
        if (child_obj_def->is_struct()) {
          stack_.push(StackEntry{
              .object_ptr = table_object->GetStruct<const flatbuffers::Struct*>(
                  field->offset()),
              .object_def = child_obj_def,
              .schema = entry.schema,
              .parent_path = field_path});
        } else {
          stack_.push(StackEntry{
              .object_ptr = table_object->GetPointer<const flatbuffers::Table*>(
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
          child_obj_ptr = table_object->GetStruct<const flatbuffers::Struct*>(
              field->offset());
        } else {
          child_obj_ptr = table_object->GetPointer<const flatbuffers::Table*>(
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
            .object_ptr = struct_object->GetStruct<const flatbuffers::Struct*>(
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
          auto vec = table_object->GetPointer<const flatbuffers::Vector<
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
          auto vec = table_object->GetPointer<const flatbuffers::Vector<
              flatbuffers::Offset<flatbuffers::Table>>*>(field->offset());
          for (decltype(vec->size()) i = 0; i < vec->size(); ++i) {
            PathSegment segment;
            segment.index = i;
            field_path.push_back(segment);
            stack_.push(StackEntry{.object_ptr = vec->Get(i),
                                   .object_def = child_obj_def,
                                   .schema = entry.schema,
                                   .parent_path = field_path});
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
            table_object->GetPointer<const flatbuffers::Vector<uint8_t>*>(
                type_field->offset());
        auto union_vec =
            table_object
                ->GetPointer<const flatbuffers::Vector<flatbuffers::Offset<>>*>(
                    field->offset());
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
            auto vec = table_object->GetPointer<const flatbuffers::Vector<
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
// The validators are called in the order they are provided for each attribute.
absl::Status FlatbuffersAttributesValidator::ValidateAttributes(
    const flatbuffers::Vector<
        flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/ attributes)
    const {
  for (const auto& validators : attribute_validators_) {
    MP_RETURN_IF_ERROR(validators(attributes));
  }
  return absl::OkStatus();
}

template <typename T>
void FlatbuffersAttributesValidator::EnumFieldVisitor::Visit(
    const reflection::Field* /*absl_nonnull*/ field) const {
  const reflection::Enum* enum_def =
      schema->enums()->Get(field->type()->index());
  T enum_value;
  if (object_def->is_struct()) {
    auto struct_ = std::get<const flatbuffers::Struct*>(object_ptr);
    enum_value = struct_->GetField<T>(field->offset());
  } else {
    auto table = std::get<const flatbuffers::Table*>(object_ptr);
    enum_value = table->GetField<T>(field->offset(), field->default_integer());
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

}  // namespace imp::split_engine
