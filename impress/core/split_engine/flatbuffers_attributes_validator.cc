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

#include <cstdint>
#include <cstring>
#include <string>

#include "flatbuffers/base.h"
#include "absl/base/nullability.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/reflection_generated.h"
#include "flatbuffers/table.h"
#include "flatbuffers/vector.h"

namespace imp::split_engine {

absl::Status ApiLevelValidator::operator()(
    const flatbuffers::Vector<
        flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/  attributes)
    const {
  if (attributes == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Attribute '", kRequiresApiAttrName, "' not found."));
  }

  // Linear search for the requires_api attribute is faster than using the
  // bsearch lookup for a few attributes.
  const reflection::KeyValue* requires_api_attr = nullptr;
  for (const auto* attr : *attributes) {
    if (attr->key()->string_view() == kRequiresApiAttrName) {
      requires_api_attr = attr;
      break;
    }
  }

  if (requires_api_attr == nullptr) {
    return absl::NotFoundError(
        absl::StrCat("Attribute '", kRequiresApiAttrName, "' not found."));
  }

  int32_t api_level;
  if (!absl::SimpleAtoi(requires_api_attr->value()->c_str(), &api_level)) {
    return absl::InvalidArgumentError(absl::StrCat(
        "Attribute '", kRequiresApiAttrName,
        "' has invalid value: ", requires_api_attr->value()->c_str()));
  }

  if (api_level > max_api_level_) {
    return absl::PermissionDeniedError(
        absl::StrCat("Requires API level ", api_level,
                     " but the maximum API level is ", max_api_level_));
  }
  return absl::OkStatus();
}

absl::Status UnstableApiAttributeValidator::operator()(
    const flatbuffers::Vector<
        flatbuffers::Offset<reflection::KeyValue>>* /*absl_nullable*/  attributes)
    const {
  if (attributes == nullptr) {
    return absl::OkStatus();
  }

  for (const auto* attr : *attributes) {
    if (attr->key()->string_view() == kUnstableApiAttrName) {
      return absl::PermissionDeniedError(
          absl::StrCat("Attribute '", kUnstableApiAttrName,
                       "' is present but not allowed for this application."));
    }
  }

  return absl::OkStatus();
}

absl::StatusOr<const reflection::Object* /*absl_nullable*/ > GetUnionChildObjectDef(
    const reflection::Schema* /*absl_nonnull*/  schema,
    const reflection::Field* /*absl_nonnull*/  field,
    const flatbuffers::Table* /*absl_nonnull*/  parent_table,
    const reflection::Object* /*absl_nonnull*/  parent_table_def) {
  

  constexpr std::string_view kUnionTypeFieldSuffix = "_type";
  const std::string type_field_name =
      absl::StrCat(field->name()->string_view(), kUnionTypeFieldSuffix);
  auto type_field =
      parent_table_def->fields()->LookupByKey(type_field_name.c_str());
  auto type_value =
      parent_table->GetField<uint8_t>(type_field->offset(),
                                      /*defaultval=*/0 /* NONE */);
  if (type_value == 0 /* NONE */) {
    return nullptr;
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

}  // namespace imp::split_engine
