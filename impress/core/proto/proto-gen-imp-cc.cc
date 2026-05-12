// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <functional>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>



#include "google/protobuf/compiler/plugin.pb.h"
#include "google/protobuf/descriptor.pb.h"
#include "absl/container/flat_hash_map.h"
#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/ascii.h"
#include "absl/strings/escaping.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/str_join.h"
#include "absl/strings/str_replace.h"
#include "absl/strings/string_view.h"
#include "absl/strings/strip.h"
#include "core/common/hash.h"
#include "core/proto/imp.pb.h"
#include "core/proto/imp_editions.pb.h"
#include "core/proto/imp_editor.pb.h"
#include "google/protobuf/compiler/code_generator.h"
#include "google/protobuf/descriptor.h"
#include "google/protobuf/io/printer.h"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "mediapipe/framework/port/status_macros.h"

namespace {

constexpr char kApprendedImpressNamespace[] = "imp_proto";

// The name of the imp.proto file used for exposing Impress C++ generator
// specific options.
constexpr char kImpOptionsfileName[] = "proto/imp.proto";

constexpr std::array<absl::string_view, 1> kWellKnownExternalPackages = {
    "google.protobuf"};

void MakeAsciiTitlecase(std::string* s, absl::string_view delimiters) {
  bool upper = true;
  for (auto& ch : *s) {
    if (upper) {
      ch = absl::ascii_toupper(ch);
    }
    upper = (absl::StrContains(delimiters, ch));
  }
}

// Helper class to access CodeGenerator APIs from ImpCodeGenerator.
//
// TODO: Refactor ImpCodeGenerator to inherit from
// google::protobuf::compiler::CodeGenerator so that we don't need this helper class.
//
// Protobuffer team does recommend refactoring to use the CodeGenerator API
// directly eventually, however this is a larger change and they have said this
// approach is supported & allowed.
class CodeGeneratorHelper : public google::protobuf::compiler::CodeGenerator {
 public:
  template <typename DescriptorT>
  static imp::ImpressFeatureSet GetResolvedFeatureSet(
      const DescriptorT& descriptor) {
    return CodeGenerator::GetResolvedSourceFeatureExtension(
        descriptor, imp::impress_feature_set);
  }
};

class ImpCodeGenerator {
 public:
  using FilesToFileProtos =
      absl::flat_hash_map<const google::protobuf::FileDescriptor*,
                          const google::protobuf::FileDescriptorProto*>;

  ImpCodeGenerator(FilesToFileProtos files_to_file_protos)
      : file_descriptor_to_file_descriptor_protos_(
            std::move(files_to_file_protos)) {}

  bool GenerateAll(
      const std::vector<const google::protobuf::FileDescriptor*>& parsed_files,
      const std::string& parameter,
      google::protobuf::compiler::CodeGeneratorResponse* response,
      std::string* error_msg) const {
    bool success = true;
    for (const auto* file : parsed_files) {
      std::string output;
      {
        // Restrict the lifetime of ostream & printer to force 'finalizing'
        // the output string before adding it to the response.
        google::protobuf::io::StringOutputStream ostream(&output);
        google::protobuf::io::Printer printer(&ostream, '$');
        success = Generate(file, parameter, &printer, error_msg);
        if (!success && error_msg && error_msg->empty()) {
          *error_msg = "Generate failed with no error.";
        }
        if (error_msg && !error_msg->empty()) {
          *error_msg = absl::StrCat(file->name(), ": ", *error_msg);
          break;
        }
        if (!success) {
          break;
        }
      }
      auto h_name = absl::StrCat(file->name(), ".imp.h");
      google::protobuf::compiler::CodeGeneratorResponse_File* file_out =
          response->add_file();
      file_out->set_name(h_name);
      file_out->set_content(output);
    }
    return success;
  }

 private:
  absl::string_view NativeType(const google::protobuf::Descriptor* msg) const {
    return msg->options().GetExtension(imp::native_type);
  }

  bool IsScalar(const google::protobuf::FieldDescriptor* field) const {
    if (field->options().GetExtension(imp::optional_type) !=
            imp::OptionalType::ABSL_OPTIONAL &&
        field->options().GetExtension(imp::optional_type) !=
            imp::OptionalType::OPTIONAL_TYPE_UNKNOWN) {
      return false;
    }
    if (field->is_map() || field->is_repeated() ||
        field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_ENUM ||
        field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_STRING ||
        field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_MESSAGE) {
      return false;
    }
    return true;
  }

  bool ShouldAppendProtoNamespace(const google::protobuf::FileDescriptor* file) const {
    if (file->options().GetExtension(imp::impress_append_namespace)) {
      return true;
    }

    for (absl::string_view well_known_external_package :
         kWellKnownExternalPackages) {
      if (absl::StartsWith(file->package(), well_known_external_package)) {
        return true;
      }
    }

    return false;
  }

  std::string AppendProtoNamespaceToPackage(
      absl::string_view full_name, const google::protobuf::FileDescriptor* file) const {
    if (ShouldAppendProtoNamespace(file)) {
      std::string result(full_name);
      result.insert(file->package().size(),
                    absl::StrCat(".", kApprendedImpressNamespace));
      return result;
    } else {
      return std::string(full_name);
    }
  }

  std::string TypeName(const google::protobuf::Descriptor* msg,
                       bool use_proto_namespace = true) const {
    const auto native_type = NativeType(msg);
    if (!native_type.empty()) {
      return std::string(native_type);
    }
    std::string full_name(msg->full_name());

    // Postfix the type with the proto namespace. This is used to
    // differentiate Impress protos from standard google protos and prevents
    // naming collisions when both the standard and Impress versions of a proto
    // are in the same binary.
    if (use_proto_namespace) {
      full_name = AppendProtoNamespaceToPackage(full_name, msg->file());
    }

    return absl::StrReplaceAll(absl::StrCat("::", full_name), {{".", "::"}});
  }

  std::string CppTypeName(const google::protobuf::FieldDescriptor* field) const {
    std::string type_name(field->cpp_type_name());
    // Int types such as int32 are not valid modern c++ - use int32_t instead.
    if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_INT32 ||
        field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_INT64 ||
        field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_UINT32 ||
        field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_UINT64) {
      absl::StrAppend(&type_name, "_t");
    }
    return type_name;
  }

  std::string TypeName(const google::protobuf::FieldDescriptor* field) const {
    if (field->message_type() != nullptr) {
      std::string type_name = TypeName(
          field->message_type(),
          !field->options().GetExtension(imp::is_imported_standard_proto));
      const std::string& template_type =
          field->options().GetExtension(imp::template_type);
      if (!template_type.empty()) {
        absl::StrAppend(&type_name, "<", template_type, ">");
      }
      return type_name;
    } else if (field->enum_type() != nullptr) {
      std::string full_name(field->enum_type()->full_name());
      full_name = AppendProtoNamespaceToPackage(full_name, field->file());
      return absl::StrReplaceAll(full_name, {{".", "::"}});
    } else if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_STRING) {
      // Note: in open source protobuf, STRING_PIECE option is not supported.
      // Instead, we have an Impress-specific STRING_VIEW option that does the
      // same thing.
      if (field->options().GetExtension(imp::string_type) ==
          imp::StringType::STRING_VIEW) {
        // Special Impress option to support a string that points to external
        // memory.
        //
        // Standard C++ protos have a feature called aliasing, which can be
        // accessed by calling 'ParseFromStringWithAliasing'. This allows users
        // to choose *at runtime* if they want the string to reference the
        // memory of the input buffer. The aliasing feature is not currently
        // supported in Impress. This special option acts as a workaround to
        // achieve similar behavior, however it is more limited since it is a
        // compile time choice to specify the option in the proto.
        //
        // This is useful for performance optimizations when working with
        // large string data that is known to live outside of the proto object's
        // lifetime.
        return "absl::string_view";
      } else if (field->options().GetExtension(imp::string_type) ==
                 imp::StringType::CORD) {
        return "absl::Cord";
      }

      switch (field->cpp_string_type()) {
        case google::protobuf::FieldDescriptor::CppStringType::kView:
          // kView has two special meanings in the standard C++ Proto:
          //
          // 1. It is used to indicate that the generated
          // setters and getters should use std::string_view instead of
          // std::string. However, the message still holds the memory for the
          // string by default, it does *not* point to external memory. This was
          // done to enable a performance optimization and code improvement for
          // how the standard plugin uses arena allocators with strings. This
          // isn't directly applicable to the Impress plugin which uses direct
          // fields with no heap allocation.
          //
          // 2. If the message is parsed with 'ParseFromStringWithAliasing' the
          // string will point to the underlying memory of the input buffer.
          // This feature is applicable to Impress, however not currently
          // supported.
          //
          // Note: kView is the default for string fields starting in edition
          // 2024.
          //
          // For now, we just treat kView as an std::string. Generating it as a
          // string_view would mean that the data is stored externally, which is
          // not desired behavior most of the time.
          //
          // TODO: Add support for proper runtime aliasing.
          return "std::string";
        case google::protobuf::FieldDescriptor::CppStringType::kCord:
          return "absl::Cord";
        case google::protobuf::FieldDescriptor::CppStringType::kString:
          return "std::string";
      }
      return "std::string";
    } else {
      return CppTypeName(field);
    }
  }

  std::string DefaultConstantTypeName(
      const google::protobuf::FieldDescriptor* field) const {
    if (field->cpp_type() == google::protobuf::FieldDescriptor::CPPTYPE_STRING) {
      return "std::string_view";
    } else {
      return TypeName(field);
    }
  }

  bool HasDefault(const google::protobuf::FieldDescriptor* field) const {
    return field->has_default_value() ||
           field->options().HasExtension(imp::msg_default);
  }

  std::string DefaultValue(const google::protobuf::FieldDescriptor* field) const {
    if (!HasDefault(field)) {
      return "";
    }

    if (field->options().HasExtension(imp::msg_default)) {
      return field->options().GetExtension(imp::msg_default);
    }

    switch (field->cpp_type()) {
      case google::protobuf::FieldDescriptor::CPPTYPE_INT32:
        return absl::StrCat(field->default_value_int32());
      case google::protobuf::FieldDescriptor::CPPTYPE_INT64:
        return absl::StrCat(field->default_value_int64());
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT32:
        return absl::StrCat(field->default_value_uint32());
      case google::protobuf::FieldDescriptor::CPPTYPE_UINT64:
        return absl::StrCat(field->default_value_uint64());
      case google::protobuf::FieldDescriptor::CPPTYPE_DOUBLE:
        return absl::StrCat(field->default_value_double());
      case google::protobuf::FieldDescriptor::CPPTYPE_FLOAT:
        return absl::StrCat(field->default_value_float());
      case google::protobuf::FieldDescriptor::CPPTYPE_BOOL:
        return field->default_value_bool() ? "true" : "false";
      case google::protobuf::FieldDescriptor::CPPTYPE_ENUM:
        return absl::StrCat(TypeName(field),
                            "::", field->default_value_enum()->name());
      case google::protobuf::FieldDescriptor::CPPTYPE_STRING:
        return absl::StrCat("\"", field->default_value_string(), "\"");
      default:
        return "UNSUPPORTED DEFAULT VALUE";
    }
  }

  std::string DefaultValueFieldName(
      const google::protobuf::FieldDescriptor* field) const {
    std::string upper_case_name = std::string(field->camelcase_name());
    upper_case_name[0] = absl::ascii_toupper(upper_case_name[0]);
    return absl::StrCat("k", upper_case_name, "Default");
  }

  absl::StatusOr<std::pair<std::string, std::string>>
  SingleFieldInfoLegacyPresence(const google::protobuf::FieldDescriptor* field) const {
    if (field->options().HasExtension(imp::msg_default)) {
      return absl::InvalidArgumentError(absl::StrCat(
          "ERROR: field ", field->containing_type()->name(),
          "::", field->name(),
          " - imp.msg_default is not supported in legacy presence mode."));
    }

    std::string type_name;
    std::string default_value;

    if (field->options().GetExtension(imp::optional_type) ==
        imp::OptionalType::UNIQUE_PTR) {
      if (!field->has_presence() || field->is_required()) {
        return absl::InvalidArgumentError(absl::StrCat(
            "ERROR: field ", field->containing_type()->name(),
            "::", field->name(),
            " - Impress Proto UNIQUE_PTR fields must be marked with the "
            "optional label or be part of a oneof."));
      }
      type_name = absl::StrCat("::imp::CopyablePtr<", TypeName(field), ">");
    } else if (field->message_type() != nullptr &&
               field->options().GetExtension(imp::optional_type) !=
                   imp::OptionalType::ABSL_OPTIONAL) {
      type_name = TypeName(field);
      const auto native_type = NativeType(field->message_type());
      if (!native_type.empty()) {
        default_value = "{}";
      }
    } else if (field->has_presence() && !field->is_required() &&
               field->real_containing_oneof() == nullptr) {
      type_name = absl::StrCat("absl::optional<", TypeName(field), ">");
    } else if (field->enum_type() != nullptr) {
      type_name = TypeName(field);
      if (default_value.empty()) {
        default_value = field->default_value_enum()->name();
        default_value = absl::StrCat(type_name, "::", default_value);
      }
    } else {
      type_name = TypeName(field);
      if (default_value.empty() &&
          field->cpp_type() != google::protobuf::FieldDescriptor::CPPTYPE_STRING) {
        default_value = "0";
      }
    }

    return std::make_pair(type_name, default_value.empty()
                                         ? default_value
                                         : absl::StrCat(" = ", default_value));
  }

  absl::StatusOr<std::pair<std::string, std::string>>
  SingleFieldInfoOptionalWithDefaultPresence(
      const google::protobuf::FieldDescriptor* field) const {
    if (field->options().HasExtension(imp::msg_default) &&
        field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      return absl::InvalidArgumentError(absl::StrCat(
          "ERROR: field ", field->containing_type()->name(),
          "::", field->name(),
          " - imp.msg_default is only supported for message fields."));
    }

    if (field->options().GetExtension(imp::optional_type) !=
        imp::OptionalType::OPTIONAL_TYPE_UNKNOWN) {
      return absl::InvalidArgumentError(absl::StrCat(
          "ERROR: field ", field->containing_type()->name(),
          "::", field->name(),
          " - imp.optional_type is not supported in editions 2024 and later."));
    }

    bool native_type_has_presence = false;
    if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
      const google::protobuf::Descriptor* message_type = field->message_type();
      native_type_has_presence =
          message_type->options().GetExtension(imp::native_type_has_presence);

      if (native_type_has_presence &&
          message_type->options().GetExtension(imp::native_type).empty()) {
        return absl::InvalidArgumentError(absl::StrCat(
            "ERROR: field ", field->containing_type()->name(),
            "::", field->name(),
            " - imp.native_type_has_presence is only supported for "
            "messages with a native type. "));
      }

      if (native_type_has_presence &&
          !field->options().GetExtension(imp::msg_default).empty()) {
        return absl::InvalidArgumentError(
            absl::StrCat("ERROR: field ", field->containing_type()->name(),
                         "::", field->name(),
                         " - imp.msg_default is not supported for messages "
                         "with imp.native_type_has_presence. "));
      }
    }

    std::string type_name;
    std::string default_value;

    if (field->has_presence() && field->real_containing_oneof() == nullptr &&
        !native_type_has_presence) {
      if (HasDefault(field)) {
        type_name = absl::StrCat("::imp::OptionalWithDefault<", TypeName(field),
                                 ", &", DefaultValueFieldName(field), ">");
      } else {
        type_name =
            absl::StrCat("::imp::OptionalWithDefault<", TypeName(field), ">");
      }
    } else {
      // Note: Message types cannot specify IMPLICIT presence, proto compiler
      // will not allow it. Therefore, only primitive types can reach this
      // branch.
      type_name = TypeName(field);
      if (field->has_default_value()) {
        default_value = DefaultValue(field);
      } else {
        default_value = "{}";
      }
    }

    // Prepend "=" sign if default value is not empty.
    if (!default_value.empty()) {
      default_value = absl::StrCat(" = ", default_value);
    }

    return std::make_pair(type_name, default_value);
  }

  absl::StatusOr<std::pair<std::string, std::string>> FieldInfo(
      const google::protobuf::FieldDescriptor* field) const {
    // Verify that STRING_VIEW is only used for string fields.
    if (field->options().GetExtension(imp::string_type) !=
            imp::StringType::STRING_TYPE_UNKNOWN &&
        field->cpp_type() != google::protobuf::FieldDescriptor::CPPTYPE_STRING) {
      return absl::InvalidArgumentError(absl::StrCat(
          "ERROR: field ", field->containing_type()->name(),
          "::", field->name(),
          " - imp.string_type is only supported for string fields."));
    }

    if (field->is_map()) {
      const auto* map_type = field->message_type();
      const auto* key = map_type->FindFieldByNumber(1);
      const auto* value = map_type->FindFieldByNumber(2);
      std::string value_type_name = TypeName(value);
      const std::string& template_type =
          field->options().GetExtension(imp::template_type);
      if (!template_type.empty()) {
        absl::StrAppend(&value_type_name, "<", template_type, ">");
      }
      std::string type_name =
          absl::StrCat("std::map<", TypeName(key), ", ", value_type_name, ">");
      return std::make_pair(type_name, "");
    } else if (field->is_repeated()) {
      std::string type_name =
          absl::StrCat("std::vector<", TypeName(field), ">");
      return std::make_pair(type_name, "");
    } else {
      imp::ImpressFeatureSet feature_set =
          CodeGeneratorHelper::GetResolvedFeatureSet(*field);

      if (feature_set.presence_mode() ==
          imp::ImpressFeatureSet::OPTIONAL_WITH_DEFAULT) {
        return SingleFieldInfoOptionalWithDefaultPresence(field);
      } else {
        return SingleFieldInfoLegacyPresence(field);
      }
    }
  }

  absl::StatusOr<std::string> GetRepeatedMergeStrategy(
      const google::protobuf::FieldDescriptor* field) const {
    switch (field->options().GetExtension(imp::repeated_merge_strategy)) {
      case imp::IMP_REPEATED_MERGE_STRATEGY_AUTOMATIC:
        if (field->type() == google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
          return "::imp::proto::RepeatedMergeStrategy::kPerElement";
        } else {
          return "::imp::proto::RepeatedMergeStrategy::kOverwrite";
        }
      case imp::IMP_REPEATED_MERGE_STRATEGY_OVERWRITE:
        return "::imp::proto::RepeatedMergeStrategy::kOverwrite";
      case imp::IMP_REPEATED_MERGE_STRATEGY_PER_ELEMENT:
        if (field->type() != google::protobuf::FieldDescriptor::TYPE_MESSAGE) {
          return absl::FailedPreconditionError(absl::StrFormat(
              "Attempting to use COMBINE merge type on non-message field "
              "%s which is not supported.",
              field->name()));
        }

        return "::imp::proto::RepeatedMergeStrategy::kPerElement";
      case imp::IMP_REPEATED_MERGE_STRATEGY_CONCAT:
        return "::imp::proto::RepeatedMergeStrategy::kConcat";
    }
  }

  absl::Status PrintEnum(google::protobuf::io::Printer* printer,
                         const google::protobuf::EnumDescriptor* edesc) const {
    printer->Print("enum $name$ {\n", "name", edesc->name());
    printer->Indent();
    int min_value = INT32_MAX;
    int max_value = INT32_MIN;
    for (int i = 0; i < edesc->value_count(); ++i) {
      const auto* value = edesc->value(i);
      printer->Print("$name$ = $number$,\n", "name",
                     absl::StrCat(value->name()), "number",
                     absl::StrCat(value->number()));
      max_value = std::max(max_value, value->number());
      min_value = std::min(min_value, value->number());
    }
    printer->Print("\n");
    printer->Print("$name$_MIN = $number$,\n", "name", edesc->name(), "number",
                   absl::StrCat(min_value));
    printer->Print("$name$_MAX = $number$,\n", "name", edesc->name(), "number",
                   absl::StrCat(max_value));
    printer->Outdent();
    printer->Print("};\n");
    printer->PrintRaw("\n");

    return absl::OkStatus();
  }

  void PrintEnumMetaData(google::protobuf::io::Printer* printer,
                         const google::protobuf::EnumDescriptor* edesc) const {
    std::string full_name = absl::StrReplaceAll(
        AppendProtoNamespaceToPackage(edesc->full_name(), edesc->file()),
        {{".", "::"}});
    printer->Print("// Enum meta data for ::$name$\n", "name", full_name);
    printer->Print("template <>\n");
    printer->Print("struct EnumMetaData<::$name$> {\n", "name", full_name);
    printer->Indent();

    // GetName method.
    printer->Print("static constexpr absl::string_view GetName(::$name$ e) {\n",
                   "name", full_name);
    printer->Indent();
    printer->Print("switch (e) {\n");
    printer->Indent();
    for (int i = 0; i < edesc->value_count(); ++i) {
      const auto* value = edesc->value(i);
      printer->Print("case ::$full_name$ : return \"$name$\";\n", "full_name",
                     absl::StrCat(full_name, "::", value->name()), "name",
                     value->name());
    }
    printer->Outdent();
    printer->Print("}\n");
    printer->Print("return \"<UNKNOWN ENUM VALUE>\";\n");
    printer->Outdent();
    printer->Print("}\n\n");

    // FromName method.
    printer->Print(
        "static constexpr std::optional<::$name$> FromName(absl::string_view "
        "name) {\n",
        "name", full_name);
    printer->Indent();
    for (int i = 0; i < edesc->value_count(); ++i) {
      const auto* value = edesc->value(i);
      printer->Print("if (name == \"$name$\") return ::$full_name$;\n", "name",
                     value->name(), "full_name",
                     absl::StrCat(full_name, "::", value->name()));
    }
    printer->Print("\nreturn std::nullopt;\n");
    printer->Outdent();
    printer->Print("}\n\n");

    // IsValid method.
    printer->Print(
        "static constexpr bool IsValid(std::underlying_type_t<::$name$> e) {\n",
        "name", full_name);
    printer->Indent();
    printer->Print("switch (e) {\n");
    printer->Indent();
    for (int i = 0; i < edesc->value_count(); ++i) {
      const auto* value = edesc->value(i);
      printer->Print("case ::$full_name$ : return true;\n", "full_name",
                     absl::StrCat(full_name, "::", value->name()));
    }
    printer->Outdent();
    printer->Print("}\n");
    printer->Print("return false;\n");
    printer->Outdent();
    printer->Print("}\n\n");

    printer->Print("static constexpr ::$name$ kMinValue = ::$name$_MIN;\n",
                   "name", full_name);
    printer->Print("static constexpr ::$name$ kMaxValue = ::$name$_MAX;\n",
                   "name", full_name);
    printer->Print(
        "static constexpr std::array<::$name$, $num_enums$> kValues = {",
        "name", full_name, "num_enums", absl::StrCat(edesc->value_count()));
    for (int i = 0; i < edesc->value_count(); ++i) {
      const auto* value = edesc->value(i);
      printer->Print("::$full_name$, ", "full_name",
                     absl::StrCat(full_name, "::", value->name()));
    }
    printer->Print("};\n");
    printer->Outdent();
    printer->Print("};\n");
    printer->PrintRaw("\n");
  }

  void PrintEnumMetaDataBlock(google::protobuf::io::Printer* printer,
                              const std::vector<const google::protobuf::EnumDescriptor*>&
                                  enum_descriptors) const {
    printer->Print("// Proto enum meta data\n");
    printer->Print("namespace imp::proto {\n");
    for (const google::protobuf::EnumDescriptor* descriptor : enum_descriptors) {
      PrintEnumMetaData(printer, descriptor);
      printer->Print("\n");
    }
    printer->Print("} // namespace imp::proto\n\n");
  }

  void PrintEnumAbslStringify(google::protobuf::io::Printer* printer,
                              const google::protobuf::EnumDescriptor* edesc) const {
    std::string full_name = absl::StrReplaceAll(
        AppendProtoNamespaceToPackage(edesc->full_name(), edesc->file()),
        {{".", "::"}});

    printer->Print("template <typename Sink>\n");
    printer->Print("void AbslStringify(Sink& sink, const $name$& value) {\n",
                   "name", full_name);
    printer->Indent();
    printer->Print(
        "sink.Append(::imp::proto::EnumMetaData<$name$>::GetName(value));\n",
        "name", full_name);
    printer->Outdent();
    printer->Print("}\n\n");
  }

  void PrintEnumAbslStringifyBlock(
      google::protobuf::io::Printer* printer,
      const std::vector<const google::protobuf::EnumDescriptor*>& enum_descriptors,
      const std::string& package) const {
    printer->Print("// Enum AbslStringify overloads\n");
    printer->Print("namespace $package$ {\n", "package", package);
    printer->PrintRaw("\n");

    for (const google::protobuf::EnumDescriptor* edesc : enum_descriptors) {
      PrintEnumAbslStringify(printer, edesc);
    }

    printer->PrintRaw("\n");
    printer->Print("}  // namespace $package$\n", "package", package);
  }

  absl::Status PrintOneof(google::protobuf::io::Printer* printer,
                          const google::protobuf::OneofDescriptor* oneof) const {
    printer->Print("absl::variant<absl::monostate");
    for (int i = 0; i < oneof->field_count(); ++i) {
      absl::StatusOr<std::pair<std::string, std::string>> field_info_or =
          FieldInfo(oneof->field(i));
      MP_RETURN_IF_ERROR(field_info_or.status());
      auto [type_name, info] = field_info_or.value();
      printer->Print(",\n              $type$", "type", type_name);
    }
    printer->Print("> $name$;\n", "name", oneof->name());
    std::string oneof_title(oneof->name());
    MakeAsciiTitlecase(&oneof_title, "_");
    printer->Print("static constexpr int k$oneof$_Unknown = 0;\n", "oneof",
                   oneof_title);
    for (int i = 0; i < oneof->field_count(); ++i) {
      std::string field_title(oneof->field(i)->camelcase_name());
      MakeAsciiTitlecase(&field_title, "");
      auto index = absl::StrCat(i + 1);
      absl::StatusOr<std::pair<std::string, std::string>> field_info_or =
          FieldInfo(oneof->field(i));
      MP_RETURN_IF_ERROR(field_info_or.status());
      auto [type_name, info] = field_info_or.value();
      std::map<std::string, std::string> vars = {
          {"oneof", oneof_title},
          {"cap", field_title},
          {"field", std::string(oneof->field(i)->name())},
          {"index", index},
          {"name", std::string(oneof->name())},
          {"type", type_name}};
      printer->Print(vars, "static constexpr int k$oneof$_$cap$ = $index$;\n");
      printer->Print(vars,
                     "const $type$* $field$() const {\n"
                     "  return absl::get_if<$index$>(&this->$name$);\n"
                     "}\n");
      printer->Print(vars,
                     "$type$* mutable_$field$() {\n"
                     "  if ($name$.index() != $index$) {\n"
                     "    this->$name$.emplace<$index$>($type${});\n"
                     "  }\n"
                     "  return absl::get_if<$index$>(&this->$name$);\n"
                     "}\n");
    }
    return absl::OkStatus();
  }

  // Stores field types of FieldDescriptor::Type and ids of fields in a oneof.
  // The index in the vector is the oneof field index excluding monostate.
  struct OneofFieldInfo {
    std::vector<int> field_types;
    std::vector<int> field_ids;
  };

  absl::Status PrintVisitAllBody(google::protobuf::io::Printer* printer,
                                 const google::protobuf::Descriptor* desc) const {
    printer->Indent();
    absl::flat_hash_map<std::string, OneofFieldInfo> printed_oneofs;
    for (int i = 0; i < desc->field_count(); ++i) {
      const google::protobuf::FieldDescriptor* field = desc->field(i);
      std::string disambiguator;
      std::string visit_method;
      if (field->is_repeated()) {
        if (field->options().GetExtension(imp::is_imported_standard_proto)) {
          return absl::FailedPreconditionError(absl::StrFormat(
              "Attempting to use is_imported_standard_proto on repeated field "
              "%s which is not supported.",
              field->name()));
        }

        MP_ASSIGN_OR_RETURN(std::string merge_type,
                         GetRepeatedMergeStrategy(field));

        disambiguator = "template ";
        visit_method =
            absl::StrFormat("Visit<%d, %s>", field->type(), merge_type);

      } else if (field->options().GetExtension(
                     imp::is_imported_standard_proto)) {
        disambiguator = "";
        visit_method = "VisitStandardProto";
      } else {
        disambiguator = "template ";
        visit_method = absl::StrFormat("Visit<%d>", field->type());
      }
      field->options().GetExtension(imp::is_imported_standard_proto)
          ? "VisitStandardProto"
          : absl::StrFormat("Visit<%s>", absl::StrCat(field->type()));

      const auto* oneof = field->real_containing_oneof();
      if (oneof != nullptr) {
        std::string oneof_name_capitalized(oneof->name());
        oneof_name_capitalized[0] = toupper(oneof_name_capitalized[0]);
        if (printed_oneofs.find(oneof->name()) == printed_oneofs.end()) {
          printer->Print(
              "constexpr bool kHasVisitVariant$oneof_capitalized$Fn = "
              "::imp::proto_traits::kHasVisitVariantFunction<Visitor, Cursor, "
              "decltype(this->$oneof$), std::integer_sequence<int, 0>>;\n",
              "oneof_capitalized", oneof_name_capitalized, "oneof",
              oneof->name());
          OneofFieldInfo fields;
          fields.field_types.resize(oneof->field_count());
          fields.field_ids.resize(oneof->field_count());
          printed_oneofs[oneof->name()] = fields;
        }
        // Store the field id so we can call the variant visit with the mapping.
        OneofFieldInfo& oneof_field_info = printed_oneofs[oneof->name()];
        oneof_field_info.field_types[field->index_in_oneof()] = field->type();
        oneof_field_info.field_ids[field->index_in_oneof()] = field->number();

        std::string ref_other = absl::StrCat(
            "(!other || other->", oneof->name(),
            ".index() != ", field->index_in_oneof() + 1,
            ") ? nullptr : absl::get_if<", field->index_in_oneof() + 1,
            ">(&other->", oneof->name(), ")");
        printer->Print(
            "cursor = (!kHasVisitVariant$oneof_capitalized$Fn && "
            "$oneof$.index() == $index$) ? "
            "v.$disambiguator$$visit_method$(cursor, $field_id$, "
            "absl::get_if<$index$>(&this->$oneof$), $ref_other$, "
            "std::forward<Args>(args)...) : cursor;\n",
            "oneof_capitalized", oneof_name_capitalized, "oneof", oneof->name(),
            "index", absl::StrCat(field->index_in_oneof() + 1), "disambiguator",
            disambiguator, "visit_method", visit_method, "field_id",
            absl::StrCat(field->number()), "ref_other", ref_other);

        if (field->index_in_oneof() == oneof->field_count() - 1) {
          std::string field_types_str =
              absl::StrFormat("std::integer_sequence<signed,%s>{}",
                              absl::StrJoin(oneof_field_info.field_types, ","));
          std::string field_ids_str = absl::StrFormat(
              "{%s}", absl::StrJoin(oneof_field_info.field_ids, ","));
          std::string oneof_name_capitalized(oneof->name());
          oneof_name_capitalized[0] = toupper(oneof_name_capitalized[0]);
          printer->Print(
              "if constexpr (kHasVisitVariant$oneof_capitalized$Fn) {\n"
              "  cursor = v.VisitVariant(cursor, &this->$oneof$, !other ? "
              "nullptr : &other->$oneof$, \"$oneof$\", $field_types_str$, "
              "$field_ids_str$, std::forward<Args>(args)...);\n"
              "}\n",
              "oneof_capitalized", oneof_name_capitalized, "oneof",
              oneof->name(), "field_types_str", field_types_str,
              "field_ids_str", field_ids_str);
        }
        continue;
      }
      std::string ref = absl::StrCat("&this->", field->name());
      std::string ref_other =
          absl::StrCat("!other ? nullptr : &other->", field->name());
      if (IsScalar(field) && !NativeType(desc).empty()) {
        ref = absl::StrCat("reinterpret_cast<", CppTypeName(field),
                           "*>(&this->", field->name(), ")");
        ref_other =
            absl::StrCat("!other ? nullptr : reinterpret_cast<",
                         CppTypeName(field), "*>(&other->", field->name(), ")");
      }
      if (field->is_map()) {
        const auto* map_type = field->message_type();
        const auto* key = map_type->FindFieldByNumber(1);
        const auto* value = map_type->FindFieldByNumber(2);
        printer->Print(
            "cursor = v.template Visit<$key_type$, $value_type$>(cursor, "
            "$field_id$, $ref$, $ref_other$, std::forward<Args>(args)...);\n",
            "key_type", absl::StrCat(key->type()), "value_type",
            absl::StrCat(value->type()), "field_id",
            absl::StrCat(field->number()), "ref", ref, "ref_other", ref_other);
      } else {
        printer->Print(
            "cursor = v.$disambiguator$$visit_method$(cursor, $field_id$, "
            "$ref$, $ref_other$, std::forward<Args>(args)...);\n",
            "disambiguator", disambiguator, "visit_method", visit_method,
            "field_id", absl::StrCat(field->number()), "ref", ref, "ref_other",
            ref_other);
      }
    }
    printer->Print("return cursor;\n");
    printer->Outdent();

    return absl::OkStatus();
  }

  absl::Status PrintVisitFieldBody(google::protobuf::io::Printer* printer,
                                   const google::protobuf::Descriptor* desc) const {
    printer->Indent();
    printer->Print("switch (field_id) {\n");
    printer->Indent();
    for (int i = 0; i < desc->field_count(); ++i) {
      const google::protobuf::FieldDescriptor* field = desc->field(i);
      printer->Print("case $field_id$:\n", "field_id",
                     absl::StrCat(field->number()));
      std::string field_ref;
      std::string field_ref_other;
      std::string disambiguator;
      std::string visit_method =
          field->options().GetExtension(imp::is_imported_standard_proto)
              ? "VisitStandardProto"
              : "Visit";
      std::string visit_method_template;
      const auto* oneof = field->real_containing_oneof();
      if (oneof != nullptr) {
        printer->Print(
            "  if ($oneof$.index() != $index$) {\n"
            "    $oneof$.emplace<$index$>($type_name${});\n"
            "  }\n",
            "oneof", oneof->name(), "index",
            absl::StrCat(field->index_in_oneof() + 1), "type_name",
            FieldInfo(field)->first);
        field_ref = absl::StrFormat("absl::get_if<%d>(&this->%s)",
                                    field->index_in_oneof() + 1, oneof->name());
        field_ref_other =
            absl::StrFormat("!other ? nullptr : absl::get_if<%d>(&other->%s)",
                            field->index_in_oneof() + 1, oneof->name());
      } else if (IsScalar(field) && !NativeType(desc).empty()) {
        field_ref = absl::StrFormat("reinterpret_cast<%s*>(&this->%s)",
                                    CppTypeName(field), field->name());
        field_ref_other = absl::StrFormat(
            "!other ? nullptr : reinterpret_cast<%s*>(&other->%s)",
            CppTypeName(field), field->name());
      } else {
        field_ref = absl::StrFormat("&this->%s", field->name());
        field_ref_other =
            absl::StrFormat("!other ? nullptr : &other->%s", field->name());
      }
      if (field->is_map()) {
        const auto* map_type = field->message_type();
        const auto* key = map_type->FindFieldByNumber(1);
        const auto* value = map_type->FindFieldByNumber(2);
        disambiguator = "template ";
        visit_method_template =
            absl::StrFormat("<%d, %d>", key->type(), value->type());
      } else if (field->is_repeated()) {
        if (field->options().GetExtension(imp::is_imported_standard_proto)) {
          return absl::FailedPreconditionError(absl::StrFormat(
              "Attempting to use is_imported_standard_proto on repeated field "
              "%s which is not supported.",
              field->name()));
        }

        MP_ASSIGN_OR_RETURN(std::string merge_type,
                         GetRepeatedMergeStrategy(field));

        disambiguator = "template ";
        visit_method_template =
            absl::StrFormat("<%d, %s>", field->type(), merge_type);
      } else if (!field->options().GetExtension(
                     imp::is_imported_standard_proto)) {
        disambiguator = "template ";
        visit_method_template = absl::StrFormat("<%d>", field->type());
      } else {
        disambiguator = "";
        visit_method_template = "";
      }
      printer->Print(
          "  return "
          "v.$disambiguator$$visit_method$$visit_method_template$(cursor, "
          "$field_id$, $field_ref$, $field_ref_other$, "
          "std::forward<Args>(args)...);\n",
          "disambiguator", disambiguator, "visit_method", visit_method,
          "visit_method_template", visit_method_template, "field_id",
          absl::StrCat(field->number()), "type", absl::StrCat(field->type()),
          "field_ref", field_ref, "field_ref_other", field_ref_other);
    }
    printer->Print(
        "default: return v.Unknown(cursor, field_id, "
        "std::forward<Args>(args)...);\n");
    printer->Outdent();
    printer->Print("}\n");
    printer->Outdent();

    return absl::OkStatus();
  }

  absl::Status PrintFieldInfo(google::protobuf::io::Printer* printer,
                              const google::protobuf::Descriptor* desc) const {
    if (desc->extension_count() > 0 || desc->extension_range_count() > 0) {
      return absl::FailedPreconditionError(absl::StrCat(
          "Extensions are not supported at this time: ", desc->full_name()));
    }
    for (int i = 0; i < desc->field_count(); ++i) {
      const google::protobuf::FieldDescriptor* field = desc->field(i);
      if (field->type() == google::protobuf::FieldDescriptor::TYPE_GROUP) {
        return absl::FailedPreconditionError(absl::StrCat(
            "Groups are not supported at this time: ", field->full_name()));
      }
    }
    printer->Print("static constexpr std::size_t kFieldsCount = $count$;\n",
                   "count", absl::StrCat(desc->field_count()));

    printer->Print("static constexpr int kFieldIds[] = {\n");
    printer->Indent();
    for (int i = 0; i < desc->field_count(); ++i) {
      const google::protobuf::FieldDescriptor* field = desc->field(i);
      printer->Print("$id$$next_element_token$\n", "id",
                     absl::StrCat(field->number()), "next_element_token",
                     i < (desc->field_count() - 1) ? "," : "");
    }
    printer->Outdent();
    printer->Print("};\n");

    // Prevent a circular reference of the imp_editor.proto to itself.
    if (desc->file()->name() !=
        "core/proto/imp_editor.proto") {
      // This code serializes the EditorControlType field annotation for each
      // field in the proto. This proto message is a description of how to
      // display the field in an Impress editor panel so the fields can be
      // modified at runtime.
      // The EditorControlType message is serialized and converted to base64 so
      // it is a valid string for a C++ .h file. It can then be deserialized by
      // the editor and used to interpret how to display each field.

      printer->Print(
          "static constexpr absl::string_view kFieldEditorControlTypes[] = "
          "{\n");
      printer->Indent();
      for (int i = 0; i < desc->field_count(); ++i) {
        const google::protobuf::FieldDescriptor* field = desc->field(i);
        if (!field->options().HasExtension(imp::editor_control)) {
          printer->Print("\"{}\",");
        } else {
          const imp::EditorControlType& editor_control_type =
              field->options().GetExtension(imp::editor_control);

          std::string encoded;
          absl::Base64Escape(editor_control_type.SerializeAsString(), &encoded);
          printer->Print("\"$editor_control_type$\"$next_element_token$\n",
                         "editor_control_type", encoded, "next_element_token",
                         i < (desc->field_count() - 1) ? "," : "");
        }
      }
      printer->Outdent();
      printer->Print("};\n");
    }

    printer->Print("static constexpr absl::string_view kFieldNames[] = {\n");
    printer->Indent();
    for (int i = 0; i < desc->field_count(); ++i) {
      const google::protobuf::FieldDescriptor* field = desc->field(i);
      printer->Print("\"$name$\"$next_element_token$\n", "name",
                     absl::StrCat(field->name()), "next_element_token",
                     i < (desc->field_count() - 1) ? "," : "");
    }
    printer->Outdent();
    printer->Print("};\n");

    printer->Print(
        "static constexpr ::imp::HashValue kFieldNameHashes[] = {\n");
    printer->Indent();
    for (int i = 0; i < desc->field_count(); ++i) {
      const google::protobuf::FieldDescriptor* field = desc->field(i);
      printer->Print("$hash$$next_element_token$\n", "hash",
                     absl::StrCat(imp::Hash(field->name())),
                     "next_element_token",
                     i < (desc->field_count() - 1) ? "," : "");
    }
    printer->Outdent();
    printer->Print("};\n");

    printer->Print(
        "static constexpr absl::string_view kFieldJsonNames[] = {\n");
    printer->Indent();
    for (int i = 0; i < desc->field_count(); ++i) {
      const google::protobuf::FieldDescriptor* field = desc->field(i);
      printer->Print("\"$json_name$\"$next_element_token$\n", "json_name",
                     absl::StrCat(field->json_name()), "next_element_token",
                     i < (desc->field_count() - 1) ? "," : "");
    }
    printer->Outdent();
    printer->Print("};\n");

    printer->Print(
        "static constexpr ::imp::HashValue kFieldJsonNameHashes[] = {\n");
    printer->Indent();
    for (int i = 0; i < desc->field_count(); ++i) {
      const google::protobuf::FieldDescriptor* field = desc->field(i);
      printer->Print("$json_hash$$next_element_token$\n", "json_hash",
                     absl::StrCat(imp::Hash(field->json_name())),
                     "next_element_token",
                     i < (desc->field_count() - 1) ? "," : "");
    }
    printer->Outdent();
    printer->Print("};\n");

    // Print declaration of FieldType struct used to expose the type of each
    // field at compile-time by index.
    printer->Print("template<std::size_t I>\n");
    printer->Print("struct FieldType;\n");

    return absl::OkStatus();
  }

  void PrintAbslStringify(google::protobuf::io::Printer* printer,
                          const google::protobuf::Descriptor* desc) const {
    printer->Print("template <typename Sink>\n");
    printer->Print(
        "friend void AbslStringify(Sink& sink, const $name$& message) {\n",
        "name", desc->name());
    printer->Indent();

    // Prevent circular dependency. Textproto writer depends on any.proto.
    // Note that because any is defined by an import, the file name here is
    // different from the one used when specifying includes.
    printer->Print("::imp::proto::AbslStringifyProto(sink, message);\n");

    printer->Outdent();
    printer->Print("}\n");
    printer->PrintRaw("\n");
  }

  absl::Status PrintMessage(
      google::protobuf::io::Printer* printer, const google::protobuf::Descriptor* desc,
      std::vector<const google::protobuf::EnumDescriptor*>& enum_descriptors) const {
    if (desc->options().map_entry()) {
      return absl::OkStatus();
    }

    if (desc->options().GetExtension(imp::no_codegen)) {
      return absl::OkStatus();
    }
    auto native_type = NativeType(desc);
    if (native_type.empty()) {
      if (desc->options().GetExtension(imp::is_event)) {
        printer->Print("struct $name$ : public imp::Event {\n", "name",
                       desc->name());
      } else {
        printer->Print("struct $name$ {\n", "name", desc->name());
      }

      printer->Indent();
      // type hash
      printer->Print(
          "static constexpr absl::string_view kTypeUrl =\n"
          "    \"type.googleapis.com/$name$\";\n"
          "static constexpr ::imp::HashValue kTypeUrlHash =\n"
          "    ::imp::Hash(kTypeUrl);\n\n",
          "name", desc->full_name());

      PrintAbslStringify(printer, desc);

      MP_RETURN_IF_ERROR(PrintFieldInfo(printer, desc));

      if (desc->enum_type_count() > 0) {
        // enums
        for (int i = 0; i < desc->enum_type_count(); ++i) {
          const auto* nested_enum = desc->enum_type(i);
          MP_RETURN_IF_ERROR(PrintEnum(printer, nested_enum));
          enum_descriptors.push_back(nested_enum);
        }
      }

      // messages
      for (int i = 0; i < desc->nested_type_count(); ++i) {
        const auto* nested_type = desc->nested_type(i);
        MP_RETURN_IF_ERROR(PrintMessage(printer, nested_type, enum_descriptors));
      }

      // Print field specified defaults.
      for (int i = 0; i < desc->field_count(); ++i) {
        const google::protobuf::FieldDescriptor* field = desc->field(i);

        imp::ImpressFeatureSet feature_set =
            CodeGeneratorHelper::GetResolvedFeatureSet(*field);

        if (HasDefault(field) &&
            feature_set.presence_mode() ==
                imp::ImpressFeatureSet::OPTIONAL_WITH_DEFAULT) {
          printer->Print(absl::StrCat(
              "static constexpr ", DefaultConstantTypeName(field), " ",
              DefaultValueFieldName(field), " = ", DefaultValue(field), ";\n"));
        }
      }

      // Print Field template specialization.
      for (int i = 0; i < desc->field_count(); ++i) {
        const google::protobuf::FieldDescriptor* field = desc->field(i);

        absl::StatusOr<std::pair<std::string, std::string>> field_info_or =
            FieldInfo(field);
        std::string type_name = field_info_or->first;

        // Print specialization of FieldType struct for each field by index.
        printer->Print("template<>\n");
        printer->Print("struct FieldType<$index$> {\n", "index",
                       absl::StrCat(i));
        printer->Indent();
        printer->Print("using Type = $type$;\n", "type", type_name);
        printer->Outdent();
        printer->Print("};\n");
      }

      // fields
      absl::flat_hash_set<const google::protobuf::OneofDescriptor*> oneofs_printed;
      for (int i = 0; i < desc->field_count(); ++i) {
        const google::protobuf::FieldDescriptor* field = desc->field(i);
        const auto* oneof = field->real_containing_oneof();
        absl::StatusOr<std::pair<std::string, std::string>> field_info_or =
            FieldInfo(field);
        MP_RETURN_IF_ERROR(field_info_or.status());
        auto [type_name, init] = field_info_or.value();
        if (oneof != nullptr) {
          if (oneofs_printed.insert(oneof).second) {
            MP_RETURN_IF_ERROR(PrintOneof(printer, oneof));
          }
        } else {
          printer->Print("$type$ $name$$init$;\n", "type", type_name, "name",
                         field->name(), "init", init);
        }
      }
      if (desc->options().GetExtension(imp::is_event)) {
        printer->Print(
            "bool ToAny(google::protobuf::imp_proto::Any* any) const override "
            "{\n");
        printer->Print("  any->type_url = kTypeUrl;\n");
        printer->Print(
            "  return ::imp::proto::SerializeTo<$name$>(this, &any->value);\n",
            "name", desc->name());
        printer->Print("}\n");
      }
      printer->PrintRaw("\n");
      // declare visitors
      if (NativeType(desc).empty()) {
        printer->Print(
            "template <typename Visitor, typename Cursor, typename... Args>\n");
        printer->Print(
            "Cursor Visit(Visitor& v, Cursor cursor, $type$* other, Args... "
            "args);\n",
            "type", desc->name());
        printer->PrintRaw("\n");
        printer->Print(
            "template <typename Visitor, typename Cursor, typename... Args>\n");
        printer->Print(
            "Cursor VisitField(int field_id, Visitor& v, Cursor cursor, "
            "$type$* other, "
            "Args... args);\n",
            "type", desc->name());
      }
      printer->Outdent();
      printer->Print("};\n");
      printer->PrintRaw("\n");
    }
    return absl::OkStatus();
  }

  absl::Status PrintVisitors(google::protobuf::io::Printer* printer,
                             const google::protobuf::Descriptor* desc,
                             std::string package) const {
    if (desc->options().map_entry()) {
      return absl::OkStatus();
    }

    if (desc->options().GetExtension(imp::no_codegen)) {
      return absl::OkStatus();
    }

    if (NativeType(desc).empty()) {
      for (int i = 0; i < desc->nested_type_count(); ++i) {
        const auto* nested_type = desc->nested_type(i);
        MP_RETURN_IF_ERROR(PrintVisitors(printer, nested_type, package));
      }
    }

    // Generate visitors.
    auto type_name = TypeName(desc);
    printer->Print("// Visitors for $type$\n", "type", type_name);
    bool is_native = !NativeType(desc).empty();
    auto qualifier = absl::StrCat(
        absl::StripPrefix(type_name, absl::StrCat("::", package, "::")), "::");
    if (is_native) {
      printer->Print("namespace ");
      printer->Print("imp::proto {\n");
      printer->Print("\n");

      printer->Print("template <>\n");
      printer->Print("struct ProtoMessage<$type$> : public $type$ {\n", "type",
                     type_name);
      printer->Indent();
      qualifier.clear();
      MP_RETURN_IF_ERROR(PrintFieldInfo(printer, desc));
    } else {
      printer->Print("namespace $package$ {\n", "package", package);
    }

    // Print Visit function bodies.
    printer->Print(
        "template <typename Visitor, typename Cursor, typename... Args>\n");
    printer->Print(
        "Cursor $qualifier$Visit(Visitor& v, Cursor cursor, $type$* other, "
        "Args... args) {\n",
        "qualifier", qualifier, "type", type_name);
    MP_RETURN_IF_ERROR(PrintVisitAllBody(printer, desc));
    printer->Print("}\n");
    printer->PrintRaw("\n");
    printer->Print(
        "template <typename Visitor, typename Cursor, typename... Args>\n");
    printer->Print(
        "Cursor $qualifier$VisitField(int field_id, Visitor& v, Cursor cursor, "
        "$type$* other, "
        "Args... args) {\n",
        "qualifier", qualifier, "type", type_name);
    MP_RETURN_IF_ERROR(PrintVisitFieldBody(printer, desc));
    printer->Print("}\n");
    printer->PrintRaw("\n");
    if (is_native) {
      printer->Outdent();
      printer->Print("};\n");
      printer->Print("} // namespace ");
      printer->Print("imp::proto\n");
    } else {
      printer->Print("} // namespace $package$\n", "package", package);
    }
    if (desc->options().GetExtension(imp::is_event)) {
      printer->Print("\n");
      printer->Print("namespace imp {\n");
      printer->Print("namespace type_traits {\n");
      printer->Print("  template <>\n");
      printer->Print(
          "  constexpr absl::string_view GetTypeName<$typename$>() {\n",
          "typename", TypeName(desc));
      printer->Print("    return $typename$::kTypeUrl;\n", "typename",
                     TypeName(desc));
      printer->Print("  }\n");
      printer->Print("}  // namespace type_traits\n");
      printer->Print("}  // namespace imp\n");
    }

    printer->PrintRaw("\n");

    return absl::OkStatus();
  }

  // Attempts to find which dependency file within the root file the msg passed
  // in belongs to. Returns nullptr if it isn't found.
  const google::protobuf::FileDescriptor* FindDependencyContainingMessage(
      const google::protobuf::FileDescriptor* file, const google::protobuf::Descriptor* msg) const {
    for (int i = 0; i < file->dependency_count(); ++i) {
      const google::protobuf::FileDescriptor* dependency = file->dependency(i);
      if (dependency == msg->file()) {
        return dependency;
      }
    }
    return nullptr;
  }

  bool FillStandardProtoFileNames(const google::protobuf::Descriptor* desc,
                                  absl::flat_hash_set<absl::string_view>&
                                      out_imported_standard_proto_file_names,
                                  std::string* error) const {
    for (int i = 0; i < desc->field_count(); ++i) {
      const google::protobuf::FieldDescriptor* field_desc = desc->field(i);
      if (field_desc->options().GetExtension(imp::is_imported_standard_proto)) {
        const google::protobuf::FileDescriptor* dependency =
            FindDependencyContainingMessage(desc->file(),
                                            field_desc->message_type());
        if (!dependency) {
          *error = "Unable to find dependency containing proto message type ";
          *error += field_desc->message_type()->name();
          *error += " that uses an external standard google3 proto.";
          return false;
        }
        out_imported_standard_proto_file_names.emplace(dependency->name());
      }
    }

    for (int i = 0; i < desc->nested_type_count(); ++i) {
      const google::protobuf::Descriptor* nested_desc = desc->nested_type(i);
      if (nested_desc->file() == desc->file()) {
        if (!FillStandardProtoFileNames(
                nested_desc, out_imported_standard_proto_file_names, error)) {
          return false;
        }
      }
    }
    return true;
  }

  bool IsDependencyUsedByMessage(
      const google::protobuf::Descriptor* desc,
      const google::protobuf::FileDescriptor* dependency) const {
    for (int i = 0; i < desc->field_count(); ++i) {
      const google::protobuf::FieldDescriptor* field_desc = desc->field(i);
      if (field_desc->message_type() &&
          field_desc->message_type()->file() == dependency) {
        return true;
      }

      if (field_desc->enum_type() &&
          field_desc->enum_type()->file() == dependency) {
        return true;
      }
    }

    for (int i = 0; i < desc->nested_type_count(); ++i) {
      const google::protobuf::Descriptor* nested_desc = desc->nested_type(i);
      if (IsDependencyUsedByMessage(nested_desc, dependency)) {
        return true;
      }
    }

    return false;
  }

  bool IsDependencyUsed(const google::protobuf::FileDescriptor* file,
                        const google::protobuf::FileDescriptor* dependency) const {
    for (int i = 0; i < file->message_type_count(); ++i) {
      if (IsDependencyUsedByMessage(file->message_type(i), dependency)) {
        return true;
      }
    }

    return false;
  }

  bool PrintFile(
      google::protobuf::io::Printer* printer, const google::protobuf::FileDescriptor* file,
      absl::flat_hash_set<const google::protobuf::FileDescriptor*>* public_dependencies,
      std::string* error) const {
    public_dependencies->insert(file);

    // Search through the fields within the messages in this file and identify
    // any the are marked as a standard google proto instead of as an Impress
    // proto.
    absl::flat_hash_set<absl::string_view> imported_standard_proto_file_names;
    for (int i = 0; i < file->message_type_count(); ++i) {
      const google::protobuf::Descriptor* desc = file->message_type(i);
      FillStandardProtoFileNames(desc, imported_standard_proto_file_names,
                                 error);
    }

    // Include dependencies.
    for (int i = 0; i < file->dependency_count(); ++i) {
      if (public_dependencies->count(file->dependency(i)) > 0) {
        continue;
      }

      // Strip out dependencies that aren't actually used.
      if (!IsDependencyUsed(file, file->dependency(i))) {
        continue;
      }

      absl::string_view include = file->dependency(i)->name();
      if (include.empty() || absl::EndsWith(include, kImpOptionsfileName)) {
        // Skip the Imp Options file, it's only used by the generator not at
        // runtime.
        continue;
      }
      if (include == "google/protobuf/any.proto") {
        include = "core/proto/any.proto";
      }

      bool is_google3_include =
          imported_standard_proto_file_names.contains(include);
      if (is_google3_include) {
        printer->Print("#include \"$include$.h\"\n", "include",
                       std::string(include));
      } else {
        printer->Print("#include \"$include$.imp.h\"\n", "include",
                       std::string(include));
      }
    }

    printer->Print("\n");
    // Include native includes.
    std::set<std::string> native_includes;
    bool has_event_message = false;
    for (int i = 0; i < file->message_type_count(); ++i) {
      const google::protobuf::MessageOptions& msg_options =
          file->message_type(i)->options();

      for (const std::string& native_include_string :
           msg_options.GetRepeatedExtension(imp::native_include)) {
        native_includes.insert(native_include_string);
      }

      if (msg_options.GetExtension(imp::is_event)) {
        has_event_message = true;
      }
    }

    if (has_event_message) {
      printer->Print(
          "#include "
          "\"core/ncsb/dispatcher/event.h\"\n");
      printer->Print(
          "#include "
          "\"core/common/type_traits.h\"\n");
      printer->Print(
          "#include "
          "\"core/proto/proto_writer.h\"\n");
    }

    for (const auto& include : native_includes) {
      printer->Print("#include \"$include$\"\n", "include", include);
    }

    // Feature only supported in editions 2023 and later.
    if (GetEdition(file) >= google::protobuf::Edition::EDITION_2023) {
      printer->Print(
          "#include "
          "\"core/common/optional_with_default.h\"\n");
    }

    printer->Print("\n");

    auto package = absl::StrReplaceAll(file->package(), {{".", "::"}});
    // Emit definitions.
    if (ShouldAppendProtoNamespace(file)) {
      absl::StrAppend(&package, "::", kApprendedImpressNamespace);
    }

    if (file->enum_type_count() > 0 || file->message_type_count() > 0) {
      std::vector<const google::protobuf::EnumDescriptor*> enum_descriptors;
      printer->Print("namespace $package$ {\n", "package", package);
      printer->Print("\n");
      if (file->enum_type_count() > 0) {
        for (int i = 0; i < file->enum_type_count(); ++i) {
          absl::Status status = PrintEnum(printer, file->enum_type(i));
          if (!status.ok()) {
            *error = status.ToString();
            return false;
          }
          printer->Print("\n");
          enum_descriptors.push_back(file->enum_type(i));
        }
      }
      // Forward declare non-native messages.
      for (int i = 0; i < file->message_type_count(); ++i) {
        const auto* msg = file->message_type(i);
        if (NativeType(msg).empty()) {
          printer->Print("struct $name$;\n", "name", msg->name());
        }
      }
      printer->Print("\n");
      // Emit non-native messages.
      for (int i = 0; i < file->message_type_count(); ++i) {
        const auto* msg = file->message_type(i);
        if (NativeType(msg).empty()) {
          absl::Status status = PrintMessage(printer, msg, enum_descriptors);
          if (!status.ok()) {
            *error = status.ToString();
            return false;
          }
        }
      }
      printer->Print("} // namespace $package$\n", "package", package);
      printer->Print("\n");

      // Emit visitors.
      for (int i = 0; i < file->message_type_count(); ++i) {
        absl::Status status =
            PrintVisitors(printer, file->message_type(i), package);
        if (!status.ok()) {
          *error = status.ToString();
          return false;
        }
      }

      PrintEnumMetaDataBlock(printer, enum_descriptors);
      PrintEnumAbslStringifyBlock(printer, enum_descriptors, package);
    }
    return true;
  }

  bool Generate(const google::protobuf::FileDescriptor* file,
                const std::string& parameter, google::protobuf::io::Printer* printer,
                std::string* error) const {
    auto h_name = absl::StrCat(file->name(), ".imp.h");

    auto include_guard = absl::AsciiStrToUpper(
        absl::StrReplaceAll(h_name, {{".", "_"}, {"/", "_"}}));

    printer->Print("#ifndef $guard$_\n", "guard", include_guard);
    printer->Print("#define $guard$_\n", "guard", include_guard);
    printer->Print("\n");
    printer->Print("#include <cstdint>\n");
    printer->Print("#include <map>\n");
    printer->Print("#include <memory>\n");
    printer->Print("#include <string>\n");
    printer->Print("#include <vector>\n");
    printer->Print("#include \"absl/strings/cord.h\"\n");
    printer->Print("#include \"absl/strings/string_view.h\"\n");
    printer->Print("#include \"absl/types/optional.h\"\n");
    printer->Print("#include \"absl/types/variant.h\"\n");
    printer->Print(
        "#include \"core/proto/proto_common.h\"\n");
    // Prevent circular dependency. Textproto writer depends on any.proto.
    if (file->name() != "core/proto/any.proto") {
      printer->Print(
          "#include \"core/proto/proto_stringify.h\"\n");
    }
    printer->Print(
        "#include \"core/proto/proto_traits.h\"\n");
    printer->Print("#include \"core/common/hash.h\"\n");
    printer->Print("\n");
    // Inject public dependencies.
    absl::flat_hash_set<const google::protobuf::FileDescriptor*> public_dependencies;
    for (int i = 0; i < file->public_dependency_count(); ++i) {
      const google::protobuf::FileDescriptor* dependency = file->public_dependency(i);
      if (absl::EndsWith(dependency->name(), kImpOptionsfileName)) {
        // Skip the Imp Options file, it's only used by the generator not at
        // runtime.
        continue;
      }

      if (!PrintFile(printer, dependency, &public_dependencies, error)) {
        return false;
      }
    }
    if (!PrintFile(printer, file, &public_dependencies, error)) {
      return false;
    }
    printer->Print("#endif // $guard$_\n", "guard", include_guard);
    return true;
  }

  // Returns the edition of the proto file.
  //
  // In most cases, the code generator does not need to know what syntax/edition
  // the proto file was written in. The FileDescriptor tells the code generator
  // what to generate, and the syntax simply influences what data the
  // FileDescriptor contains.
  //
  // However, in some cases we want to branch the code generation based on
  // syntax/edition to improve the generated code for newer editions/syntaxes
  // without breaking existing protos written in older editions/syntaxes.
  ::google::protobuf::Edition GetEdition(const google::protobuf::FileDescriptor* file) const {
    auto it = file_descriptor_to_file_descriptor_protos_.find(file);
    if (it == file_descriptor_to_file_descriptor_protos_.end()) {
      return google::protobuf::Edition::EDITION_UNKNOWN;
    }
    return it->second->edition();
  }

  // Stores a mapping of FileDescriptor to FileDescriptorProto.
  //
  // The FileDescriptorProto contains metadata about the file that isn't
  // available in the FileDescriptor itself.
  //
  // This is used to detect what syntax/edition the proto file was written in.
  FilesToFileProtos file_descriptor_to_file_descriptor_protos_;
};

bool GenerateCode(const google::protobuf::compiler::CodeGeneratorRequest& request,
                  google::protobuf::compiler::CodeGeneratorResponse* response,
                  std::string* error_msg) {
  absl::flat_hash_map<std::string, const google::protobuf::FileDescriptorProto*>
      source_file_descriptors_by_name;
  google::protobuf::DescriptorPool pool;

  for (int i = 0; i < request.source_file_descriptors_size(); ++i) {
    const ::google::protobuf::FileDescriptorProto& file_proto =
        request.source_file_descriptors(i);
    source_file_descriptors_by_name[file_proto.name()] = &file_proto;
  }

  for (int i = 0; i < request.proto_file_size(); ++i) {
    const ::google::protobuf::FileDescriptorProto* file_proto = &request.proto_file(i);

    // If the file is available via source_file_descriptors, use that instead.
    // This allows features that use RETENTION_SOURCE to be detected correctly.
    //
    // The order the files are built in must be preserved according to the
    // proto_file field ordering, so the map is used to look up the file.
    auto itr = source_file_descriptors_by_name.find(file_proto->name());
    if (itr != source_file_descriptors_by_name.end()) {
      file_proto = itr->second;
    }

    const ::google::protobuf::FileDescriptor* file = pool.BuildFile(*file_proto);
    if (!file) {
      return false;
    }
  }

  std::vector<const google::protobuf::FileDescriptor*> parsed_files;
  ImpCodeGenerator::FilesToFileProtos files_to_file_protos;

  

  for (int i = 0; i < request.file_to_generate_size(); ++i) {
    const ::google::protobuf::FileDescriptor* file =
        pool.FindFileByName(request.file_to_generate(i));

    const ::google::protobuf::FileDescriptorProto& file_proto =
        request.source_file_descriptors(i);
    

    parsed_files.push_back(file);
    files_to_file_protos[file] = &file_proto;

    if (parsed_files.back() == nullptr) {
      *error_msg =
          "protoc asked to generate code, but didn't provide "
          "a descriptor for " +
          request.file_to_generate(i);
      return false;
    }
  }

  std::string error;
  ImpCodeGenerator generator(std::move(files_to_file_protos));
  bool success = generator.GenerateAll(parsed_files, request.parameter(),
                                       response, &error);
  if (!success && error.empty()) {
    error = "code generator failed but didn't set error message.";
  }
  if (!error.empty()) {
    response->set_error(error);
  }
  return true;
}

}  // namespace

int main(int argc, char* argv[]) {
  
  

  if (argc > 1) {
    std::cerr << argv[0] << " unknown option: " << argv[1] << std::endl;
    return 1;
  }

  google::protobuf::compiler::CodeGeneratorRequest request;
  if (!request.ParseFromFileDescriptor(STDIN_FILENO)) {
    std::cerr << argv[0] << ": protoc sent unparseable request" << std::endl;
    return 1;
  }

  std::string error_msg;
  google::protobuf::compiler::CodeGeneratorResponse response;
  response.set_supported_features(
      google::protobuf::compiler::CodeGeneratorResponse::FEATURE_PROTO3_OPTIONAL |
      google::protobuf::compiler::CodeGeneratorResponse::FEATURE_SUPPORTS_EDITIONS);
  response.set_minimum_edition(google::protobuf::Edition::EDITION_PROTO2);
  response.set_maximum_edition(google::protobuf::Edition::EDITION_2024);

  if (GenerateCode(request, &response, &error_msg)) {
    if (!response.SerializeToFileDescriptor(STDOUT_FILENO)) {
      std::cerr << argv[0] << ": Error writing to stdout." << std::endl;
      return 1;
    }
  } else {
    if (!error_msg.empty()) {
      std::cerr << argv[0] << ": " << error_msg << std::endl;
    }
    return 1;
  }

  return 0;
}
