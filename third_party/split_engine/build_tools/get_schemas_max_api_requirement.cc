// Copyright 2025 Google LLC
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

#include <cstdint>
#include <iostream>
#include <iterator>
#include <optional>
#include <string>

#include "absl/base/log_severity.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"
#include "absl/log/globals.h"
#include "absl/log/initialize.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/numbers.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "flatbuffers/idl.h"
#include "flatbuffers/util.h"
#include "mediapipe/framework/port/status_macros.h"

constexpr absl::string_view kRequiresApiAttribute = "requires_api";

absl::StatusOr<std::optional<int>> GetRequiresApiAttribute(
    const flatbuffers::SymbolTable<flatbuffers::Value>& attributes) {
  auto* requires_api_attr = attributes.Lookup(kRequiresApiAttribute.data());
  if (requires_api_attr == nullptr) {
    return std::nullopt;
  }
  int requires_api;
  if (!absl::SimpleAtoi(requires_api_attr->constant, &requires_api)) {
    return absl::InvalidArgumentError(
        absl::StrCat("Failed to parse ", kRequiresApiAttribute,
                     " attribute: ", requires_api_attr->constant));
  }
  return requires_api;
}

template <typename T>
absl::StatusOr<int> GetMaxRequiresApiValueFromSymbolTable(
    const flatbuffers::SymbolTable<T>& symbol_table) {
  static_assert(std::is_base_of_v<flatbuffers::Definition, T>,
                "T must derive from flatbuffers::Definition");
  int max_requires_api = 0;
  for (T* def : symbol_table.vec) {
    MP_ASSIGN_OR_RETURN(auto requires_api,
                     GetRequiresApiAttribute(def->attributes));
    if (requires_api.has_value() && *requires_api > max_requires_api) {
      max_requires_api = *requires_api;
    }
  }
  return max_requires_api;
}

absl::StatusOr<uint32_t> GetSchemaMaxRequiresApiValue(
    flatbuffers::Parser& parser) {
  int max_schema_requires_api = 0;
  // Process tables and structs (`parser.structs_` contains both).
  MP_ASSIGN_OR_RETURN(auto max_struct_requires_api,
                   GetMaxRequiresApiValueFromSymbolTable(parser.structs_));
  if (max_struct_requires_api > max_schema_requires_api) {
    max_schema_requires_api = max_struct_requires_api;
  }
  for (const auto* struct_def : parser.structs_.vec) {
    MP_ASSIGN_OR_RETURN(auto struct_fields_requires_api,
                     GetMaxRequiresApiValueFromSymbolTable(struct_def->fields));
    if (struct_fields_requires_api > max_schema_requires_api) {
      max_schema_requires_api = struct_fields_requires_api;
    }
  }

  // Process enums and unions (`parser.enums_` contains both).
  MP_ASSIGN_OR_RETURN(auto max_enum_requires_api,
                   GetMaxRequiresApiValueFromSymbolTable(parser.enums_));
  if (max_enum_requires_api > max_schema_requires_api) {
    max_schema_requires_api = max_enum_requires_api;
  }
  for (const auto* enum_def : parser.enums_.vec) {
    for (const auto* enum_val : enum_def->Vals()) {
      MP_ASSIGN_OR_RETURN(auto enum_val_requires_api,
                       GetRequiresApiAttribute(enum_val->attributes));
      if (enum_val_requires_api.has_value() &&
          *enum_val_requires_api > max_schema_requires_api) {
        max_schema_requires_api = *enum_val_requires_api;
      }
    }
  }

  return max_schema_requires_api;
}

int main(int argc, char** argv) {
  absl::InitializeLog();
  absl::SetStderrThreshold(absl::LogSeverityAtLeast::kInfo);
  absl::SetMinLogLevel(absl::LogSeverityAtLeast::kInfo);
  absl::SetProgramUsageMessage(
      absl::StrCat("Usage: ", argv[0], " INPUT_FILES..."));
  auto positional_args = absl::ParseCommandLine(argc, argv);

  if (positional_args.size() == 1) {
    LOG(ERROR) << "Missing input file.\n" << absl::ProgramUsageMessage();
    return 1;
  }

  int max_requires_api_value = 0;

  // Skip the first argument, which is the program name.
  for (auto it = std::next(positional_args.begin());
       it != positional_args.end(); ++it) {
    const char* input_filepath = *it;
    std::string contents;
    if (!flatbuffers::LoadFile(input_filepath, true, &contents)) {
      LOG(ERROR) << "Unable to load file: " << input_filepath;
      return 1;
    }

    flatbuffers::IDLOptions parser_options;
    parser_options.lang_to_generate = flatbuffers::IDLOptions::kBinary;

    flatbuffers::Parser parser(parser_options);
    if (!parser.Parse(contents.data(), nullptr, input_filepath)) {
      LOG(ERROR) << "Failed to parse input file: " << parser.error_;
      return 1;
    }

    auto max_schema_requires_api = GetSchemaMaxRequiresApiValue(parser);
    if (!max_schema_requires_api.ok()) {
      LOG(ERROR) << "Failed to get max schema requires api value: "
                 << max_schema_requires_api.status();
      return 1;
    }
    if (*max_schema_requires_api > max_requires_api_value) {
      max_requires_api_value = *max_schema_requires_api;
    }
  }
  std::cout << max_requires_api_value << std::endl;
  return 0;
}
