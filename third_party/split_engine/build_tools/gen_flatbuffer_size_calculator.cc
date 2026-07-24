// Copyright 2026 Google LLC
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
#include <cstddef>
#include <iterator>
#include <string>
#include <vector>

#include "absl/base/log_severity.h"
#include "absl/flags/flag.h"
#include "absl/flags/parse.h"
#include "absl/flags/usage.h"
#include "absl/log/globals.h"
#include "absl/log/initialize.h"
#include "core/common/log.h"
#include "absl/strings/match.h"
#include "absl/strings/str_cat.h"
#include "flatbuffers/code_generators.h"
#include "flatbuffers/idl.h"
#include "flatbuffers/util.h"

ABSL_FLAG(std::string, out, "", "Output file path");
ABSL_FLAG(bool, scoped_enums, false, "Output scoped enum syntax (enum class)");
ABSL_FLAG(std::vector<std::string>, include_paths, {},
          "Include paths for the schema files.");

// Helper to get fully qualified or relative C++ name for a StructDef or EnumDef
template <typename DefT>
std::string GetCppName(const DefT& def, const flatbuffers::Namespace* cur_ns) {
  if (cur_ns == def.defined_namespace) {
    return def.name;
  }
  std::string res = "::";
  if (def.defined_namespace) {
    for (const auto& comp : def.defined_namespace->components) {
      res += comp + "::";
    }
  }
  res += def.name;
  return res;
}

// Helper to get fully qualified C++ name
template <typename DefT>
std::string GetFullyQualifiedName(const DefT& def) {
  std::string res = "::";
  if (def.defined_namespace) {
    for (const auto& comp : def.defined_namespace->components) {
      res += comp + "::";
    }
  }
  res += def.name;
  return res;
}

// Helper to get the C++ type name for a given FlatBuffers Type
std::string GetCppType(const flatbuffers::Type& type, bool in_vector,
                       const flatbuffers::FieldDef& field,
                       const flatbuffers::Namespace* cur_ns) {
  if (type.enum_def && !type.enum_def->is_union) {
    return GetCppName(*type.enum_def, cur_ns);
  }
  switch (type.base_type) {
    case flatbuffers::BASE_TYPE_BOOL:
      return "bool";
    case flatbuffers::BASE_TYPE_CHAR:
      return "int8_t";
    case flatbuffers::BASE_TYPE_UCHAR:
      return "uint8_t";
    case flatbuffers::BASE_TYPE_SHORT:
      return "int16_t";
    case flatbuffers::BASE_TYPE_USHORT:
      return "uint16_t";
    case flatbuffers::BASE_TYPE_INT:
      return "int32_t";
    case flatbuffers::BASE_TYPE_UINT:
      return "uint32_t";
    case flatbuffers::BASE_TYPE_LONG:
      return "int64_t";
    case flatbuffers::BASE_TYPE_ULONG:
      return "uint64_t";
    case flatbuffers::BASE_TYPE_FLOAT:
      return "float";
    case flatbuffers::BASE_TYPE_DOUBLE:
      return "double";
    case flatbuffers::BASE_TYPE_STRING:
      return "flatbuffers::Offset<flatbuffers::String>";
    case flatbuffers::BASE_TYPE_STRUCT: {
      std::string qname = GetCppName(*type.struct_def, cur_ns);
      if (type.struct_def->fixed) {
        return "const " + qname + "*";
      } else {
        return "flatbuffers::Offset<" + qname + ">";
      }
    }
    case flatbuffers::BASE_TYPE_VECTOR:
    case flatbuffers::BASE_TYPE_VECTOR64: {
      flatbuffers::Type el_type = type.VectorType();
      if (el_type.base_type == flatbuffers::BASE_TYPE_STRUCT) {
        std::string qname = GetCppName(*el_type.struct_def, cur_ns);
        if (el_type.struct_def->fixed) {
          return "flatbuffers::Offset<flatbuffers::Vector<" + qname + ">>";
        } else {
          return "flatbuffers::Offset<flatbuffers::Vector<flatbuffers::"
                 "Offset<" +
                 qname + ">>>";
        }
      } else if (el_type.base_type == flatbuffers::BASE_TYPE_STRING) {
        return "flatbuffers::Offset<flatbuffers::Vector<flatbuffers::Offset<"
               "flatbuffers::String>>>";
      } else {
        return "flatbuffers::Offset<flatbuffers::Vector<" +
               GetCppType(el_type, true, field, cur_ns) + ">>";
      }
    }
    case flatbuffers::BASE_TYPE_UNION:
      return "flatbuffers::Offset<void>";
    case flatbuffers::BASE_TYPE_UTYPE:
      return GetCppName(*type.enum_def, cur_ns);
    default:
      return "void*";
  }
}

// Helper to get the C++ parameter type for EstimateFooSize
std::string GetParamType(const flatbuffers::Type& type,
                         const flatbuffers::FieldDef& field,
                         const flatbuffers::Namespace* cur_ns) {
  return GetCppType(type, false, field, cur_ns);
}

// Helper to get default value for EstimateFooSize parameter
std::string GetDefaultValue(const flatbuffers::FieldDef& field,
                            const flatbuffers::Namespace* cur_ns) {
  const flatbuffers::Type& type = field.value.type;
  if (type.base_type == flatbuffers::BASE_TYPE_BOOL) {
    return field.value.constant == "0" ? "false" : "true";
  } else if (type.base_type == flatbuffers::BASE_TYPE_UTYPE ||
             (type.enum_def && !type.enum_def->is_union)) {
    return "static_cast<" + GetCppName(*type.enum_def, cur_ns) + ">(" +
           field.value.constant + ")";
  } else if (flatbuffers::IsScalar(type.base_type)) {
    return "static_cast<" + GetParamType(type, field, cur_ns) + ">(" +
           field.value.constant + ")";
  } else if (type.base_type == flatbuffers::BASE_TYPE_STRUCT &&
             type.struct_def->fixed) {
    return "nullptr";
  } else {
    return "0";
  }
}

class FlatbufferSizeGenerator {
 public:
  explicit FlatbufferSizeGenerator(const flatbuffers::Parser& parser);

  std::string Generate(const std::vector<std::string>& include_filepaths);

 private:
  void SetNameSpace(const flatbuffers::Namespace* ns);
  std::string GenIncludeGuard(const std::string& file_name,
                              const flatbuffers::Namespace& name_space,
                              const std::string& postfix = "");

  void GeneratePreamble(const std::string& include_guard);
  void GenerateIncludes(const std::vector<std::string>& include_filepaths);
  void GenerateForwardDeclarations();
  void GenerateTableDefinitions();
  void GenerateTableSizeEstimator(const flatbuffers::StructDef* struct_def);
  void GenerateEstimateTableSize(const flatbuffers::StructDef* struct_def);
  void GenerateFieldOffsetCalculation(const flatbuffers::StructDef* struct_def,
                                      const flatbuffers::FieldDef* field);
  void GenerateEstimateOffset(const flatbuffers::StructDef* struct_def);
  void GenerateEstimateSize(const flatbuffers::StructDef* struct_def);
  void GenerateUnionDefinitions();
  void GeneratePostamble(const std::string& include_guard);

  const flatbuffers::Parser& parser_;
  const flatbuffers::Namespace* cur_name_space_;
  flatbuffers::CodeWriter code_;
};

FlatbufferSizeGenerator::FlatbufferSizeGenerator(
    const flatbuffers::Parser& parser)
    : parser_(parser), cur_name_space_(nullptr), code_("  ") {}

void FlatbufferSizeGenerator::SetNameSpace(const flatbuffers::Namespace* ns) {
  if (cur_name_space_ == ns) {
    return;
  }
  size_t old_size = cur_name_space_ ? cur_name_space_->components.size() : 0;
  size_t new_size = ns ? ns->components.size() : 0;

  size_t common_prefix_size = 0;
  while (common_prefix_size < old_size && common_prefix_size < new_size &&
         ns->components[common_prefix_size] ==
             cur_name_space_->components[common_prefix_size]) {
    common_prefix_size++;
  }

  for (size_t j = old_size; j > common_prefix_size; --j) {
    code_.DecrementIdentLevel();
    code_ += "}  // namespace " + cur_name_space_->components[j - 1];
  }
  if (old_size != common_prefix_size) {
    code_ += "";
  }

  for (size_t j = common_prefix_size; j != new_size; ++j) {
    code_ += "namespace " + ns->components[j] + " {";
    code_.IncrementIdentLevel();
  }
  if (new_size != common_prefix_size) {
    code_ += "";
  }
  cur_name_space_ = ns;
}

std::string FlatbufferSizeGenerator::GenIncludeGuard(
    const std::string& file_name, const flatbuffers::Namespace& name_space,
    const std::string& postfix) {
  // Generate include guard.
  std::string guard = file_name;
  // Remove any non-alpha-numeric characters that may appear in a filename.
  struct IsAlnum {
    bool operator()(char c) const { return !flatbuffers::is_alnum(c); }
  };
  guard.erase(std::remove_if(guard.begin(), guard.end(), IsAlnum()),
              guard.end());
  guard = "FLATBUFFERS_GENERATED_" + guard + "_";
  // For further uniqueness, also add the namespace.
  for (const std::string& component : name_space.components) {
    guard += component + "_";
  }
  // Anything extra to add to the guard?
  if (!postfix.empty()) {
    guard += postfix + "_";
  }
  guard += "H_";
  std::transform(guard.begin(), guard.end(), guard.begin(),
                 flatbuffers::CharToUpper);
  return guard;
}

std::string FlatbufferSizeGenerator::Generate(
    const std::vector<std::string>& include_filepaths) {
  code_.Clear();
  std::string file_name;
  if (!include_filepaths.empty()) {
    std::string path = include_filepaths[0];
    size_t slash = path.find_last_of("/\\");
    if (slash != std::string::npos) path = path.substr(slash + 1);
    size_t dot = path.find_last_of('.');
    if (dot != std::string::npos) path = path.substr(0, dot);
    file_name = path;
  }
  std::string include_guard =
      file_name.empty()
          ? "FLATBUFFER_SIZE_CALCULATOR_GENERATED_H_"
          : GenIncludeGuard(file_name, *parser_.current_namespace_,
                            "SIZE_CALCULATOR");

  GeneratePreamble(include_guard);
  GenerateIncludes(include_filepaths);
  GenerateForwardDeclarations();
  GenerateTableDefinitions();
  GenerateUnionDefinitions();
  GeneratePostamble(include_guard);
  return code_.ToString();
}

// Emits warning comments, include guards, and standard headers.
void FlatbufferSizeGenerator::GeneratePreamble(
    const std::string& include_guard) {
  code_ += " // Automatically generated by gen_flatbuffer_size_calculator.cc";
  code_ += " // Do not modify!";
  code_ += "";
  code_ += "#ifndef " + include_guard;
  code_ += "#define " + include_guard;
  code_ += "";
  code_ += "#include <cstddef>";
  code_ += "#include <cstdint>";
  code_ += "#include <memory>";
  code_ += "#include <string>";
  code_ += "#include <vector>";
  code_ += "";
  code_ += "#include \"split_engine/flatbuffer_size_calculator.h\"";
}

// Resolves include paths and emits #include directives for generated headers.
void FlatbufferSizeGenerator::GenerateIncludes(
    const std::vector<std::string>& include_filepaths) {
  std::string dir_prefix;
  if (!include_filepaths.empty()) {
    std::string first_path = include_filepaths[0];
    size_t slash_pos = first_path.find_last_of("/\\");
    if (slash_pos != std::string::npos) {
      dir_prefix = first_path.substr(0, slash_pos + 1);
    }
  }

  for (const auto& filepath : include_filepaths) {
    std::string header_path =
        filepath.substr(0, filepath.find_last_of('.')) + "_generated.h";
    code_ += "#include \"" + header_path + "\"";
  }
  for (const auto& inc : parser_.GetIncludedFiles()) {
    std::string inc_base =
        inc.schema_name.substr(0, inc.schema_name.find_last_of('.'));
    std::string prefix = absl::StrContains(inc_base, "/") ? "" : dir_prefix;
    code_ += "#include \"" + prefix + inc_base + "_generated.h\"";
    code_ +=
        "#include \"" + prefix + inc_base + "_size_calculator_generated.h\"";
  }
  code_ += "";
}

// Emits forward declarations for tables and unions.
void FlatbufferSizeGenerator::GenerateForwardDeclarations() {
  // Forward declarations for tables
  for (const auto* struct_def : parser_.structs_.vec) {
    if (struct_def->generated) continue;
    if (!struct_def->fixed) {
      SetNameSpace(struct_def->defined_namespace);
      std::string qname = struct_def->name;
      code_.SetValue("NAME", qname);
      code_ += "struct {{NAME}}SizeEstimator;";
      std::string params;
      for (const auto* field : struct_def->fields.vec) {
        if (!field->deprecated) {
          params += ", " +
                    GetParamType(field->value.type, *field, cur_name_space_) +
                    " " + field->name;
        }
      }
      code_.SetValue("PARAMS", params);
      code_ +=
          "flatbuffers::Offset<{{NAME}}> "
          "Estimate{{NAME}}Size(::split_engine::FlatbufferSizeCalculator& "
          "calc{{PARAMS}});";
      code_ +=
          "flatbuffers::Offset<{{NAME}}> "
          "EstimateOffset(::split_engine::FlatbufferSizeCalculator& calc, "
          "const {{NAME}}T* obj);";
      code_ +=
          "size_t EstimateSize(::split_engine::FlatbufferSizeCalculator& calc, "
          "const {{NAME}}T& obj);";
      code_ += "size_t EstimateSize(const {{NAME}}T& obj);";
    }
  }
  code_ += "";

  // Forward declarations for unions
  for (const auto* enum_def : parser_.enums_.vec) {
    if (enum_def->generated) continue;
    if (enum_def->is_union) {
      SetNameSpace(enum_def->defined_namespace);
      code_.SetValue("NAME", enum_def->name);
      code_ +=
          "flatbuffers::Offset<void> "
          "EstimateOffset(::split_engine::FlatbufferSizeCalculator& calc, "
          "const {{NAME}}Union& u);";
    }
  }
  code_ += "";
}

// Iterates over parser_.structs_ and emits table estimators and calculation
// functions.
void FlatbufferSizeGenerator::GenerateTableDefinitions() {
  for (const auto* struct_def : parser_.structs_.vec) {
    if (struct_def->generated) continue;
    if (!struct_def->fixed) {
      SetNameSpace(struct_def->defined_namespace);
      GenerateTableSizeEstimator(struct_def);
      GenerateEstimateTableSize(struct_def);
      GenerateEstimateOffset(struct_def);
      GenerateEstimateSize(struct_def);
    }
  }
}

// Emits the builder-like size estimator struct:
// struct FooSizeEstimator { calc_; void add_x(...); Finish(); };
void FlatbufferSizeGenerator::GenerateTableSizeEstimator(
    const flatbuffers::StructDef* struct_def) {
  std::string qname = struct_def->name;
  code_.SetValue("NAME", qname);
  code_ += "struct {{NAME}}SizeEstimator {";
  code_.IncrementIdentLevel();
  code_ += "::split_engine::FlatbufferSizeCalculator& calc_;";
  for (const auto* field : struct_def->fields.vec) {
    if (!field->deprecated) {
      const flatbuffers::Type& type = field->value.type;
      std::string ptype = GetParamType(type, *field, cur_name_space_);
      code_.SetValue("FNAME", field->name);
      code_.SetValue("PTYPE", ptype);
      code_.SetValue("VOFF", std::to_string(field->value.offset));
      code_ += "void add_{{FNAME}}({{PTYPE}} {{FNAME}}) {";
      code_.IncrementIdentLevel();
      if (flatbuffers::IsScalar(type.base_type) ||
          type.base_type == flatbuffers::BASE_TYPE_UTYPE ||
          (type.enum_def && !type.enum_def->is_union)) {
        code_.SetValue("DEFVAL", GetDefaultValue(*field, cur_name_space_));
        code_ +=
            "calc_.AddElement<{{PTYPE}}>({{VOFF}}, {{FNAME}}, {{DEFVAL}});";
      } else if (type.base_type == flatbuffers::BASE_TYPE_STRUCT &&
                 type.struct_def->fixed) {
        code_ += "calc_.AddStruct({{VOFF}}, {{FNAME}});";
      } else {
        code_ += "calc_.AddOffset({{VOFF}}, {{FNAME}});";
      }
      code_.DecrementIdentLevel();
      code_ += "}";
    }
  }
  code_ +=
      "explicit "
      "{{NAME}}SizeEstimator(::split_engine::FlatbufferSizeCalculator& "
      "calc) : calc_(calc) { calc_.StartTable(); }";
  code_ +=
      "flatbuffers::Offset<{{NAME}}> Finish() { return "
      "calc_.EndTable<{{NAME}}>(); }";
  code_.DecrementIdentLevel();
  code_ += "};";
  code_ += "";
}

// Emits the clean estimation helper:
// inline flatbuffers::Offset<Foo> EstimateFooSize(calc, x, y, z) { ... }
void FlatbufferSizeGenerator::GenerateEstimateTableSize(
    const flatbuffers::StructDef* struct_def) {
  std::string qname = struct_def->name;
  code_.SetValue("NAME", qname);
  std::string create_params;
  for (const auto* field : struct_def->fields.vec) {
    if (!field->deprecated) {
      create_params +=
          ", " + GetParamType(field->value.type, *field, cur_name_space_) +
          " " + field->name + " = " + GetDefaultValue(*field, cur_name_space_);
    }
  }
  code_.SetValue("CPARAMS", create_params);
  code_ +=
      "inline flatbuffers::Offset<{{NAME}}> "
      "Estimate{{NAME}}Size(::split_engine::FlatbufferSizeCalculator& "
      "calc{{CPARAMS}}) {";
  code_.IncrementIdentLevel();
  code_ += "{{NAME}}SizeEstimator estimator_(calc);";
  for (size_t size = struct_def->sortbysize ? 8 : 1; size; size /= 2) {
    for (auto it = struct_def->fields.vec.rbegin();
         it != struct_def->fields.vec.rend(); ++it) {
      const auto* field = *it;
      if (!field->deprecated &&
          (!struct_def->sortbysize ||
           size == flatbuffers::SizeOf(field->value.type.base_type))) {
        code_.SetValue("FNAME", field->name);
        code_ += "estimator_.add_{{FNAME}}({{FNAME}});";
      }
    }
  }
  code_ += "return estimator_.Finish();";
  code_.DecrementIdentLevel();
  code_ += "}";
  code_ += "";
}

// Generates the calculation logic for a single field (scalar, string, struct,
// vector). Handles IsRequired() checks cleanly without cluttering the main
// table traversal.
void FlatbufferSizeGenerator::GenerateFieldOffsetCalculation(
    const flatbuffers::StructDef* struct_def,
    const flatbuffers::FieldDef* field) {
  const flatbuffers::Type& type = field->value.type;
  code_.SetValue("FNAME", field->name);
  if (type.base_type == flatbuffers::BASE_TYPE_UTYPE) {
    code_.SetValue("SIBLING", field->sibling_union_field->name);
    code_ += "auto {{FNAME}}_val = obj->{{SIBLING}}.type;";
  } else if (flatbuffers::IsScalar(type.base_type) ||
             (type.enum_def && !type.enum_def->is_union)) {
    code_ += "auto {{FNAME}}_val = obj->{{FNAME}};";
  } else if (type.base_type == flatbuffers::BASE_TYPE_UNION) {
    code_ += "auto {{FNAME}}_val = EstimateOffset(calc, obj->{{FNAME}});";
  } else if (type.base_type == flatbuffers::BASE_TYPE_STRING) {
    if (field->IsRequired()) {
      code_ += "auto {{FNAME}}_val = calc.CreateString(obj->{{FNAME}});";
    } else {
      code_ +=
          "auto {{FNAME}}_val = obj->{{FNAME}}.empty() ? 0 : "
          "calc.CreateString(obj->{{FNAME}});";
    }
  } else if (type.base_type == flatbuffers::BASE_TYPE_STRUCT) {
    if (type.struct_def->fixed) {
      code_ += "auto {{FNAME}}_val = obj->{{FNAME}}.get();";
    } else {
      code_ +=
          "auto {{FNAME}}_val = obj->{{FNAME}} ? EstimateOffset(calc, "
          "obj->{{FNAME}}.get()) : 0;";
    }
  } else if (type.base_type == flatbuffers::BASE_TYPE_VECTOR ||
             type.base_type == flatbuffers::BASE_TYPE_VECTOR64) {
    flatbuffers::Type el_type = type.VectorType();
    if (el_type.base_type == flatbuffers::BASE_TYPE_STRUCT) {
      std::string qelname = GetCppName(*el_type.struct_def, cur_name_space_);
      code_.SetValue("QELNAME", qelname);
      if (el_type.struct_def->fixed) {
        if (field->IsRequired()) {
          code_ +=
              "auto {{FNAME}}_val = "
              "calc.CreateVector<{{QELNAME}}>(obj->{{FNAME}}.size(), "
              "sizeof({{QELNAME}}), alignof({{QELNAME}}));";
        } else {
          code_ +=
              "auto {{FNAME}}_val = obj->{{FNAME}}.size() ? "
              "calc.CreateVector<{{QELNAME}}>(obj->{{FNAME}}.size(), "
              "sizeof({{QELNAME}}), alignof({{QELNAME}})) : 0;";
        }
      } else {
        code_ +=
            "for (const auto& el : obj->{{FNAME}}) { if (el) "
            "EstimateOffset(calc, el.get()); }";
        if (field->IsRequired()) {
          code_ +=
              "auto {{FNAME}}_val = "
              "calc.CreateOffsetVector<{{QELNAME}}>(obj->{{FNAME}}.size());";
        } else {
          code_ +=
              "auto {{FNAME}}_val = obj->{{FNAME}}.size() ? "
              "calc.CreateOffsetVector<{{QELNAME}}>(obj->{{FNAME}}.size()) : "
              "0;";
        }
      }
    } else if (el_type.base_type == flatbuffers::BASE_TYPE_STRING) {
      code_ +=
          "for (const auto& el : obj->{{FNAME}}) { calc.CreateString(el); }";
      if (field->IsRequired()) {
        code_ +=
            "auto {{FNAME}}_val = "
            "calc.CreateOffsetVector<flatbuffers::String>(obj->{{"
            "FNAME}}.size());";
      } else {
        code_ +=
            "auto {{FNAME}}_val = obj->{{FNAME}}.size() ? "
            "calc.CreateOffsetVector<flatbuffers::String>(obj->{{FNAME}}."
            "size()) : 0;";
      }
    } else if (el_type.base_type == flatbuffers::BASE_TYPE_UTYPE &&
               el_type.enum_def && el_type.enum_def->is_union) {
      std::string union_vec_name =
          field->name.substr(0, field->name.size() - 5);
      std::string cpp_el_type = GetCppName(*el_type.enum_def, cur_name_space_);
      code_.SetValue("UNIONVECNAME", union_vec_name);
      code_.SetValue("CPPELTYPE", cpp_el_type);
      if (field->IsRequired()) {
        code_ +=
            "auto {{FNAME}}_val = "
            "calc.CreateVector<{{CPPELTYPE}}>(obj->{{UNIONVECNAME}}.size(), "
            "sizeof({{CPPELTYPE}}), alignof({{CPPELTYPE}}));";
      } else {
        code_ +=
            "auto {{FNAME}}_val = obj->{{UNIONVECNAME}}.size() ? "
            "calc.CreateVector<{{CPPELTYPE}}>(obj->{{UNIONVECNAME}}.size(), "
            "sizeof({{CPPELTYPE}}), alignof({{CPPELTYPE}})) : 0;";
      }
    } else if (el_type.base_type == flatbuffers::BASE_TYPE_UNION) {
      code_ +=
          "for (const auto& el : obj->{{FNAME}}) { EstimateOffset(calc, el); }";
      if (field->IsRequired()) {
        code_ +=
            "auto {{FNAME}}_val = "
            "calc.CreateOffsetVector<void>(obj->{{FNAME}}.size());";
      } else {
        code_ +=
            "auto {{FNAME}}_val = obj->{{FNAME}}.size() ? "
            "calc.CreateOffsetVector<void>(obj->{{FNAME}}.size()) : 0;";
      }
    } else {
      std::string cpp_el_type =
          GetCppType(el_type, true, *field, cur_name_space_);
      code_.SetValue("CPPELTYPE", cpp_el_type);
      if (field->IsRequired()) {
        code_ +=
            "auto {{FNAME}}_val = "
            "calc.CreateVector<{{CPPELTYPE}}>(obj->{{FNAME}}.size(), "
            "sizeof({{CPPELTYPE}}), alignof({{CPPELTYPE}}));";
      } else {
        code_ +=
            "auto {{FNAME}}_val = obj->{{FNAME}}.size() ? "
            "calc.CreateVector<{{CPPELTYPE}}>(obj->{{FNAME}}.size(), "
            "sizeof({{CPPELTYPE}}), alignof({{CPPELTYPE}})) : 0;";
      }
    }
  }
}

// Emits the C++ Object API traversal helper:
// inline flatbuffers::Offset<Foo> EstimateOffset(calc, const FooT* obj) { ... }
void FlatbufferSizeGenerator::GenerateEstimateOffset(
    const flatbuffers::StructDef* struct_def) {
  std::string qname = struct_def->name;
  code_.SetValue("NAME", qname);
  code_ +=
      "inline flatbuffers::Offset<{{NAME}}> "
      "EstimateOffset(::split_engine::FlatbufferSizeCalculator& calc, "
      "const {{NAME}}T* obj) {";
  code_.IncrementIdentLevel();
  code_ += "if (!obj) return 0;";
  for (const auto* field : struct_def->fields.vec) {
    if (field->deprecated) continue;
    GenerateFieldOffsetCalculation(struct_def, field);
  }
  std::string est_args;
  for (const auto* field : struct_def->fields.vec) {
    if (!field->deprecated) {
      est_args += ", " + field->name + "_val";
    }
  }
  code_.SetValue("ESTARGS", est_args);
  code_ += "return Estimate{{NAME}}Size(calc{{ESTARGS}});";
  code_.DecrementIdentLevel();
  code_ += "}";
  code_ += "";
}

// Emits the public top-level estimation function:
// inline size_t EstimateSize(const FooT& obj) { ... }
void FlatbufferSizeGenerator::GenerateEstimateSize(
    const flatbuffers::StructDef* struct_def) {
  std::string qname = struct_def->name;
  code_.SetValue("NAME", qname);
  code_ +=
      "inline size_t EstimateSize(::split_engine::FlatbufferSizeCalculator& "
      "calc, "
      "const {{NAME}}T& obj) {";
  code_.IncrementIdentLevel();
  code_ += "calc.Clear();";
  code_ += "auto root = EstimateOffset(calc, &obj);";
  code_ += "calc.Finish(root);";
  code_ += "return calc.ComputeSize();";
  code_.DecrementIdentLevel();
  code_ += "}";
  code_ += "";
  code_ += "inline size_t EstimateSize(const {{NAME}}T& obj) {";
  code_.IncrementIdentLevel();
  code_ += "::split_engine::FlatbufferSizeCalculator calc;";
  code_ += "return EstimateSize(calc, obj);";
  code_.DecrementIdentLevel();
  code_ += "}";
  code_ += "";
}

// Emits the switch statement for union offset estimation:
// inline flatbuffers::Offset<void> EstimateOffset(calc, const FooUnion& u) {
// switch(u.type) ... }
void FlatbufferSizeGenerator::GenerateUnionDefinitions() {
  for (const auto* enum_def : parser_.enums_.vec) {
    if (enum_def->generated) continue;
    if (enum_def->is_union) {
      SetNameSpace(enum_def->defined_namespace);
      code_.SetValue("NAME", enum_def->name);
      code_ +=
          "inline flatbuffers::Offset<void> "
          "EstimateOffset(::split_engine::FlatbufferSizeCalculator& calc, "
          "const {{NAME}}Union& u) {";
      code_.IncrementIdentLevel();
      code_ += "switch (u.type) {";
      code_.IncrementIdentLevel();
      for (const auto* ev : enum_def->Vals()) {
        if (ev->IsZero()) continue;
        std::string label = parser_.opts.scoped_enums
                                ? (enum_def->name + "::" + ev->name)
                                : (enum_def->name + "_" + ev->name);
        code_.SetValue("LABEL", label);
        if (ev->union_type.base_type == flatbuffers::BASE_TYPE_STRUCT) {
          std::string qelname =
              GetCppName(*ev->union_type.struct_def, cur_name_space_);
          code_.SetValue("QELNAME", qelname);
          if (ev->union_type.struct_def->fixed) {
            code_ += "case {{LABEL}}: {";
            code_.IncrementIdentLevel();
            code_ +=
                "auto ptr = reinterpret_cast<const {{QELNAME}}*>(u.value);";
            code_ += "return ptr ? calc.CreateStruct(*ptr).Union() : 0;";
            code_.DecrementIdentLevel();
            code_ += "}";
          } else {
            code_ += "case {{LABEL}}: {";
            code_.IncrementIdentLevel();
            code_ +=
                "auto ptr = reinterpret_cast<const {{QELNAME}}T*>(u.value);";
            code_ += "return ptr ? EstimateOffset(calc, ptr).Union() : 0;";
            code_.DecrementIdentLevel();
            code_ += "}";
          }
        } else if (ev->union_type.base_type == flatbuffers::BASE_TYPE_STRING) {
          code_ += "case {{LABEL}}: {";
          code_.IncrementIdentLevel();
          code_ += "auto ptr = reinterpret_cast<const std::string*>(u.value);";
          code_ +=
              "return ptr && !ptr->empty() ? calc.CreateString(*ptr).Union() "
              ": 0;";
          code_.DecrementIdentLevel();
          code_ += "}";
        }
      }
      code_ += "default: return 0;";
      code_.DecrementIdentLevel();
      code_ += "}";
      code_.DecrementIdentLevel();
      code_ += "}";
      code_ += "";
    }
  }
}

// Closes namespaces and include guards.
void FlatbufferSizeGenerator::GeneratePostamble(
    const std::string& include_guard) {
  SetNameSpace(nullptr);
  code_ += "";
  code_ += "#endif  // " + include_guard;
}

int main(int argc, char** argv) {
  absl::InitializeLog();
  absl::SetStderrThreshold(absl::LogSeverityAtLeast::kInfo);
  absl::SetMinLogLevel(absl::LogSeverityAtLeast::kInfo);
  absl::SetProgramUsageMessage(
      absl::StrCat("Usage: ", argv[0], " --out=OUTPUT_FILE INPUT_FILES..."));
  auto positional_args = absl::ParseCommandLine(argc, argv);

  if (positional_args.size() == 1) {
    IMP_LOG(imp::ERROR) << "Missing input file.\n" << absl::ProgramUsageMessage();
    return 1;
  }

  std::string out_path = absl::GetFlag(FLAGS_out);
  if (out_path.empty()) {
    IMP_LOG(imp::ERROR) << "Missing --out flag.\n" << absl::ProgramUsageMessage();
    return 1;
  }

  flatbuffers::IDLOptions parser_options;
  parser_options.lang_to_generate = flatbuffers::IDLOptions::kCpp;
  parser_options.scoped_enums = absl::GetFlag(FLAGS_scoped_enums);
  flatbuffers::Parser parser(parser_options);

  std::vector<const char*> include_paths;
  for (const auto& include_path : absl::GetFlag(FLAGS_include_paths)) {
    include_paths.push_back(include_path.c_str());
  }

  std::vector<std::string> input_filepaths;
  for (auto it = std::next(positional_args.begin());
       it != positional_args.end(); ++it) {
    const char* input_filepath = *it;
    input_filepaths.push_back(input_filepath);
    std::string contents;
    if (!flatbuffers::LoadFile(input_filepath, true, &contents)) {
      IMP_LOG(imp::ERROR) << "Unable to load file: " << input_filepath;
      return 1;
    }

    if (!parser.Parse(contents.data(), include_paths.data(), input_filepath)) {
      IMP_LOG(imp::ERROR) << "Failed to parse input file " << input_filepath << ": "
                 << parser.error_;
      return 1;
    }
  }

  FlatbufferSizeGenerator generator(parser);
  std::string code = generator.Generate(input_filepaths);
  if (!flatbuffers::SaveFile(out_path.c_str(), code, false)) {
    IMP_LOG(imp::ERROR) << "Failed to save file: " << out_path;
    return 1;
  }

  return 0;
}
