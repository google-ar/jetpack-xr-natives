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

#include "core/recipes/language/recipe_scope.h"

#include <functional>
#include <optional>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp {

std::optional<std::reference_wrapper<recipe::Variable>>
RecipeScope::GetVariable(absl::string_view name) {
  auto itr = variables_.find(name);
  if (itr != variables_.end()) {
    return itr.value();
  }

  if (parent_scope_) {
    return parent_scope_->GetVariable(name);
  }

  return {};
}

recipe::Variables RecipeScope::GetVariables() {
  recipe::Variables result;
  result.reserve(variables_.size());
  for (const auto& [name, variable] : variables_) {
    result[name] = variable;
  }

  if (!parent_scope_) {
    return result;
  }

  recipe::Variables parent_result = parent_scope_->GetVariables();
  for (const auto& [name, variable] : parent_result) {
    result[name] = variable;
  }

  return result;
}

void RecipeScope::ClearLocalVariables() { variables_.clear(); }

absl::Status RecipeScope::DeclareVariable(
    const VariableDeclaration& variable_declaration) {
#ifdef IMP_ENABLE_RECIPE_EXPERIMENTAL
  // The experimental path allows for declaring variables with the same name at
  // different levels of scope, and overwriting local variables.

  auto itr = variables_.find(variable_declaration.name);
  if (itr != variables_.end()) {
    return OverwriteVariable(variable_declaration);
  }
#else
  std::optional<std::reference_wrapper<recipe::Variable>> existing_variable =
      GetVariable(variable_declaration.name);
  if (existing_variable) {
    return absl::FailedPreconditionError(
        absl::StrFormat("Can't declare two variables with the same name. %s",
                        variable_declaration.name));
  }
#endif

  std::string type_name =
      std::string(recipe::ToTypeName(variable_declaration.type));
  recipe::Variable& result = variables_[variable_declaration.name];

#ifdef IMP_ENABLE_RECIPE_EXPERIMENTAL
  // This path allows for support of initially declaring a variable type with
  // unknown type, and then initializing it with a value of the correct type.

  if (variable_declaration.init_value) {
    recipe::Variable init_value = variable_declaration.init_value->value;
    if (variable_declaration.type ==
        VariableDeclaration::UNKNOWN_VARIABLE_TYPE) {
      // If the type is unknown, then we can set the value to the init
      // value.
      result = init_value;
    } else if (init_value.index() ==
                   VariableDeclaration::UNKNOWN_VARIABLE_TYPE &&
               variable_declaration.type !=
                   VariableDeclaration::UNKNOWN_VARIABLE_TYPE) {
      // If init_value is UNKNOWN_VARIABLE_TYPE, and result is not, then we
      // should set the init value to the default value of the result type.
      // This can guard from 0 values disappearing.
      recipe::SetToDefault(variable_declaration.type, result);
    } else if (init_value.index() != variable_declaration.type) {
      IMP_LOG(imp::INFO) << "Mismatched indexes: " << init_value.index() << " "
                << result.index();
      return absl::FailedPreconditionError(absl::StrFormat(
          "Mismatched types for initializing variable %s. Expected %s. Got %s",
          variable_declaration.name,
          proto::EnumMetaData<VariableDeclaration::Type>::GetName(
              variable_declaration.type),
          proto::EnumMetaData<VariableDeclaration::Type>::GetName(
              VariableDeclaration::Type(init_value.index()))));
    } else {
      result = init_value;
    }
  } else {
    // No init value, so set to default for the type.
    recipe::SetToDefault(variable_declaration.type, result);
  }

#else
  recipe::SetToDefault(variable_declaration.type, result);
  if (variable_declaration.type == VariableDeclaration::UNKNOWN_VARIABLE_TYPE) {
    return absl::FailedPreconditionError(
        absl::StrFormat("Can't declare a variable with an unknown type. %s",
                        variable_declaration.name));
  }

  if (variable_declaration.init_value) {
    recipe::Variable init_value = variable_declaration.init_value->value;
    if (init_value.index() != result.index()) {
      return absl::FailedPreconditionError(absl::StrFormat(
          "Mismatched types for initializing variable %s. Expected %s. Got %s",
          variable_declaration.name,
          proto::EnumMetaData<VariableDeclaration::Type>::GetName(
              variable_declaration.type),
          proto::EnumMetaData<VariableDeclaration::Type>::GetName(
              VariableDeclaration::Type(init_value.index()))));
    }
    result = init_value;
  }
#endif

  imp::output::Recipe("Declared variable %s of type %s",
                      variable_declaration.name, type_name);

  return absl::OkStatus();
}

#if IMP_ENABLE_RECIPE_EXPERIMENTAL
absl::Status RecipeScope::OverwriteVariable(
    const VariableDeclaration& variable_declaration) {
  // If the variable is already in the scope, delete it.
  auto itr = variables_.find(variable_declaration.name);
  if (itr == variables_.end()) {
    if (parent_scope_) {
      return parent_scope_->OverwriteVariable(variable_declaration);
    }
    return absl::FailedPreconditionError(absl::StrFormat(
        "Variable %s not found in this scope or any parent scopes.",
        variable_declaration.name));
  }
  // Delete the variable from the scope.
  variables_.erase(itr);

  // Redeclare the variable.
  return DeclareVariable(variable_declaration);
}
#endif

}  // namespace imp
