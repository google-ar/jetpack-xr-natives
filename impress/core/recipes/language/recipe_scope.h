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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_SCOPE_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_SCOPE_H_

#include <functional>
#include <optional>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp {

class RecipeSystem;

// RecipeScope holds a collection of Recipe variables.
// Variables are registered/retrieved by their names.
// When creating a RecipeScope, a parent RecipeScope can be passed in so that
// it's possible to look up variables defined in parent RecipeScope. Please note
// that this is a recursive action, which means it will search all the way to
// the root scope.
class RecipeScope {
 public:
  RecipeScope(RecipeScope* parent_scope = nullptr)
      : parent_scope_(parent_scope) {}

  // Returns the variable with the given name.
  // Returns nullopt if the variable is not found.
  std::optional<std::reference_wrapper<recipe::Variable>> GetVariable(
      absl::string_view name);

  // Returns all variables accessible in this scope.
  recipe::Variables GetVariables();

  // Clear all variables in this scope (will leave parent variables intact).
  void ClearLocalVariables();

  // Declares a variable with the specified `VariableDeclaration`.
  // This can fail and return an error when:
  //   - The variable is already registered (either in this scope or the parent
  //   scopes).
  //   - The VariableDeclaration has contradictory value types between declare
  //   type and init value type.
  absl::Status DeclareVariable(const VariableDeclaration& variable_declaration);

  // Returns true if the RecipeScope has variable of a given name.
  // NOTE: If the given variable is not in the RecipeScope, the method will
  // check for it in the parent RecipeScope.
  bool HasVariable(absl::string_view name) const;

#if IMP_ENABLE_RECIPE_EXPERIMENTAL
  // Overwrites an already declare variable with the specified
  // `VariableDeclaration`. This can fail and return an error when:
  //   - The variable is not already declared in this scope or any parent
  //   scopes.
  absl::Status OverwriteVariable(
      const VariableDeclaration& variable_declaration);
#endif

 private:
  RecipeScope* parent_scope_ = nullptr;
  recipe::Variables variables_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_SCOPE_H_
