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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_SYSTEM_H_

#include <memory>
#include <string>
#include <string_view>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/common/invocable.h"
#include "core/ncsb/node.h"
#include "core/ncsb/system.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/recipe_custom_statement.h"
#include "core/recipes/language/recipe_scope.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/language/registered_function.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

namespace imp {

// RecipeSystem holds a collection of data that can be referenced by
// RecipeRunner and RecipeRuntimeGraph for executing functions, reading/writing
// variable values, etc.
//
// RecipeRunner is intended to be referenced from imp::Registry. For example:
//   RecipeSystem& recipe_system =
//    GetView().GetRegistry().GetOrCreate<RecipeSystem>(GetView());
class RecipeSystem : public BaseRecipeSystem {
 public:
  RecipeSystem(BaseView& view);
  ~RecipeSystem() override = default;

  absl::Status DeclareGlobalVariable(std::string_view name,
                                     VariableDeclaration::Type type,
                                     recipe::Variable value) override {
    VariableDeclaration variable_declaration = {
        .name = std::string(name),
        .type = type,
        .init_value = Literal{.value = value},
    };

    return GetRootScope().DeclareVariable(variable_declaration);
  }

  // Executes a function with a given name and a vector of Variable.
  absl::StatusOr<recipe::ReturnValue> ExecuteFunction(
      absl::string_view name, recipe::Args& args,
      recipe::NamedArgs& named_args) const {
    imp::output::Recipe("Calling function %s", name);
    auto it = registered_functions_.find(name);
    if (it == registered_functions_.end()) {
      return absl::NotFoundError(
          absl::StrFormat("Unable to find global function named %s", name));
    }

    absl::StatusOr<recipe::ReturnValue> result =
        it->second->Execute(args, named_args);

    if (!result.ok()) {
      return absl::InternalError(
          absl::StrFormat("Failed to execute Recipe function %s: %s", name,
                          result.status().message()));
    }

    return result;
  }

  absl::StatusOr<recipe::ReturnValue> ExecuteFunction(
      absl::string_view name, recipe::Args& args) const {
    recipe::NamedArgs named_args;
    return ExecuteFunction(name, args, named_args);
  }

  // Returns the root RecipeScope.
  // This should be the root for all RecipeScope, which means Variables in the
  // root scope are basically global variables.
  // TODO: Figure out a way to automatically enforce all
  // RecipeScope to have root_scope_ as the root.
  RecipeScope& GetRootScope() { return root_scope_; }

  std::unique_ptr<RecipeCustomStatement> CreateCustomStatement(
      absl::string_view name) const;

 protected:
  void RegisterFunctionImpl(
      absl::string_view name,
      std::unique_ptr<recipe::RegisteredFunction> function) override;

  void RegisterCustomStatementTypeImpl(
      absl::string_view name,
      Invocable<std::unique_ptr<RecipeCustomStatement>()> creation_fn) override;

 private:
  StringMap<std::unique_ptr<recipe::RegisteredFunction>> registered_functions_;
  StringMap<Invocable<std::unique_ptr<RecipeCustomStatement>()>>
      custom_statement_creators_;
  RecipeScope root_scope_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_SYSTEM_H_
