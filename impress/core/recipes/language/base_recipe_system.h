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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_BASE_RECIPE_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_BASE_RECIPE_SYSTEM_H_

#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/common/invocable.h"
#include "core/recipes/language/recipe_custom_statement.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/language/registered_function.h"

namespace imp {

// The base class for the RecipeSystem.
//
// Used in cases where the RecipeSystem cannot be used/accessed directly, such
// as in the files under core/recipes/language/functions.
class BaseRecipeSystem {
 public:
  virtual ~BaseRecipeSystem() = default;
  virtual absl::Status DeclareGlobalVariable(std::string_view name,
                                             VariableDeclaration::Type type,
                                             recipe::Variable value) = 0;

  template <typename Fn>
  void RegisterFunction(absl::string_view name, Fn fn);

  template <typename Fn>
  void RegisterFunction(recipe::RegisteredFunction::Builder& builder, Fn fn);

  // Registers a RecipeCustomStatement type.
  //
  // T must be derived from RecipeCustomStatement and has T::kName.
  // Internally, T::kName is used for identifying the custom statement type.
  template <typename T>
  void RegisterCustomStatementType();

 protected:
  // Registers a function that takes a vector of Variable.
  // The function will return sync values and an optional async values.
  //
  // All types in the function signature (parameters and return values) need to
  // be types supported by recipe system (i.e recipe::Variable).
  // Functions can have arbitrary number of parameters.
  // Function return types can be one of the following:
  //  - void
  //. - one of the recipe::Varaiable type
  //. - recipe::Varaibles
  //  - Future<absl::Status>
  //  - Future<one of the recipe::Variable type>
  //  - Future<recipe::Variables>
  //  - recipe::ReturnValueDeclaration (contains sync and async return values).
  //
  // Single return values will be stored in recipe::kDefaultOutputSocket.
  // For example:
  //  []() { return 0; }
  //  is equivalent to
  //  []() {
  //    Variables return_values;
  //    return_values[recipe::kDefaultOutputSocket] = Variable(0);
  //    return return_values;
  //  }
  //
  // Registering a function that returns void or Future<absl::Status> will
  // result in an empty Variables being returned.
  //
  // It is also possible to register a function that returns
  // recipe::ReturnValueDeclaration, which contains a string map of values that
  // can be sync or async. For example,
  //
  // []() {
  //   recipe::ReturnValueDeclaration return_values;
  //   return_values.socket_values["sync_value"] = foo;
  //
  //   return_values.socket_values["async_value"] =
  //   Future<recipe::Variable>::Schedule([](){
  //     Do some async work
  //     return recipe::Variable(bar);
  //   });
  //
  //   return_values.socket_values["another_async_value"] =
  //     Future<recipe::Variable>(baz);
  //
  //.  return_values.status_futures.push_back(
  //     Future<absl::Status>::Schedule([]() {
  //       Do some other async work
  //       return absl::OkStatus();
  //     }));
  //
  //   return return_values;
  // }
  //
  // In this example, `sync_value` will be ready immediately for the `next`
  // nodes to reference. Async return values will be ready for `done` nodes to
  // reference once all async variable futures are ready.
  //
  // Please note that registered recipe functions are globally accessible so
  // please be extra careful when registering a function that has capture. To
  // avoid capturing, always pass in the value through arguments. Take
  // NodeHandle->GetLocalRotation() as example: Instead of [Nodehandle node]() {
  // return node->GetLocalRotation(); } Register a function that takes
  // NodeHandle as an argument:
  // [](NodeHandle node) { return node->GetLocalRotation(); }
  virtual void RegisterFunctionImpl(
      std::unique_ptr<recipe::RegisteredFunction> function) = 0;

  virtual void RegisterCustomStatementTypeImpl(
      absl::string_view name,
      Invocable<std::unique_ptr<RecipeCustomStatement>()> creation_fn) = 0;

 private:
  recipe::RegisteredFunction::Builder registered_function_builder_;
};

template <typename Fn>
void BaseRecipeSystem::RegisterFunction(absl::string_view name, Fn fn) {
  registered_function_builder_.SetName(name);
  registered_function_builder_.ClearParams();
  RegisterFunctionImpl(registered_function_builder_.Build<Fn>(std::move(fn)));
}

template <typename Fn>
void BaseRecipeSystem::RegisterFunction(
    recipe::RegisteredFunction::Builder& builder, Fn fn) {
  RegisterFunctionImpl(builder.Build<Fn>(std::move(fn)));
}

template <typename T>
void BaseRecipeSystem::RegisterCustomStatementType() {
  RegisterCustomStatementTypeImpl(T::kName, []() {
    return std::unique_ptr<RecipeCustomStatement>(std::make_unique<T>());
  });
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_BASE_RECIPE_SYSTEM_H_
