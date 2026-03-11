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

#include "core/recipes/language/recipe_function_utils.h"

#include <optional>
#include <string>
#include <tuple>
#include <utility>
#include <variant>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/async/future.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/view/utils/string_map.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::recipe {

namespace internal {

template <>
absl::StatusOr<std::tuple<recipe::Args>> ArgsToTupleHelper(
    recipe::Args& args, std::index_sequence<0>) {
  return std::tuple<recipe::Args>(args);
}

absl::StatusOr<recipe::ReturnValue> ConvertStatusToStatusOrReturnValue(
    absl::Status status) {
  if (!status.ok()) {
    return status;
  }

  return recipe::ReturnValue{};
}

recipe::ReturnValue ConvertFutureStatusToReturnValue(
    Future<absl::Status> future_status) {
  // Handles the case where the return type is Future<absl::Status>.
  return recipe::ReturnValue{
      .async_values = future_status.Then([]() { return recipe::Variables(); })};
}

absl::StatusOr<recipe::ReturnValue>
ConvertStatusOrReturnValueDeclarationToStatusOrReturnValue(
    absl::StatusOr<ReturnValueDeclaration> return_value_declaration) {
  // Handles the case where the return type is
  // recipe::ReturnValueDeclaration or
  // absl::StatusOr<recipe::ReturnValueDeclaration>.
  if (!return_value_declaration.ok()) {
    return return_value_declaration.status();
  }

  return ConvertReturnValueDeclarationToReturnValue(*return_value_declaration);
}

recipe::ReturnValue ConvertReturnValueDeclarationToReturnValue(
    ReturnValueDeclaration return_value_declaration) {
  recipe::ReturnValue return_value;

  // combined_future combines all of the async variables to one
  // future. If there's no async variables, then this will be
  // std::nullopt. Otherwise, this will become ready once all variable
  // futures are ready.
  std::optional<Future<absl::Status>> combined_future;
  StringMap<Future<recipe::Variable>> variable_futures;
  for (auto& [socket_name, variable] : return_value_declaration.socket_values) {
    if (std::holds_alternative<recipe::Variable>(variable)) {
      return_value.values[socket_name] = std::get<recipe::Variable>(variable);
    } else if (std::holds_alternative<Future<recipe::Variable>>(variable)) {
      if (!combined_future) {
        combined_future = Future<absl::Status>(absl::OkStatus());
      }
      combined_future = combined_future->Combine(
          std::get<Future<recipe::Variable>>(variable));
      variable_futures[socket_name] =
          std::get<Future<recipe::Variable>>(variable);
    }
  }

  if (return_value_declaration.status_future.has_value()) {
    if (!combined_future) {
      combined_future = Future<absl::Status>(absl::OkStatus());
    }
    combined_future =
        combined_future->Combine(*return_value_declaration.status_future);
  }

  if (combined_future) {
    return_value.async_values = combined_future->Then(
        [variable_futures]() -> absl::StatusOr<recipe::Variables> {
          recipe::Variables async_values;
          // All async values should be ready by this point.
          for (auto& [socket_name, future] : variable_futures) {
            MP_ASSIGN_OR_RETURN(recipe::Variable value, future.Get());
            async_values[socket_name] = value;
          }
          return async_values;
        });
  }

  return return_value;
}

recipe::ReturnValue ConvertFutureRecipeVariablesToReturnValue(
    Future<recipe::Variables> variables) {
  return recipe::ReturnValue{.async_values = variables};
}

recipe::ReturnValue ConvertFutureRecipeVariableToReturnValue(
    Future<recipe::Variable> variable) {
  return recipe::ReturnValue{
      .async_values =
          variable.Then([](recipe::Variable return_value) -> recipe::Variables {
            recipe::Variables result;
            result[std::string(recipe::kDefaultOutputSocketName.data(),
                               recipe::kDefaultOutputSocketName.length())] =
                return_value;
            return result;
          })};
}

recipe::ReturnValue ConvertRecipeVariablesToReturnValue(
    recipe::Variables variables) {
  return recipe::ReturnValue{.values = variables};
}

absl::StatusOr<recipe::ReturnValue>
ConvertStatusOrRecipeVariablesToStatusOrReturnValue(
    absl::StatusOr<recipe::Variables> variables) {
  if (!variables.ok()) {
    return variables.status();
  }

  return recipe::ReturnValue{.values = *variables};
}

absl::StatusOr<recipe::ReturnValue> ConvertStatusOrRecipeVariableToReturnValue(
    absl::StatusOr<recipe::Variable> variable) {
  if (!variable.ok()) {
    return variable.status();
  }

  return recipe::ReturnValue{
      .values = {{std::string(recipe::kDefaultOutputSocketName.data(),
                              recipe::kDefaultOutputSocketName.length()),
                  *variable}}};
}

recipe::ReturnValue ConvertRecipeVariableToReturnValue(
    recipe::Variable variable) {
  return recipe::ReturnValue{
      .values = {{std::string(recipe::kDefaultOutputSocketName.data(),
                              recipe::kDefaultOutputSocketName.length()),
                  variable}}};
}

}  // namespace internal

}  // namespace imp::recipe
