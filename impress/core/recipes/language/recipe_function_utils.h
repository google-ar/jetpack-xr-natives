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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_FUNCTION_UTILS_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_FUNCTION_UTILS_H_

#include <cstddef>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "absl/meta/type_traits.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "core/async/future.h"
#include "core/async/future_traits.h"
#include "core/recipes/language/recipe_traits.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {

namespace internal {

template <typename TupleT, std::size_t Index>
std::tuple_element_t<Index, TupleT>& ArgsToTupleElementHelper(
    const recipe::Args& args) {
  using ElementT = std::decay_t<std::tuple_element_t<Index, TupleT>>;
  const recipe::Variable& variable = args.at(Index);

  if constexpr (std::is_same_v<ElementT, recipe::Variable>) {
    return variable;
  } else {
    return absl::get<ElementT>(variable);
  }
}

template <typename TupleT, std::size_t... Indices>
TupleT ArgsToTupleHelper(const recipe::Args& args,
                         std::index_sequence<Indices...>) {
  return std::forward_as_tuple(
      ArgsToTupleElementHelper<TupleT, Indices>(args)...);
}

template <>
std::tuple<const recipe::Args&> ArgsToTupleHelper(const recipe::Args& args,
                                                  std::index_sequence<0>);

// Helper Method that takes in a `absl::Status` message and converts it to a
// `absl::StatusOr<recipe::ReturnValue>` object that is returned from the
// method. Will return store an empty `recipe::Variables` object in the returned
// `absl::StatusOr<recipe::ReturnValue>` object if an `absl::OkStatus` message
// was passed into the method.
//
// - status - the `absl::Status` message to convert to a
// `absl::StatusOr<recipe::ReturnValue>` object
absl::StatusOr<recipe::ReturnValue> ConvertStatusToStatusOrReturnValue(
    const absl::Status& status);

// Helper Method that takes in a `Future<absl::Status>`, converts it to a
// `Future<recipe::Variables>`, and stores it in a `recipe::ReturnValue` that is
// returned by the method.
//
// - future_status - the `Future<absl::Status>` object to convert to a
// `Future<recipe::Variables>`
recipe::ReturnValue ConvertFutureStatusToReturnValue(
    const Future<absl::Status>& future_status);

// Helper Method that takes in a `absl::StatusOr<recipe::ReturnValue>` and
// converts internal data to a `recipe::ReturnValue` object that should be
// returned by the method. Will return an `absl::Status` message if that was
// passed into the method instead.
//
// - return_value_declaration - the `ReturnValueDeclaration` object to convert
// to a `recipe::ReturnValue`
absl::StatusOr<recipe::ReturnValue>
ConvertStatusOrReturnValueDeclarationToStatusOrReturnValue(
    const absl::StatusOr<ReturnValueDeclaration>& return_value_declaration);

// Helper Method that takes in a `ReturnValueDeclaration` and converts its
// internal data to a `recipe::ReturnValue` object that should be returned by
// the method.
//
// - return_value_declaration - the `ReturnValueDeclaration` object to convert
// to a `recipe::ReturnValue`
recipe::ReturnValue ConvertReturnValueDeclarationToReturnValue(
    const ReturnValueDeclaration& return_value_declaration);

// Helper Method that takes in a `Future<recipe::Variables>` and returns a
// `recipe::ReturnValue` object containing the `Future<recipe::Variables>`.
//
// - variable - the `Future<recipe::Variables>` object to store in the returned
// `recipe::ReturnValue`
recipe::ReturnValue ConvertFutureRecipeVariablesToReturnValue(
    const Future<recipe::Variables>& variables);

// Helper Method that takes in a `Future<recipe::Variable>` and returns a
// `recipe::ReturnValue` object containing the `Future<recipe::Variable>`.
//
// - variable - the `Future<recipe::Variable>` object to store in the returned
// `recipe::ReturnValue`
recipe::ReturnValue ConvertFutureRecipeVariableToReturnValue(
    const Future<recipe::Variable>& variable);

// Helper Method that takes in a `Future<T>`, converts it into a
// `Future<recipe::Variable>` (if possible), and returns a `recipe::ReturnValue`
// object containing the converted `Future<recipe::Variable>`.
//
// - variable - the `Future<T>` to convert to a `Future<recipe::Variable>` and
// stores it in the returned `recipe::ReturnValue`
template <typename T>
recipe::ReturnValue ConvertFutureTemplateToReturnValue(
    const Future<T>& variable) {
  Future<recipe::Variable> future_recipe_variable = variable.Then(
      [](T return_value) -> recipe::Variable { return return_value; });
  return ConvertFutureRecipeVariableToReturnValue(future_recipe_variable);
}

// Helper Method that takes in a `absl::StatusOr<recipe::Variables>` object and
// should return a `recipe::ReturnValue` object holding a list of
// `recipe::Variable`s inside of it. However, if the
// `absl::StatusOr<recipe::Variables>` object held an absl::Status message
// instead, then that absl::Status message will be returned instead.
//
// - variables - the list of the `recipe::Variable`s to be placed inside of the
// returned `recipe::ReturnValue` object or the `absl::Status` message to be
// returned immediately
absl::StatusOr<recipe::ReturnValue>
ConvertStatusOrRecipeVariablesToStatusOrReturnValue(
    const absl::StatusOr<recipe::Variables>& variables);

// Helper Method that takes in a `recipe::Variables` object and returns a
// `recipe::ReturnValue` object holding a list of `recipe::Variable`s inside of
// said `recipe::Variables` object.
//
// - variables - the list of the `recipe::Variable`s to be placed inside of the
// returned `recipe::ReturnValue` object
recipe::ReturnValue ConvertRecipeVariablesToReturnValue(
    const recipe::Variables& variables);

// Helper method that takes in an `absl::StatusOr` object of `recipe::Variable`
// or one of its raw value types and returns a `recipe::ReturnValue` struct
// containing it. Will return an absl::Status object if the
// `absl::StatusOr` parameter contained an `absl::Status` message instead.
//
// - variable - the `absl::StatusOr<recipe::Variable>` to be placed inside of
// the returned `recipe::ReturnValue` or returned immediately depending on if it
// holds an `absl::Status` message or not
absl::StatusOr<recipe::ReturnValue> ConvertStatusOrRecipeVariableToReturnValue(
    const absl::StatusOr<recipe::Variable>& variable);

// Helper method that takes in a `recipe::Variable` or one of it's raw value
// types and returns a `recipe::ReturnValue` struct that contains
// the recipe::Variable.
//
// - variable - the `recipe::Variable` to be placed inside of the returned
// `recipe::ReturnValue`
recipe::ReturnValue ConvertRecipeVariableToReturnValue(
    const recipe::Variable& variable);

template <typename NonRefT, int Index>
static constexpr int GetRecipeVariableIndexHelper() {
  if constexpr (Index > Literal::kFieldsCount || Index <= 0) {
    return Literal::kValue_Unknown;
  } else {
    using VariantType = std::variant_alternative_t<Index, recipe::Variable>;
    return std::is_same_v<NonRefT, VariantType>
               ? Index
               : GetRecipeVariableIndexHelper<NonRefT, Index + 1>();
  }
}

}  // namespace internal

// Helper Method for retrieving a type index value for a supported
// recipe::Variable raw type at compile time
template <typename T>
static constexpr int GetRecipeVariableIndex() {
  using NonRefT = absl::remove_cvref_t<T>;

  if constexpr (std::is_same_v<NonRefT, recipe::Variable>) {
    return std::numeric_limits<int>::max() - 1;
  } else if constexpr (std::is_constructible_v<std::string, NonRefT>) {
    return Literal::kValue_StringValue;
  } else {
    return internal::GetRecipeVariableIndexHelper<NonRefT, 1>();
  }
}

// Helper Method for retrieving the argument types out of a `RecipeFunction`
// that takes in fixed set of `recipe::Variable`s or any of its raw value types
// and returns a list of integers representing each argument's type.
template <typename TupleT, size_t... Indices>
std::vector<int> GetParameterTypeIds(std::index_sequence<Indices...>) {
  if constexpr (std::tuple_size<TupleT>::value == 1) {
    if constexpr (std::is_same_v<std::tuple_element_t<0, TupleT>,
                                 recipe::Args>) {
      return {};
    }
  }

  return {GetRecipeVariableIndex<std::tuple_element_t<Indices, TupleT>>()...};
}

// TODO: Update this function to take in the built
// RegisteredFunction.
template <typename Fn>
recipe::RecipeFunction MakeRecipeFunction(Fn fn) {
  using FnType = std::decay_t<Fn>;
  using FunctorUnpacker =
      decltype(recipe_traits::FunctorUnpacker(&FnType::operator()));
  using ReturnT = typename FunctorUnpacker::ReturnT;
  using ConstArgsRefTuple = typename FunctorUnpacker::ConstArgsRefTuple;

  return [fn = std::move(fn)](
             const recipe::Args& args) -> absl::StatusOr<recipe::ReturnValue> {
    constexpr int kRequiredArgs = std::tuple_size<ConstArgsRefTuple>::value;
    ConstArgsRefTuple args_tuple =
        internal::ArgsToTupleHelper<ConstArgsRefTuple>(
            args, std::make_index_sequence<kRequiredArgs>());

    if constexpr (std::is_same_v<ReturnT, void>) {
      // Handles the case where the return type is void.
      std::apply(fn, args_tuple);
      return recipe::ReturnValue{};
    } else if constexpr (std::is_same_v<ReturnT, absl::Status>) {
      // Handles the case where the return type is an absl::Status
      return recipe::internal::ConvertStatusToStatusOrReturnValue(
          std::forward<absl::Status>(std::apply(fn, args_tuple)));
    } else if constexpr (std::is_same_v<ReturnT, Future<absl::Status>>) {
      // Handles the case where the return type is a Future<absl::Status>
      return recipe::internal::ConvertFutureStatusToReturnValue(
          std::forward<Future<absl::Status>>(std::apply(fn, args_tuple)));
    } else if constexpr (std::is_same_v<ReturnT,
                                        recipe::ReturnValueDeclaration>) {
      // Handles the case where the return type is a
      // recipe::ReturnValueDeclaration
      return recipe::internal::ConvertReturnValueDeclarationToReturnValue(
          std::forward<recipe::ReturnValueDeclaration>(
              std::apply(fn, args_tuple)));
    } else if constexpr (std::is_same_v<
                             ReturnT,
                             absl::StatusOr<recipe::ReturnValueDeclaration>>) {
      // Handles the case where the return type is a
      // absl::StatusOr<recipe::ReturnValueDeclaration>
      return recipe::internal::
          ConvertStatusOrReturnValueDeclarationToStatusOrReturnValue(
              std::forward<absl::StatusOr<recipe::ReturnValueDeclaration>>(
                  std::apply(fn, args_tuple)));
    } else if constexpr (imp::internal::future_traits::IsFutureV<ReturnT>) {
      // Handles the case where a Future is returned.
      if constexpr (std::is_same_v<ReturnT, Future<recipe::Variables>>) {
        // Handles the case where the return type is a Future<recipe::Variables>
        return recipe::internal::ConvertFutureRecipeVariablesToReturnValue(
            std::forward<Future<recipe::Variables>>(
                std::apply(fn, args_tuple)));
      } else if constexpr (std::is_same_v<ReturnT, Future<recipe::Variable>>) {
        // Handles the case where the return type is a Future<recipe::Variable>
        return recipe::internal::ConvertFutureRecipeVariableToReturnValue(
            std::forward<Future<recipe::Variable>>(std::apply(fn, args_tuple)));
      } else {
        // If the function returns a Future<T> where T can be converted
        // into Recipe Variable, we need to transform the return type
        // into a Future<Variables> once the Future becomes ready.
        return recipe::internal::ConvertFutureTemplateToReturnValue(
            std::forward<ReturnT>(std::apply(fn, args_tuple)));
      }
    } else if constexpr (std::is_same_v<ReturnT, recipe::Variables>) {
      // Handle methods that return a String Map of recipe::Variables
      return recipe::internal::ConvertRecipeVariablesToReturnValue(
          std::forward<recipe::Variables>(std::apply(fn, args_tuple)));
    } else if constexpr (std::is_same_v<ReturnT,
                                        absl::StatusOr<recipe::Variables>>) {
      // Handle methods that return a String Map of recipe::Variables or a
      // absl::Status Message
      return recipe::internal::
          ConvertStatusOrRecipeVariablesToStatusOrReturnValue(
              std::forward<absl::StatusOr<recipe::Variables>>(
                  std::apply(fn, args_tuple)));
    } else if constexpr (recipe_traits::kIsStatusOrV<ReturnT>) {
      // Handle methods that return an absl::StatusOr object of a
      // recipe::Variable or one it's raw value types
      return recipe::internal::ConvertStatusOrRecipeVariableToReturnValue(
          std::forward<absl::StatusOr<recipe::Variable>>(
              std::apply(fn, args_tuple)));
    } else {
      // Handles the case where the return type is a recipe::Variable or one of
      // it's raw value types
      return recipe::internal::ConvertRecipeVariableToReturnValue(
          std::forward<recipe::Variable>(std::apply(fn, args_tuple)));
    }
  };
}

}  // namespace imp::recipe

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_FUNCTION_UTILS_H_
