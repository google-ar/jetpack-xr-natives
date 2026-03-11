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
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "core/async/future.h"
#include "core/async/future_traits.h"
#include "core/recipes/language/recipe_traits.h"
#include "core/recipes/language/recipe_utils.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::recipe {

namespace internal {

template <typename TupleT, std::size_t Index>
auto ArgsToTupleElementHelper(recipe::Args& args, absl::Status& out_status)
    -> std::tuple_element_t<Index, TupleT> {
  // It's possible for a default-constructed tuple element, e.g.
  //   return std::tuple_element_t<Index, TupleT>{}
  // to return a reference type, which emits a compiler warning for returning a
  // reference to a local object on the stack. Returning a static,
  // default-constructed object is a (slightly hacky) workaround to always
  // return a non-local object.
  static const std::tuple_element_t<Index, TupleT> kErrorDefault{};

  if (!out_status.ok()) {
    return kErrorDefault;
  }

  using ElementT = std::decay_t<std::tuple_element_t<Index, TupleT>>;
  recipe::Variable& variable = args.at(Index);

#if IMP_ENABLE_RECIPE_EXPERIMENTAL
  // The experimental path attempts to coerce the variable into the correct
  // type, if possible.
  if constexpr (std::is_same_v<ElementT, recipe::Variable>) {
    return variable;
  } else if constexpr (std::is_same_v<ElementT, NodeHandle>) {
    // Special case for coercing NodeHandle - if the variable can coerce, but
    // it is an invalid NodeHandle, return an error.
    auto coerced_node = recipe::CoerceToNode(variable);

    if (coerced_node.IsValid()) {
      return coerced_node;
    } else {
      out_status = absl::InvalidArgumentError(
          absl::StrFormat("Function expected arg %d of type NodeHandle. "
                          "Received invalid Node.",
                          Index));
      return kErrorDefault;
    }
  } else if (!absl::holds_alternative<ElementT>(variable)) {
    // When the variable is not the correct type, attempt to coerce it.
    if constexpr (std::is_same_v<ElementT, bool>) {
      auto coerced_bool = recipe::CoerceToBool(variable);
      if (coerced_bool != std::nullopt) {
        return coerced_bool.value();
      }
    }
    if constexpr (std::is_same_v<ElementT, int>) {
      auto coerced_int = recipe::CoerceToInt(variable);
      if (coerced_int != std::nullopt) {
        return coerced_int.value();
      }
    }
    if constexpr (std::is_same_v<ElementT, float>) {
      auto coerced_float = recipe::CoerceToFloat(variable);
      if (coerced_float != std::nullopt) {
        return coerced_float.value();
      }
    }
    if constexpr (std::is_same_v<ElementT, float3>) {
      auto coerced_float3 = recipe::CoerceToFloat3(variable);
      if (coerced_float3 != std::nullopt) {
        return coerced_float3.value();
      }
    }
    if constexpr (std::is_same_v<ElementT, float4>) {
      auto coerced_float4 = recipe::CoerceToFloat4(variable);
      if (coerced_float4 != std::nullopt) {
        return coerced_float4.value();
      }
    }
    if constexpr (std::is_same_v<ElementT, quatf>) {
      auto coerced_quatf = recipe::CoerceToQuatf(variable);
      if (coerced_quatf != std::nullopt) {
        return coerced_quatf.value();
      }
    }
    if constexpr (std::is_same_v<ElementT, double>) {
      auto coerced_double = recipe::CoerceToDouble(variable);
      if (coerced_double != std::nullopt) {
        return coerced_double.value();
      }
    }
    if constexpr (std::is_same_v<ElementT, std::string>) {
      return recipe::ToString(variable);
    }
    out_status = absl::InvalidArgumentError(absl::StrFormat(
        "Function expected arg %d of type %s. Received: %s", Index,
        type_traits::kTypeName<ElementT>, recipe::ToTypeName(variable)));

    return kErrorDefault;
  } else {
    return absl::get<ElementT>(variable);
  }
#else
  if constexpr (std::is_same_v<ElementT, recipe::Variable>) {
    return variable;
  } else if (!absl::holds_alternative<ElementT>(variable)) {
    out_status = absl::InvalidArgumentError(
        absl::StrFormat("Function expected arg %d of type %s.", Index,
                        type_traits::kTypeName<ElementT>));

    return kErrorDefault;
  } else {
    return absl::get<ElementT>(variable);
  }
#endif
}

template <typename TupleT, std::size_t... Indices>
absl::StatusOr<TupleT> ArgsToTupleHelper(recipe::Args& args,
                                         std::index_sequence<Indices...>) {
  absl::Status status = absl::OkStatus();

  TupleT tuple =
      TupleT(ArgsToTupleElementHelper<TupleT, Indices>(args, status)...);

  MP_RETURN_IF_ERROR(status);

  return tuple;
}

template <>
absl::StatusOr<std::tuple<recipe::Args>> ArgsToTupleHelper(
    recipe::Args& args, std::index_sequence<0>);

// Helper Method that takes in a `absl::Status` message and converts it to a
// `absl::StatusOr<recipe::ReturnValue>` object that is returned from the
// method. Will return store an empty `recipe::Variables` object in the returned
// `absl::StatusOr<recipe::ReturnValue>` object if an `absl::OkStatus` message
// was passed into the method.
//
// - status - the `absl::Status` message to convert to a
// `absl::StatusOr<recipe::ReturnValue>` object
absl::StatusOr<recipe::ReturnValue> ConvertStatusToStatusOrReturnValue(
    absl::Status status);

// Helper Method that takes in a `Future<absl::Status>`, converts it to a
// `Future<recipe::Variables>`, and stores it in a `recipe::ReturnValue` that is
// returned by the method.
//
// - future_status - the `Future<absl::Status>` object to convert to a
// `Future<recipe::Variables>`
recipe::ReturnValue ConvertFutureStatusToReturnValue(
    Future<absl::Status> future_status);

// Helper Method that takes in a `absl::StatusOr<recipe::ReturnValue>` and
// converts internal data to a `recipe::ReturnValue` object that should be
// returned by the method. Will return an `absl::Status` message if that was
// passed into the method instead.
//
// - return_value_declaration - the `ReturnValueDeclaration` object to convert
// to a `recipe::ReturnValue`
absl::StatusOr<recipe::ReturnValue>
ConvertStatusOrReturnValueDeclarationToStatusOrReturnValue(
    absl::StatusOr<ReturnValueDeclaration> return_value_declaration);

// Helper Method that takes in a `ReturnValueDeclaration` and converts its
// internal data to a `recipe::ReturnValue` object that should be returned by
// the method.
//
// - return_value_declaration - the `ReturnValueDeclaration` object to convert
// to a `recipe::ReturnValue`
recipe::ReturnValue ConvertReturnValueDeclarationToReturnValue(
    ReturnValueDeclaration return_value_declaration);

// Helper Method that takes in a `Future<recipe::Variables>` and returns a
// `recipe::ReturnValue` object containing the `Future<recipe::Variables>`.
//
// - variable - the `Future<recipe::Variables>` object to store in the returned
// `recipe::ReturnValue`
recipe::ReturnValue ConvertFutureRecipeVariablesToReturnValue(
    Future<recipe::Variables> variables);

// Helper Method that takes in a `Future<recipe::Variable>` and returns a
// `recipe::ReturnValue` object containing the `Future<recipe::Variable>`.
//
// - variable - the `Future<recipe::Variable>` object to store in the returned
// `recipe::ReturnValue`
recipe::ReturnValue ConvertFutureRecipeVariableToReturnValue(
    Future<recipe::Variable> variable);

// Helper Method that takes in a `Future<T>`, converts it into a
// `Future<recipe::Variable>` (if possible), and returns a `recipe::ReturnValue`
// object containing the converted `Future<recipe::Variable>`.
//
// - variable - the `Future<T>` to convert to a `Future<recipe::Variable>` and
// stores it in the returned `recipe::ReturnValue`
template <typename T>
recipe::ReturnValue ConvertFutureTemplateToReturnValue(Future<T> variable) {
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
    absl::StatusOr<recipe::Variables> variables);

// Helper Method that takes in a `recipe::Variables` object and returns a
// `recipe::ReturnValue` object holding a list of `recipe::Variable`s inside of
// said `recipe::Variables` object.
//
// - variables - the list of the `recipe::Variable`s to be placed inside of the
// returned `recipe::ReturnValue` object
recipe::ReturnValue ConvertRecipeVariablesToReturnValue(
    recipe::Variables variables);

// Helper method that takes in an `absl::StatusOr` object of `recipe::Variable`
// or one of its raw value types and returns a `recipe::ReturnValue` struct
// containing it. Will return an absl::Status object if the
// `absl::StatusOr` parameter contained an `absl::Status` message instead.
//
// - variable - the `absl::StatusOr<recipe::Variable>` to be placed inside of
// the returned `recipe::ReturnValue` or returned immediately depending on if it
// holds an `absl::Status` message or not
absl::StatusOr<recipe::ReturnValue> ConvertStatusOrRecipeVariableToReturnValue(
    absl::StatusOr<recipe::Variable> variable);

// Helper method that takes in a `recipe::Variable` or one of it's raw value
// types and returns a `recipe::ReturnValue` struct that contains
// the recipe::Variable.
//
// - variable - the `recipe::Variable` to be placed inside of the returned
// `recipe::ReturnValue`
recipe::ReturnValue ConvertRecipeVariableToReturnValue(
    recipe::Variable variable);

}  // namespace internal

// TODO: Update this function to take in the built
// RegisteredFunction.
template <typename Fn>
recipe::RecipeFunction MakeRecipeFunction(Fn fn) {
  using FnType = std::decay_t<Fn>;
  using FunctorUnpacker =
      decltype(recipe_traits::FunctorUnpacker(&FnType::operator()));
  using ReturnT = typename FunctorUnpacker::ReturnT;
  using ArgsTuple = typename FunctorUnpacker::ArgsTuple;

  return [fn = std::move(fn)](
             recipe::Args& args) -> absl::StatusOr<recipe::ReturnValue> {
    constexpr int kRequiredArgs = std::tuple_size<ArgsTuple>::value;
    MP_ASSIGN_OR_RETURN(ArgsTuple args_tuple,
                     internal::ArgsToTupleHelper<ArgsTuple>(
                         args, std::make_index_sequence<kRequiredArgs>()));

    if constexpr (std::is_same_v<ReturnT, void>) {
      // Handles the case where the return type is void.
      std::apply(fn, args_tuple);
      return recipe::ReturnValue{};
    } else if constexpr (std::is_same_v<ReturnT, absl::Status>) {
      // Handles the case where the return type is an absl::Status
      return recipe::internal::ConvertStatusToStatusOrReturnValue(
          std::apply(fn, args_tuple));
    } else if constexpr (std::is_same_v<ReturnT, Future<absl::Status>>) {
      // Handles the case where the return type is a Future<absl::Status>
      return recipe::internal::ConvertFutureStatusToReturnValue(
          std::apply(fn, args_tuple));
    } else if constexpr (std::is_same_v<ReturnT,
                                        recipe::ReturnValueDeclaration>) {
      // Handles the case where the return type is a
      // recipe::ReturnValueDeclaration
      return recipe::internal::ConvertReturnValueDeclarationToReturnValue(
          std::apply(fn, args_tuple));
    } else if constexpr (std::is_same_v<
                             ReturnT,
                             absl::StatusOr<recipe::ReturnValueDeclaration>>) {
      // Handles the case where the return type is a
      // absl::StatusOr<recipe::ReturnValueDeclaration>
      return recipe::internal::
          ConvertStatusOrReturnValueDeclarationToStatusOrReturnValue(
              std::apply(fn, args_tuple));
    } else if constexpr (imp::internal::future_traits::IsFutureV<ReturnT>) {
      // Handles the case where a Future is returned.
      if constexpr (std::is_same_v<ReturnT, Future<recipe::Variables>>) {
        // Handles the case where the return type is a Future<recipe::Variables>
        return recipe::internal::ConvertFutureRecipeVariablesToReturnValue(
            std::apply(fn, args_tuple));
      } else if constexpr (std::is_same_v<ReturnT, Future<recipe::Variable>>) {
        // Handles the case where the return type is a Future<recipe::Variable>
        return recipe::internal::ConvertFutureRecipeVariableToReturnValue(
            std::apply(fn, args_tuple));
      } else {
        // If the function returns a Future<T> where T can be converted
        // into Recipe Variable, we need to transform the return type
        // into a Future<Variables> once the Future becomes ready.
        return recipe::internal::ConvertFutureTemplateToReturnValue(
            std::apply(fn, args_tuple));
      }
    } else if constexpr (std::is_same_v<ReturnT, recipe::Variables>) {
      // Handle methods that return a String Map of recipe::Variables
      return recipe::internal::ConvertRecipeVariablesToReturnValue(
          std::apply(fn, args_tuple));
    } else if constexpr (std::is_same_v<ReturnT,
                                        absl::StatusOr<recipe::Variables>>) {
      // Handle methods that return a String Map of recipe::Variables or a
      // absl::Status Message
      return recipe::internal::
          ConvertStatusOrRecipeVariablesToStatusOrReturnValue(
              std::apply(fn, args_tuple));
    } else if constexpr (recipe_traits::kIsStatusOrV<ReturnT>) {
      // Handle methods that return an absl::StatusOr object of a
      // recipe::Variable or one it's raw value types
      return recipe::internal::ConvertStatusOrRecipeVariableToReturnValue(
          std::apply(fn, args_tuple));
    } else {
      // Handles the case where the return type is a recipe::Variable or one of
      // it's raw value types
      return recipe::internal::ConvertRecipeVariableToReturnValue(
          std::apply(fn, args_tuple));
    }
  };
}

}  // namespace imp::recipe

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_RECIPE_FUNCTION_UTILS_H_
