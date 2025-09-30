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

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <type_traits>
#include <utility>
#include <variant>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/invocable.h"
#include "core/recipes/language/recipe_custom_statement.h"
#include "core/recipes/language/recipe_traits.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/recipes/language/registered_function.h"
#include "core/view/utils/string_map.h"
#include "mediapipe/framework/port/status_macros.h"

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
      absl::string_view name,
      std::unique_ptr<recipe::RegisteredFunction> function) = 0;

  template <typename TupleT, std::size_t Index>
  static auto ArgsToTupleElementHelper(recipe::Args& args,
                                       absl::Status& out_status)
      -> std::tuple_element_t<Index, TupleT>;

  template <typename TupleT, std::size_t... Indices>
  static absl::StatusOr<TupleT> ArgsToTupleHelper(
      recipe::Args& args, std::index_sequence<Indices...>);

  template <>
  absl::StatusOr<std::tuple<recipe::Args>> ArgsToTupleHelper(
      recipe::Args& args, std::index_sequence<0>) {
    return std::tuple<recipe::Args>(args);
  }

  template <typename Fn>
  std::unique_ptr<recipe::RegisteredFunction> MakeRecipeFunction(
      recipe::RegisteredFunction::Builder& builder, Fn fn);

  virtual void RegisterCustomStatementTypeImpl(
      absl::string_view name,
      Invocable<std::unique_ptr<RecipeCustomStatement>()> creation_fn) = 0;
};

template <typename Fn>
void BaseRecipeSystem::RegisterFunction(absl::string_view name, Fn fn) {
  using FnType = std::decay_t<Fn>;
  using FunctorUnpacker =
      decltype(recipe_traits::FunctorUnpacker(&FnType::operator()));
  using ArgsTuple = typename FunctorUnpacker::ArgsTuple;
  int kRequiredArgs = std::tuple_size<ArgsTuple>::value;
  recipe::RegisteredFunction::Builder builder =
      recipe::RegisteredFunction::Builder(name);
  for (int i = 0; i < kRequiredArgs; ++i) {
    builder.AddParam(absl::StrCat(recipe::kDefaultArgPrefix, i));
  }

  RegisterFunction(builder, std::move(fn));
}

template <typename Fn>
void BaseRecipeSystem::RegisterFunction(
    recipe::RegisteredFunction::Builder& builder, Fn fn) {
  RegisterFunctionImpl(builder.GetName(),
                       MakeRecipeFunction<Fn>(builder, std::move(fn)));
}

template <typename TupleT, std::size_t Index>
auto BaseRecipeSystem::ArgsToTupleElementHelper(recipe::Args& args,
                                                absl::Status& out_status)
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
absl::StatusOr<TupleT> BaseRecipeSystem::ArgsToTupleHelper(
    recipe::Args& args, std::index_sequence<Indices...>) {
  absl::Status status = absl::OkStatus();

  TupleT tuple =
      TupleT(ArgsToTupleElementHelper<TupleT, Indices>(args, status)...);

  MP_RETURN_IF_ERROR(status);

  return tuple;
}

// TODO: Update this function to take in the built
// RegisteredFunction.
template <typename Fn>
std::unique_ptr<recipe::RegisteredFunction>
BaseRecipeSystem::MakeRecipeFunction(
    recipe::RegisteredFunction::Builder& builder, Fn fn) {
  using FnType = std::decay_t<Fn>;
  using FunctorUnpacker =
      decltype(recipe_traits::FunctorUnpacker(&FnType::operator()));
  using ReturnT = typename FunctorUnpacker::ReturnT;
  using ArgsTuple = typename FunctorUnpacker::ArgsTuple;

  std::string name = builder.GetName();

  return builder.Build([fn = std::move(fn), name = name](recipe::Args& args)
                           -> absl::StatusOr<recipe::ReturnValue> {
    constexpr int kRequiredArgs = std::tuple_size<ArgsTuple>::value;
    constexpr bool kFnTakesArgs =
        std::is_same_v<ArgsTuple, std::tuple<recipe::Args>>;
    if constexpr (!kFnTakesArgs) {
      if (args.size() != kRequiredArgs) {
        return absl::FailedPreconditionError(
            absl::StrFormat("function %s requires %d arguments but got %d.",
                            name, kRequiredArgs, args.size()));
      }
    }

    MP_ASSIGN_OR_RETURN(ArgsTuple args_tuple,
                     ArgsToTupleHelper<ArgsTuple>(
                         args, std::make_index_sequence<kRequiredArgs>()));

    if constexpr (std::is_same_v<ReturnT, void>) {
      // Handles the case where the return type is void.
      std::apply(fn, args_tuple);
      return recipe::ReturnValue{};
    } else if constexpr (std::is_same_v<ReturnT, absl::Status>) {
      MP_RETURN_IF_ERROR(std::apply(fn, args_tuple));
      return recipe::ReturnValue{};
    } else if constexpr (std::is_same_v<ReturnT, Future<absl::Status>>) {
      // Handles the case where the return type is Future<absl::Status>.
      return recipe::ReturnValue{.async_values =
                                     std::apply(fn, args_tuple).Then([]() {
                                       return recipe::Variables();
                                     })};
    } else if constexpr (std::is_same_v<ReturnT,
                                        recipe::ReturnValueDeclaration> ||
                         std::is_same_v<
                             ReturnT,
                             absl::StatusOr<recipe::ReturnValueDeclaration>>) {
      // Handles the case where the return type is
      // recipe::ReturnValueDeclaration or
      // absl::StatusOr<recipe::ReturnValueDeclaration>.
      recipe::ReturnValue return_value;
      MP_ASSIGN_OR_RETURN(recipe::ReturnValueDeclaration variables,
                       absl::StatusOr<recipe::ReturnValueDeclaration>(
                           std::apply(fn, args_tuple)));

      // combined_future combines all of the async variables to one
      // future. If there's no async variables, then this will be
      // std::nullopt. Otherwise, this will become ready once all variable
      // futures are ready.
      std::optional<Future<absl::Status>> combined_future;
      StringMap<Future<recipe::Variable>> variable_futures;
      for (auto& [socket_name, variable] : variables.socket_values) {
        if (std::holds_alternative<recipe::Variable>(variable)) {
          return_value.values[socket_name] =
              std::get<recipe::Variable>(variable);
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

      if (variables.status_future.has_value()) {
        if (!combined_future) {
          combined_future = Future<absl::Status>(absl::OkStatus());
        }
        combined_future = combined_future->Combine(*variables.status_future);
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
    } else {
      // Handles the case where Recipe Variables are returned.
      if constexpr (internal::future_traits::IsFutureV<ReturnT>) {
        // Handles the case where a Future is returned.
        if constexpr (std::is_same_v<ReturnT, Future<recipe::Variables>>) {
          return recipe::ReturnValue{.async_values =
                                         std::apply(fn, args_tuple)};
        } else {
          // If the function returns a Future<T> where T can be converted
          // into Recipe Variable, we need to transform the return type
          // into a Future<Variables> once the Future becomes ready.
          return recipe::ReturnValue{
              .async_values =
                  std::apply(fn, args_tuple)
                      .Then([](typename ReturnT::Value return_value)
                                -> recipe::Variables {
                        recipe::Variables result;
                        result[std::string(recipe::kDefaultOutputSocketName)] =
                            return_value;
                        return result;
                      })};
        }
      } else {
        // If the return values are not Futures, we just need to populate
        // the sync return values.
        if constexpr (std::is_same_v<ReturnT, recipe::Variables>) {
          return recipe::ReturnValue{.values = std::apply(fn, args_tuple)};
        } else if constexpr (std::is_same_v<
                                 ReturnT, absl::StatusOr<recipe::Variables>>) {
          MP_ASSIGN_OR_RETURN(
              recipe::Variables variables,
              absl::StatusOr<recipe::Variables>(std::apply(fn, args_tuple)));

          return recipe::ReturnValue{.values = variables};
        } else {
          // Handles the case where the return type is a single Variable.
          recipe::Variables result;
          absl::StatusOr<recipe::Variable> variable =
              std::apply(fn, args_tuple);
          if (!variable.ok()) {
            return variable.status();
          }

          result[std::string(recipe::kDefaultOutputSocketName)] = *variable;
          return recipe::ReturnValue{.values = result};
        }
      }
    }
  });
}

template <typename T>
void BaseRecipeSystem::RegisterCustomStatementType() {
  RegisterCustomStatementTypeImpl(T::kName, []() {
    return std::unique_ptr<RecipeCustomStatement>(std::make_unique<T>());
  });
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_BASE_RECIPE_SYSTEM_H_
