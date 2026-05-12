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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_REGISTERED_FUNCTION_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_REGISTERED_FUNCTION_H_

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/recipes/language/recipe_function_utils.h"
#include "core/recipes/language/recipe_traits.h"
#include "core/recipes/language/recipe_utils.h"
#include "core/view/utils/string_map.h"

namespace imp::recipe {

// Represents a function that has been registered with the recipe system.
// This class encapsulates the function's name, argument information, and the
// actual function pointer/lambda that will be executed.
class RegisteredFunction {
 public:
  // Represents information about a single parameter of a registered recipe
  // function.  This includes the parameter's name and an optional default
  // value.
  struct Param {
    std::string name;
    int type;
    recipe::Variable default_value;
  };

  // Argument limit used when the RegisteredFunction is checking for explicit
  // argument types or named parameters. Is needed to prevent any accidental
  // overflow when operating with the bit flags in the `Execute()` method. If a
  // Recipe Function requires more arguments than what is listed here, please
  // use a `recipe::Args` variable by itself to hold the arguments for the
  // RegisteredFunction instead of using explicit argument names or types.
  static constexpr size_t kMaxArgumentLimit = 16;

  explicit RegisteredFunction(std::string_view name,
                              const std::vector<Param>& params,
                              RecipeFunction fn,
                              bool uses_explicit_arg_type_checking);

  absl::string_view GetName() const { return name_; }

  class Builder {
   public:
    explicit Builder(std::string_view name = "");

    absl::string_view GetName() const { return name_; }

    // TODO: Remove dedicated name variable and pass the name to
    // the builder through the `Build` method
    ABSL_ATTRIBUTE_NOINLINE void SetName(absl::string_view name) {
      name_ = name;
    }

    ABSL_ATTRIBUTE_NOINLINE void ClearParams() { params_.clear(); }

    Builder& AddParam(std::string_view name) {
      params_.push_back(
          Param{.name = std::string(name),
                .type = recipe::GetRecipeVariableIndex<recipe::Variable>()});
      return *this;
    }

    template <typename T>
    Builder& AddParamWithDefault(std::string_view name, T default_value) {
      params_.push_back(Param{.name = std::string(name),
                              .type = recipe::GetRecipeVariableIndex<T>(),
                              .default_value = default_value});
      return *this;
    }

    template <typename Fn>
    std::unique_ptr<RegisteredFunction> Build(Fn fn) {
      using FnType = std::decay_t<Fn>;
      if constexpr (std::is_constructible_v<RecipeFunction, FnType>) {
        return BuildInternal(std::move(fn), {}, false);
      } else {
        using FunctorUnpacker =
            decltype(recipe_traits::FunctorUnpacker(&FnType::operator()));
        using ConstArgsRefTuple = typename FunctorUnpacker::ConstArgsRefTuple;

        constexpr bool kUsesRecipeArgsVariable =
            std::is_same_v<ConstArgsRefTuple, std::tuple<const recipe::Args&>>;
        constexpr int kRequiredArgsCount =
            kUsesRecipeArgsVariable ? 0
                                    : std::tuple_size<ConstArgsRefTuple>::value;

        return BuildInternal(
            recipe::MakeRecipeFunction<Fn>(std::move(fn)),
            recipe::GetParameterTypeIds<ConstArgsRefTuple>(
                std::make_index_sequence<kRequiredArgsCount>()),
            !kUsesRecipeArgsVariable);
      }
    }

   private:
    std::unique_ptr<RegisteredFunction> BuildInternal(
        RecipeFunction fn, const std::vector<int>& parameter_types,
        bool uses_explicit_arg_type_checking);

    bool FillParameterTypes(const std::vector<int>& parameter_types);

    // The name of the RegisteredFunction to build
    std::string name_;

    // The parameters to add to the built RegisteredFunction
    std::vector<Param> params_;
  };

  // Executes the registered function with the provided arguments.
  //
  // Argument matching logic:
  // 1. Positional arguments are matched first, in order, with the declared
  //    parameters.
  // 2. Named arguments are then matched by name to the declared parameters.
  //    If a parameter has already been assigned a value by a positional
  //    argument, an error is returned.
  // 3. For any parameters that have not been assigned a value through
  //    positional or named arguments, if a default value is defined for that
  //    parameter, the default value is used.
  // 4. If there are still unassigned parameters that do not have default
  //    values, an error is returned, indicating missing required arguments.
  absl::StatusOr<ReturnValue> Execute(const Args& args,
                                      const NamedArgs& named_args) const;

 private:
  // The name of this RegisteredFunction
  std::string name_;

  // The Parameters used in this RegisteredFunction
  std::vector<Param> params_;

  // The backing RecipeFunction that will be called when this RegisteredFunction
  // is executed
  RecipeFunction fn_;

  // A map of variable names to argument indices
  StringMap<size_t> arg_positions_;

  // A bit flag where each bit represents index of an argument that holds a
  // default value. Is not used if `uses_explicit_arg_type_checking_` set to
  // false.
  size_t params_with_default_values_bit_flag_;

  // States whether or not this RegisteredFunction will check for explicit
  // argument types passed into it
  bool uses_explicit_arg_type_checking_;
};

}  // namespace imp::recipe

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_REGISTERED_FUNCTION_H_
