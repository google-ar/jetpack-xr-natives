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
#include <utility>
#include <vector>

#include "absl/status/statusor.h"
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
    std::optional<recipe::Variable> default_value = std::nullopt;
  };

  explicit RegisteredFunction(std::string name, std::vector<Param> params,
                              RecipeFunction fn);

  class Builder {
   public:
    explicit Builder(std::string_view name);

    std::string GetName() const { return name_; }

    Builder& AddParam(std::string_view name) {
      params_.push_back(Param{.name = std::string(name)});
      return *this;
    }

    template <typename T>
    Builder& AddParamWithDefault(std::string_view name, T default_value) {
      params_.push_back(
          Param{.name = std::string(name), .default_value = default_value});
      return *this;
    }

    std::unique_ptr<RegisteredFunction> Build(RecipeFunction fn) {
      return std::make_unique<RegisteredFunction>(
          std::move(name_), std::move(params_), std::move(fn));
    }

   private:
    std::string name_;
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
  absl::StatusOr<ReturnValue> Execute(Args& args, NamedArgs& named_args) const;

 private:
  std::string name_;
  std::vector<Param> params_;
  RecipeFunction fn_;
  StringMap<size_t> arg_positions_;
  int num_defaults_ = 0;
};

}  // namespace imp::recipe

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_REGISTERED_FUNCTION_H_
