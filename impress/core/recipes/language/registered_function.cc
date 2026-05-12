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

#include "core/recipes/language/registered_function.h"

#include <cstddef>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/recipes/language/recipe_function_utils.h"
#include "core/recipes/language/recipe_types.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {
using Param = RegisteredFunction::Param;
}  // namespace

RegisteredFunction::Builder::Builder(std::string_view name) : name_(name) {}

std::unique_ptr<RegisteredFunction> RegisteredFunction::Builder::BuildInternal(
    RecipeFunction fn, const std::vector<int>& parameter_types,
    bool uses_explicit_arg_type_checking) {
  if (!fn || params_.size() > kMaxArgumentLimit) {
    return nullptr;
  }

  if (uses_explicit_arg_type_checking && !FillParameterTypes(parameter_types)) {
    return nullptr;
  }

  return std::make_unique<RegisteredFunction>(name_, params_, std::move(fn),
                                              uses_explicit_arg_type_checking);
}

RegisteredFunction::RegisteredFunction(std::string_view name,
                                       const std::vector<Param>& params,
                                       RecipeFunction fn,
                                       bool uses_explicit_arg_type_checking)
    : name_(name),
      params_(params),
      fn_(std::move(fn)),
      params_with_default_values_bit_flag_(0),
      uses_explicit_arg_type_checking_(uses_explicit_arg_type_checking) {
  for (size_t i = 0; i < params_.size(); ++i) {
    arg_positions_[params_[i].name] = i;
    if (!std::holds_alternative<std::monostate>(params[i].default_value)) {
      params_with_default_values_bit_flag_ |= (1 << i);
    }
  }
}

bool RegisteredFunction::Builder::FillParameterTypes(
    const std::vector<int>& parameter_types) {
  if (!parameter_types.empty() && parameter_types.size() < params_.size()) {
    return false;
  }

  for (int i = params_.size(); i < parameter_types.size(); ++i) {
    AddParam(absl::StrCat(recipe::kDefaultArgPrefix, i));
  }

  for (int i = 0; i < parameter_types.size(); ++i) {
    if (parameter_types[i] !=
            recipe::GetRecipeVariableIndex<recipe::Variable>() &&
        params_[i].type != recipe::GetRecipeVariableIndex<recipe::Variable>() &&
        params_[i].type != parameter_types[i]) {
      return false;
    }

    params_[i].type = parameter_types[i];
  }

  return true;
}

absl::StatusOr<recipe::ReturnValue> RegisteredFunction::Execute(
    const Args& args, const NamedArgs& named_args) const {
  imp::output::Recipe("Calling function %s", name_);

  // Special case to handle registered functions that take in N positional
  // arguments through the recipe::Args vector or don't take any arguments. This
  // skips the argument matching logic and directly passes the args  vector to
  // the function.
  if (!uses_explicit_arg_type_checking_ && params_.empty() &&
      named_args.empty()) {
    return fn_(args);
  }

  if (args.size() > kMaxArgumentLimit) {
    return absl::InvalidArgumentError("Too Many Arguments provided.");
  }

  // Return an error message if there are too many positional arguments.
  if ((!named_args.empty() || params_with_default_values_bit_flag_ != 0) &&
      args.size() > params_.size()) {
    return absl::InvalidArgumentError(absl::StrFormat(
        "Too many positional arguments provided to function %s. "
        "Expected at most %d, got %d positional and %d named arguments.",
        name_, params_.size(), args.size(), named_args.size()));
  }

  // Translate used parameter indexes to a bit flag mask
  size_t args_provided_bit_flag = ((1 << args.size()) - 1);
  size_t params_bit_flag = ((1 << params_.size()) - 1);

  // For every named parameter passed to the RecipeFunction...
  for (const auto& [arg_name, arg_value] : named_args) {
    // Return an error if the named argument does not exist in the
    // RecipeFunction.
    auto pos_it = arg_positions_.find(arg_name);
    if (pos_it == arg_positions_.end()) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Unknown named argument '%s' provided to function '%s'.", arg_name,
          name_));
    }

    // Return an error if the parameter's value had already been provided.
    size_t arg_position_bit = (1 << pos_it->second);
    if ((args_provided_bit_flag & arg_position_bit) != 0) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Argument '%s' to function '%s' was already provided.", arg_name,
          name_));
    }

    args_provided_bit_flag |= arg_position_bit;
  }

  // Return an error message if there are still some missing arguments.
  if ((args_provided_bit_flag | params_with_default_values_bit_flag_) !=
      params_bit_flag) {
    return params_with_default_values_bit_flag_ == 0
               ? absl::InvalidArgumentError(absl::StrFormat(
                     "function %s requires %d arguments but got %d.", name_,
                     params_.size(),
                     __builtin_popcount(args_provided_bit_flag)))
               : absl::InvalidArgumentError(absl::StrFormat(
                     "Missing required argument '%s' for function '%s'.",
                     params_[__builtin_ctz(~args_provided_bit_flag)].name,
                     name_));
  }

  std::vector<Variable> final_args(params_.size());

  // Fill in positional arguments first.
  for (int i = 0; i < args.size(); ++i) {
    final_args[i] = args[i];
  }

  // Fill in named arguments.
  for (const auto& [arg_name, arg_value] : named_args) {
    final_args[arg_positions_.at(arg_name)] = arg_value;
  }

  // Fill in default arguments for missing arguments (if any exist).
  while ((args_provided_bit_flag & params_with_default_values_bit_flag_) !=
         params_with_default_values_bit_flag_) {
    size_t index = __builtin_ctz(
        (~args_provided_bit_flag & params_with_default_values_bit_flag_));
    final_args[index] = params_[index].default_value;
    args_provided_bit_flag |= (1 << index);
  }

  // Ensure final arguments have the correct parameter types
  for (int i = 0; i < params_.size(); ++i) {
    if (params_[i].type == recipe::GetRecipeVariableIndex<recipe::Variable>() ||
        static_cast<int>(final_args[i].index()) == params_[i].type) {
      continue;
    }

#if IMP_ENABLE_RECIPE_EXPERIMENTAL
    bool was_able_to_coerce_variable = false;
    switch (params_[i].type) {
      case recipe::GetRecipeVariableIndex<bool>(): {
        std::optional<bool> coerced_val = recipe::CoerceToBool(final_args[i]);
        if (coerced_val != std::nullopt) {
          final_args[i].emplace<bool>(coerced_val.value());
          was_able_to_coerce_variable = true;
        }
        break;
      }
      case recipe::GetRecipeVariableIndex<int>(): {
        std::optional<int> coerced_val = recipe::CoerceToInt(final_args[i]);
        if (coerced_val != std::nullopt) {
          final_args[i].emplace<int>(coerced_val.value());
          was_able_to_coerce_variable = true;
        }
        break;
      }
      case recipe::GetRecipeVariableIndex<float>(): {
        std::optional<float> coerced_val = recipe::CoerceToFloat(final_args[i]);
        if (coerced_val != std::nullopt) {
          final_args[i].emplace<float>(coerced_val.value());
          was_able_to_coerce_variable = true;
        }
        break;
      }
      case recipe::GetRecipeVariableIndex<double>(): {
        std::optional<double> coerced_val =
            recipe::CoerceToDouble(final_args[i]);
        if (coerced_val != std::nullopt) {
          final_args[i].emplace<double>(coerced_val.value());
          was_able_to_coerce_variable = true;
        }
        break;
      }
      case recipe::GetRecipeVariableIndex<float3>(): {
        std::optional<float3> coerced_val =
            recipe::CoerceToFloat3(final_args[i]);
        if (coerced_val != std::nullopt) {
          final_args[i].emplace<float3>(coerced_val.value());
          was_able_to_coerce_variable = true;
        }
        break;
      }
      case recipe::GetRecipeVariableIndex<float4>(): {
        std::optional<float4> coerced_val =
            recipe::CoerceToFloat4(final_args[i]);
        if (coerced_val != std::nullopt) {
          final_args[i].emplace<float4>(coerced_val.value());
          was_able_to_coerce_variable = true;
        }
        break;
      }
      case recipe::GetRecipeVariableIndex<quatf>(): {
        std::optional<quatf> coerced_val = recipe::CoerceToQuatf(final_args[i]);
        if (coerced_val != std::nullopt) {
          final_args[i].emplace<quatf>(coerced_val.value());
          was_able_to_coerce_variable = true;
        }
        break;
      }
      case recipe::GetRecipeVariableIndex<std::string>():
        final_args[i].emplace<std::string>(recipe::ToString(final_args[i]));
        was_able_to_coerce_variable = true;
        break;
      case recipe::GetRecipeVariableIndex<NodeHandle>(): {
        // Special case for coercing NodeHandle - if the variable can coerce,
        // but it is an invalid NodeHandle, return an error.
        auto coerced_node = recipe::CoerceToNode(final_args[i]);

        if (coerced_node.IsValid()) {
          final_args[i].emplace<NodeHandle>(coerced_node);
          was_able_to_coerce_variable = true;
        } else {
          return absl::InvalidArgumentError(
              absl::StrFormat("Function expected arg %d of type NodeHandle. "
                              "Received invalid Node.",
                              i));
        }
        break;
      }
    }

    if (!was_able_to_coerce_variable) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Function expected arg %d of type %s.", i,
          recipe::ToTypeName(
              static_cast<VariableDeclaration::Type>(params_[i].type))));
    }
#else
    return absl::InvalidArgumentError(absl::StrFormat(
        "Function expected arg %d of type %s.", i,
        recipe::ToTypeName(
            static_cast<VariableDeclaration::Type>(params_[i].type))));
#endif
  }

  return fn_(final_args);
}

}  // namespace imp::recipe
