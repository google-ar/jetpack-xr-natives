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
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {
using Param = RegisteredFunction::Param;
}  // namespace

RegisteredFunction::Builder::Builder(std::string_view name) : name_(name) {}

std::unique_ptr<RegisteredFunction> RegisteredFunction::Builder::BuildInternal(
    RecipeFunction fn, bool uses_explicit_arg_type_checking) {
  if (!fn || params_.size() > kMaxArgumentLimit) {
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

absl::StatusOr<recipe::ReturnValue> RegisteredFunction::Execute(
    Args& args, NamedArgs& named_args) const {
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
  for (size_t i = 0; i < args.size(); ++i) {
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

  return fn_(final_args);
}

}  // namespace imp::recipe
