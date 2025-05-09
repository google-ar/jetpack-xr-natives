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
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {
using Param = RegisteredFunction::Param;
}  // namespace

RegisteredFunction::Builder::Builder(std::string_view name) : name_(name) {}

RegisteredFunction::RegisteredFunction(std::string name,
                                       std::vector<Param> params,
                                       RecipeFunction fn)
    : name_(std::move(name)), params_(std::move(params)), fn_(std::move(fn)) {
  for (size_t i = 0; i < params_.size(); ++i) {
    Param& param = params_[i];
    arg_positions_[param.name] = i;
    if (param.default_value.has_value()) {
      num_defaults_++;
    }
  }
}

absl::StatusOr<recipe::ReturnValue> RegisteredFunction::Execute(
    Args& args, NamedArgs& named_args) const {
  imp::output::Recipe("Calling function %s", name_);

  // Special case to handle registered functions that take in N positional
  // arguments through the recipe::Args vector. This skips the argument matching
  // logic and directly passes the args vector to the function.
  if (named_args.empty() && args.size() >= params_.size() &&
      num_defaults_ == 0) {
    return fn_(args);
  }

  if (named_args.empty() && num_defaults_ == 0 &&
      args.size() != params_.size()) {
    return absl::InvalidArgumentError(
        absl::StrFormat("function %s requires %d arguments but got %d.", name_,
                        params_.size(), args.size()));
  } else if (args.size() > params_.size()) {
    // There are too many positional arguments.
    return absl::InvalidArgumentError(absl::StrFormat(
        "Too many positional arguments provided to function %s. "
        "Expected at most %d, got %d positional and %d named arguments.",
        name_, params_.size(), args.size(), named_args.size()));
  }

  std::vector<Variable> final_args(params_.size());
  std::vector<bool> arg_provided(params_.size(), false);

  // Fill in positional arguments first
  for (size_t i = 0; i < args.size(); ++i) {
    final_args[i] = args[i];
    arg_provided[i] = true;
  }

  // Fill in named arguments
  for (const auto& [arg_name, arg_value] : named_args) {
    auto pos_it = arg_positions_.find(arg_name);
    if (pos_it == arg_positions_.end()) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Unknown named argument '%s' provided to function '%s'.", arg_name,
          name_));
    }

    size_t arg_position = pos_it->second;
    if (arg_provided[arg_position]) {
      return absl::InvalidArgumentError(absl::StrFormat(
          "Argument '%s' to function '%s' was already provided.", arg_name,
          name_));
    }

    final_args[arg_position] = arg_value;
    arg_provided[arg_position] = true;
  }

  // Fill in default arguments for missing arguments
  for (size_t i = 0; i < params_.size(); ++i) {
    if (!arg_provided[i]) {
      if (params_[i].default_value.has_value()) {
        final_args[i] = *params_[i].default_value;
      } else {
        return absl::InvalidArgumentError(
            absl::StrFormat("Missing required argument '%s' for function '%s'.",
                            params_[i].name, name_));
      }
    }
  }

  return fn_(final_args);
}

}  // namespace imp::recipe
