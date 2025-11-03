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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_BINARY_EXPRESSION_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_BINARY_EXPRESSION_H_

#include "absl/status/statusor.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {

absl::StatusOr<Variable> EvaluateBinaryExpression(
    const BinaryExpression::BinaryOps& op, const Variable& left,
    const Variable& right);

}  // namespace imp::recipe

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_BINARY_EXPRESSION_H_
