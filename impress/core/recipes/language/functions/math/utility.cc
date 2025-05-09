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

#include <cmath>
#include <variant>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {

absl::StatusOr<recipe::Variable> Select(bool condition,
                                        const recipe::Variable& a,
                                        const recipe::Variable& b) {
  if (a.index() != b.index()) {
    return absl::InvalidArgumentError("b must be of the same type as a.");
  }

  return condition ? a : b;
}

}  // namespace

void RegisterMathUtilityFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction("IsInf", [](float input) -> bool {
#ifdef __FINITE_MATH_ONLY__
    static_assert(__FINITE_MATH_ONLY__ == 0, "Infinite math is required.");
#endif
    return std::isinf(input);
  });

  recipe_system->RegisterFunction("IsNan", [](float input) -> bool {
#ifdef __FINITE_MATH_ONLY__
    static_assert(__FINITE_MATH_ONLY__ == 0, "Infinite math is required.");
#endif
    return std::isnan(input);
  });

  recipe_system->RegisterFunction(
      "Select",
      [](bool condition, recipe::Variable a,
         recipe::Variable b) -> absl::StatusOr<recipe::Variable> {
        return Select(condition, a, b);
      });
}

}  // namespace imp::recipe
