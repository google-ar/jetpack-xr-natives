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

#include <limits>

#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {

void RegisterMathConstants(BaseRecipeSystem* recipe_system) {
  (void)recipe_system->DeclareGlobalVariable(
      recipe::kMathPi, VariableType::FLOAT, static_cast<float>(M_PI));
  (void)recipe_system->DeclareGlobalVariable(
      recipe::kMathE, VariableType::FLOAT, static_cast<float>(M_E));
  (void)recipe_system->DeclareGlobalVariable(
      recipe::kMathNan, VariableType::FLOAT,
      std::numeric_limits<float>::quiet_NaN());
  (void)recipe_system->DeclareGlobalVariable(
      recipe::kMathInf, VariableType::FLOAT,
      std::numeric_limits<float>::infinity());
}

}  // namespace imp::recipe
