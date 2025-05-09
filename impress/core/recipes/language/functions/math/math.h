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

#ifndef THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_FUNCTIONS_MATH_H_
#define THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_FUNCTIONS_MATH_H_

#include "core/recipes/language/base_recipe_system.h"

namespace imp::recipe {
// Each method below registers a category of math functions, the categorization
// roughly follows the groups defined in Section 4.1.1 of the GLTF spec:
// https://github.com/KhronosGroup/glTF/blob/interactivity/extensions/2.0/Khronos/KHR_interactivity/Specification.adoc#math-nodes

void RegisterMathAngleFunctions(BaseRecipeSystem* recipe_system);

void RegisterMathArithmeticFunctions(BaseRecipeSystem* recipe_system);

void RegisterMathBitWiseFunctions(BaseRecipeSystem* recipe_system);

void RegisterMathConstants(BaseRecipeSystem* recipe_system);

/*
 * Register all the Exponential Nodes
 * See
 * https://github.com/KhronosGroup/glTF/blob/interactivity/extensions/2.0/Khronos/KHR_interactivity/Specification.adoc#4114-exponential-nodes
 */
void RegisterMathExponentialFunctions(BaseRecipeSystem* recipe_system);

/*
 * Register all the Hyperbolic Nodes
 * See
 * https://github.com/KhronosGroup/glTF/blob/interactivity/extensions/2.0/Khronos/KHR_interactivity/Specification.adoc#4116-hyperbolic-nodes
 */
void RegisterMathHyperbolicFunctions(BaseRecipeSystem* recipe_system);

void RegisterMathUtilityFunctions(BaseRecipeSystem* recipe_system);

void RegisterMathVectorFunctions(BaseRecipeSystem* recipe_system);

}  // namespace imp::recipe

#endif  // THIRD_PARTY_IMPRESS_CORE_RECIPES_LANGUAGE_FUNCTIONS_MATH_H_
