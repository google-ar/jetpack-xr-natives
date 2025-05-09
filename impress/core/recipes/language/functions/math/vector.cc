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

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/math/vec.h"
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"
#include "core/recipes/language/recipe_graph.proto.imp.h"
#include "core/recipes/language/recipe_utils.h"

namespace imp::recipe {
namespace {

absl::StatusOr<float> GetVectorLength(const recipe::Variable& value) {
  switch (value.index()) {
    case Literal::kValue_Float2Value:
      return length(std::get<float2>(value));
    case Literal::kValue_Float3Value:
      return length(std::get<float3>(value));
    case Literal::kValue_Float4Value:
      return length(std::get<float4>(value));
    default:
      return absl::InvalidArgumentError("input must be a floatN type.");
  }
}

recipe::Variables Extract2(const float2 input) {
  recipe::Variables variables;
  variables["a"] = input.x;
  variables["b"] = input.y;
  return variables;
}

recipe::Variables Extract3(const float3 input) {
  recipe::Variables variables;
  variables["a"] = input.x;
  variables["b"] = input.y;
  variables["c"] = input.z;
  return variables;
}

recipe::Variables Extract4(const float4 input) {
  recipe::Variables variables;
  variables["a"] = input.x;
  variables["b"] = input.y;
  variables["c"] = input.z;
  variables["d"] = input.w;
  return variables;
}

}  // namespace

void RegisterMathVectorFunctions(BaseRecipeSystem* recipe_system) {
  recipe_system->RegisterFunction(
      "GetVectorLength", [](recipe::Variable value) -> absl::StatusOr<float> {
        return GetVectorLength(value);
      });

  recipe_system->RegisterFunction(
      "Combine2", [](float a, float b) { return float2(a, b); });

  recipe_system->RegisterFunction(
      "Combine3", [](float a, float b, float c) { return float3(a, b, c); });

  recipe_system->RegisterFunction(
      "Combine4",
      [](float a, float b, float c, float d) { return float4(a, b, c, d); });

  recipe_system->RegisterFunction("Extract2",
                                  [](float2 input) { return Extract2(input); });

  recipe_system->RegisterFunction("Extract3",
                                  [](float3 input) { return Extract3(input); });

  recipe_system->RegisterFunction("Extract4",
                                  [](float4 input) { return Extract4(input); });
}

}  // namespace imp::recipe
