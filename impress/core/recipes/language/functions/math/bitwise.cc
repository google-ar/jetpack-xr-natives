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
#include "core/recipes/language/base_recipe_system.h"
#include "core/recipes/language/functions/math/math.h"

namespace imp::recipe {
namespace {

absl::StatusOr<int> ShiftBitsRight(int value, int shift_amount) {
  if (shift_amount < 0 || shift_amount > 31) {
    return absl::InternalError("invalid value for shift amount.");
  }

  return (value >> shift_amount);
}

absl::StatusOr<int> ShiftBitsLeft(int value, int shift_amount) {
  if (shift_amount < 0 || shift_amount > 31) {
    return absl::InternalError("invalid value for shift amount.");
  }

  return (value << shift_amount);
}

}  // namespace

void RegisterMathBitWiseFunctions(BaseRecipeSystem* recipe_system) {
  // Performs a right bitwise shift on an integer.
  recipe_system->RegisterFunction(
      "Asr", [](int value, int shift_amount) -> absl::StatusOr<int> {
        return ShiftBitsRight(value, shift_amount);
      });

  // Performs a left bitwise shift on an integer.
  recipe_system->RegisterFunction(
      "Lsl", [](int value, int shift_amount) -> absl::StatusOr<int> {
        return ShiftBitsLeft(value, shift_amount);
      });

  // Counts leading zeros of the given number in binary representation.
  recipe_system->RegisterFunction("Clz", [](int value) -> int {
    return value == 0 ? 32 : (value < 0 ? 0 : __builtin_clz(value));
  });

  // Counts trailing zeros of the given number in binary representation.
  recipe_system->RegisterFunction("Ctz", [](int value) -> int {
    return value == 0 ? 32 : __builtin_ctz(value);
  });

  // Counts the number of bits set to 1 in an integer.
  recipe_system->RegisterFunction(
      "PopCnt", [](int value) { return __builtin_popcount(value); });
}

}  // namespace imp::recipe
