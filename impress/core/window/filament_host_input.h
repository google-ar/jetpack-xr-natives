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

#ifndef THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAMENT_HOST_INPUT_H_
#define THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAMENT_HOST_INPUT_H_

#include "absl/types/variant.h"
#include "core/math/vec.h"

namespace imp::window::detail {

struct PointerDown {
  int id;
};

struct PointerMove {
  int2 position;
  int2 travel;

  // Temporarily added for compatibility.
  explicit PointerMove(int2 travel) : travel(travel) {}

  PointerMove(int2 position, int2 travel)
      : position(position), travel(travel) {}
};

struct PointerUp {
  int id;
};

struct Wheel {
  int2 travel;
};
using MouseInput = std::variant<PointerDown, PointerMove, PointerUp, Wheel>;

}  // namespace imp::window::detail

#endif  // THIRD_PARTY_IMPRESS_CORE_WINDOW_FILAMENT_HOST_INPUT_H_
