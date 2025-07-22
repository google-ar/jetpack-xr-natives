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

#include "core/assets/gltf/object_model/token_parser.h"

#include <optional>

#include "absl/strings/numbers.h"
#include "absl/strings/string_view.h"
namespace imp::gltf {

namespace {

std::optional<ParsedToken> ParseIntToken(absl::string_view token) {
  int value;
  if (absl::SimpleAtoi(token, &value)) {
    return value;
  }
  return std::nullopt;
}

}  // namespace

TokenParser GetIntTokenParser() { return ParseIntToken; }

}  // namespace imp::gltf
