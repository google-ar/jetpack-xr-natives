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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_TOKEN_PARSER_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_TOKEN_PARSER_H_

#include <optional>
#include <string>
#include <variant>

#include "absl/strings/string_view.h"

namespace imp::gltf {

// ParsedToken contains the result of parsing a token.
//
// std::monostate is used for tokens that do not contain any additional
// information. For example, a fixed token like "translation" can only either be
// parsed or fails to be parsed and does not contain any additional information.
// While an int token like "123" can be parsed into an int which can be
// referenced later.
using ParsedToken = std::variant<std::monostate, int>;

// TokenParserFn is pointer to a function that parses a token and returns a
// ParsedToken. The function should return std::nullopt if the token cannot be
// parsed.
using TokenParserFn = std::optional<ParsedToken> (*)(absl::string_view);

// TokenParser is a variant of a TokenParserFn and a fixed token.
//
// TokenParserFn is used for parsing variable tokens that can be parsed into a
// ParsedToken. For example, a token like "123" can be parsed into an int.
//
// std::string is used for parsing fixed tokens. For example, "translation".
using TokenParser = std::variant<TokenParserFn, std::string>;

// TokenParser for parsing tokens into ints.
TokenParser GetIntTokenParser();

}  // namespace imp::gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_TOKEN_PARSER_H_
