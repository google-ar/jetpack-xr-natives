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

#include "core/assets/gltf/object_model/pointer_parser.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "core/common/log.h"
#include "absl/strings/str_split.h"
#include "absl/strings/string_view.h"
#include "core/assets/gltf/object_model/property_pointer.h"
#include "core/assets/gltf/object_model/token_parser.h"

namespace imp::gltf {

using PointerValue = PropertyPointer::PointerValue;
using PointerDeclaration = PropertyPointer::PointerDeclaration;

void PointerParser::RegisterPointerDeclaration(
    std::unique_ptr<PointerDeclaration> pointer_declaration) {
  ParserNode* current_node = &root_node_;

  for (const TokenParser& token_parser :
       pointer_declaration->GetTokenParsers()) {
    auto it = current_node->children.find(token_parser);
    if (it != current_node->children.end()) {
      // The token parser already exists, so we can continue to the next node.
      current_node = it->second.get();
    } else {
      // Create a new node for the token parser if it doesn't exist.
      current_node->children[token_parser] = std::make_unique<ParserNode>();
      current_node = current_node->children[token_parser].get();
    }
  }

  current_node->pointer_declaration = std::move(pointer_declaration);
}

void PointerParser::RegisterPointerDeclarations(
    std::vector<std::unique_ptr<PointerDeclaration>> declarations) {
  for (std::unique_ptr<PointerDeclaration>& declaration : declarations) {
    RegisterPointerDeclaration(std::move(declaration));
  }
}

const PointerDeclaration* PointerParser::TryParse(
    std::vector<std::string>& tokens, int token_index,
    const ParserNode* current_node,
    std::vector<ParsedToken>& parsed_tokens) const {
  if (token_index > tokens.size() || token_index < 0) {
    IMP_LOG(imp::FATAL) << "Bad token index (" << token_index
               << ") provided to PointerParser::TryParse.";
  }

  if (token_index == tokens.size()) {
    // We have reached the end of the tokens.
    return current_node->pointer_declaration.get();
  }

  absl::string_view token = tokens[token_index];

  // Check if there's any matching fixed token children.
  auto fixed_token_it = current_node->children.find(std::string(token));
  if (fixed_token_it != current_node->children.end()) {
    parsed_tokens.push_back(std::monostate());

    const PointerDeclaration* result = TryParse(
        tokens, token_index + 1, fixed_token_it->second.get(), parsed_tokens);
    if (result) {
      return result;
    }

    parsed_tokens.pop_back();
  } else {
    // Iterate through all token parsers to see if any variable token parser can
    // parse the token.
    for (const auto& [token_parser, child_node] : current_node->children) {
      // If the token parser is a variable token parser, try to parse it.
      if (std::holds_alternative<TokenParserFn>(token_parser)) {
        std::optional<ParsedToken> parsed_token =
            std::get<TokenParserFn>(token_parser)(token);
        if (!parsed_token.has_value()) {
          continue;
        }

        parsed_tokens.push_back(parsed_token.value());

        const PointerDeclaration* result =
            TryParse(tokens, token_index + 1, child_node.get(), parsed_tokens);
        if (result) {
          return result;
        }

        parsed_tokens.pop_back();
      }
    }
  }

  return nullptr;
}

std::optional<PropertyPointer> PointerParser::TryParse(
    absl::string_view path) const {
  // Splits the path into individual tokens.
  std::vector<std::string> tokens = absl::StrSplit(path, '/');

  // Since pointer paths must start with a slash, the path must have at
  // least 2 tokens and the first token must be empty.
  if (tokens.size() < 2 || !tokens[0].empty()) {
    return std::nullopt;
  }

  std::vector<ParsedToken> parsed_tokens;
  // Starting from 1 to skip the first prefix "/".
  const PointerDeclaration* result =
      TryParse(tokens, 1, &root_node_, parsed_tokens);
  if (result) {
    return PropertyPointer(result, parsed_tokens);
  }

  return std::nullopt;
}

}  // namespace imp::gltf
