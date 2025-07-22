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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_PARSER_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_PARSER_H_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/container/flat_hash_map.h"
#include "absl/strings/string_view.h"
#include "core/assets/gltf/object_model/property_pointer.h"
#include "core/assets/gltf/object_model/token_parser.h"

namespace imp::gltf {

// PointerParser is a class that parses pointer paths into PropertyPointers.
//
// A pointer path is a string that contains a series of tokens separated by
// forward slashes. For example, "nodes/2/translation".
//
// PointerParser keeps a search tree of TokenParsers to find matching
// PointerDeclarations for a given pointer path.
//
// For more information on pointer paths, see:
// https://github.com/KhronosGroup/glTF/blob/main/specification/2.0/ObjectModel.adoc
//
// TODO: Cache parsed pointers to avoid duplicate work.
class PointerParser {
 public:
  // Registers a pointer declaration.
  //
  // If a pointer declaration with the same token parsers already exists, the
  // existing declaration will be overridden.
  void RegisterPointerDeclaration(
      std::unique_ptr<PropertyPointer::PointerDeclaration> declaration);

  // Registers a vector of pointer declarations.
  void RegisterPointerDeclarations(
      std::vector<std::unique_ptr<PropertyPointer::PointerDeclaration>>
          declarations);

  // Tries to parse a pointer path into a PropertyPointer.
  // Returns std::nullopt if the pointer path cannot be parsed.
  std::optional<PropertyPointer> TryParse(absl::string_view path) const;

 private:
  // ParserNode is a node in the parser tree.
  // Each node contains a map of token parsers to child nodes.
  // The node also contains a pointer declaration to indicate the node can be a
  // valid end of a pointer path.
  struct ParserNode {
    absl::flat_hash_map<TokenParser, std::unique_ptr<ParserNode>> children;

    std::unique_ptr<PropertyPointer::PointerDeclaration> pointer_declaration;
  };

  // Internal implementation of TryParse.
  // The function tries to search recursively for a valid pointer declaration in
  // the parser tree.
  const PropertyPointer::PointerDeclaration* TryParse(
      std::vector<std::string>& tokens, int token_index,
      const ParserNode* current_node,
      std::vector<ParsedToken>& parsed_token) const;

  // The root node of the parser tree.
  ParserNode root_node_;
};

}  // namespace imp::gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_PARSER_H_
