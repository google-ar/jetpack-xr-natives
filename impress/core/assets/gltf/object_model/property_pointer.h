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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_PROPERTY_POINTER_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_PROPERTY_POINTER_H_

#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/assets/gltf/object_model/token_parser.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/ncsb/node_handle.h"

namespace imp::gltf {

// PropertyPointer represents a pointer to a property in a gltf model.
//
// Essentially, a PropertyPointer is a wrapper around a set of functions that
// retrieves and sets values of gltf properties.
//
// For more information on pointers, see:
// https://github.com/KhronosGroup/glTF/blob/main/specification/2.0/ObjectModel.adoc
class PropertyPointer {
 public:
  // PointerValue is a value of a gltf property.
  using PointerValue = std::variant<int, float, float2, float3, float4, mat4f>;

  // PointerDeclaration contains the information needed to parse a pointer path
  // and construct a PropertyPointer.
  class PointerDeclaration {
   public:
    virtual ~PointerDeclaration() = default;

    // Returns the token parsers that are required to parse the pointer path.
    virtual std::vector<TokenParser> GetTokenParsers() const = 0;

    // Gets the value of the gltf property pointed to by this PropertyPointer.
    virtual absl::StatusOr<PointerValue> GetValue(
        NodeHandle gltf_model,
        absl::Span<const ParsedToken> parsed_tokens) const = 0;

    // Sets the value of the gltf property pointed to by this PropertyPointer.
    virtual absl::Status SetValue(NodeHandle gltf_model,
                                  absl::Span<const ParsedToken> parsed_tokens,
                                  PointerValue value) const = 0;
  };

  // Constructs a PropertyPointer with the given value retriever and setter.
  // Retriever and setter are optional. If not provided, the default
  // implementation will return an unimplemented error.
  PropertyPointer(const PointerDeclaration* pointer_declaration,
                  const std::vector<ParsedToken>& parsed_tokens);

  // Gets the value of the gltf property pointed to by this PropertyPointer.
  absl::StatusOr<PointerValue> GetValue(NodeHandle gltf_model) const;

  // Sets the value of the gltf property pointed to by this PropertyPointer.
  absl::Status SetValue(NodeHandle gltf_model, PointerValue value) const;

 private:
  const PointerDeclaration* pointer_declaration_;
  std::vector<ParsedToken> parsed_tokens_;
};

}  // namespace imp::gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_PROPERTY_POINTER_H_
