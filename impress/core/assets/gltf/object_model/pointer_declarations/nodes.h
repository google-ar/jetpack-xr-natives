/*
 * Copyright 2025 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_DECLARATIONS_NODES_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_DECLARATIONS_NODES_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/assets/gltf/object_model/property_pointer.h"
#include "core/assets/gltf/object_model/token_parser.h"
#include "core/ncsb/node_handle.h"

namespace imp::gltf {

class NodesTranslationPointerDeclaration
    : public PropertyPointer::PointerDeclaration {
 public:
  std::vector<TokenParser> GetTokenParsers() const override;

  absl::StatusOr<PropertyPointer::PointerValue> GetValue(
      NodeHandle gltf_model,
      absl::Span<const ParsedToken> parsed_tokens) const override;

  absl::Status SetValue(NodeHandle gltf_model,
                        absl::Span<const ParsedToken> parsed_tokens,
                        PropertyPointer::PointerValue value) const override;
};

class NodesRotationPointerDeclaration
    : public PropertyPointer::PointerDeclaration {
 public:
  std::vector<TokenParser> GetTokenParsers() const override;

  absl::StatusOr<PropertyPointer::PointerValue> GetValue(
      NodeHandle gltf_model,
      absl::Span<const ParsedToken> parsed_tokens) const override;

  absl::Status SetValue(NodeHandle gltf_model,
                        absl::Span<const ParsedToken> parsed_tokens,
                        PropertyPointer::PointerValue value) const override;
};

class NodesScalePointerDeclaration
    : public PropertyPointer::PointerDeclaration {
 public:
  std::vector<TokenParser> GetTokenParsers() const override;

  absl::StatusOr<PropertyPointer::PointerValue> GetValue(
      NodeHandle gltf_model,
      absl::Span<const ParsedToken> parsed_tokens) const override;

  absl::Status SetValue(NodeHandle gltf_model,
                        absl::Span<const ParsedToken> parsed_tokens,
                        PropertyPointer::PointerValue value) const override;
};

class NodesMatrixPointerDeclaration
    : public PropertyPointer::PointerDeclaration {
 public:
  std::vector<TokenParser> GetTokenParsers() const override;

  absl::StatusOr<PropertyPointer::PointerValue> GetValue(
      NodeHandle gltf_model,
      absl::Span<const ParsedToken> parsed_tokens) const override;

  absl::Status SetValue(NodeHandle gltf_model,
                        absl::Span<const ParsedToken> parsed_tokens,
                        PropertyPointer::PointerValue value) const override;
};

class NodesGlobalMatrixPointerDeclaration
    : public PropertyPointer::PointerDeclaration {
 public:
  std::vector<TokenParser> GetTokenParsers() const override;

  absl::StatusOr<PropertyPointer::PointerValue> GetValue(
      NodeHandle gltf_model,
      absl::Span<const ParsedToken> parsed_tokens) const override;

  absl::Status SetValue(NodeHandle gltf_model,
                        absl::Span<const ParsedToken> parsed_tokens,
                        PropertyPointer::PointerValue value) const override;
};

class NodesLengthPointerDeclaration
    : public PropertyPointer::PointerDeclaration {
 public:
  std::vector<TokenParser> GetTokenParsers() const override;

  absl::StatusOr<PropertyPointer::PointerValue> GetValue(
      NodeHandle gltf_model,
      absl::Span<const ParsedToken> parsed_tokens) const override;

  absl::Status SetValue(NodeHandle gltf_model,
                        absl::Span<const ParsedToken> parsed_tokens,
                        PropertyPointer::PointerValue value) const override;
};

class NodesChildrenLengthPointerDeclaration
    : public PropertyPointer::PointerDeclaration {
 public:
  std::vector<TokenParser> GetTokenParsers() const override;

  absl::StatusOr<PropertyPointer::PointerValue> GetValue(
      NodeHandle gltf_model,
      absl::Span<const ParsedToken> parsed_tokens) const override;

  absl::Status SetValue(NodeHandle gltf_model,
                        absl::Span<const ParsedToken> parsed_tokens,
                        PropertyPointer::PointerValue value) const override;
};

class NodesParentPointerDeclaration
    : public PropertyPointer::PointerDeclaration {
 public:
  std::vector<TokenParser> GetTokenParsers() const override;

  absl::StatusOr<PropertyPointer::PointerValue> GetValue(
      NodeHandle gltf_model,
      absl::Span<const ParsedToken> parsed_tokens) const override;

  absl::Status SetValue(NodeHandle gltf_model,
                        absl::Span<const ParsedToken> parsed_tokens,
                        PropertyPointer::PointerValue value) const override;
};

}  // namespace imp::gltf

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_DECLARATIONS_NODES_H_
