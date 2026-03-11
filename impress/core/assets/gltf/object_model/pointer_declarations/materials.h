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

#ifndef THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_DECLARATIONS_MATERIALS_H_
#define THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_DECLARATIONS_MATERIALS_H_

#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/types/span.h"
#include "core/assets/gltf/object_model/property_pointer.h"
#include "core/assets/gltf/object_model/token_parser.h"
#include "core/ncsb/node_handle.h"

namespace imp::gltf {

// Pointer declaration for the alpha cutoff property of a material.
// Pointer path: materials/{int}/alphaCutoff
class MaterialsAlphaCutoffPointerDeclaration
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

// Pointer declaration for the emissive factor property of a material.
// Pointer path: materials/{int}/emissiveFactor
class MaterialsEmissiveFactorPointerDeclaration
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

// Pointer declaration for the normal texture scale property of a material.
// Pointer path: materials/{int}/normalTexture/scale
class MaterialsNormalTextureScalePointerDeclaration
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

// Pointer declaration for the ambient occlusion texture strength property of a
// material.
// Pointer path: materials/{int}/occlusionTexture/strength
class MaterialsOcclusionTextureStrengthPointerDeclaration
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

// Pointer declaration for the base color factor property of a material.
// Pointer path: materials/{int}/pbrMetallicRoughness/baseColorFactor
class MaterialsPbrMetallicRoughnessBaseColorFactorPointerDeclaration
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

// Pointer declaration for the metallic factor property of a material.
// Pointer path: materials/{int}/pbrMetallicRoughness/metallicFactor
class MaterialsPbrMetallicRoughnessMetallicFactorPointerDeclaration
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

// Pointer declaration for the roughness factor property of a material.
// Pointer path: materials/{int}/pbrMetallicRoughness/roughnessFactor
class MaterialsPbrMetallicRoughnessRoughnessFactorPointerDeclaration
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

// Pointer declaration for the length of the materials array.
// Pointer path: materials.length
class MaterialsLengthPointerDeclaration
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

// Pointer declaration for whether the material is double sided.
// Pointer path: materials/{int}/doubleSided
class MaterialsDoubleSidedPointerDeclaration
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

#endif  // THIRD_PARTY_IMPRESS_CORE_ASSETS_GLTF_OBJECT_MODEL_POINTER_DECLARATIONS_MATERIALS_H_
