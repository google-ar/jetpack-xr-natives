// Copyright 2025 Google LLC
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

#include "core/assets/gltf/object_model/pointer_declarations/materials.h"

#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "core/assets/gltf/object_model/property_pointer.h"
#include "core/assets/gltf/object_model/token_parser.h"
#include "core/material_library/generic_material.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/ncsb/component_handle.h"
#include "core/ncsb/node_handle.h"
#include "core/view/framework/assets/gltf_renderer.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::gltf {

namespace {

absl::StatusOr<GenericMaterial*> GetMaterial(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) {
  if (!std::holds_alternative<int>(parsed_tokens[1])) {
    return absl::InvalidArgumentError("Material index must be an int.");
  }

  int material_index = std::get<int>(parsed_tokens[1]);

  return gltf_model->GetComponent<GltfRenderer>()->GetGenericMaterialByIndex(
      material_index);
}

}  // namespace

std::vector<TokenParser>
MaterialsAlphaCutoffPointerDeclaration::GetTokenParsers() const {
  return {"materials", GetIntTokenParser(), "alphaCutoff"};
}

absl::StatusOr<PropertyPointer::PointerValue>
MaterialsAlphaCutoffPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  return material->GetAlphaCutoff();
}

absl::Status MaterialsAlphaCutoffPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  if (!std::holds_alternative<float>(value)) {
    return absl::InvalidArgumentError("Alpha cutoff must be a float.");
  }

  material->SetAlphaCutoff(std::get<float>(value));

  return absl::OkStatus();
}

std::vector<TokenParser>
MaterialsEmissiveFactorPointerDeclaration::GetTokenParsers() const {
  return {"materials", GetIntTokenParser(), "emissiveFactor"};
}

absl::StatusOr<PropertyPointer::PointerValue>
MaterialsEmissiveFactorPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  return material->GetEmissiveFactor();
}

absl::Status MaterialsEmissiveFactorPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  if (!std::holds_alternative<float3>(value)) {
    return absl::InvalidArgumentError("Emissive factor must be a float3.");
  }

  material->SetEmissiveFactor(std::get<float3>(value));

  return absl::OkStatus();
}

std::vector<TokenParser>
MaterialsNormalTextureScalePointerDeclaration::GetTokenParsers() const {
  return {"materials", GetIntTokenParser(), "normalTexture", "scale"};
}

absl::StatusOr<PropertyPointer::PointerValue>
MaterialsNormalTextureScalePointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  return material->GetNormalScale();
}

absl::Status MaterialsNormalTextureScalePointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  if (!std::holds_alternative<float>(value)) {
    return absl::InvalidArgumentError("Normal scale must be a float.");
  }

  material->SetNormalScale(std::get<float>(value));

  return absl::OkStatus();
}

std::vector<TokenParser>
MaterialsOcclusionTextureStrengthPointerDeclaration::GetTokenParsers() const {
  return {"materials", GetIntTokenParser(), "occlusionTexture", "strength"};
}

absl::StatusOr<PropertyPointer::PointerValue>
MaterialsOcclusionTextureStrengthPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  return material->GetAmbientOcclusionStrength();
}

absl::Status MaterialsOcclusionTextureStrengthPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  if (!std::holds_alternative<float>(value)) {
    return absl::InvalidArgumentError(
        "Ambient occlusion strength must be a float.");
  }

  material->SetAmbientOcclusionStrength(std::get<float>(value));
  return absl::OkStatus();
}

std::vector<TokenParser>
MaterialsPbrMetallicRoughnessBaseColorFactorPointerDeclaration::
    GetTokenParsers() const {
  return {"materials", GetIntTokenParser(), "pbrMetallicRoughness",
          "baseColorFactor"};
}

absl::StatusOr<PropertyPointer::PointerValue>
MaterialsPbrMetallicRoughnessBaseColorFactorPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  return material->GetBaseColorFactor();
}

absl::Status
MaterialsPbrMetallicRoughnessBaseColorFactorPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  if (!std::holds_alternative<float4>(value)) {
    return absl::InvalidArgumentError("Base color factor must be a float4.");
  }

  material->SetBaseColorFactor(std::get<float4>(value));

  return absl::OkStatus();
}

std::vector<TokenParser>
MaterialsPbrMetallicRoughnessMetallicFactorPointerDeclaration::GetTokenParsers()
    const {
  return {"materials", GetIntTokenParser(), "pbrMetallicRoughness",
          "metallicFactor"};
}

absl::StatusOr<PropertyPointer::PointerValue>
MaterialsPbrMetallicRoughnessMetallicFactorPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  return material->GetMetallicFactor();
}

absl::Status
MaterialsPbrMetallicRoughnessMetallicFactorPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  if (!std::holds_alternative<float>(value)) {
    return absl::InvalidArgumentError("Metallic factor must be a float.");
  }

  material->SetMetallicFactor(std::get<float>(value));

  return absl::OkStatus();
}

std::vector<TokenParser>
MaterialsPbrMetallicRoughnessRoughnessFactorPointerDeclaration::
    GetTokenParsers() const {
  return {"materials", GetIntTokenParser(), "pbrMetallicRoughness",
          "roughnessFactor"};
}

absl::StatusOr<PropertyPointer::PointerValue>
MaterialsPbrMetallicRoughnessRoughnessFactorPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  return material->GetRoughnessFactor();
}

absl::Status
MaterialsPbrMetallicRoughnessRoughnessFactorPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  if (!std::holds_alternative<float>(value)) {
    return absl::InvalidArgumentError("Roughness factor must be a float.");
  }

  material->SetRoughnessFactor(std::get<float>(value));

  return absl::OkStatus();
}

std::vector<TokenParser> MaterialsLengthPointerDeclaration::GetTokenParsers()
    const {
  return {"materials.length"};
}

absl::StatusOr<PropertyPointer::PointerValue>
MaterialsLengthPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  ComponentHandle<GltfRenderer> gltf_renderer =
      gltf_model->GetComponent<GltfRenderer>();
  if (!gltf_renderer) {
    return absl::InvalidArgumentError("GltfRenderer is not found.");
  }

  return static_cast<int>(gltf_renderer->GetMaterials().size());
}

absl::Status MaterialsLengthPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  return absl::FailedPreconditionError("materials.length is readonly.");
}

std::vector<TokenParser>
MaterialsDoubleSidedPointerDeclaration::GetTokenParsers() const {
  return {"materials", GetIntTokenParser(), "doubleSided"};
}

absl::StatusOr<PropertyPointer::PointerValue>
MaterialsDoubleSidedPointerDeclaration::GetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens) const {
  MP_ASSIGN_OR_RETURN(GenericMaterial * material,
                   GetMaterial(gltf_model, parsed_tokens));
  return material->GetMaterial()
      ->GetFilamentMaterialInstance()
      ->isDoubleSided();
}

absl::Status MaterialsDoubleSidedPointerDeclaration::SetValue(
    NodeHandle gltf_model, absl::Span<const ParsedToken> parsed_tokens,
    PropertyPointer::PointerValue value) const {
  return absl::FailedPreconditionError(
      "/materials/{}/doubleSided is readonly.");
}

}  // namespace imp::gltf
