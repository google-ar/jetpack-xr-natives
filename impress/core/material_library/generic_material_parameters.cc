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

#include "core/material_library/generic_material_parameters.h"

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/common/schemas/math_generated.h"
#include "core/material_library/schemas/generic_material_generated.h"

namespace imp {

namespace {

// This helper creates imp::schemas versions of a TextureSampler.
class GenericMaterialTextureSamplerCreator {
 public:
  using TextureSampler = schemas::TextureSampler;
  static constexpr auto CreateTextureSampler = schemas::CreateTextureSampler;

  using MinFilter = schemas::MinFilter;
  using MagFilter = schemas::MagFilter;
  using WrapMode = schemas::WrapMode;
  using CompareMode = schemas::CompareMode;
  using CompareFunc = schemas::CompareFunc;
};

// This helper creates imp::schemas versions of GenericMaterialParameters.
class GenericMaterialParametersSchemaCreator {
 public:
  using Schema = schemas::GenericMaterialParameters;
  static constexpr auto CreateGenericMaterialParameters =
      schemas::CreateGenericMaterialParameters;

  using GenericMaterialTextureParameter =
      schemas::GenericMaterialTextureParameter;
  static constexpr auto CreateGenericMaterialTextureParameter =
      schemas::CreateGenericMaterialTextureParameter;

  using GenericMaterialParametersBaseColor =
      schemas::GenericMaterialParametersBaseColor;
  static constexpr auto CreateGenericMaterialParametersBaseColor =
      schemas::CreateGenericMaterialParametersBaseColor;

  using GenericMaterialParametersMetallicRoughness =
      schemas::GenericMaterialParametersMetallicRoughness;
  static constexpr auto CreateGenericMaterialParametersMetallicRoughness =
      schemas::CreateGenericMaterialParametersMetallicRoughness;

  using GenericMaterialParametersNormal =
      schemas::GenericMaterialParametersNormal;
  static constexpr auto CreateGenericMaterialParametersNormal =
      schemas::CreateGenericMaterialParametersNormal;

  using GenericMaterialParametersAmbientOcclusion =
      schemas::GenericMaterialParametersAmbientOcclusion;
  static constexpr auto CreateGenericMaterialParametersAmbientOcclusion =
      schemas::CreateGenericMaterialParametersAmbientOcclusion;

  using GenericMaterialParametersEmissive =
      schemas::GenericMaterialParametersEmissive;
  static constexpr auto CreateGenericMaterialParametersEmissive =
      schemas::CreateGenericMaterialParametersEmissive;

  using GenericMaterialParametersClearcoat =
      schemas::GenericMaterialParametersClearcoat;
  static constexpr auto CreateGenericMaterialParametersClearcoat =
      schemas::CreateGenericMaterialParametersClearcoat;

  using GenericMaterialParametersSheen =
      schemas::GenericMaterialParametersSheen;
  static constexpr auto CreateGenericMaterialParametersSheen =
      schemas::CreateGenericMaterialParametersSheen;

  using GenericMaterialParametersTransmission =
      schemas::GenericMaterialParametersTransmission;
  static constexpr auto CreateGenericMaterialParametersTransmission =
      schemas::CreateGenericMaterialParametersTransmission;

  using GenericMaterialParametersRefraction =
      schemas::GenericMaterialParametersRefraction;
  static constexpr auto CreateGenericMaterialParametersRefraction =
      schemas::CreateGenericMaterialParametersRefraction;

  using GenericMaterialParametersMasking =
      schemas::GenericMaterialParametersMasking;
  static constexpr auto CreateGenericMaterialParametersMasking =
      schemas::CreateGenericMaterialParametersMasking;

  using TextureSamplerCreator = GenericMaterialTextureSamplerCreator;

  using Bool = schemas::Bool;
  using Float = schemas::Float;
  using Float2 = schemas::Float2;
  using Float3 = schemas::Float3;
  using Float4 = schemas::Float4;
  using Mat3f = schemas::Mat3f;
};

}  // namespace

flatbuffers::Offset<schemas::GenericMaterialParameters>
GenericMaterialParameters::ToFlatbuffer(
    flatbuffers::FlatBufferBuilder& builder) const {
  return ToFlatbufferT<GenericMaterialParametersSchemaCreator>(builder);
}

}  // namespace imp
