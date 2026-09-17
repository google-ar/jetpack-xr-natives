/*
 * Copyright 2026 Google LLC
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

#include "core/material_library/generic_material.h"

#include <optional>
#include <utility>

#include "absl/base/no_destructor.h"
#include "absl/container/flat_hash_set.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/material_library/generic_material_constants.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"

namespace imp {
namespace {
void LogUnsupportedParameter(absl::string_view name) {
  IMP_LOG(imp::ERROR) << "SetParameter: Unsupported parameter: " << name;
}

void LogParameterNotDirectlyAssignable(absl::string_view name) {
  IMP_LOG(imp::ERROR) << "SetParameter: Parameter is not directly assignable: " << name;
}

void LogTextureParameterNotDirectlyAssignable(absl::string_view name) {
  IMP_LOG(imp::ERROR)
      << "SetParameter: Texture parameter type is not directly assignable: "
      << name << " Use OwnedTexturePtr or BorrowedTexturePtr overloads.";
}

std::optional<absl::string_view> MapTextureParameterToChannel(
    absl::string_view parameter_name) {
  if (parameter_name == kBaseColorTexture) {
    return kBaseColorIndex;
  } else if (parameter_name == kMetallicRoughnessTexture) {
    return kMetallicRoughnessIndex;
  } else if (parameter_name == kNormalTexture) {
    return kNormalIndex;
  } else if (parameter_name == kAoTexture) {
    return kAoIndex;
  } else if (parameter_name == kEmissiveTexture) {
    return kEmissiveIndex;
  } else if (parameter_name == kClearcoatTexture) {
    return kClearcoatIndex;
  } else if (parameter_name == kClearcoatNormalTexture) {
    return kClearcoatNormalIndex;
  } else if (parameter_name == kClearcoatRoughnessTexture) {
    return kClearcoatRoughnessIndex;
  } else if (parameter_name == kSheenColorTexture) {
    return kSheenColorIndex;
  } else if (parameter_name == kSheenRoughnessTexture) {
    return kSheenRoughnessIndex;
  } else if (parameter_name == kTransmissionTexture) {
    return kTransmissionIndex;
  } else if (parameter_name == kThicknessTexture) {
    return kThicknessIndex;
  }
  return std::nullopt;
}

// Returns a singleton hash set tracking all parameters supported by
// GenericMaterial implementations. Always keep this set in sync with the string
// constants defined in generic_material_parameter_constants.h.
const absl::flat_hash_set<absl::string_view>& GetSupportedParameters() {
  // LINT.IfChange(supported_parameters)
  static const absl::NoDestructor<absl::flat_hash_set<absl::string_view>>
      kSupportedParameters({
          // Float parameters
          kMetallicFactor,
          kRoughnessFactor,
          kNormalScale,
          kAoStrength,
          kSheenRoughnessFactor,
          kTransmissionFactor,
          kIndexOfRefraction,
          kThicknessFactor,
          kAttenuationDistance,
          // Float3 / Float4 parameters
          kEmissiveFactor,
          kSheenColorFactor,
          kClearcoatRoughnessNormalFactors,
          kAttenuationColor,
          kBaseColorFactor,
          // Int parameters (internal binding indices/flags)
          kSamplersUvBitflags,
          kBaseColorIndex,
          kMetallicRoughnessIndex,
          kNormalIndex,
          kAoIndex,
          kEmissiveIndex,
          kClearcoatIndex,
          kClearcoatRoughnessIndex,
          kClearcoatNormalIndex,
          kSheenColorIndex,
          kSheenRoughnessIndex,
          kTransmissionIndex,
          kThicknessIndex,
          kEstimatedDepthTexture,
          kCameraTexture,
          kFeatureIdTexture0,
          kFeatureIdTexture1,
          kFeatureIdTexture2,
          kFeatureIdTexture3,
          // Texture parameters
          kBaseColorTexture,
          kMetallicRoughnessTexture,
          kNormalTexture,
          kAoTexture,
          kEmissiveTexture,
          kClearcoatTexture,
          kClearcoatNormalTexture,
          kClearcoatRoughnessTexture,
          kSheenColorTexture,
          kSheenRoughnessTexture,
          kTransmissionTexture,
          kThicknessTexture,
      });
  // LINT.ThenChange(//depot/google3/third_party/impress/core/material_library/generic_material_parameter_constants.h:generic_material_parameters)
  return *kSupportedParameters;
}
}  // namespace

bool GenericMaterial::HasParameter(absl::string_view name) {
  return GetSupportedParameters().contains(name);
}

void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   bool value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   bool2 value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   bool3 value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   bool4 value) {
  LogUnsupportedParameter(parameter_name);
}

void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   float value) {
  if (parameter_name == kMetallicFactor) {
    SetMetallicFactor(value);
  } else if (parameter_name == kRoughnessFactor) {
    SetRoughnessFactor(value);
  } else if (parameter_name == kNormalScale) {
    SetNormalScale(value);
  } else if (parameter_name == kSheenRoughnessFactor) {
    SetSheenRoughnessFactor(value);
  } else if (parameter_name == kIndexOfRefraction) {
    SetIndexOfRefraction(value);
  } else if (parameter_name == kThicknessFactor) {
    SetThicknessFactor(value);
  } else if (parameter_name == kAttenuationDistance) {
    SetAttenuationDistance(value);
  } else if (parameter_name == kTransmissionFactor) {
    SetTransmissionFactor(value);
  } else if (parameter_name == kAoStrength) {
    SetAmbientOcclusionStrength(value);
  } else {
    LogUnsupportedParameter(parameter_name);
  }
}

void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   float2 value) {
  LogUnsupportedParameter(parameter_name);
}

void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   float3 value) {
  if (parameter_name == kEmissiveFactor) {
    SetEmissiveFactor(value);
  } else if (parameter_name == kSheenColorFactor) {
    SetSheenColorFactor(value);
  } else if (parameter_name == kClearcoatRoughnessNormalFactors) {
    SetClearcoatFactors(value);
  } else if (parameter_name == kAttenuationColor) {
    SetAttenuationColor(value);
  } else {
    LogUnsupportedParameter(parameter_name);
  }
}

void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   float4 value) {
  if (parameter_name == kBaseColorFactor) {
    SetBaseColorFactor(value);
  } else {
    LogUnsupportedParameter(parameter_name);
  }
}

void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   int value) {
  if (parameter_name == kSamplersUvBitflags ||       //
      parameter_name == kBaseColorIndex ||           //
      parameter_name == kMetallicRoughnessIndex ||   //
      parameter_name == kNormalIndex ||              //
      parameter_name == kAoIndex ||                  //
      parameter_name == kEmissiveIndex ||            //
      parameter_name == kClearcoatIndex ||           //
      parameter_name == kClearcoatRoughnessIndex ||  //
      parameter_name == kClearcoatNormalIndex ||     //
      parameter_name == kSheenColorIndex ||          //
      parameter_name == kSheenRoughnessIndex ||      //
      parameter_name == kThicknessIndex ||           //
      parameter_name == kTransmissionIndex) {
    LogParameterNotDirectlyAssignable(parameter_name);
  } else {
    LogUnsupportedParameter(parameter_name);
  }
}

void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   int2 value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   int3 value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   int4 value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   uint value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   uint2 value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   uint3 value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   uint4 value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   mat3f value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   mat4f value) {
  LogUnsupportedParameter(parameter_name);
}

// Spans
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const bool> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const bool2> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const bool3> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const bool4> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const float> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const float2> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const float3> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const float4> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const int> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const int2> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const int3> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const int4> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const uint> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const uint2> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const uint3> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const uint4> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const mat3f> value) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   absl::Span<const mat4f> value) {
  LogUnsupportedParameter(parameter_name);
}

void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   filament::RgbaType type,
                                   filament::math::float4 color) {
  LogUnsupportedParameter(parameter_name);
}
void GenericMaterial::SetParameter(absl::string_view parameter_name,
                                   filament::RgbType type,
                                   filament::math::float3 color) {
  LogUnsupportedParameter(parameter_name);
}

void GenericMaterial::SetParameter(
    absl::string_view parameter_name, const imp::Texture* texture,
    std::optional<filament::TextureSampler> sampler_override) {
  LogTextureParameterNotDirectlyAssignable(parameter_name);
}
void GenericMaterial::SetParameter(
    absl::string_view parameter_name, TexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  LogTextureParameterNotDirectlyAssignable(parameter_name);
}
void GenericMaterial::SetParameter(
    absl::string_view parameter_name, OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  auto channel_name = MapTextureParameterToChannel(parameter_name);
  if (!channel_name) {
    LogUnsupportedParameter(parameter_name);
    return;
  }

  absl::Status status =
      SetTextureForChannel(*channel_name, std::move(texture), sampler_override);
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "SetParameter failed for " << parameter_name << ": "
               << status.message();
  }
}

void GenericMaterial::SetParameter(
    absl::string_view parameter_name, BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  auto channel_name = MapTextureParameterToChannel(parameter_name);
  if (!channel_name) {
    LogUnsupportedParameter(parameter_name);
    return;
  }

  absl::Status status =
      SetTextureForChannel(*channel_name, std::move(texture), sampler_override);
  if (!status.ok()) {
    IMP_LOG(imp::ERROR) << "SetParameter failed for " << parameter_name << ": "
               << status.message();
  }
}

absl::Status GenericMaterial::SetBaseColorTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kBaseColorIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetBaseColorTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kBaseColorIndex, texture, sampler_override);
}

absl::Status GenericMaterial::SetMetallicRoughnessTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kMetallicRoughnessIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetMetallicRoughnessTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kMetallicRoughnessIndex, texture,
                              sampler_override);
}

absl::Status GenericMaterial::SetNormalTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kNormalIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetNormalTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kNormalIndex, texture, sampler_override);
}

absl::Status GenericMaterial::SetAmbientOcclusionTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kAoIndex, std::move(texture), sampler_override);
}
absl::Status GenericMaterial::SetAmbientOcclusionTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kAoIndex, texture, sampler_override);
}

absl::Status GenericMaterial::SetEmissiveTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kEmissiveIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetEmissiveTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kEmissiveIndex, texture, sampler_override);
}

absl::Status GenericMaterial::SetClearcoatTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kClearcoatIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetClearcoatTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kClearcoatIndex, texture, sampler_override);
}

absl::Status GenericMaterial::SetClearcoatRoughnessTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kClearcoatRoughnessIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetClearcoatRoughnessTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kClearcoatRoughnessIndex, texture,
                              sampler_override);
}

absl::Status GenericMaterial::SetClearcoatNormalTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kClearcoatNormalIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetClearcoatNormalTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kClearcoatNormalIndex, texture, sampler_override);
}

absl::Status GenericMaterial::SetSheenColorTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kSheenColorIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetSheenColorTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kSheenColorIndex, texture, sampler_override);
}

absl::Status GenericMaterial::SetSheenRoughnessTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kSheenRoughnessIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetSheenRoughnessTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kSheenRoughnessIndex, texture, sampler_override);
}

absl::Status GenericMaterial::SetThicknessTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kThicknessIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetThicknessTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kThicknessIndex, texture, sampler_override);
}

absl::Status GenericMaterial::SetTransmissionTexture(
    OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kTransmissionIndex, std::move(texture),
                              sampler_override);
}
absl::Status GenericMaterial::SetTransmissionTexture(
    BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  return SetTextureForChannel(kTransmissionIndex, texture, sampler_override);
}

}  // namespace imp
