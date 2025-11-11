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

#include "core/split_engine/materials/split_engine_custom_material.h"

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/functional/function_ref.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/utils/string_map.h"

namespace imp::split_engine {

SplitEngineCustomMaterial::SplitEngineCustomMaterial(
    SplitEngineSerializer& serializer, OwnedMaterialPtr material)
    : serializer_(serializer), material_(std::move(material)) {}

SplitEngineCustomMaterial::~SplitEngineCustomMaterial() {
  serializer_.RemoveMaterialInstance(GetFilamentMaterialInstance());
}

filament::MaterialInstance*
SplitEngineCustomMaterial::GetFilamentMaterialInstance() {
  return material_->GetFilamentMaterialInstance();
}

const std::string& SplitEngineCustomMaterial::GetName() const {
  return material_->GetName();
}

void SplitEngineCustomMaterial::SetName(absl::string_view name) {
  material_->SetName(name);
}

void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             bool value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             bool2 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             bool3 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             bool4 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             float value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             float2 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             float3 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             float4 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             int value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             int2 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             int3 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             int4 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             uint value) {
  if (value > std::numeric_limits<int>::max()) {
    IMP_LOG(imp::ERROR) << "uint value " << value << " is out of range";
    return;
  }
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, static_cast<int>(value));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             uint2 value) {
  int2 cast_value;
  for (int i = 0; i < value.size(); ++i) {
    if (value[i] > std::numeric_limits<int>::max()) {
      IMP_LOG(imp::ERROR) << "uint value " << value[i] << " is out of range";
      return;
    }
    cast_value[i] = static_cast<int>(value[i]);
  }
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, cast_value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             uint3 value) {
  int3 cast_value;
  for (int i = 0; i < value.size(); ++i) {
    if (value[i] > std::numeric_limits<int>::max()) {
      IMP_LOG(imp::ERROR) << "uint value " << value[i] << " is out of range";
      return;
    }
    cast_value[i] = static_cast<int>(value[i]);
  }
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, cast_value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             uint4 value) {
  int4 cast_value;
  for (int i = 0; i < value.size(); ++i) {
    if (value[i] > std::numeric_limits<int>::max()) {
      IMP_LOG(imp::ERROR) << "uint value " << value[i] << " is out of range";
      return;
    }
    cast_value[i] = static_cast<int>(value[i]);
  }
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, cast_value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             mat3f value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             mat4f value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const bool> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const bool2> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const bool3> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const bool4> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const float> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const float2> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const float3> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const float4> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const int> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const int2> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const int3> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const int4> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const uint> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const uint2> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const uint3> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const uint4> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const mat3f> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const mat4f> value) {
  IMP_LOG(imp::ERROR) << "span parameters are not supported for " << parameter_name;
}

void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             filament::RgbaType type,
                                             filament::math::float4 color) {
  // TODO: (broken link) - Add support for parameters with RgbaType.
  IMP_LOG(imp::ERROR) << "RGBA parameters are not supported for " << parameter_name;
}

void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             filament::RgbType type,
                                             filament::math::float3 color) {
  // TODO: (broken link) - Add support for parameters with RgbaType.
  IMP_LOG(imp::ERROR) << "RGB parameters are not supported for " << parameter_name;
}

void SplitEngineCustomMaterial::SetParameter(
    absl::string_view parameter_name, const imp::Texture* texture,
    std::optional<filament::TextureSampler> sampler_override) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name, texture->GetTexture(),
      sampler_override ? *sampler_override : texture->GetSampler());

  parameters_to_raw_textures_.insert_or_assign(std::string(parameter_name),
                                               texture->GetTexture());
  parameters_to_owned_or_borrowed_textures_.erase(parameter_name);
}

void SplitEngineCustomMaterial::SetParameter(
    absl::string_view parameter_name, TexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  SetParameter(parameter_name, OwnedTexturePtr(std::move(texture)),
               sampler_override);
}

void SplitEngineCustomMaterial::SetParameter(
    absl::string_view parameter_name, OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name, texture->GetTexture(),
      sampler_override ? *sampler_override : texture->GetSampler());

  parameters_to_owned_or_borrowed_textures_.insert_or_assign(
      std::string(parameter_name),
      OwnedOrBorrowedPtr<Texture>(std::move(texture)));
  parameters_to_raw_textures_.erase(parameter_name);
}

void SplitEngineCustomMaterial::SetParameter(
    absl::string_view parameter_name, BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name, texture->GetTexture(),
      sampler_override ? *sampler_override : texture->GetSampler());

  parameters_to_owned_or_borrowed_textures_.insert_or_assign(
      std::string(parameter_name),
      OwnedOrBorrowedPtr<Texture>(std::move(texture)));
  parameters_to_raw_textures_.erase(parameter_name);
}

bool SplitEngineCustomMaterial::HasParameter(absl::string_view parameter_name) {
#if IMP_USE_LOCAL_SPLIT_ENGINE_MATERIALS
  return material_->HasParameter(parameter_name);
#endif
  // We don't know if the parameter exists because the material is held on the
  // SplitEngine renderer side.
  // NOTE: This means that TrySetParameter will behave the same as SetParameter
  // and crash the app if the parameter does not exist.
  return true;
}

SplitEngineCustomMaterial::HeldTextureType
SplitEngineCustomMaterial::GetAssignedTextureType(
    absl::string_view parameter_name) {
  auto itr = parameters_to_raw_textures_.find(parameter_name);
  if (itr != parameters_to_raw_textures_.end()) {
    return HeldTextureType::kRawPointer;
  }

  auto owned_or_borrowed_itr =
      parameters_to_owned_or_borrowed_textures_.find(parameter_name);
  if (owned_or_borrowed_itr !=
      parameters_to_owned_or_borrowed_textures_.end()) {
    return owned_or_borrowed_itr.value().IsOwned()
               ? HeldTextureType::kOwnedPointer
               : HeldTextureType::kBorrowedPointer;
  }

  return HeldTextureType::kNone;
}

imp::StringMap<const filament::Texture*>
SplitEngineCustomMaterial::GetUnownedFilamentTextures() const {
  imp::StringMap<const filament::Texture*> result;
  for (auto& pair : parameters_to_raw_textures_) {
    result.emplace(pair.first, pair.second);
  }
  return result;
}

void SplitEngineCustomMaterial::ForEachTexture(
    absl::FunctionRef<void(BorrowedTexturePtr)> fn, SmallSourceLocation loc) {
  for (auto& [_, texture] : parameters_to_owned_or_borrowed_textures_) {
    if (texture) {
      fn(texture.Borrow(loc));
    }
  }
}

}  // namespace imp::split_engine
