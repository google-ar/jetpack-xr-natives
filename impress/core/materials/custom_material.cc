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

#include "core/materials/custom_material.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/functional/function_ref.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/libs/math/include/math/mathfwd.h"
#include "core/assets/asset_ptr.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_ptr.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

namespace imp {

CustomMaterial::CustomMaterial(filament::MaterialInstance* material_instance,
                               AssetPtr<MaterialAsset> material_asset)
    : engine_(BaseView::GetSharedEngine()),
      material_instance_(material_instance),
      material_asset_(material_asset) {
  SetName(material_instance_->getName());
}

CustomMaterial::~CustomMaterial() {
  if (engine_ && material_instance_) {
    engine_->destroy(material_instance_);
  }
  engine_ = nullptr;
  material_instance_ = nullptr;
}

filament::MaterialInstance* CustomMaterial::GetFilamentMaterialInstance() {
  return material_instance_;
}

void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  bool value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  bool2 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  bool3 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  bool4 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  float value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  float2 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  float3 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  float4 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name, int value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  int2 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  int3 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  int4 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  uint value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  uint2 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  uint3 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  uint4 value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  mat3f value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  mat4f value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value);
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const bool> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const bool2> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const bool3> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const bool4> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const float> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const float2> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const float3> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const float4> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const int> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const int2> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const int3> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const int4> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const uint> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const uint2> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const uint3> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const uint4> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const mat3f> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}
void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  absl::Span<const mat4f> value) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   value.data(), value.size());
}

void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  filament::RgbaType type,
                                  filament::math::float4 color) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   type, color);
}

void CustomMaterial::SetParameter(absl::string_view parameter_name,
                                  filament::RgbType type,
                                  filament::math::float3 color) {
  material_instance_->setParameter(parameter_name.data(), parameter_name.size(),
                                   type, color);
}

void CustomMaterial::SetParameter(
    absl::string_view parameter_name, const imp::Texture* texture,
    std::optional<filament::TextureSampler> sampler_override) {
  material_instance_->setParameter(
      parameter_name.data(), parameter_name.size(), texture->GetTexture(),
      sampler_override.value_or(texture->GetSampler()));

  parameters_to_raw_textures_.insert_or_assign(std::string(parameter_name),
                                               texture->GetTexture());
  parameters_to_owned_or_borrowed_textures_.erase(parameter_name);
}

void CustomMaterial::SetParameter(
    absl::string_view parameter_name, TexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  SetParameter(parameter_name, OwnedTexturePtr(std::move(texture)),
               sampler_override);
}

void CustomMaterial::SetParameter(
    absl::string_view parameter_name, OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  material_instance_->setParameter(
      parameter_name.data(), parameter_name.size(), texture->GetTexture(),
      sampler_override.value_or(texture->GetSampler()));

  parameters_to_owned_or_borrowed_textures_.insert_or_assign(
      std::string(parameter_name),
      OwnedOrBorrowedPtr<Texture>(std::move(texture)));
  parameters_to_raw_textures_.erase(parameter_name);
}

void CustomMaterial::SetParameter(
    absl::string_view parameter_name, BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  material_instance_->setParameter(
      parameter_name.data(), parameter_name.size(), texture->GetTexture(),
      sampler_override.value_or(texture->GetSampler()));

  parameters_to_owned_or_borrowed_textures_.insert_or_assign(
      std::string(parameter_name),
      OwnedOrBorrowedPtr<Texture>(std::move(texture)));
  parameters_to_raw_textures_.erase(parameter_name);
}

bool CustomMaterial::HasParameter(absl::string_view name) {
  return GetFilamentMaterialInstance()->getMaterial()->hasParameter(
      std::string(name).c_str());
}

CustomMaterial::HeldTextureType CustomMaterial::GetAssignedTextureType(
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
CustomMaterial::GetUnownedFilamentTextures() const {
  imp::StringMap<const filament::Texture*> result;
  for (auto& pair : parameters_to_raw_textures_) {
    result.emplace(pair.first, pair.second);
  }
  return result;
}

void CustomMaterial::ForEachTexture(
    absl::FunctionRef<void(BorrowedTexturePtr)> fn, SmallSourceLocation loc) {
  for (auto& [_, texture] : parameters_to_owned_or_borrowed_textures_) {
    if (texture) {
      fn(texture.Borrow(loc));
    }
  }
}

}  // namespace imp
