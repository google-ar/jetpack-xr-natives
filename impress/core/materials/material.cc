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

#include "core/materials/material.h"

#include <memory>
#include <optional>
#include <string>
#include <utility>

#include "absl/memory/memory.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "filament/libs/math/include/math/mathfwd.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_ptr.h"
#include "core/render/texture.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

namespace imp {

std::unique_ptr<Material> Material::WrapMaterial(
    BaseView& view, filament::MaterialInstance* material_instance) {
  return absl::WrapUnique(new Material(view, material_instance, {}));
}

Material::Material(BaseView& view,
                   filament::MaterialInstance* material_instance,
                   const AssetPtr<MaterialAsset>& material_asset)
    : view_(view),
      engine_(BaseView::GetSharedEngine()),
      material_instance_(material_instance),
      material_asset_(material_asset) {
  if (split_engine::SplitEngineSerializer* serializer =
          view_.GetSplitEngineSerializer()) {
    serializer->AddMaterialInstance(material_instance_->getMaterial(),
                                    material_instance_);
  }
  SetName(material_instance_->getName());
}

Material::~Material() {
  if (split_engine::SplitEngineSerializer* serializer =
          view_.GetSplitEngineSerializer()) {
    serializer->RemoveMaterialInstance(material_instance_);
  }
  if (engine_ && material_instance_) {
    engine_->destroy(material_instance_);
  }
  engine_ = nullptr;
  material_instance_ = nullptr;
}

filament::MaterialInstance* Material::GetFilamentMaterialInstance() {
  return material_instance_;
}

const std::string& Material::GetName() const { return name_; }

void Material::SetName(absl::string_view name) { name_ = std::string(name); }

void Material::SetParameter(absl::string_view parameter_name,
                            filament::RgbaType type,
                            filament::math::float4 color) {
  // TODO: add support for params with RgbaType.
  GetFilamentMaterialInstance()->setParameter(
      parameter_name.data(), parameter_name.size(), type, color);
}
void Material::SetParameter(absl::string_view parameter_name,
                            filament::RgbType type,
                            filament::math::float3 color) {
  // TODO: add support for params with RgbType.
  GetFilamentMaterialInstance()->setParameter(
      parameter_name.data(), parameter_name.size(), type, color);
}

void Material::SetTextureImpl(
    absl::string_view parameter_name, const Texture* texture,
    std::optional<filament::TextureSampler> sampler_override) {
  filament::TextureSampler sampler =
      sampler_override ? *sampler_override : texture->GetSampler();

  if (split_engine::SplitEngineSerializer* serializer =
          view_.GetSplitEngineSerializer()) {
    serializer->SetMaterialParameter(GetFilamentMaterialInstance(),
                                     parameter_name, texture->GetTexture(),
                                     sampler);
  }
  GetFilamentMaterialInstance()->setParameter(parameter_name.data(),
                                              parameter_name.size(),
                                              texture->GetTexture(), sampler);
}

void Material::SetParameter(
    absl::string_view parameter_name, imp::Texture* texture,
    std::optional<filament::TextureSampler> sampler_override) {
  SetTextureImpl(parameter_name, texture, sampler_override);
  parameters_to_raw_textures_.insert_or_assign(std::string(parameter_name),
                                               texture->GetTexture());
  parameters_to_owned_or_borrowed_textures_.erase(parameter_name);
}

void Material::SetParameter(
    absl::string_view parameter_name, const imp::Texture* texture,
    std::optional<filament::TextureSampler> sampler_override) {
  SetTextureImpl(parameter_name, texture, sampler_override);
  parameters_to_raw_textures_.insert_or_assign(std::string(parameter_name),
                                               texture->GetTexture());
  parameters_to_owned_or_borrowed_textures_.erase(parameter_name);
}

void Material::SetParameter(
    absl::string_view parameter_name, TexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  SetParameter(parameter_name, OwnedTexturePtr(std::move(texture)),
               sampler_override);
}

void Material::SetParameter(
    absl::string_view parameter_name, OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  SetTextureImpl(parameter_name, texture.operator->(), sampler_override);
  parameters_to_owned_or_borrowed_textures_.insert_or_assign(
      std::string(parameter_name),
      OwnedOrBorrowedPtr<Texture>(std::move(texture)));
  parameters_to_raw_textures_.erase(parameter_name);
}

void Material::SetParameter(
    absl::string_view parameter_name, BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  SetTextureImpl(parameter_name, texture.operator->(), sampler_override);
  parameters_to_owned_or_borrowed_textures_.insert_or_assign(
      std::string(parameter_name),
      OwnedOrBorrowedPtr<Texture>(std::move(texture)));
  parameters_to_raw_textures_.erase(parameter_name);
}

bool Material::HasParameter(absl::string_view name) {
  return GetFilamentMaterialInstance()->getMaterial()->hasParameter(
      std::string(name).c_str());
}

Material::HeldTextureType Material::GetAssignedTextureType(
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

imp::StringMap<const filament::Texture*> Material::GetUnownedFilamentTextures()
    const {
  imp::StringMap<const filament::Texture*> result;
  for (auto& pair : parameters_to_raw_textures_) {
    result.emplace(pair.first, pair.second);
  }
  return result;
}

}  // namespace imp
