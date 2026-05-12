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

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "absl/functional/function_ref.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/assets/asset_ptr.h"
#include "core/assets/material/material_asset.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/split_engine/android/split_engine_android_bridge.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/placeholder_material_asset.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/framework/assets/asset_manager.h"
#include "core/view/framework/assets/material_factory.h"
#include "core/view/utils/string_map.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

Future<absl::Status> SplitEngineCustomMaterial::RequestCustomFilamentMaterial(
    BaseView& view, absl::string_view material_source,
    filament::Material* filament_material,
    const MaterialPreCompileOptions& precompile_options) {
  const uint64_t material_id = SplitEngineSerializer::GetId(filament_material);

  flatbuffers::FlatBufferBuilder fbb;
  flatbuffers::Offset<android_xr::schemas::FilamentMaterialSpec>
      material_spec_offset = android_xr::schemas::CreateFilamentMaterialSpec(
          fbb, fbb.CreateString(material_source),
          Pack(fbb, precompile_options));
  flatbuffers::Offset<android_xr::schemas::AddCustomMaterialRequest> request =
      android_xr::schemas::CreateAddCustomMaterialRequest(
          fbb, material_id,
          android_xr::schemas::CustomMaterialSpec::FilamentMaterialSpec,
          material_spec_offset.Union());

  return SendRequest<android_xr::schemas::AddCustomMaterialRequest,
                     absl::Status>(view.GetSplitEngineSerializer()->GetBridge(),
                                   fbb, request);
}

SplitEngineCustomMaterial::SplitEngineCustomMaterial(
    SplitEngineSerializer& serializer, OwnedMaterialPtr material)
    : serializer_(serializer), material_(std::move(material)) {}

SplitEngineCustomMaterial::~SplitEngineCustomMaterial() {
  serializer_.RemoveMaterialInstance(GetFilamentMaterialInstance());
}

const filament::MaterialInstance*
SplitEngineCustomMaterial::GetFilamentMaterialInstance() const {
  return material_->GetFilamentMaterialInstance();
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
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             uint2 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             uint3 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             uint4 value) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name, value);
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
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<bool>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const bool2> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<bool2>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const bool3> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<bool3>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const bool4> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<bool4>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const float> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<float>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const float2> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<float2>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const float3> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<float3>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const float4> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<float4>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const int> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<int>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const int2> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<int2>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const int3> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<int3>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const int4> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<int4>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const uint> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<uint>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const uint2> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<uint2>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const uint3> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<uint3>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const uint4> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<uint4>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const mat3f> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<mat3f>(value.begin(), value.end()));
}
void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             absl::Span<const mat4f> value) {
  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name,
      std::vector<mat4f>(value.begin(), value.end()));
}

void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             filament::RgbaType type,
                                             filament::math::float4 color) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name,
                                   filament::Color::toLinear(type, color));
}

void SplitEngineCustomMaterial::SetParameter(absl::string_view parameter_name,
                                             filament::RgbType type,
                                             filament::math::float3 color) {
  serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                   parameter_name,
                                   filament::Color::toLinear(type, color));
}

void SplitEngineCustomMaterial::SetParameter(
    absl::string_view parameter_name, const imp::Texture* texture,
    std::optional<filament::TextureSampler> sampler_override) {
  // Unassign a previously assigned owned or borrowed texture if it exists.
  auto it = parameters_to_owned_or_borrowed_textures_.find(parameter_name);
  if (it != parameters_to_owned_or_borrowed_textures_.end()) {
    it->second->OnUnassignedFromMaterial(*this, parameter_name);
  }

  serializer_.SetMaterialParameter(
      GetFilamentMaterialInstance(), parameter_name, texture->GetTexture(),
      sampler_override ? *sampler_override : texture->GetSampler());

  parameters_to_raw_textures_.insert_or_assign(std::string(parameter_name),
                                               texture);
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
  SetOwnedOrBorrowedTexture(parameter_name, std::move(texture),
                            sampler_override);
}

void SplitEngineCustomMaterial::SetParameter(
    absl::string_view parameter_name, BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  SetOwnedOrBorrowedTexture(parameter_name, std::move(texture),
                            sampler_override);
}

void SplitEngineCustomMaterial::SetOwnedOrBorrowedTexture(
    absl::string_view parameter_name, OwnedOrBorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  // Unassign a previously assigned owned or borrowed texture if it exists.
  auto it = parameters_to_owned_or_borrowed_textures_.find(parameter_name);
  if (it != parameters_to_owned_or_borrowed_textures_.end()) {
    it->second->OnUnassignedFromMaterial(*this, parameter_name);
  }

  filament::TextureSampler sampler =
      sampler_override ? *sampler_override : texture->GetSampler();

  Texture::UpdateTextureFn update_texture_fn =
      [this, sampler](absl::string_view parameter_name,
                      filament::Texture* texture) {
        serializer_.SetMaterialParameter(GetFilamentMaterialInstance(),
                                         parameter_name, texture, sampler);
      };
  update_texture_fn(parameter_name, texture->GetTexture());
  texture->OnAssignedToMaterial(*this, parameter_name,
                                std::move(update_texture_fn));

  parameters_to_owned_or_borrowed_textures_.insert_or_assign(
      std::string(parameter_name), std::move(texture));
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

absl::string_view SplitEngineCustomMaterial::GetParameterTransformName(
    absl::string_view sampler_name) const {
#if IMP_USE_LOCAL_SPLIT_ENGINE_MATERIALS
  return material_->GetParameterTransformName(sampler_name);
#endif

  // We don't know the mapping between sampler names and transform names
  // because the material is held on the SplitEngine renderer side.
  return {};
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
    result.emplace(pair.first, pair.second->GetTexture());
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
