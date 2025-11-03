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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_CUSTOM_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_CUSTOM_MATERIAL_H_

#include <optional>

#include "absl/base/attributes.h"
#include "absl/functional/function_ref.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/libs/math/include/math/mathfwd.h"
#include "core/assets/asset_ptr.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/view/utils/string_map.h"

namespace imp {

// A thin wrapper around filament::MaterialInstance.
// This is the "canonical" implementation of imp::Material that is used when
// Impress runs without Split Engine.
class CustomMaterial : public Material {
 public:
  ~CustomMaterial() override;

  filament::MaterialInstance* GetFilamentMaterialInstance() override;

  void SetParameter(absl::string_view parameter_name, bool value) override;
  void SetParameter(absl::string_view parameter_name, bool2 value) override;
  void SetParameter(absl::string_view parameter_name, bool3 value) override;
  void SetParameter(absl::string_view parameter_name, bool4 value) override;
  void SetParameter(absl::string_view parameter_name, float value) override;
  void SetParameter(absl::string_view parameter_name, float2 value) override;
  void SetParameter(absl::string_view parameter_name, float3 value) override;
  void SetParameter(absl::string_view parameter_name, float4 value) override;
  void SetParameter(absl::string_view parameter_name, int value) override;
  void SetParameter(absl::string_view parameter_name, int2 value) override;
  void SetParameter(absl::string_view parameter_name, int3 value) override;
  void SetParameter(absl::string_view parameter_name, int4 value) override;
  void SetParameter(absl::string_view parameter_name, uint value) override;
  void SetParameter(absl::string_view parameter_name, uint2 value) override;
  void SetParameter(absl::string_view parameter_name, uint3 value) override;
  void SetParameter(absl::string_view parameter_name, uint4 value) override;
  void SetParameter(absl::string_view parameter_name, mat3f value) override;
  void SetParameter(absl::string_view parameter_name, mat4f value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const bool> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const bool2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const bool3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const bool4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const float> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const float2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const float3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const float4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const int> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const int2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const int3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const int4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const uint> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const uint2> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const uint3> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const uint4> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const mat3f> value) override;
  void SetParameter(absl::string_view parameter_name,
                    absl::Span<const mat4f> value) override;

  void SetParameter(absl::string_view parameter_name, filament::RgbaType type,
                    filament::math::float4 color) override;

  void SetParameter(absl::string_view parameter_name, filament::RgbType type,
                    filament::math::float3 color) override;

  ABSL_DEPRECATED(
      "Use imp::BorrowedTexturePtr overload instead. See "
      "(broken link).")
  void SetParameter(absl::string_view parameter_name,
                    const imp::Texture* texture,
                    std::optional<filament::TextureSampler> sampler_override =
                        std::nullopt) override;

  void SetParameter(absl::string_view parameter_name, TexturePtr texture,
                    std::optional<filament::TextureSampler> sampler_override =
                        std::nullopt) override;

  void SetParameter(absl::string_view parameter_name, OwnedTexturePtr texture,
                    std::optional<filament::TextureSampler> sampler_override =
                        std::nullopt) override;

  void SetParameter(absl::string_view parameter_name,
                    BorrowedTexturePtr texture,
                    std::optional<filament::TextureSampler> sampler_override =
                        std::nullopt) override;

  bool HasParameter(absl::string_view parameter_name) override;

  HeldTextureType GetAssignedTextureType(
      absl::string_view parameter_name) override;

  imp::StringMap<const filament::Texture*> GetUnownedFilamentTextures()
      const override;

  void ForEachTexture(
      absl::FunctionRef<void(BorrowedTexturePtr)> fn,
      SmallSourceLocation loc = SmallSourceLocation::Current()) override;

 private:
  CustomMaterial(filament::MaterialInstance* material_instance,
                 AssetPtr<MaterialAsset> material_asset);

  filament::Engine* engine_;
  filament::MaterialInstance* material_instance_;

  // Keep a pointer to the material asset so it is not cleared as long as
  // instances of the material are in use.
  AssetPtr<MaterialAsset> material_asset_;

  // Tracks textures used by the material
  // If the texture is represented by an OwnedTexturePtr, then it is owned
  // here and will be released when the material is destroyed, or if the
  // property is reassigned.
  //
  // If the texture is a BorrowedTexturePtr it is owned elsewhere and only
  // used here. This level of tracking  is provided for debugging purposes to
  // detect if the filament texture is destroyed while it is in use.
  StringMap<OwnedOrBorrowedPtr<Texture>>
      parameters_to_owned_or_borrowed_textures_;

  // Tracks raw textures used by the material for backwards compatibility with
  // deprecated SetParameter overloads.
  //
  // This is only used for debugging purposes to detect if the filament
  // texture is destroyed while it is in use.
  StringMap<const Texture*> parameters_to_raw_textures_;

  friend class MaterialFactory;
  friend class GenericMaterialImpl;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_CUSTOM_MATERIAL_H_
