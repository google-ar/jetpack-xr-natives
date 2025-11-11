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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_CUSTOM_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_CUSTOM_MATERIAL_H_

#include <optional>
#include <string>

#include "absl/base/attributes.h"
#include "absl/functional/function_ref.h"
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
#include "core/split_engine/materials/split_engine_material.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/utils/string_map.h"

namespace imp::split_engine {

// A SplitEngineCustomMaterial is used when an Impress application defines a
// material that is used through Split Engine. The material can be sent in
// compiled or source form and is stored on the Split Engine renderer side. This
// class provides access to the material for the app and is responsible for
// serializing material changes to the Split Engine host.
class SplitEngineCustomMaterial : public Material {
 public:
  explicit SplitEngineCustomMaterial(SplitEngineSerializer& serializer,
                                     OwnedMaterialPtr material);

  ~SplitEngineCustomMaterial() override;

  filament::MaterialInstance* GetFilamentMaterialInstance() override;

  const std::string& GetName() const override;

  void SetName(absl::string_view name) override;

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
  SplitEngineSerializer& serializer_;
  OwnedMaterialPtr material_;

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
  StringMap<const filament::Texture*> parameters_to_raw_textures_;
};
}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_SPLIT_ENGINE_CUSTOM_MATERIAL_H_
