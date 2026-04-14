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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_MATERIAL_WRAPPER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_MATERIAL_WRAPPER_H_

#include <optional>
#include <string>
#include <utility>

#include "absl/base/attributes.h"
#include "absl/functional/function_ref.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "core/common/owned_ptr.h"
#include "core/common/small_source_location.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/view/utils/string_map.h"

namespace imp::split_engine {

// A wrapper around a material that implements the Material interface forwarding
// it to the underlying material, and the BuiltInMaterial interface. This is a
// common base class for all built-in materials that wrap a Material instance.
template <typename MaterialT>
class BuiltInMaterialWrapper : public BuiltInMaterial {
  static_assert(std::is_base_of_v<Material, MaterialT>,
                "MaterialT must be a subclass of Material.");

 public:
  BuiltInMaterialWrapper(OwnedPtr<MaterialT> material);

  const std::string& GetName() const override;

  void SetName(absl::string_view name) override;

  const filament::MaterialInstance* GetFilamentMaterialInstance()
      const override;
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

  // Sets the value of a RGBA parameter in the underlying Filament structures.
  void SetParameter(absl::string_view parameter_name, filament::RgbaType type,
                    filament::math::float4 color) override;

  // Sets the value of a RGB parameter in the underlying Filament structures.
  void SetParameter(absl::string_view parameter_name, filament::RgbType type,
                    filament::math::float3 color) override;

  // Set a parameter in filament structures using texture.
  // Does not take ownership of texture.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  ABSL_DEPRECATED(
      "Use imp::BorrowedTexturePtr overload instead. See "
      "(broken link).")
  void SetParameter(
      absl::string_view parameter_name, const imp::Texture* texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  // Forwards the TexturePtr to the overload that takes an OwnedTexturePtr.
  //
  // This method is kept for backwards compatibility. Typically, TexturePtr can
  // be implicitly converted to OwnedTexturePtr. However, in this overload is
  // needed to disambiguate which overload of SetParameter is called.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  void SetParameter(
      absl::string_view parameter_name, TexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  // Set a parameter in filament structures using texture.
  //
  // Takes full ownership of texture.  The texture will be destroyed when this
  // material is.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  void SetParameter(
      absl::string_view parameter_name, OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  // Set a parameter in filament structures using texture.
  //
  // The OwnedTexturePtr that texture was borrowed from must not be destroyed
  // until after the material is either destroyed or SetParameter has been
  // called again to change to a different texture.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  void SetParameter(
      absl::string_view parameter_name, BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  // Returns true if parameter_name exists in the underlying Filament
  // structures.
  bool HasParameter(absl::string_view name) override;

  // Gets the name of the transform field associated for the given sampler
  // parameter. In the case where the parameter does not have a transform name
  // field, it will return an empty string.
  absl::string_view GetParameterTransformName(
      absl::string_view sampler_name) const override;

  // Returns the type of texture assignment for the given parameter name.
  HeldTextureType GetAssignedTextureType(
      absl::string_view parameter_name) override;

  // Returns a map of unowned filament textures used by the material.
  imp::StringMap<const filament::Texture*> GetUnownedFilamentTextures()
      const override;

  // Invokes the given function on each texture used by the material.
  void ForEachTexture(absl::FunctionRef<void(BorrowedTexturePtr)> fn,
                      SmallSourceLocation loc) override;

 protected:
  std::string name_;
  OwnedPtr<MaterialT> material_;

  BorrowedMaterialPtr GetMaterialInternal(
      SmallSourceLocation loc) const override;
};

template <typename MaterialT>
BuiltInMaterialWrapper<MaterialT>::BuiltInMaterialWrapper(
    OwnedPtr<MaterialT> material)
    : material_(std::move(material)) {}

template <typename MaterialT>
const std::string& BuiltInMaterialWrapper<MaterialT>::GetName() const {
  return name_;
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetName(absl::string_view name) {
  name_ = name;
}

template <typename MaterialT>
const filament::MaterialInstance*
BuiltInMaterialWrapper<MaterialT>::GetFilamentMaterialInstance() const {
  return material_->GetFilamentMaterialInstance();
}

template <typename MaterialT>
filament::MaterialInstance*
BuiltInMaterialWrapper<MaterialT>::GetFilamentMaterialInstance() {
  return material_->GetFilamentMaterialInstance();
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, bool value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, bool2 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, bool3 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, bool4 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, float value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, float2 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, float3 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, float4 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, int value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, int2 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, int3 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, int4 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, uint value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, uint2 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, uint3 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, uint4 value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, mat3f value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, mat4f value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const bool> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const bool2> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const bool3> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const bool4> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const float> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const float2> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const float3> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const float4> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const int> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const int2> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const int3> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const int4> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const uint> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const uint2> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const uint3> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const uint4> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const mat3f> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, absl::Span<const mat4f> value) {
  material_->SetParameter(parameter_name, value);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, filament::RgbaType type,
    filament::math::float4 color) {
  material_->SetParameter(parameter_name, type, color);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, filament::RgbType type,
    filament::math::float3 color) {
  material_->SetParameter(parameter_name, type, color);
}

template <typename MaterialT>
ABSL_DEPRECATED(
    "Use imp::BorrowedTexturePtr overload instead. See "
    "(broken link).")
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, const imp::Texture* texture,
    std::optional<filament::TextureSampler> sampler_override) {
  material_->SetParameter(parameter_name, texture, sampler_override);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, TexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  material_->SetParameter(parameter_name, std::move(texture), sampler_override);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, OwnedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  material_->SetParameter(parameter_name, std::move(texture), sampler_override);
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::SetParameter(
    absl::string_view parameter_name, BorrowedTexturePtr texture,
    std::optional<filament::TextureSampler> sampler_override) {
  material_->SetParameter(parameter_name, texture, sampler_override);
}

template <typename MaterialT>
bool BuiltInMaterialWrapper<MaterialT>::HasParameter(absl::string_view name) {
  return material_->HasParameter(name);
}

template <typename MaterialT>
absl::string_view BuiltInMaterialWrapper<MaterialT>::GetParameterTransformName(
    absl::string_view sampler_name) const {
  return material_->GetParameterTransformName(sampler_name);
}

template <typename MaterialT>
Material::HeldTextureType
BuiltInMaterialWrapper<MaterialT>::GetAssignedTextureType(
    absl::string_view parameter_name) {
  return material_->GetAssignedTextureType(parameter_name);
}

template <typename MaterialT>
imp::StringMap<const filament::Texture*>
BuiltInMaterialWrapper<MaterialT>::GetUnownedFilamentTextures() const {
  return material_->GetUnownedFilamentTextures();
}

template <typename MaterialT>
void BuiltInMaterialWrapper<MaterialT>::ForEachTexture(
    absl::FunctionRef<void(BorrowedTexturePtr)> fn, SmallSourceLocation loc) {
  material_->ForEachTexture(fn, loc);
}

template <typename MaterialT>
BorrowedMaterialPtr BuiltInMaterialWrapper<MaterialT>::GetMaterialInternal(
    SmallSourceLocation loc) const {
  return material_.Borrow(loc);
}

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_MATERIALS_BUILTIN_BUILTIN_MATERIAL_WRAPPER_H_
