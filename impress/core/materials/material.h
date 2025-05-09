/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIALS_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIALS_MATERIAL_H_

#include <array>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "absl/base/attributes.h"
#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/libs/math/include/math/mathfwd.h"
#include "core/assets/asset_ptr.h"
#include "core/common/owned_or_borrowed_ptr.h"
#include "core/common/owned_or_unowned_memory.h"
#include "core/common/owned_ptr.h"
#include "core/material_library/material_param_value.h"
#include "core/render/texture.h"
#include "core/split_engine/split_engine_serializer.h"
#include "core/view/base_view.h"
#include "core/view/utils/string_map.h"

namespace imp {

class MaterialAsset;
class MaterialFactory;
class GltfRenderer;

// A thin wrapper around filament::MaterialInstance.
class Material {
 public:
  ~Material();

  // Creates an Impress Material wrapping the given filament::MaterialInstance.
  // The lifetime of the filament::MaterialInstance is managed automatically.
  static std::unique_ptr<Material> WrapMaterial(
      BaseView& view, filament::MaterialInstance* material_instance);

  // Returns the underlying filament::MaterialInstance*;
  filament::MaterialInstance* GetFilamentMaterialInstance();

  // Returns the name of this Material.  The name is stored in this class and
  // not connected to the underlying filament structures.
  const std::string& GetName() const;

  // Assigns a name for this Material.  The name set in this class is not copied
  // into underlying filament structures.
  void SetName(absl::string_view name);

  // Sets the value of a parameter in the underlying Filament structures if
  // parameter_name is valid.  Does nothing if parameter_name is invalid.
  template <typename... T>
  void TrySetParameter(absl::string_view parameter_name, T&&... values);

  // Sets the value of a parameter in the underlying Filament structures.
  // Errors out if parameter_name is invalid.
  template <typename T>
  void SetParameter(absl::string_view parameter_name, T value);

  // Set an Array of values to a parameter
  // in the underlying Filament structures if parameter_name is valid.
  // Does nothing if parameter_name is invalid.
  template <typename T>
  void TrySetParameter(absl::string_view parameter_name, absl::Span<T> values);

  template <typename T, size_t Size>
  void SetParameter(absl::string_view parameter_name,
                    std::array<T, Size> values);

  // Set an Array of values to a parameter
  // in the underlying Filament structures.
  // Errors out if parameter_name is invalid.
  template <typename T>
  void SetParameter(absl::string_view parameter_name, absl::Span<T> values);

  // Sets the value of a RGBA parameter in the underlying Filament structures.
  void SetParameter(absl::string_view parameter_name, filament::RgbaType type,
                    filament::math::float4 color);

  // Sets the value of a RGB parameter in the underlying Filament structures.
  void SetParameter(absl::string_view parameter_name, filament::RgbType type,
                    filament::math::float3 color);

  // Set a parameter in filament structures using texture.
  // Does not take ownership of texture, the caller must ensure the lifetime of
  // the texture while it is in use.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  ABSL_DEPRECATED(
      "Use imp::BorrowedTexturePtr overload instead. See "
      "(broken link).")
  void SetParameter(
      absl::string_view parameter_name, imp::Texture* texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

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
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

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
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Set a parameter in filament structures using texture.
  //
  // Takes full ownership of texture.  The texture will be destroyed when this
  // material is.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  void SetParameter(
      absl::string_view parameter_name, OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

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
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  /// Returns true if parameter_name exists in the underlying Filament
  /// structures.
  bool HasParameter(absl::string_view parameter_name);

  enum class HeldTextureType {
    kNone,
    kRawPointer,
    kOwnedPointer,
    kBorrowedPointer
  };

  HeldTextureType GetAssignedTextureType(absl::string_view parameter_name);

  imp::StringMap<const filament::Texture*> GetUnownedFilamentTextures() const;

 private:
  void SetTextureImpl(absl::string_view parameter_name, const Texture* texture,
                      std::optional<filament::TextureSampler> sampler_override);

  Material(BaseView& view, filament::MaterialInstance* material_instance,
           const AssetPtr<MaterialAsset>& material_asset);

  BaseView& view_;
  filament::Engine* engine_;
  filament::MaterialInstance* material_instance_;

  AssetPtr<MaterialAsset> material_asset_;
  std::string name_;

  // Tracks textures used by the material
  // If the texture is represented by an OwnedTexturePtr, then it is owned here
  // and will be released when the material is destroyed, or if the property is
  // reassigned.
  //
  // If the texture is a BorrowedTexturePtr it is owned elsewhere and only used
  // here. This level of tracking  is provided for debugging purposes to
  // detect if the filament texture is destroyed while it is in use.
  StringMap<OwnedOrBorrowedPtr<Texture>>
      parameters_to_owned_or_borrowed_textures_;

  // Tracks raw textures used by the material for backwards compatibility with
  // deprecated SetParameter overloads.
  //
  // This is only used for debugging purposes to detect if the filament
  // texture is destroyed while it is in use.
  StringMap<filament::Texture*> parameters_to_raw_textures_;

  friend class MaterialFactory;
  friend class MaterialAsset;
};

// For now, we only support move semantics and single ownership.
using MaterialPtr ABSL_DEPRECATED(
    "Prefer using OwnedMaterialPtr instead. See "
    "(broken link).") = std::unique_ptr<Material>;
using OwnedOrUnownedMaterial = OwnedOrUnownedMemory<Material>;

// Track lifetime of materials using OwnedPtr and BorrowedPtr.
using OwnedMaterialPtr = OwnedPtr<Material>;
using BorrowedMaterialPtr = BorrowedPtr<Material>;

template <typename T>
void Material::SetParameter(absl::string_view parameter_name, T value) {
  if (split_engine::SplitEngineSerializer* serializer =
          view_.GetSplitEngineSerializer()) {
    // TODO: This is needed because there are parameter types that
    // are used by Impress client apps that are not covered by
    // MaterialParamValue. We should support all parameter types.
    if constexpr (std::is_convertible<T, MaterialParamValue>::value) {
      serializer->SetMaterialParameter(material_instance_, parameter_name,
                                       value);
    } else {
      IMP_LOG(imp::FATAL) << "Attempt to call SetParameter() with type unsupported by "
                    "SplitEngine.";
    }
  } else {
    GetFilamentMaterialInstance()->setParameter<T>(
        parameter_name.data(), parameter_name.size(), value);
  }
}

// TODO: send the rest of these to the serializer as well.
template <typename... T>
void Material::TrySetParameter(absl::string_view parameter_name,
                               T&&... values) {
  if (HasParameter(parameter_name)) {
    SetParameter(parameter_name, std::forward<T>(values)...);
  }
}

template <typename T>
void Material::SetParameter(absl::string_view parameter_name,
                            absl::Span<T> values) {
  if (split_engine::SplitEngineSerializer* serializer =
          view_.GetSplitEngineSerializer()) {
    // TODO: This is needed because there are parameter types that
    // are used by Impress client apps that are not covered by
    // MaterialParamValue. We should support all parameter types.
    if constexpr (std::is_convertible<std::vector<T>,
                                      MaterialParamValue>::value) {
      // One of the supported types in the MaterialParamValue variant is a
      // vector of other supported types. So we can send the vector directly to
      // the serializer.
      std::vector<T> value_vector(values.begin(), values.end());
      serializer->SetMaterialParameter(material_instance_, parameter_name,
                                       value_vector);
    } else {
      IMP_LOG(imp::FATAL) << "Attempt to call SetParameter() with type unsupported by "
                    "SplitEngine.";
    }
  } else {
    GetFilamentMaterialInstance()->setParameter<T>(
        std::string(parameter_name).c_str(), values.data(), values.size());
  }
}

template <typename T, size_t Size>
void Material::SetParameter(absl::string_view parameter_name,
                            std::array<T, Size> values) {
  SetParameter(parameter_name, absl::MakeSpan(values));
}

template <typename T>
void Material::TrySetParameter(absl::string_view parameter_name,
                               absl::Span<T> values) {
  if (HasParameter(parameter_name)) {
    SetParameter(parameter_name, values);
  }
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_MATERIAL_H_
