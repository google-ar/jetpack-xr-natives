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
#include <utility>

#include "absl/base/attributes.h"
#include "absl/functional/function_ref.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/Color.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "filament/libs/math/include/math/mathfwd.h"
#include "core/assets/material/material_asset.h"  // IWYU pragma: export
#include "core/common/owned_or_unowned_memory.h"
#include "core/common/owned_ptr.h"
#include "core/common/small_source_location.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/view/utils/string_map.h"

namespace imp {

class MaterialAsset;
class MaterialFactory;
class GltfRenderer;

// For now, we only support move semantics and single ownership.
using MaterialPtr ABSL_DEPRECATED(
    "Prefer using OwnedMaterialPtr instead. See "
    "(broken link).") = std::unique_ptr<Material>;
using OwnedOrUnownedMaterial = OwnedOrUnownedMemory<Material>;

// Track lifetime of materials using OwnedPtr and BorrowedPtr.
using OwnedMaterialPtr = OwnedPtr<Material>;
using BorrowedMaterialPtr = BorrowedPtr<Material>;

// A base class that represents a Material in Impress. It can be used with
// rendering components (i.e. MeshRenderer, GltfRenderer) and provides a
// high-level API for interacting with materials, such as setting parameters,
// managing textures, and handling lifetime. A Material is typically created
// from the MaterialFactory using an AssetPtr<MaterialAsset>.
//
// When rendering with Impress normally, the Material subclass is a wrapper
// around a filament::MaterialInstance. When rendering with Split Engine, the
// subclass acts as a proxy to a material on the remote Split Engine Renderer.
class Material {
 public:
  Material() = default;

  virtual ~Material() = default;

  // Returns the underlying const filament::MaterialInstance*;
  virtual const filament::MaterialInstance* GetFilamentMaterialInstance()
      const = 0;
  // Returns the underlying filament::MaterialInstance*;
  virtual filament::MaterialInstance* GetFilamentMaterialInstance() = 0;

  // Returns the name of this Material.
  virtual const std::string& GetName() const = 0;

  // Assigns a name for this Material.
  virtual void SetName(absl::string_view name) = 0;

  virtual void SetParameter(absl::string_view parameter_name, bool value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, bool2 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, bool3 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, bool4 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, float value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, float2 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, float3 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, float4 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, int value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, int2 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, int3 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, int4 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, uint value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, uint2 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, uint3 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, uint4 value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, mat3f value) = 0;
  virtual void SetParameter(absl::string_view parameter_name, mat4f value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const bool> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const bool2> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const bool3> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const bool4> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const float> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const float2> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const float3> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const float4> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const int> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const int2> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const int3> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const int4> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const uint> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const uint2> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const uint3> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const uint4> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const mat3f> value) = 0;
  virtual void SetParameter(absl::string_view parameter_name,
                            absl::Span<const mat4f> value) = 0;

  // Sets the value of a RGBA parameter in the underlying Filament structures.
  virtual void SetParameter(absl::string_view parameter_name,
                            filament::RgbaType type,
                            filament::math::float4 color) = 0;

  // Sets the value of a RGB parameter in the underlying Filament structures.
  virtual void SetParameter(absl::string_view parameter_name,
                            filament::RgbType type,
                            filament::math::float3 color) = 0;

  // Set a parameter in filament structures using texture.
  // Does not take ownership of texture.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  ABSL_DEPRECATED(
      "Use imp::BorrowedTexturePtr overload instead. See "
      "(broken link).")
  virtual void SetParameter(absl::string_view parameter_name,
                            const imp::Texture* texture);
  ABSL_DEPRECATED(
      "Use imp::BorrowedTexturePtr overload instead. See "
      "(broken link).")
  virtual void SetParameter(
      absl::string_view parameter_name, const imp::Texture* texture,
      std::optional<filament::TextureSampler> sampler_override) = 0;

  // Forwards the TexturePtr to the overload that takes an OwnedTexturePtr.
  //
  // This method is kept for backwards compatibility. Typically, TexturePtr can
  // be implicitly converted to OwnedTexturePtr. However, in this overload is
  // needed to disambiguate which overload of SetParameter is called.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  void SetParameter(absl::string_view parameter_name, TexturePtr texture);
  virtual void SetParameter(
      absl::string_view parameter_name, TexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) = 0;

  // Set a parameter in filament structures using texture.
  //
  // Takes full ownership of texture.  The texture will be destroyed when this
  // material is.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  void SetParameter(absl::string_view parameter_name, OwnedTexturePtr texture);
  virtual void SetParameter(
      absl::string_view parameter_name, OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) = 0;

  // Set a parameter in filament structures using texture.
  //
  // The OwnedTexturePtr that texture was borrowed from must not be destroyed
  // until after the material is either destroyed or SetParameter has been
  // called again to change to a different texture.
  //
  // If sampler_override is provided, it will be used instead of the sampler
  // from the texture.
  void SetParameter(absl::string_view parameter_name,
                    BorrowedTexturePtr texture);
  virtual void SetParameter(
      absl::string_view parameter_name, BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) = 0;

  // Sets an Array of values to a parameter in the underlying Filament
  // structures. Errors out if parameter_name is invalid.
  template <typename T, size_t Size>
  void SetParameter(absl::string_view parameter_name,
                    std::array<T, Size> values) {
    SetParameter(parameter_name, absl::MakeConstSpan(values));
  }

  // Sets the value of a parameter in the underlying Filament structures. Does
  // nothing if parameter_name is invalid.
  template <typename... T>
  void TrySetParameter(absl::string_view parameter_name, T&&... values) {
    if (HasParameter(parameter_name)) {
      SetParameter(parameter_name, std::forward<T>(values)...);
    }
  }

  // Sets an Array of values to a parameter in the underlying Filament
  // structures. Does nothing if parameter_name is invalid.
  template <typename T>
  void TrySetParameter(absl::string_view parameter_name, absl::Span<T> values) {
    if (HasParameter(parameter_name)) {
      SetParameter(parameter_name, values);
    }
  }

  // Returns true if parameter_name exists in the underlying Filament
  // structures.
  virtual bool HasParameter(absl::string_view parameter_name) = 0;

  // Gets the name of the transform field associated for the given sampler
  // parameter. In the case where the parameter does not have a transform name
  // field, it will return an empty string.
  virtual absl::string_view GetParameterTransformName(
      absl::string_view sampler_name) const = 0;

  enum class HeldTextureType {
    kNone,
    kRawPointer,
    kOwnedPointer,
    kBorrowedPointer
  };

  // Returns the type of texture assignment for the given parameter name.
  virtual HeldTextureType GetAssignedTextureType(
      absl::string_view parameter_name) = 0;

  // Returns a map of unowned filament textures used by the material.
  virtual imp::StringMap<const filament::Texture*> GetUnownedFilamentTextures()
      const = 0;

  // Invokes the given function on each texture used by the material.
  virtual void ForEachTexture(absl::FunctionRef<void(BorrowedTexturePtr)> fn,
                              SmallSourceLocation loc) = 0;
};

inline void Material::SetParameter(absl::string_view parameter_name,
                                   const imp::Texture* texture) {
  SetParameter(parameter_name, texture, std::nullopt);
}

inline void Material::SetParameter(absl::string_view parameter_name,
                                   TexturePtr texture) {
  SetParameter(parameter_name, std::move(texture), std::nullopt);
}

inline void Material::SetParameter(absl::string_view parameter_name,
                                   OwnedTexturePtr texture) {
  SetParameter(parameter_name, std::move(texture), std::nullopt);
}

inline void Material::SetParameter(absl::string_view parameter_name,
                                   BorrowedTexturePtr texture) {
  SetParameter(parameter_name, texture, std::nullopt);
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIALS_MATERIAL_H_
