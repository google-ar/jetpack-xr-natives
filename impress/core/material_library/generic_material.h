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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_H_

#include <memory>
#include <optional>
#include <vector>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "absl/types/span.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/common/owned_ptr.h"
#include "core/common/small_source_location.h"
#include "core/common/typed_vector.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/model/entity_data.h"
#include "core/render/texture.h"
#include "core/view/utils/string_map.h"

namespace imp {

// GenericMaterial is a specialization of Material representing generic
// ubershaders that conform to the glTF 2.0 Specification.
//
// It can be used like a regular Impress Material (e.g., passed to
// MeshRenderer), but also provides type-safe, explicit getters and setters for
// glTF-specific properties (e.g. factors, textures).
//
// Creation:
//   GenericMaterial instances should be created via MaterialFactory:
//
//   GetMaterialFactory().LoadGltfGenericMaterial(spec).Then(
//       [](OwnedGenericMaterialPtr material) {
//         // Use the material here.
//       });
//
//   The GenericMaterialSpec controls which underlying material is
//   picked from a MaterialPackage. Optionally, GltfAsset::LoadOptions can be
//   used to specify which MaterialPackage to use.
//
//   The choice of MaterialPackage affects package file size and performance
//   (as more features mean more shader permutations and shader code).
//   GltfAsset::LoadOptions determines which package is used:
//     - Default (Full): Uses the full default package (all features).
//     - Lite: Opt-in via `use_lite_materials = true` (excludes heavy features
//             like clearcoat and ambient occlusion to reduce size).
//     - Custom: Specified via `materials_url_override` to load a custom package
//               built with `imp_gltf_materials_package` with a tailored set
//               of features.
//
// Parameter Setting:
//   You can set parameters in two ways:
//   1. Explicit Setters/Getters: Use methods like SetBaseColorTexture() or
//      SetBaseColorFactor().
//   2. String-based: The inherited SetParameter() calls are still safe to use
//      for setting parameters dynamically. Refer to
//      generic_material_parameter_constants.h for the list of supported
//      public parameter names.
//
// IMPORTANT:
//   GenericMaterial require specific vertex attributes to be present in the
//   mesh they are applied to. If any of these are missing, rendering may fail
//   or behave unexpectedly:
//     - POSITION (always required)
//     - COLOR (required even if not using vertex colors to avoid multiplication
//              with zero defaults)
//     - UV0 (primary texture coordinates)
//     - UV1 (secondary texture coordinates)
//     - TANGENTS (required ONLY if using a Lit lighting model)
//
//   If you are using GenericMaterial on custom meshes or primitives (like a
//   simple Quad), ensure these attributes are generated. For example, when
//   creating a Quad, set the vertex color to white (to avoid multiplying color
//   factors with zero) and ensure UVs are generated.
class GenericMaterial : public Material {
 public:
  //////////////////////////////////////////////////////////////////////////////
  // Base Color Parameters
  //////////////////////////////////////////////////////////////////////////////

  // Sets the base color texture transferring ownership of the texture.
  absl::Status SetBaseColorTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the base color texture without transferring ownership of the texture.
  absl::Status SetBaseColorTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the base color texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetBaseColorTexture() const = 0;

  virtual absl::Status SetBaseColorUvTransform(const mat3f& uv_transform) = 0;

  // Sets the base color factor (RGBA).
  virtual void SetBaseColorFactor(const float4& factor) = 0;

  // Gets the base color factor (RGBA).
  virtual float4 GetBaseColorFactor() const = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Metallic Roughness Parameters
  //////////////////////////////////////////////////////////////////////////////

  // Sets the metallic roughness texture transferring ownership of the texture.
  absl::Status SetMetallicRoughnessTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the metallic roughness texture without transferring ownership of the
  // texture.
  absl::Status SetMetallicRoughnessTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the metallic roughness texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetMetallicRoughnessTexture() const = 0;

  virtual absl::Status SetMetallicRoughnessUvTransform(
      const mat3f& uv_transform) = 0;

  virtual void SetMetallicFactor(float factor) = 0;

  virtual float GetMetallicFactor() const = 0;

  virtual void SetRoughnessFactor(float factor) = 0;

  virtual float GetRoughnessFactor() const = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Normal Map Parameters
  //////////////////////////////////////////////////////////////////////////////

  // Sets the normal texture transferring ownership of the texture.
  absl::Status SetNormalTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the normal texture without transferring ownership of the texture.
  absl::Status SetNormalTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the normal texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetNormalTexture() const = 0;

  virtual absl::Status SetNormalUvTransform(const mat3f& uv_transform) = 0;

  virtual void SetNormalScale(float scale) = 0;

  virtual float GetNormalScale() const = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Ambient Occlusion Parameters
  //////////////////////////////////////////////////////////////////////////////

  // Sets the ambient occlusion texture transferring ownership of the texture.
  absl::Status SetAmbientOcclusionTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the ambient occlusion texture without transferring ownership of the
  // texture.
  absl::Status SetAmbientOcclusionTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the ambient occlusion texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetAmbientOcclusionTexture() const = 0;

  virtual absl::Status SetAmbientOcclusionUvTransform(
      const mat3f& uv_transform) = 0;

  virtual void SetAmbientOcclusionStrength(float strength) = 0;

  virtual float GetAmbientOcclusionStrength() const = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Emissive Parameters
  //////////////////////////////////////////////////////////////////////////////

  // Sets the emissive texture transferring ownership of the texture.
  absl::Status SetEmissiveTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the emissive texture without transferring ownership of the texture.
  absl::Status SetEmissiveTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the emissive texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetEmissiveTexture() const = 0;

  virtual absl::Status SetEmissiveUvTransform(const mat3f& uv_transform) = 0;

  // Sets the emissive factor (RGB).
  virtual void SetEmissiveFactor(const float3& factor) = 0;

  // Gets the emissive factor (RGB).
  virtual float3 GetEmissiveFactor() const = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Clearcoat Parameters (KHR_materials_clearcoat extension)
  //////////////////////////////////////////////////////////////////////////////

  // Sets the clearcoat texture transferring ownership of the texture.
  absl::Status SetClearcoatTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the clearcoat texture without transferring ownership of the texture.
  absl::Status SetClearcoatTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the clearcoat texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetClearcoatTexture() const = 0;

  // Sets the clearcoat roughness texture transferring ownership of the texture.
  absl::Status SetClearcoatRoughnessTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the clearcoat roughness texture without transferring ownership of the
  // texture.
  absl::Status SetClearcoatRoughnessTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the clearcoat roughness texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetClearcoatRoughnessTexture() const = 0;

  // Sets the clearcoat normal texture transferring ownership of the texture.
  absl::Status SetClearcoatNormalTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the clearcoat normal texture without transferring ownership of the
  // texture.
  absl::Status SetClearcoatNormalTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the clearcoat normal texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetClearcoatNormalTexture() const = 0;

  // Sets clearcoat factors: clearcoat factor, clearcoat roughness factor,
  // and clearcoat normal scale in a float3.
  virtual void SetClearcoatFactors(const float3& factor) = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Sheen Parameters (KHR_materials_sheen extension)
  //////////////////////////////////////////////////////////////////////////////

  // Sets the sheen color texture transferring ownership of the texture.
  absl::Status SetSheenColorTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the sheen color texture without transferring ownership of the texture.
  absl::Status SetSheenColorTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the sheen color texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetSheenColorTexture() const = 0;

  // Sets the sheen color factor (RGB).
  virtual void SetSheenColorFactor(const float3& factor) = 0;

  // Sets the sheen roughness texture transferring ownership of the texture.
  absl::Status SetSheenRoughnessTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the sheen roughness texture without transferring ownership of the
  // texture.
  absl::Status SetSheenRoughnessTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the sheen roughness texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetSheenRoughnessTexture() const = 0;

  virtual void SetSheenRoughnessFactor(float factor) = 0;

  //////////////////////////////////////////////////////////////////////////////
  // IOR Parameters (KHR_materials_ior extension)
  //////////////////////////////////////////////////////////////////////////////

  virtual void SetIndexOfRefraction(float index_of_refraction) = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Volume Parameters (KHR_materials_volume extension)
  //////////////////////////////////////////////////////////////////////////////

  // Sets the thickness texture transferring ownership of the texture.
  absl::Status SetThicknessTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the thickness texture without transferring ownership of the texture.
  absl::Status SetThicknessTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the thickness texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetThicknessTexture() const = 0;

  virtual void SetThicknessFactor(float factor) = 0;

  virtual void SetAttenuationDistance(float distance) = 0;

  // Sets the attenuation color (RGB).
  virtual void SetAttenuationColor(const float3& color) = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Transmission Parameters (KHR_materials_transmission extension)
  //////////////////////////////////////////////////////////////////////////////

  // Sets the transmission texture transferring ownership of the texture.
  absl::Status SetTransmissionTexture(
      OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Sets the transmission texture without transferring ownership of the
  // texture.
  absl::Status SetTransmissionTexture(
      BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override = std::nullopt);

  // Returns the transmission texture and sampler.
  // TODO: Explore switching to BorrowedTexturePtr.
  virtual TextureAndSampler GetTransmissionTexture() const = 0;

  virtual absl::Status SetTransmissionUvTransform(
      const mat3f& uv_transform) = 0;

  virtual void SetTransmissionFactor(float factor) = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Alpha Cutoff Parameters
  //////////////////////////////////////////////////////////////////////////////

  virtual void SetAlphaCutoff(float alpha_cutoff) = 0;

  virtual float GetAlphaCutoff() const = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Feature ID Parameters (EXT_mesh_features extension)
  //////////////////////////////////////////////////////////////////////////////

  // Returns the feature id texture and sampler for the given index.
  //
  // If the material does not have a feature id texture at the given index or
  // the mesh_features extension is disabled, this will return std::nullopt.
  virtual std::optional<TextureAndSampler> GetFeatureIdTexture(
      int index) const = 0;

  // Creates a copy of the material with all the same textures and parameters.
  virtual GenericMaterialPtr Duplicate() const = 0;

  //////////////////////////////////////////////////////////////////////////////
  // Overrides to route parameter setting via name.
  //////////////////////////////////////////////////////////////////////////////

  bool HasParameter(absl::string_view parameter_name) override;

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
  void SetParameter(
      absl::string_view parameter_name, const imp::Texture* texture,
      std::optional<filament::TextureSampler> sampler_override) override;
  ABSL_DEPRECATED(
      "Use imp::OwnedTexturePtr overload instead. See "
      "(broken link).")
  void SetParameter(
      absl::string_view parameter_name, TexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;
  void SetParameter(
      absl::string_view parameter_name, OwnedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;
  void SetParameter(
      absl::string_view parameter_name, BorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override) override;

  // Assigns all textures and parameters to the material from the spec schema in
  // one operation.
  //
  // This flow is primarily used when loading from a glTF file and when crossing
  // the Split Engine boundary.
  virtual absl::Status AssignTexturesAndParams(
      const GenericMaterialParameters& generic_material_parameters,
      const TextureBorrower& texture_borrower) = 0;

  // Legacy methods for MaterialConfig.
  // TODO: Remove these methods once MaterialConfig is removed.
  virtual std::vector<model::MaterialParameter> GetParameters() const = 0;
  virtual TypedVector<model::MaterialTexture> GetTextures() const = 0;
  virtual StringMap<int> GetSamplerIndexLookup() const = 0;

  // Gets the underlying material you can assign to things.
  //
  // TODO: Explore removing this now that GenericMaterial itself is
  // a material.
  BorrowedMaterialPtr GetMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const {
    return GetMaterialInternal(loc);
  }

 protected:
  // Set texture for a given channel (e.g. baseColorIndex).
  virtual absl::Status SetTextureForChannel(
      absl::string_view channel_name, OwnedOrBorrowedTexturePtr texture,
      std::optional<filament::TextureSampler> sampler_override =
          std::nullopt) = 0;

  virtual BorrowedMaterialPtr GetMaterialInternal(
      SmallSourceLocation loc) const = 0;
};

using GenericMaterialPtr = std::unique_ptr<GenericMaterial>;
using OwnedGenericMaterialPtr = OwnedPtr<GenericMaterial>;
using BorrowedGenericMaterialPtr = BorrowedPtr<GenericMaterial>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_H_
