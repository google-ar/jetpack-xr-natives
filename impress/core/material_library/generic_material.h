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

#include <vector>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/common/small_source_location.h"
#include "core/common/typed_vector.h"
#include "core/material_library/generic_material_parameters.h"
#include "core/material_library/material_param_value.h"
#include "core/materials/material.h"
#include "core/math/mat.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/view/utils/string_map.h"

namespace imp {

// A class to manage Impress' generic glTF materials by ingesting a schema with
// structured material parameters and textures for known texture channels such
// as "base-color", "normal", "roughness", "ambient-occlusion", etc.
//
// Create a GenericMaterial using the Create method, passing in a schema, a
// material cache, and the textures required by the material.
class GenericMaterial {
 public:
  virtual ~GenericMaterial() = default;

  // Creates a copy of the material with all the same textures and parameters.
  virtual GenericMaterialPtr Duplicate() const = 0;

  // Assigns all textures and parameters to the material from the spec schema.
  virtual absl::Status AssignTexturesAndParams(
      const GenericMaterialParameters& generic_material_parameters,
      const TextureBorrower& texture_borrower) = 0;

  // TODO: Remove this once we fully migrate to TextureBorrower.
  // Assigns all textures and parameters to the material from the spec schema.
  virtual absl::Status AssignTexturesAndParams(
      const GenericMaterialParameters& generic_material_parameters,
      const TextureProvider& texture_provider) = 0;

  // Legacy methods for MaterialConfig.
  // TODO: (broken link) - Remove these methods once MaterialConfig is removed.
  virtual absl::string_view GetName() const = 0;
  virtual std::vector<MaterialParameter> GetParameters() const = 0;
  virtual TypedVector<MaterialTexture> GetTextures() const = 0;
  virtual StringMap<int> GetSamplerIndexLookup() const = 0;

  // Gets the underlying material you can assign to things.
  BorrowedMaterialPtr GetMaterial(
      SmallSourceLocation loc = SmallSourceLocation::Current()) const {
    return GetMaterialInternal(loc);
  }

  // Gets the base color texture and sampler. Note: this may be a placeholder
  // white texture if the material does not have a base color channel.
  virtual TextureAndSampler GetBaseColorTexture() const = 0;
  // Sets the UV transform for the base color texture.
  virtual absl::Status SetBaseColorUvTransform(const mat3f& uv_transform) = 0;
  virtual void SetBaseColorFactor(const float4& factor) = 0;
  virtual float4 GetBaseColorFactor() const = 0;

  // Gets the metallic roughness texture and sampler. Note: this may be a
  // placeholder white texture if the material does not have a metallic
  // roughness channel.
  virtual TextureAndSampler GetMetallicRoughnessTexture() const = 0;
  virtual absl::Status SetMetallicRoughnessUvTransform(
      const mat3f& uv_transform) = 0;
  virtual void SetMetallicFactor(float factor) = 0;
  virtual float GetMetallicFactor() const = 0;
  virtual void SetRoughnessFactor(float factor) = 0;
  virtual float GetRoughnessFactor() const = 0;

  // Gets the normal texture and sampler. Note: this may be a placeholder white
  // texture if the material does not have a normal channel.
  virtual TextureAndSampler GetNormalTexture() const = 0;
  virtual absl::Status SetNormalUvTransform(const mat3f& uv_transform) = 0;
  virtual void SetNormalScale(float scale) = 0;
  virtual float GetNormalScale() const = 0;

  virtual TextureAndSampler GetAmbientOcclusionTexture() const = 0;
  virtual absl::Status SetAmbientOcclusionUvTransform(
      const mat3f& uv_transform) = 0;
  virtual void SetAmbientOcclusionStrength(float strength) = 0;
  virtual float GetAmbientOcclusionStrength() const = 0;

  virtual TextureAndSampler GetEmissiveTexture() const = 0;
  virtual absl::Status SetEmissiveUvTransform(const mat3f& uv_transform) = 0;
  virtual void SetEmissiveFactor(const float3& factor) = 0;
  virtual float3 GetEmissiveFactor() const = 0;

  virtual TextureAndSampler GetClearcoatTexture() const = 0;
  virtual TextureAndSampler GetClearcoatNormalTexture() const = 0;
  virtual TextureAndSampler GetClearcoatRoughnessTexture() const = 0;
  virtual void SetClearcoatFactors(const float3& factor) = 0;
  virtual TextureAndSampler GetSheenColorTexture() const = 0;
  virtual void SetSheenColorFactor(const float3& factor) = 0;
  virtual TextureAndSampler GetSheenRoughnessTexture() const = 0;
  virtual void SetSheenRoughnessFactor(float factor) = 0;
  virtual TextureAndSampler GetTransmissionTexture() const = 0;
  virtual absl::Status SetTransmissionUvTransform(
      const mat3f& uv_transform) = 0;
  virtual void SetTransmissionFactor(float factor) = 0;
  virtual void SetIndexOfRefraction(float index_of_refraction) = 0;

  virtual void SetAlphaCutoff(float alpha_cutoff) = 0;
  virtual float GetAlphaCutoff() const = 0;

 protected:
  virtual BorrowedMaterialPtr GetMaterialInternal(
      SmallSourceLocation loc) const = 0;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_GENERIC_MATERIAL_H_
