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

#ifndef THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_MATERIAL_PARAM_VALUE_H_
#define THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_MATERIAL_PARAM_VALUE_H_

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Texture.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "core/common/data_helpers.h"
#include "core/common/invocable.h"
#include "core/common/paired_vector.h"
#include "core/common/typed_id.h"
#include "core/math/almost_equal.h"
#include "core/math/mat.h"
#include "core/math/vec.h"

namespace imp {

using SamplerId = TypedIdWithSentinel<filament::TextureSampler, uint16_t,
                                      kMaxValue<uint16_t>>;
using TextureId =
    TypedIdWithSentinel<filament::Texture*, uint16_t, kMaxValue<uint16_t>>;
struct MaterialTexture;
using MaterialTextureId =
    TypedIdWithSentinel<MaterialTexture, uint16_t, kMaxValue<uint16_t>>;
struct MaterialParameter;
using MaterialParameterId =
    TypedIdWithSentinel<MaterialParameter, uint16_t, kMaxValue<uint16_t>>;
// using PrimitiveType = filament::RenderableManager::PrimitiveType;

template <typename T>
using SamplerLookup = PairedVector<T, SamplerId::ReferredType>;
template <typename T>
using TextureLookup = PairedVector<T, TextureId::ReferredType>;

struct MaterialTexture {
  MaterialTexture(TextureId in_texture, SamplerId in_sampler)
      : texture(in_texture), sampler(in_sampler) {}

  TextureId texture;
  SamplerId sampler;
};

// Holds a filament texture and the associated texture sampler.
struct TextureAndSampler {
  const filament::Texture* texture;
  filament::TextureSampler sampler;
  mat3f uv_transform;

  TextureAndSampler(const filament::Texture* in_texture,
                    filament::TextureSampler in_sampler)
      : texture(in_texture), sampler(in_sampler) {}

  TextureAndSampler(const filament::Texture* in_texture,
                    filament::TextureSampler in_sampler, mat3f in_uv_transform)
      : texture(in_texture),
        sampler(in_sampler),
        uv_transform(in_uv_transform) {}

  bool operator==(const TextureAndSampler& other) const {
    return texture == other.texture &&
           sampler.getAnisotropy() == other.sampler.getAnisotropy() &&
           sampler.getCompareFunc() == other.sampler.getCompareFunc() &&
           sampler.getCompareMode() == other.sampler.getCompareMode() &&
           sampler.getMagFilter() == other.sampler.getMagFilter() &&
           sampler.getMinFilter() == other.sampler.getMinFilter() &&
           sampler.getWrapModeS() == other.sampler.getWrapModeS() &&
           sampler.getWrapModeT() == other.sampler.getWrapModeT() &&
           sampler.getWrapModeR() == other.sampler.getWrapModeR() &&
           RoughlyEqual(uv_transform, other.uv_transform);
  }
};

struct MaterialParameter {
  using ParamVariant =
      std::variant<float, float2, float3, float4,  //
                   int, int2, int3, int4,          //
                   bool, bool2, bool3, bool4,      //
                   mat3f, std::vector<mat3f>, TextureAndSampler>;

  MaterialParameter(absl::string_view in_name, ParamVariant in_value)
      : name(in_name), value(std::move(in_value)) {}

  std::string name;
  ParamVariant value;

  inline bool operator==(const MaterialParameter& other) const {
    return name == other.name && value == other.value;
  }
};

using TextureProvider = Invocable<const filament::Texture*(uint64_t)>;
class GenericMaterial;
using GenericMaterialPtr = std::unique_ptr<GenericMaterial>;
using MaterialId =
    TypedIdWithSentinel<GenericMaterialPtr, uint16_t, kMaxValue<uint16_t>>;

using MaterialParamValue =
    std::variant<float, float2, float3, float4,  //^
                 int, int2, int3, int4,          //^
                 bool, bool2, bool3, bool4,      //^
                 MaterialTextureId, mat3f, std::vector<mat3f>, mat4f,
                 std::vector<mat4f>>;

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_MATERIAL_LIBRARY_MATERIAL_PARAM_VALUE_H_
