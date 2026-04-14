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

#ifndef THIRD_PARTY_SPLIT_ENGINE_MATERIALS_WATER_REFLECTION_MATERIAL_H_
#define THIRD_PARTY_SPLIT_ENGINE_MATERIALS_WATER_REFLECTION_MATERIAL_H_

#include <memory>
#include <optional>
#include <utility>

#include "absl/base/attributes.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/render/texture.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "imp.h"

namespace android_xr {

// Displays a flowing water effect by reflecting the current IBL with custom
// normal map and tiling parameters. This uses a Split Engine Built-in material.
class WaterReflectionMaterial
    : public imp::split_engine::SplitEngineBuiltinMaterial {
 public:
  // TODO: Use the unified TextureAndSampler once it is compatible.
  using TextureAndSampler = std::pair<imp::OwnedOrBorrowedTexturePtr,
                                      std::optional<filament::TextureSampler>>;

  static imp::Future<std::unique_ptr<WaterReflectionMaterial>> Create(
      imp::BaseView& view, bool transparent = true);

  ~WaterReflectionMaterial() override;

  void SetReflectionCube(
      imp::OwnedOrBorrowedTexturePtr reflection_cube,
      std::optional<filament::TextureSampler> sampler = std::nullopt);
  void SetNormalMap(
      imp::OwnedOrBorrowedTexturePtr normal_map,
      std::optional<filament::TextureSampler> sampler = std::nullopt);
  void SetNormalTiling(float normal_tiling);
  void SetNormalSpeed(float normal_speed);
  void SetAlphaMap(
      imp::OwnedOrBorrowedTexturePtr alpha_map,
      std::optional<filament::TextureSampler> sampler = std::nullopt);
  void SetAlphaStepMultiplier(float alpha_step_multiplier);
  void SetNormalZ(float normal_z);
  void SetNormalBoundary(float normal_boundary);

  ABSL_DEPRECATED("Remove when water material migration is complete.")
  void SetAlphaStepU(imp::float4 alpha_step_u);
  ABSL_DEPRECATED("Remove when water material migration is complete.")
  void SetAlphaStepV(imp::float4 alpha_step_v);

 protected:
  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

 private:
  WaterReflectionMaterial(imp::BaseView& view, imp::OwnedMaterialPtr material);

  std::optional<TextureAndSampler> reflection_cube_;
  std::optional<TextureAndSampler> normal_map_;
  std::optional<float> normal_tiling_;
  std::optional<float> normal_speed_;
  std::optional<TextureAndSampler> alpha_map_;
  std::optional<float> alpha_step_multiplier_;
  std::optional<float> normal_z_;
  std::optional<float> normal_boundary_;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_WATER_REFLECTION_MATERIAL_H_
