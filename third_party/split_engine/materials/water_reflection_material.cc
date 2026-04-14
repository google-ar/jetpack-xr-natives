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

#include "split_engine/materials/water_reflection_material.h"

#include <memory>
#include <optional>
#include <utility>

#include "absl/memory/memory.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/render/texture.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace android_xr {

namespace {

constexpr float kDefaultNormalTiling = 2.5f;
constexpr float kDefaultNormalSpeed = 0.025f;
constexpr float kDefaultAlphaStepMultiplier = 1.0f;
constexpr float kDefaultNormalZ = 0.5f;
constexpr float kDefaultNormalBoundary = 0.6f;

}  // namespace

imp::Future<std::unique_ptr<WaterReflectionMaterial>>
WaterReflectionMaterial::Create(imp::BaseView& view, bool transparent) {
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterial5cf26af8>
      spec_offset =
          android_xr::schemas::CreateBuiltInMaterial5cf26af8(*fbb, transparent);
  return imp::split_engine::SplitEngineBuiltinMaterial::RequestBuiltInMaterial(
             view, std::move(fbb),
             android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial5cf26af8,
             spec_offset.Union())
      .Then([&view](imp::OwnedMaterialPtr material) {
        return absl::WrapUnique(
            new WaterReflectionMaterial(view, std::move(material)));
      });
}

WaterReflectionMaterial::WaterReflectionMaterial(imp::BaseView& view,
                                                 imp::OwnedMaterialPtr material)
    : SplitEngineBuiltinMaterial(
          view,
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterial5cf26af8Parameters,
          std::move(material)) {}

WaterReflectionMaterial::~WaterReflectionMaterial() { Cleanup(); }

flatbuffers::Offset<void> WaterReflectionMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter>
      reflection_cube;
  if (reflection_cube_) {
    reflection_cube = texture_parameter_creator.Create(
        fbb, reflection_cube_->first.Borrow(), reflection_cube_->second);
  }
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter> normal_map;
  if (normal_map_) {
    normal_map = texture_parameter_creator.Create(
        fbb, normal_map_->first.Borrow(), normal_map_->second);
  }
  android_xr::schemas::Float normal_tiling =
      imp::split_engine::Pack(normal_tiling_.value_or(kDefaultNormalTiling));
  android_xr::schemas::Float normal_speed =
      imp::split_engine::Pack(normal_speed_.value_or(kDefaultNormalSpeed));
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter> alpha_map;
  if (alpha_map_) {
    alpha_map = texture_parameter_creator.Create(
        fbb, alpha_map_->first.Borrow(), alpha_map_->second);
  }

  android_xr::schemas::Float alpha_step_multiplier = imp::split_engine::Pack(
      alpha_step_multiplier_.value_or(kDefaultAlphaStepMultiplier));
  android_xr::schemas::Float normal_z =
      imp::split_engine::Pack(normal_z_.value_or(kDefaultNormalZ));
  android_xr::schemas::Float normal_boundary = imp::split_engine::Pack(
      normal_boundary_.value_or(kDefaultNormalBoundary));

  // TODO: (broken link) - Change the parameters to all be optional and only
  // serialize the ones that are set / changed.
  return android_xr::schemas::CreateBuiltInMaterial5cf26af8Parameters(
             fbb, reflection_cube, normal_map, &normal_tiling, &normal_speed,
             alpha_map, &alpha_step_multiplier, &normal_z, &normal_boundary)
      .Union();
}

void WaterReflectionMaterial::SetReflectionCube(
    imp::OwnedOrBorrowedTexturePtr reflection_cube,
    std::optional<filament::TextureSampler> sampler) {
  reflection_cube_ =
      std::make_pair(std::move(reflection_cube), std::move(sampler));
  MarkParametersDirty();
}

void WaterReflectionMaterial::SetNormalMap(
    imp::OwnedOrBorrowedTexturePtr normal_map,
    std::optional<filament::TextureSampler> sampler) {
  normal_map_ = std::make_pair(std::move(normal_map), std::move(sampler));
  MarkParametersDirty();
}

void WaterReflectionMaterial::SetNormalTiling(float normal_tiling) {
  normal_tiling_ = normal_tiling;
  MarkParametersDirty();
}

void WaterReflectionMaterial::SetNormalSpeed(float normal_speed) {
  normal_speed_ = normal_speed;
  MarkParametersDirty();
}

void WaterReflectionMaterial::SetAlphaMap(
    imp::OwnedOrBorrowedTexturePtr alpha_map,
    std::optional<filament::TextureSampler> sampler) {
  alpha_map_ = std::make_pair(std::move(alpha_map), std::move(sampler));
  MarkParametersDirty();
}

void WaterReflectionMaterial::SetAlphaStepMultiplier(
    float alpha_step_multiplier) {
  alpha_step_multiplier_ = alpha_step_multiplier;
  MarkParametersDirty();
}

void WaterReflectionMaterial::SetNormalZ(float normal_z) {
  normal_z_ = normal_z;
  MarkParametersDirty();
}

void WaterReflectionMaterial::SetNormalBoundary(float normal_boundary) {
  normal_boundary_ = normal_boundary;
  MarkParametersDirty();
}

void WaterReflectionMaterial::SetAlphaStepU(imp::float4 alpha_step_u) {}
void WaterReflectionMaterial::SetAlphaStepV(imp::float4 alpha_step_v) {}

}  // namespace android_xr
