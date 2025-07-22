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

#ifndef THIRD_PARTY_SPLIT_ENGINE_MATERIALS_SVXR_FOOTPRINT_MATERIAL_H_
#define THIRD_PARTY_SPLIT_ENGINE_MATERIALS_SVXR_FOOTPRINT_MATERIAL_H_

#include <memory>
#include <optional>

#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "imp.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace android_xr {

// Displays a 3D texture with support for various formats and parameters.
class SVXRFootprintMaterial : public imp::split_engine::SplitEngineMaterial {
 public:
  static imp::Future<std::unique_ptr<SVXRFootprintMaterial>> Create(
      imp::BaseView& view);

  ~SVXRFootprintMaterial() override;

  void SetPrimaryTouchPoint(imp::float3 primary_touch_point);
  void SetTouchControl(imp::float2 touch_control);
  void SetTouchResponse(imp::float4 touch_response);
  void SetSecondaryTouchPoint(imp::float3 secondary_touch_point);
  void SetFalloffColor(imp::float4 falloff_color);
  void SetCutoffColor(imp::float4 cutoff_color);

 protected:
  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

 private:
  SVXRFootprintMaterial(
      imp::BaseView& view,
      imp::split_engine::PlaceholderOrBuiltInMaterialPtr material);

  std::optional<android_xr::schemas::Float3> primary_touch_point_;
  std::optional<android_xr::schemas::Float2> touch_control_;
  std::optional<android_xr::schemas::Float4> touch_response_;
  std::optional<android_xr::schemas::Float3> secondary_touch_point_;
  std::optional<android_xr::schemas::Float4> falloff_color_;
  std::optional<android_xr::schemas::Float4> cutoff_color_;
};

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_MATERIALS_SVXR_FOOTPRINT_MATERIAL_H_
