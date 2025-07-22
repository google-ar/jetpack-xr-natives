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

#ifndef VR_ANDROID_XR_SPLIT_ENGINE_MATERIALS_SVXR_PLANE_MATERIAL_H_
#define VR_ANDROID_XR_SPLIT_ENGINE_MATERIALS_SVXR_PLANE_MATERIAL_H_

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
class SVXRPlaneMaterial : public imp::split_engine::SplitEngineMaterial {
 public:
  static imp::Future<std::unique_ptr<SVXRPlaneMaterial>> Create(
      imp::BaseView& view);

  ~SVXRPlaneMaterial() override;

  void SetHighlightPoint(const imp::float3& highlight_point);
  void SetDotPattern(imp::OwnedOrBorrowedTexturePtr dot_pattern);
  void SetPlaneControl(const imp::float3& plane_control);
  void SetFalloffColor(const imp::float4& falloff_color);
  void SetCutoffColor(const imp::float4& cutoff_color);
  void SetUVScale(const imp::float2& uv_scale);

 protected:
  flatbuffers::Offset<void> SerializeParameters(
      flatbuffers::FlatBufferBuilder& fbb,
      imp::split_engine::BuiltInTextureParameterCreator&
          texture_parameter_creator) const override;

 private:
  SVXRPlaneMaterial(
      imp::BaseView& view,
      imp::split_engine::PlaceholderOrBuiltInMaterialPtr material);

  std::optional<android_xr::schemas::Float3> highlight_point_;
  imp::OwnedOrBorrowedTexturePtr dot_pattern_;
  std::optional<android_xr::schemas::Float3> plane_control_;
  std::optional<android_xr::schemas::Float4> falloff_color_;
  std::optional<android_xr::schemas::Float4> cutoff_color_;
  std::optional<android_xr::schemas::Float2> uv_scale_;
};

}  // namespace android_xr

#endif  // VR_ANDROID_XR_SPLIT_ENGINE_MATERIALS_SVXR_PLANE_MATERIAL_H_
