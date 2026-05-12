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

#include "split_engine/materials/svxr_plane_material.h"

#include <memory>
#include <optional>
#include <utility>

#include "absl/memory/memory.h"
#include "filament/filament/include/filament/TextureSampler.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace android_xr {

imp::Future<std::unique_ptr<SVXRPlaneMaterial>> SVXRPlaneMaterial::Create(
    imp::BaseView& view) {
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterialbd7fe08c>
      spec_offset = android_xr::schemas::CreateBuiltInMaterialbd7fe08c(*fbb);
  return imp::split_engine::SplitEngineBuiltinMaterial::RequestBuiltInMaterial(
             view, std::move(fbb),
             android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialbd7fe08c,
             spec_offset.Union())
      .Then(
          [&view](imp::split_engine::PlaceholderOrBuiltInMaterialPtr material) {
            return absl::WrapUnique(
                new SVXRPlaneMaterial(view, std::move(material)));
          });
}

SVXRPlaneMaterial::SVXRPlaneMaterial(
    imp::BaseView& view,
    imp::split_engine::PlaceholderOrBuiltInMaterialPtr material)
    : SplitEngineBuiltinMaterial(
          view,
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterialbd7fe08cParameters,
          std::move(material)) {}

SVXRPlaneMaterial::~SVXRPlaneMaterial() { Cleanup(); }

flatbuffers::Offset<void> SVXRPlaneMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  flatbuffers::Offset<android_xr::schemas::BuiltInTextureParameter> dot_pattern;
  if (dot_pattern_) {
    std::optional<filament::TextureSampler> texture_sampler =
        filament::TextureSampler(
            filament::TextureSampler::MinFilter::LINEAR_MIPMAP_LINEAR,
            filament::TextureSampler::MagFilter::LINEAR,
            filament::TextureSampler::WrapMode::REPEAT);

    dot_pattern = texture_parameter_creator.Create(fbb, dot_pattern_.Borrow(),
                                                   texture_sampler);
  }

  return android_xr::schemas::CreateBuiltInMaterialbd7fe08cParameters(
             fbb, imp::split_engine::PointerFromOptional(highlight_point_),
             dot_pattern,
             imp::split_engine::PointerFromOptional(plane_control_),
             imp::split_engine::PointerFromOptional(falloff_color_),
             imp::split_engine::PointerFromOptional(cutoff_color_),
             imp::split_engine::PointerFromOptional(uv_scale_))
      .Union();
}

void SVXRPlaneMaterial::SetHighlightPoint(const imp::float3& highlight_point) {
  highlight_point_ = imp::split_engine::Pack(highlight_point);
  MarkParametersDirty();
}

void SVXRPlaneMaterial::SetDotPattern(
    imp::OwnedOrBorrowedTexturePtr dot_pattern) {
  dot_pattern_ = std::move(dot_pattern);
  MarkParametersDirty();
}

void SVXRPlaneMaterial::SetPlaneControl(const imp::float3& plane_control) {
  plane_control_ = imp::split_engine::Pack(plane_control);
  MarkParametersDirty();
}

void SVXRPlaneMaterial::SetFalloffColor(const imp::float4& falloff_color) {
  falloff_color_ = imp::split_engine::Pack(falloff_color);
  MarkParametersDirty();
}

void SVXRPlaneMaterial::SetCutoffColor(const imp::float4& cutoff_color) {
  cutoff_color_ = imp::split_engine::Pack(cutoff_color);
  MarkParametersDirty();
}

void SVXRPlaneMaterial::SetUVScale(const imp::float2& uv_scale) {
  uv_scale_ = imp::split_engine::Pack(uv_scale);
  MarkParametersDirty();
}

}  // namespace android_xr
