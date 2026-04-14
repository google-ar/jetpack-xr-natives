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

#include "split_engine/materials/svxr_footprint_material.h"

#include <memory>
#include <optional>
#include <utility>

#include "absl/memory/memory.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/materials/material.h"
#include "core/math/vec.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_builtin_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace android_xr {

imp::Future<std::unique_ptr<SVXRFootprintMaterial>>
SVXRFootprintMaterial::Create(imp::BaseView& view) {
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterial0d0cb9aa>
      spec_offset = android_xr::schemas::CreateBuiltInMaterial0d0cb9aa(*fbb);
  return imp::split_engine::SplitEngineBuiltinMaterial::RequestBuiltInMaterial(
             view, std::move(fbb),
             android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterial0d0cb9aa,
             spec_offset.Union())
      .Then([&view](imp::OwnedMaterialPtr material) {
        return absl::WrapUnique(
            new SVXRFootprintMaterial(view, std::move(material)));
      });
}

SVXRFootprintMaterial::SVXRFootprintMaterial(imp::BaseView& view,
                                             imp::OwnedMaterialPtr material)
    : SplitEngineBuiltinMaterial(
          view,
          android_xr::schemas::BuiltInMaterialParameters::
              BuiltInMaterial0d0cb9aaParameters,
          std::move(material)) {}

SVXRFootprintMaterial::~SVXRFootprintMaterial() { Cleanup(); }

flatbuffers::Offset<void> SVXRFootprintMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  // TODO: (broken link) - Change the parameters to all be optional and only
  // serialize the ones that are set / changed.
  return android_xr::schemas::CreateBuiltInMaterial0d0cb9aaParameters(
             fbb, imp::split_engine::PointerFromOptional(primary_touch_point_),
             imp::split_engine::PointerFromOptional(touch_control_),
             imp::split_engine::PointerFromOptional(touch_response_),
             imp::split_engine::PointerFromOptional(secondary_touch_point_),
             imp::split_engine::PointerFromOptional(falloff_color_),
             imp::split_engine::PointerFromOptional(cutoff_color_))
      .Union();
}

void SVXRFootprintMaterial::SetPrimaryTouchPoint(
    imp::float3 primary_touch_point) {
  primary_touch_point_ = imp::split_engine::Pack(primary_touch_point);
  MarkParametersDirty();
}
void SVXRFootprintMaterial::SetTouchControl(imp::float2 touch_control) {
  touch_control_ = imp::split_engine::Pack(touch_control);
  MarkParametersDirty();
}
void SVXRFootprintMaterial::SetTouchResponse(imp::float4 touch_response) {
  touch_response_ = imp::split_engine::Pack(touch_response);
  MarkParametersDirty();
}
void SVXRFootprintMaterial::SetSecondaryTouchPoint(
    imp::float3 secondary_touch_point) {
  secondary_touch_point_ = imp::split_engine::Pack(secondary_touch_point);
  MarkParametersDirty();
}
void SVXRFootprintMaterial::SetFalloffColor(imp::float4 falloff_color) {
  falloff_color_ = imp::split_engine::Pack(falloff_color);
  MarkParametersDirty();
}
void SVXRFootprintMaterial::SetCutoffColor(imp::float4 cutoff_color) {
  cutoff_color_ = imp::split_engine::Pack(cutoff_color);
  MarkParametersDirty();
}

}  // namespace android_xr
