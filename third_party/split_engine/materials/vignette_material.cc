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

#include "split_engine/materials/vignette_material.h"

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"
#include "split_engine/schemas/split_engine_primitive_generated.h"

namespace android_xr {

constexpr imp::float4 kDefaultColor = {0.f, 0.f, 0.f, 1.f};
constexpr float kDefaultCoverage = 0.55f;
constexpr float kDefaultFeather = 0.15f;

imp::Future<std::unique_ptr<VignetteMaterial>> VignetteMaterial::Create(
    imp::BaseView& view) {
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterialE3ca0ab9>
      spec_offset = android_xr::schemas::CreateBuiltInMaterialE3ca0ab9(*fbb);
  return imp::split_engine::SplitEngineMaterial::RequestBuiltInMaterial(
             view, std::move(fbb),
             android_xr::schemas::BuiltInMaterialSpec::BuiltInMaterialE3ca0ab9,
             spec_offset.Union())
      .Then(
          [&view](imp::split_engine::PlaceholderOrBuiltInMaterialPtr material) {
            return absl::WrapUnique(
                new VignetteMaterial(view, std::move(material)));
          });
}

VignetteMaterial::VignetteMaterial(
    imp::BaseView& view,
    imp::split_engine::PlaceholderOrBuiltInMaterialPtr material)
    : SplitEngineMaterial(view,
                          android_xr::schemas::BuiltInMaterialParameters::
                              BuiltInMaterialE3ca0ab9Parameters,
                          std::move(material)) {}

void VignetteMaterial::SetColor(const imp::float4& color) {
  color_ = color;
  MarkParametersDirty();
}
void VignetteMaterial::SetCoverage(float coverage) {
  coverage_ = coverage;
  MarkParametersDirty();
}
void VignetteMaterial::SetFeather(float feather) {
  feather_ = feather;
  MarkParametersDirty();
}

flatbuffers::Offset<void> VignetteMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  android_xr::schemas::Float4 color =
      imp::split_engine::Pack(color_.value_or(kDefaultColor));
  // TODO: (broken link) - use the new BuiltInMaterialE3ca0ab9Parameters instead.
  return android_xr::schemas::CreateBuiltInMaterialE3ca0ab9Parameters(
             fbb, &color, coverage_.value_or(kDefaultCoverage),
             feather_.value_or(kDefaultFeather))
      .Union();
}

}  // namespace android_xr
