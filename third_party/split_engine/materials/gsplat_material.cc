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

#include "split_engine/materials/gsplat_material.h"

#include <sys/stat.h>

#include <memory>
#include <utility>

#include "absl/memory/memory.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/async/future.h"
#include "core/split_engine/materials/builtin_texture_parameter_creator.h"
#include "core/split_engine/materials/split_engine_material.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace android_xr {

imp::Future<std::unique_ptr<GsplatMaterial>> GsplatMaterial::Create(
    imp::BaseView& view) {
  auto fbb = std::make_unique<flatbuffers::FlatBufferBuilder>();
  flatbuffers::Offset<android_xr::schemas::BuiltInMaterialGsplatSpec>
      spec_offset = android_xr::schemas::CreateBuiltInMaterialGsplatSpec(*fbb);
  return RequestBuiltInMaterial(view, std::move(fbb),
                                android_xr::schemas::BuiltInMaterialSpec::
                                    BuiltInMaterialGsplatSpec,
                                spec_offset.Union())
      .Then([&view](
                imp::split_engine::PlaceholderOrBuiltInMaterialPtr material) {
        return absl::WrapUnique(new GsplatMaterial(view, std::move(material)));
      });
}

GsplatMaterial::GsplatMaterial(
    imp::BaseView& view,
    imp::split_engine::PlaceholderOrBuiltInMaterialPtr material)
    : SplitEngineMaterial(view,
                          android_xr::schemas::BuiltInMaterialParameters::
                              BuiltInMaterialGsplatParameters,
                          std::move(material)) {}

GsplatMaterial::~GsplatMaterial() { Cleanup(); }

flatbuffers::Offset<void> GsplatMaterial::SerializeParameters(
    flatbuffers::FlatBufferBuilder& fbb,
    imp::split_engine::BuiltInTextureParameterCreator&
        texture_parameter_creator) const {
  return android_xr::schemas::CreateBuiltInMaterialGsplatParameters(fbb)
      .Union();
}

}  // namespace android_xr
