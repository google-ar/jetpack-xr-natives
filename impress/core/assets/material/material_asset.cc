// Copyright 2024 Google LLC
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

#include "core/assets/material/material_asset.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "core/assets/material/material_helpers.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp {
namespace {

filament::Material::Builder::ShadowSamplingQuality
ToFilamentShadowSamplingQuality(
    MaterialPreCompileOptions::ShadowSamplingQuality quality) {
  switch (quality) {
    case MaterialPreCompileOptions::ShadowSamplingQuality::
        SHADOW_SAMPLING_QUALITY_UNSPECIFIED:
      return filament::Material::Builder::ShadowSamplingQuality::LOW;
    case MaterialPreCompileOptions::ShadowSamplingQuality::
        SHADOW_SAMPLING_QUALITY_HARD:
      return filament::Material::Builder::ShadowSamplingQuality::HARD;
    case MaterialPreCompileOptions::ShadowSamplingQuality::
        SHADOW_SAMPLING_QUALITY_LOW:
      return filament::Material::Builder::ShadowSamplingQuality::LOW;
  }
}

}  // namespace

// Specifies which material variants to precompile and if the loading needs to
// wait for the high priority variants' compilation.
const MaterialPreCompileOptions& MaterialAsset::kDefaultPreCompileOptions =
    *new MaterialPreCompileOptions{};

filament::Material* MaterialAsset::BuildMaterial(
    filament::Engine& engine, const uint8_t* data, size_t size,
    const MaterialPreCompileOptions& material_pre_compile_options) {
  filament::Material::Builder builder;
  builder.package(data, size);
  for (const auto& constant : material_pre_compile_options.constants) {
    switch (constant.value.index()) {
      case MaterialPreCompileConstant::kValue_IntValue:
        builder.constant(constant.name.c_str(), *constant.int_value());
        break;
      case MaterialPreCompileConstant::kValue_FloatValue:
        builder.constant(constant.name.c_str(), *constant.float_value());
        break;
      case MaterialPreCompileConstant::kValue_BoolValue:
        builder.constant(constant.name.c_str(), *constant.bool_value());
        break;
    }
  }
  builder.sphericalHarmonicsBandCount(
      material_pre_compile_options.spherical_harmonics_bands.value_or(3));
  builder.shadowSamplingQuality(ToFilamentShadowSamplingQuality(
      material_pre_compile_options.shadow_sampling_quality));
  return builder.build(engine);
}

Future<std::unique_ptr<MaterialAsset>> MaterialAsset::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future,
    MaterialPreCompileOptions material_pre_compile_options) {
  IMP_TRACE();
  return resource_future.Then([view, material_pre_compile_options = std::move(
                                         material_pre_compile_options)](
                                  resources::Resource resource)
                                  -> Future<std::unique_ptr<MaterialAsset>> {
    IMP_TRACE_BLOCK("Then");
    auto material_asset = std::make_unique<MaterialAsset>(
        view, resource.GetData(), material_pre_compile_options);

    if (view->GetEngineConfig().disableParallelShaderCompile) {
      return Future<std::unique_ptr<MaterialAsset>>(std::move(material_asset));
    }
    // Pre compiles variants of the material.
    Future<absl::Status> pre_compile_status =
        material_helpers::PreCompileMaterial(material_asset->material_,
                                             material_pre_compile_options);

    // Waits for high priority variants' compilation to complete before
    // returning the MaterialAsset.
    return pre_compile_status.Then(
        [asset = std::move(material_asset)]() mutable {
          return std::move(asset);
        });
  });
}

MaterialAsset::MaterialAsset(
    BaseView* view, const BufferAccess& data,
    const MaterialPreCompileOptions& material_pre_compile_options)
    : view_(view) {
  material_ = BuildMaterial(*view_->GetSharedEngine(), data.Data(), data.Size(),
                            material_pre_compile_options);
  if (auto* serializer = view_->GetSplitEngineSerializer()) {
    serializer->AddMaterial(material_, data);
  }
}

MaterialAsset::~MaterialAsset() {
  if (view_ != nullptr && view_->GetSharedEngine() != nullptr &&
      material_ != nullptr) {
    if (auto* serializer = view_->GetSplitEngineSerializer()) {
      serializer->RemoveMaterial(material_);
    }
    view_->GetSharedEngine()->destroy(material_);
    material_ = nullptr;
  }
}

}  // namespace imp
