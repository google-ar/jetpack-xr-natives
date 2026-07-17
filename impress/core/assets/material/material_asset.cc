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
#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "filament/filament/include/filament/Engine.h"
#include "filament/filament/include/filament/Material.h"
#include "filament/filament/include/filament/MaterialInstance.h"
#include "core/assets/material/material_helpers.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/trace.h"
#include "core/resources/resource_manager.h"
#include "core/view/base_view.h"

namespace imp {
namespace {

bool IsPlaceholderSplitEngineMaterial(const filament::Material* material) {
  if (material) {
    // TODO: (broken link) - Find a better way to check the placeholder material.
    return strcmp(material->getName(), "Split Engine Placeholder") == 0;
  }
  return false;
}

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

filament::Material::UboBatchingMode ToFilamentUboBatchingMode(
    MaterialPreCompileOptions::UboBatchingOption option) {
  switch (option) {
    case MaterialPreCompileOptions::UboBatchingOption::
        UBO_BATCHING_OPTION_UNSPECIFIED:
    case MaterialPreCompileOptions::UboBatchingOption::
        UBO_BATCHING_OPTION_DEFAULT:
      return filament::Material::UboBatchingMode::DEFAULT;
    case MaterialPreCompileOptions::UboBatchingOption::
        UBO_BATCHING_OPTION_DISABLED:
      return filament::Material::UboBatchingMode::DISABLED;
  }
}

Future<std::unique_ptr<MaterialAsset>> CreateMaterialFromResource(
    BaseView* view, MaterialPreCompileOptions material_pre_compile_options,
    resources::Resource resource) {
  IMP_TRACE();
  BufferAccess compiled_material_data = resource.GetData();

  if (compiled_material_data.Empty()) {
    return Future<std::unique_ptr<MaterialAsset>>(
        absl::InternalError("MaterialAsset has no compiled data."));
  }

  // Parse and create the filament material from the cmat data.
  filament::Material* filament_material = MaterialAsset::BuildMaterial(
      *view->GetSharedEngine(), compiled_material_data.Data(),
      compiled_material_data.Size(), material_pre_compile_options);

  if (filament_material == nullptr) {
    return Future<std::unique_ptr<MaterialAsset>>(
        absl::InternalError("Failed to build material from compiled data."));
  }

  // MaterialAsset is a thin wrapper around the filament material.
  auto material_asset =
      std::make_unique<MaterialAsset>(view, filament_material);

  if (view->GetSplitEngineSerializer()) {
    // Material source is available when built with flag:
    // --define=IMP_SPLIT_ENGINE_ALLOW_EXPERIMENTAL_APIS=1
    // or when imp_material sets include_source_mat true.
    std::string material_source = std::string(filament_material->getSource());

    // The placeholder material is used for built-in materials as a kind of
    // app-side handle. The built-in materials expect it to be loaded as a
    // normal app-side material. Because of this, we should not request it
    // from split engine.
    if (!IsPlaceholderSplitEngineMaterial(filament_material) &&
        !material_source.empty()) {
      Future<absl::Status> request_material_status =
          view->GetSplitEngineSerializer()->RequestCustomFilamentMaterial(
              material_source, filament_material, material_pre_compile_options);

      return request_material_status.Then(
          [material_asset = std::move(material_asset)]() mutable {
            return std::move(material_asset);
          });
    }

    return Future<std::unique_ptr<MaterialAsset>>(std::move(material_asset));
  } else {
    if (view->GetEngineConfig().disableParallelShaderCompile) {
      return Future<std::unique_ptr<MaterialAsset>>(std::move(material_asset));
    }

    // Pre compiles variants of the material.
    Future<absl::Status> pre_compile_status =
        material_helpers::PreCompileMaterial(filament_material,
                                             material_pre_compile_options);

    // Waits for high priority variants' compilation to complete before
    // returning the MaterialAsset.
    return pre_compile_status.Then(
        [asset = std::move(material_asset)]() mutable {
          return std::move(asset);
        });
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
  builder.uboBatching(ToFilamentUboBatchingMode(
      material_pre_compile_options.ubo_batching_option));
  return builder.build(engine);
}

Future<std::unique_ptr<MaterialAsset>> MaterialAsset::Load(
    BaseView* view, absl::string_view asset_url,
    Future<resources::Resource> resource_future,
    MaterialPreCompileOptions material_pre_compile_options) {
  return resource_future.Then([view, material_pre_compile_options = std::move(
                                         material_pre_compile_options)](
                                  resources::Resource resource) mutable
                                  -> Future<std::unique_ptr<MaterialAsset>> {
    return CreateMaterialFromResource(
        view, std::move(material_pre_compile_options), std::move(resource));
  });
}

MaterialAsset::MaterialAsset(BaseView* view, filament::Material* material)
    : view_(view), material_(material) {}

MaterialAsset::~MaterialAsset() {
  if (view_ != nullptr && view_->GetSharedEngine() != nullptr &&
      material_ != nullptr) {
    if (auto serializer = view_->GetSplitEngineSerializer()) {
      if (!IsPlaceholderSplitEngineMaterial(material_) &&
          !material_->getSource().empty()) {
        serializer->RemoveMaterial(material_);
      }
    }
    view_->GetSharedEngine()->destroy(material_);
    material_ = nullptr;
  }
}

}  // namespace imp
