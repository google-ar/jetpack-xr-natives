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

#include "core/split_engine/materials/split_engine_custom_material_factory.h"

#include <functional>
#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Material.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/materials/compiler/runtime_material_compiler_creator.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/split_engine_filament_resource_ptrs.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

SplitEngineCustomMaterialFactory::SplitEngineCustomMaterialFactory(
    BaseView& view)
    : view_(view) {}

// Handles a request to create a custom material.
Future<OwnedFilamentMaterialPtr>
SplitEngineCustomMaterialFactory::CreateCustomMaterial(
    BaseView& view,
    const android_xr::schemas::AddCustomMaterialRequest& request) {
  switch (request.spec_type()) {
    case android_xr::schemas::CustomMaterialSpec::NONE:
      return Future<OwnedFilamentMaterialPtr>(
          absl::InvalidArgumentError("Custom material spec type is not set."));
    case android_xr::schemas::CustomMaterialSpec::FilamentMaterialSpec:
      if (!request.spec()) {
        return Future<OwnedFilamentMaterialPtr>(
            absl::InvalidArgumentError("Custom material spec is not set."));
      }
      return CreateFilamentMaterial(*request.spec_as_FilamentMaterialSpec());
    default:
      return Future<OwnedFilamentMaterialPtr>(absl::UnimplementedError(
          "Custom material spec type is not supported."));
  }
}

Future<std::reference_wrapper<RuntimeMaterialCompiler>>
SplitEngineCustomMaterialFactory::GetOrCreateMaterialCompiler() {
  if (!material_compiler_future_) {
    material_compiler_future_ =
        RuntimeMaterialCompilerCreator::Create(view_).Then(
            [this](std::unique_ptr<RuntimeMaterialCompiler> compiler) {
              material_compiler_ = std::move(compiler);
              return std::ref(*material_compiler_);
            });
  }
  return *material_compiler_future_;
}

Future<OwnedFilamentMaterialPtr>
SplitEngineCustomMaterialFactory::CreateFilamentMaterial(
    const android_xr::schemas::FilamentMaterialSpec& spec) {
  if (!spec.source() || spec.source()->string_view().empty()) {
    return Future<OwnedFilamentMaterialPtr>(absl::InvalidArgumentError(
        "Filament custom material source is empty."));
  }
  IMP_TRACE();

#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(WASM) || IMP_PLATFORM(IOS) || \
    IMP_PLATFORM(IOS_SIMULATOR)
  auto platform = RuntimeMaterialCompiler::Platform::Mobile;
#else
  auto platform = RuntimeMaterialCompiler::Platform::Desktop;
#endif

#if IMP_MATERIAL_API(OPENGL)
  auto target_api = RuntimeMaterialCompiler::TargetApi::OpenGL;
#elif IMP_MATERIAL_API(VULKAN)
  auto target_api = RuntimeMaterialCompiler::TargetApi::Vulkan;
#elif IMP_MATERIAL_API(METAL)
  auto target_api = RuntimeMaterialCompiler::TargetApi::Metal;
#endif

  return GetOrCreateMaterialCompiler().Then(
      [&spec, platform, target_api](RuntimeMaterialCompiler& compiler)
          -> Future<OwnedFilamentMaterialPtr> {
        MaterialPreCompileOptions precompile_options;
        if (spec.precompile_options()) {
          precompile_options = UnPack(*spec.precompile_options());
        }
        absl::Time start = absl::Now();
        return compiler
            .CompileMaterial(spec.source()->string_view(), platform, target_api,
                             precompile_options)
            .Then([start](filament::Material* material) {
              absl::Duration duration = absl::Now() - start;
              IMP_LOG(imp::INFO) << "Compiled material '" << material->getName()
                        << "' in " << duration;
              return OwnedFilamentMaterialPtr{material};
            });
      });
}

}  // namespace imp::split_engine
