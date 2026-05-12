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

#include "core/split_engine/request_handlers/add_custom_material_request_handler.h"

#include <functional>
#include <memory>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "filament/filament/include/filament/Material.h"
#include "flatbuffers/detached_buffer.h"
#include "core/assets/material/material_load_options.proto.imp.h"
#include "core/async/future.h"
#include "core/common/trace.h"
#include "core/config.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/materials/compiler/runtime_material_compiler_creator.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "core/split_engine/request_handlers/request_handler_registry.h"
#include "core/split_engine/split_engine_filament_resource_ptrs.h"
#include "core/split_engine/split_engine_renderer_context.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

Future<flatbuffers::DetachedBuffer> AddCustomMaterialHandler::HandleRequest(
    SharedContext& shared_context, AppContext& app_context,
    const android_xr::schemas::Request& request) {
  const android_xr::schemas::AddCustomMaterialRequest* custom_material_request =
      request.request_as_AddCustomMaterialRequest();
  if (!custom_material_request) {
    return Future<flatbuffers::DetachedBuffer>{
        absl::InvalidArgumentError("Custom material request is not set.")};
  }
  if (!custom_material_request->spec()) {
    return Future<flatbuffers::DetachedBuffer>{
        absl::InvalidArgumentError("Custom material spec is not set.")};
  }

  if (app_context.materials.contains(custom_material_request->material_id())) {
    return Future<flatbuffers::DetachedBuffer>{absl::AlreadyExistsError(
        absl::StrFormat("Material with id %d already exists.",
                        custom_material_request->material_id()))};
  }

  Future<OwnedFilamentMaterialPtr> filament_material_future;
  switch (custom_material_request->spec_type()) {
    case android_xr::schemas::CustomMaterialSpec::FilamentMaterialSpec:
      filament_material_future = CreateFilamentMaterial(
          app_context.view,
          *custom_material_request->spec_as_FilamentMaterialSpec());
      break;
    default:
      return Future<flatbuffers::DetachedBuffer>{absl::UnimplementedError(
          "Custom material spec type is not supported.")};
  }

  return filament_material_future.Then(
      [&app_context, material_id = custom_material_request->material_id()](
          OwnedFilamentMaterialPtr material)
          -> absl::StatusOr<flatbuffers::DetachedBuffer> {
        app_context.materials[material_id] = std::move(material);
        return SerializeStatusResponse(absl::OkStatus());
      });
}

Future<std::reference_wrapper<RuntimeMaterialCompiler>>
AddCustomMaterialHandler::GetOrCreateMaterialCompiler(BaseView& view) {
  if (material_compiler_ == nullptr) {
    return RuntimeMaterialCompilerCreator::Create(view).Then(
        [this](std::unique_ptr<RuntimeMaterialCompiler> compiler) {
          material_compiler_ = std::move(compiler);
          material_compiler_future_.Return(*material_compiler_);
          return material_compiler_future_;
        });
  } else {
    return material_compiler_future_;
  }
}

Future<OwnedFilamentMaterialPtr>
AddCustomMaterialHandler::CreateFilamentMaterial(
    BaseView& view, const android_xr::schemas::FilamentMaterialSpec& spec) {
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

  return GetOrCreateMaterialCompiler(view).Then(
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

// Register the handler for the AddCustomMaterialRequest.
const bool kUnused = RequestHandlerRegistry::RegisterOrDie(
    android_xr::schemas::RequestTypes::AddCustomMaterialRequest,
    []() { return std::make_unique<AddCustomMaterialHandler>(); });

}  // namespace imp::split_engine
