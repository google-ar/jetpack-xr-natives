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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_REQUEST_HANDLERS_ADD_CUSTOM_MATERIAL_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_REQUEST_HANDLERS_ADD_CUSTOM_MATERIAL_HANDLER_H_

#include <functional>
#include <memory>

#include "flatbuffers/detached_buffer.h"
#include "core/async/future.h"
#include "core/materials/compiler/runtime_material_compiler.h"
#include "core/split_engine/request_handlers/request_handler.h"
#include "core/split_engine/split_engine_filament_resource_ptrs.h"
#include "core/split_engine/split_engine_renderer_context.h"
#include "core/view/base_view.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

// Handler for the AddCustomMaterialRequest, which creates a custom materials
// from specified specs.
// It registers itself to the SplitEngineRenderer statically.
class AddCustomMaterialHandler : public RequestHandler {
 public:
  // Handles a request from the app.
  //
  // The `app_context` is the AppContext of the app that is making the request.
  // The `shared_context` is the SharedContext of the SplitEngineRenderer.
  // The `request` is the request that is being handled.
  //
  // Returns a Future of the response buffer.
  Future<flatbuffers::DetachedBuffer> HandleRequest(
      SharedContext& shared_context, AppContext& app_context,
      const android_xr::schemas::Request& request) override;

 private:
  Future<std::reference_wrapper<RuntimeMaterialCompiler>>
  GetOrCreateMaterialCompiler(BaseView& view);

  Future<OwnedFilamentMaterialPtr> CreateFilamentMaterial(
      BaseView& view, const android_xr::schemas::FilamentMaterialSpec& spec);

  Future<std::reference_wrapper<RuntimeMaterialCompiler>>
      material_compiler_future_;
  std::unique_ptr<RuntimeMaterialCompiler> material_compiler_;
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_REQUEST_HANDLERS_ADD_CUSTOM_MATERIAL_HANDLER_H_
