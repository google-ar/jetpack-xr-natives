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

#include "core/split_engine/request_handlers/built_in_material_request_handler.h"

#include <functional>
#include <memory>
#include <utility>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_format.h"
#include "flatbuffers/detached_buffer.h"
#include "core/async/future.h"
#include "core/split_engine/materials/builtin/builtin_material.h"
#include "core/split_engine/materials/split_engine_builtin_material_factory.h"
#include "core/split_engine/request_handlers/request_handler_registry.h"
#include "core/split_engine/split_engine_renderer_context.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"
#include "split_engine/schemas/split_engine_material_generated.h"

namespace imp::split_engine {

Future<flatbuffers::DetachedBuffer>
BuiltInMaterialRequestHandler::HandleRequest(
    SharedContext& shared_context, AppContext& app_context,
    const android_xr::schemas::Request& request) {
  const android_xr::schemas::BuiltInMaterialRequest* built_in_material_request =
      request.request_as_BuiltInMaterialRequest();
  if (!built_in_material_request) {
    return Future<flatbuffers::DetachedBuffer>{
        absl::InvalidArgumentError("Built-in material request is not set.")};
  }

  if (app_context.material_instances.contains(
          built_in_material_request->material_instance_id())) {
    return Future<flatbuffers::DetachedBuffer>{absl::AlreadyExistsError(
        absl::StrFormat("Material instance with id %d already exists.",
                        built_in_material_request->material_instance_id()))};
  }

  return shared_context.builtin_material_factory
      .HandleCreateRequest(app_context.bridge_id, *built_in_material_request)
      .Then(
          [&app_context, material_instance_id =
                             built_in_material_request->material_instance_id()](
              BuiltInMaterialPtr material)
              -> absl::StatusOr<flatbuffers::DetachedBuffer> {
            app_context.material_instances[material_instance_id] =
                std::move(material);
            return SerializeStatusResponse(absl::OkStatus());
          });
}

// Register the handler for the BuiltInMaterialRequest.
const bool kUnused = RequestHandlerRegistry::RegisterOrDie(
    android_xr::schemas::RequestTypes::BuiltInMaterialRequest,
    []() { return std::make_unique<BuiltInMaterialRequestHandler>(); });

}  // namespace imp::split_engine
