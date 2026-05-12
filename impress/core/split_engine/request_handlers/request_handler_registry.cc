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
#include "core/split_engine/request_handlers/request_handler_registry.h"

#include <functional>
#include <memory>
#include <type_traits>

#include "absl/base/no_destructor.h"
#include "absl/container/flat_hash_map.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "core/common/small_source_location.h"
#include "core/split_engine/request_handlers/request_handler.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

namespace {
absl::flat_hash_map<android_xr::schemas::RequestTypes,
                    std::unique_ptr<RequestHandler>>&
GetRequestHandlers() {
  static absl::NoDestructor<absl::flat_hash_map<
      android_xr::schemas::RequestTypes, std::unique_ptr<RequestHandler>>>
      request_handlers;
  return *request_handlers;
}
}  // namespace

absl::StatusOr<RequestHandler&> RequestHandlerRegistry::Get(
    android_xr::schemas::RequestTypes request_type) {
  absl::flat_hash_map<android_xr::schemas::RequestTypes,
                      std::unique_ptr<RequestHandler>>& request_handlers =
      GetRequestHandlers();
  auto it = request_handlers.find(request_type);
  if (it != request_handlers.end()) {
    return *it->second;
  }
  return absl::UnimplementedError(absl::StrCat(
      "Request handler not found for type: ",
      EnumNameRequestTypes(request_type), " (",
      static_cast<std::underlying_type_t<android_xr::schemas::RequestTypes>>(
          request_type),
      ")"));
}

bool RequestHandlerRegistry::RegisterOrDie(
    android_xr::schemas::RequestTypes type,
    std::function<std::unique_ptr<RequestHandler>()> handler_factory,
    SmallSourceLocation loc) {
  IMP_LOG(imp::INFO) << "Registering request handler for type: "
            << EnumNameRequestTypes(type);
  auto [it, inserted] = GetRequestHandlers().insert({type, handler_factory()});
  if (!inserted) {
    IMP_LOG(imp::FATAL)
        << "Request handler already registered for type: "
        << EnumNameRequestTypes(type) << " ("
        << static_cast<
               std::underlying_type_t<android_xr::schemas::RequestTypes>>(type)
        << ") registered at " << loc.GetFileName() << ":" << loc.GetLineNumber()
        << ".";
  }
  return true;
}
}  // namespace imp::split_engine
