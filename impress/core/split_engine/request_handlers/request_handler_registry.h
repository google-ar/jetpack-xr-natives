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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_REQUEST_HANDLERS_REQUEST_HANDLER_REGISTERER_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_REQUEST_HANDLERS_REQUEST_HANDLER_REGISTERER_H_

#include <functional>
#include <memory>

#include "absl/status/statusor.h"
#include "core/common/small_source_location.h"
#include "core/split_engine/request_handlers/request_handler.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

// The RequestHandlerRegister is responsible for registering request handlers
// for the Split Engine.
struct RequestHandlerRegistry {
  // Returns a request handler for the given request type.
  static absl::StatusOr<RequestHandler&> Get(
      android_xr::schemas::RequestTypes request_type);

  // Registers a request handler for the given request type. It will cause a
  // crash if a handler is registered twice for the same request type.
  // This must be called at the global scope, for example in a .cc file.
  // Returns a boolean to prevent the compiler from optimizing the call away.
  static bool RegisterOrDie(
      android_xr::schemas::RequestTypes type,
      std::function<std::unique_ptr<RequestHandler>()> handler_factory,
      SmallSourceLocation loc = SmallSourceLocation::Current());
};
}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_REQUEST_HANDLERS_REQUEST_HANDLER_REGISTERER_H_
