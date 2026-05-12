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

#ifndef THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_REQUEST_HANDLERS_TYPES_H_
#define THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_REQUEST_HANDLERS_TYPES_H_

#include "absl/status/status.h"
#include "flatbuffers/detached_buffer.h"
#include "core/async/future.h"
#include "core/split_engine/split_engine_renderer_context.h"
#include "core/split_engine/split_engine_shared_context.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

// Abstract base class for all renderer request handlers.
class RequestHandler {
 public:
  virtual ~RequestHandler() = default;

  // Handles a request, must be implemented by each handler.
  //
  // The `app_context` is the AppContext of the app that is making the request.
  // The `request` is the request that is being handled.
  //
  // Returns a Future of the response buffer.
  virtual Future<flatbuffers::DetachedBuffer> HandleRequest(
      SharedContext& shared_context, AppContext& app_context,
      const android_xr::schemas::Request& request) = 0;

 protected:
  static flatbuffers::DetachedBuffer SerializeStatusResponse(
      absl::Status status);
};

}  // namespace imp::split_engine

#endif  // THIRD_PARTY_IMPRESS_CORE_SPLIT_ENGINE_REQUEST_HANDLERS_TYPES_H_
