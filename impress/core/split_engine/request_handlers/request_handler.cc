/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "core/split_engine/request_handlers/request_handler.h"

#include "absl/status/status.h"
#include "flatbuffers/buffer.h"
#include "flatbuffers/detached_buffer.h"
#include "flatbuffers/flatbuffer_builder.h"
#include "core/split_engine/flatbuffer_utils.h"
#include "split_engine/schemas/split_engine_ipc_generated.h"

namespace imp::split_engine {

flatbuffers::DetachedBuffer RequestHandler::SerializeStatusResponse(
    absl::Status status) {
  flatbuffers::FlatBufferBuilder fbb;
  if (status.ok()) {
    // Create an empty response to signal an OK status (similar to
    // absl::StatusOr<T>).
    fbb.Finish(android_xr::schemas::CreateResponse(fbb));
    return fbb.Release();
  }
  fbb.Finish(android_xr::schemas::CreateResponse(
      fbb, android_xr::schemas::ResponseTypes::ErrorResponse,
      android_xr::schemas::CreateErrorResponseDirect(
          fbb, StatusToErrorCode(status), status.message().data())
          .Union()));
  return fbb.Release();
}

}  // namespace imp::split_engine
