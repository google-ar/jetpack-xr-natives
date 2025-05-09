/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_WEB_BASE_MESSAGE_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_WEB_BASE_MESSAGE_HANDLER_H_

#include <optional>
#include <vector>

#include "absl/types/optional.h"
#include "core/async/future.h"
#include "core/proto/any.proto.imp.h"
#include "core/view/scripting/script_message_handler.h"

namespace imp::scripting {

// Base class for all MessageHandlers.
// Used by ScriptingSystem to store and access MessageHandlers without needing
// access to the concrete types of requests and responses.
//
// It is not recommended to inherit from this directly, instead inherit from the
// templated MessageHandler type or MultiMessageHandler.
struct BaseMessageHandler {
  using Any = google::protobuf::imp_proto::Any;

  // Generic return type used by ImpWeb - An optional proto response.
  using OptionalResponse = absl::optional<Any>;

  BaseMessageHandler() = default;
  virtual ~BaseMessageHandler() = default;

  BaseMessageHandler(const BaseMessageHandler&) = delete;
  BaseMessageHandler& operator=(const BaseMessageHandler&) = delete;

  // Implemented by subclasses to return the list of protobuf type urls for
  // message types that the handler supports.
  virtual std::vector<absl::string_view> GetSupportedRequestTypeUrls() = 0;

  // Implemented by subclasses to handle requests.
  // Handles an incoming message from the script side of the scripting system.
  // Returns a response message corresponding to the request type.
  //
  // The future return value represents one of three outcomes:
  // 1) Status  - an error occurred in the handler.
  // 2) nullopt - success with no return value.
  // 3) Any     - success and the Any is the proto to return as the response.
  virtual Future<OptionalResponse> HandleAnyMessage(const Any& message,
                                                    const PlatformArgs& args,
                                                    PlatformArgs& out) = 0;

  // Handles an incoming message from the script side of the scripting system.
  // Returns a response message corresponding to the request type.
  //
  // The future return value represents one of three outcomes:
  // 1) Status  - an error occurred in the handler.
  // 2) nullopt - success with no return value.
  // 3) Any     - success and the Any is the proto to return as the response.
  Future<OptionalResponse> HandleAnyMessage(const Any& message) {
    PlatformArgs out;
    return HandleAnyMessage(message, PlatformArgs(), out);
  }

  template <typename T>
  absl::optional<absl::Status> CheckInvalidNodeTarget(
      const T& message_with_target) {
    if (!message_with_target.target) {
      return absl::InvalidArgumentError("Invalid target Node");
    }
    return {};
  }
};

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_WEB_BASE_MESSAGE_HANDLER_H_
