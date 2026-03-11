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
#include <type_traits>
#include <variant>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "absl/types/optional.h"
#include "core/async/future.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_writer.h"
#include "core/view/scripting/script_message_handler.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::scripting {

// Base class for all MessageHandlers.
// Used by ScriptingSystem to store and access MessageHandlers without needing
// access to the concrete types of requests and responses.
//
// It is not recommended to inherit from this directly, instead inherit from the
// templated MessageHandler type or MultiMessageHandler.
class BaseMessageHandler {
 public:
  using Any = google::protobuf::imp_proto::Any;

  // Generic return type for all message handlers.
  class Response {
   public:
    // Creates a Response from a value of valid type T, which could be one of
    // absl::Status, a proto message, or a platform-specific object (void*).
    // If status and not ok, return the status, else return an empty Response.
    // If the value is a proto message, packs it into an Any proto & Response.
    // Packing the Any can fail, which will be returned as an error status.
    // If the value is a platform-specific object, return a void* Response.
    template <typename T>
    static absl::StatusOr<Response> Create(const T& value);

    // Returns true if Response has a value (could be either proto or void*).
    bool HasValue() const { return value_.has_value(); }

    // Returns true if Response specifically contains a proto Any.
    bool HasProtoValue() const {
      return value_.has_value() && std::holds_alternative<const Any>(*value_);
    }

    // Returns true if Response specifically contains a platform object (void*).
    bool HasPlatformObjectValue() const {
      return value_.has_value() && std::holds_alternative<void*>(*value_);
    }

    // Assumes HasProtoValue() is true and returns the proto value as an Any.
    // This is a helper for the common case where the response value is a proto.
    const Any& Value() const { return ValueAsProto(); }
    // Returns the proto response value as an Any. Caller is responsible for
    // first checking that the value is an Any with HasProtoValue().
    const Any& ValueAsProto() const { return std::get<const Any>(*value_); }
    // Returns the void* (platform-specific object) response value. Caller is
    // responsible for first checking that the value is a void* with
    // HasPlatformObjectValue().
    void* ValueAsPlatformObject() const { return std::get<void*>(*value_); }

   private:
    Response() : value_(std::nullopt) {}
    explicit Response(const Any& value) : value_(value) {}
    explicit Response(void* value) : value_(value) {}

    const std::optional<std::variant<const Any, void*>> value_;
  };

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
  // Platform args is an optional vector of void* arguments such as a jobject.
  // Subclasses can override this version if they need the platform args.
  // Otherwise, they should override the version without the PlatformArgs.
  virtual Future<Response> HandleAnyMessage(const Any& message,
                                            const PlatformArgs& args) = 0;

  // Handles an incoming message from the script side of the scripting system.
  // Returns a response message corresponding to the request type.
  Future<Response> HandleAnyMessage(const Any& message) {
    return HandleAnyMessage(message, PlatformArgs());
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

template <typename T>
absl::StatusOr<BaseMessageHandler::Response>
BaseMessageHandler::Response::Create(const T& value) {
  if constexpr (std::is_same<T, absl::Status>::value) {
    MP_RETURN_IF_ERROR(value);
    return Response();
  } else if constexpr (std::is_same<T, void*>::value) {
    return Response(value);
  } else {
    MP_ASSIGN_OR_RETURN(Any packed_value, proto::PackAny(value));
    return Response(packed_value);
  }
}

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_WEB_BASE_MESSAGE_HANDLER_H_
