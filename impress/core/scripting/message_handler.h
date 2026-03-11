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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLER_H_

#include <vector>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/scripting/base_message_handler.h"
#include "core/scripting/message_helpers.h"
#include "core/view/scripting/script_message_handler.h"

namespace imp::scripting {

class WebView;

// Templated base class for handling a message from Javascript.
//
// TRequest is the type of proto that this class will handle
// requests for.
//
// TResponse is the type of response that this handler will provide when the
// request is received. The response can either be another proto, or it can be
// absl::Status.
//
// Subclasses must implement the pure-virtual method HandleMessage to process
// the request. HandleMessage returns a future to allow the request to be
// handled asynchronously.
template <typename TRequest, typename TResponse>
struct MessageHandler : public BaseMessageHandler {
  // The type of request that the handler handles. Must be a proto.
  using MessageRequestType = TRequest;

  // The type of response that the handler returns.
  // Must be a proto or absl::Status.
  using MessageResponseType = TResponse;

  // Override in subclass to process the request and return the response.
  //
  // Example:
  //
  // Future<FooResponseProto> HandleMessage(const FooRequestProto& message) {
  //   FooResponseProto response;
  //   response.bazz = message.bar;
  //   return Future<FooResponseProto>(response);
  // }
  virtual Future<MessageResponseType> HandleMessage(
      const MessageRequestType& message) = 0;

  // Override this version instead if you need PlatformArgs args (i.e. jobject).
  virtual Future<MessageResponseType> HandleMessage(
      const MessageRequestType& message, const PlatformArgs& args) {
    return HandleMessage(message);
  }

  // Expose the base version that only takes message.
  using BaseMessageHandler::HandleAnyMessage;

  Future<Response> HandleAnyMessage(const Any& message,
                                    const PlatformArgs& args) override;

  std::vector<absl::string_view> GetSupportedRequestTypeUrls() override {
    return std::vector<absl::string_view>{MessageRequestType::kTypeUrl};
  }
};

template <typename TRequest, typename TResponse>
Future<BaseMessageHandler::Response>
MessageHandler<TRequest, TResponse>::HandleAnyMessage(
    const Any& message, const PlatformArgs& args) {
  // First, assert that the protobuf passed in is actually the type of protobuf
  // that this handler handles.
  

  // Unpack the proto into the actual concrete request type.
  absl::StatusOr<TRequest> unpacked_message_or =
      proto::UnpackAny<TRequest>(message);
  if (!unpacked_message_or.ok()) {
    return Future<Response>(absl::InvalidArgumentError(
        absl::StrFormat("Unable to unpack message: %s",
                        unpacked_message_or.status().ToString())));
  }
  TRequest& unpacked_message = unpacked_message_or.value();

  // Call HandleMessage with the concrete type to process the actual request.
  return HandleMessage(unpacked_message, args)
      .Then(&BaseMessageHandler::Response::Create<MessageResponseType>);
}

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLER_H_
