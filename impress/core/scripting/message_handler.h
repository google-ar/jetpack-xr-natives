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

#include <tuple>
#include <vector>

#include "absl/types/optional.h"
#include "core/async/future.h"
#include "core/scripting/base_message_handler.h"
#include "core/scripting/message_helpers.h"

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

  // Override this version instead if you need to return PlatformArgs out.
  virtual Future<MessageResponseType> HandleMessage(
      const MessageRequestType& message, const PlatformArgs& args,
      PlatformArgs& out) {
    return HandleMessage(message, args);
  }

  // Expose the base version that only takes message.
  using BaseMessageHandler::HandleAnyMessage;

  Future<OptionalResponse> HandleAnyMessage(const Any& message,
                                            const PlatformArgs& args,
                                            PlatformArgs& out) override;

  std::vector<absl::string_view> GetSupportedRequestTypeUrls() override {
    return std::vector<absl::string_view>{MessageRequestType::kTypeUrl};
  }
};

template <typename TRequest, typename TResponse>
Future<BaseMessageHandler::OptionalResponse>
MessageHandler<TRequest, TResponse>::HandleAnyMessage(const Any& message,
                                                      const PlatformArgs& args,
                                                      PlatformArgs& out) {
  // First, assert that the protobuf passed in is actually the type of protobuf
  // that this handler handles.
  

  // Unpack the proto into the actual concrete request type.
  absl::StatusOr<TRequest> unpacked_message_or =
      proto::UnpackAny<TRequest>(message);
  if (!unpacked_message_or.ok()) {
    return Future<OptionalResponse>(absl::InvalidArgumentError(
        absl::StrFormat("Unable to unpack message: %s",
                        unpacked_message_or.status().ToString())));
  }
  TRequest& unpacked_message = unpacked_message_or.value();

  // Call HandleMessage with the concrete type to process the actual request.
  return HandleMessage(unpacked_message, args, out)
      .Then([](MessageResponseType response) -> Future<OptionalResponse> {
        // Convert the result of HandleMessage into the OptionalResponse type.
        if constexpr (std::is_same<MessageResponseType, absl::Status>::value) {
          if (response.ok()) {
            // If TResponse is absl::Status, and HandleMessage succeeded,
            // then return an empty response because there is no response proto.
            return Future<OptionalResponse>(OptionalResponse());
          } else {
            // TResponse is absl::Status and HandleMessage failed, just pass
            // along the failure status.
            return Future<OptionalResponse>(response);
          }
        } else {
          // TResponse is a proto and HandleMessage succeeded, so pack it into
          // an Any and forward it along.
          // If TResponse is a proto and HandleMessage failed, then this lambda
          // is never called, the failure status is just automatically returned
          // by the future.
          return Future<OptionalResponse>(proto::PackAny(response));
        }
      });
}

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MESSAGE_HANDLER_H_
