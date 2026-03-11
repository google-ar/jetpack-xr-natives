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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MULTI_MESSAGE_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MULTI_MESSAGE_HANDLER_H_

#include <memory>
#include <vector>

#include "core/common/log.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/scripting/base_message_handler.h"
#include "core/scripting/message_handler.h"
#include "core/view/scripting/script_message_handler.h"
#include "core/view/utils/string_map.h"

namespace imp::scripting {

// Base class for handling messages from javascript that supports responding to
// multiple different types of messages. If you only need to handle one type of
// message, use MessageHandler instead.
//
// Example Usage:
//
// struct FooHandler : public MultiMessageHandler {
//   FooHandler() {
//     AddHandler<BarRequest, BarResponse>(this);
//     AddHandler<BazzRequest, absl::Status>(this);
//   }
//
//   Future<BarResponse> HandleMessage(const BarRequest& request);
//   Future<absl::Status> HandleMessage(const BazzRequest& request);
// }
//
class MultiMessageHandler : public BaseMessageHandler {
 public:
  // Expose the base version that only takes message.
  using BaseMessageHandler::HandleAnyMessage;

  Future<Response> HandleAnyMessage(const Any& message,
                                    const PlatformArgs& args) override;
  std::vector<absl::string_view> GetSupportedRequestTypeUrls() override;

 protected:
  // Adds a handler for the given TRequest/TResponse pair, which informs the
  // MultiMessageHandler that it supports handling messages of type TRequest.
  //
  // The THandler passed in must be a pointer to an object that implements a
  // method with this signature:
  //
  // Future<TResponse> HandleMessage(const TRequest& request);
  //
  // It can be any object with that method, but in a typical use-case it is the
  // MultiMessageHandler subclass itself.
  //
  // This does not take ownership of the THandler passed in. It should remain
  // alive as long as the MultiMessageHandler remains alive.
  //
  // Note: AddHandler must be called prior to the MultiMessageHandler being
  // added to the ScriptingSystem. Typically, from the subclasses constructor.
  template <typename TRequest, typename TResponse, typename THandler>
  void AddHandler(THandler* handler);

 private:
  // Helper class forwards HandleMessage calls for a given TRequest/TResponse
  // pair to another object.
  template <typename TRequest, typename TResponse, typename THandler>
  class HandlerForwarder : public MessageHandler<TRequest, TResponse> {
   public:
    // Does not take ownership of the handler.
    explicit HandlerForwarder(THandler* handler) : handler_(*handler) {}

    Future<TResponse> HandleMessage(const TRequest& message) override;

   private:
    THandler& handler_;
  };

  // The key is a string_view, which doesn't own it's data. This means that the
  // string_view's data must outlive the lifetime of the registry. This is
  // guaranteed because the string_view for type_urls for generated protos
  // is a static constexpr that lives for the lifetime of the application.
  // The advantage of this is that we don't need pay any extra memory or cpu
  // cost for storing or copying duplicate strings.
  using HandlersMap = StringViewMap<std::unique_ptr<BaseMessageHandler>>;
  HandlersMap handler_forwarders_;
};

template <typename TRequest, typename TResponse, typename THandler>
void MultiMessageHandler::AddHandler(THandler* handler) {
  HandlersMap::const_iterator iter =
      handler_forwarders_.find(TRequest::kTypeUrl);

  if (iter != handler_forwarders_.end()) {
    IMP_LOG(imp::WARNING)
        << "AddHandler has already been called for the request type "
        << TRequest::kTypeUrl
        << " on this MultiMessageHandler. Overriding with the new handler.";
  }

  handler_forwarders_.insert_or_assign(
      iter, TRequest::kTypeUrl,
      std::make_unique<HandlerForwarder<TRequest, TResponse, THandler>>(
          handler));
}

template <typename TRequest, typename TResponse, typename THandler>
Future<TResponse> MultiMessageHandler::HandlerForwarder<
    TRequest, TResponse, THandler>::HandleMessage(const TRequest& message) {
  // Forward the HandleMessage call to the handler type passed in.
  // If you get a compiler error here, then that likely means you are missing
  // a HandleMessage declaration that should look like this:
  // Future<TResponse> HandleMessage(const TRequest& request);
  // Likely in the MultiMessageHandler subclass.
  return handler_.HandleMessage(message);
}

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_MULTI_MESSAGE_HANDLER_H_
