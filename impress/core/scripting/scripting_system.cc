// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/scripting/scripting_system.h"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "absl/container/flat_hash_set.h"
#include "absl/log/check.h"
#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/common/hash.h"
#include "core/common/platform_helpers.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_reader.h"
#include "core/proto/proto_writer.h"
#include "core/scripting/base_message_handler.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/scripting/proto/events.proto.imp.h"
#include "core/scripting/web/web_view.h"
#include "core/view/scripting/script_message_handler.h"
#include "core/view/scripting/script_message_handler_provider.h"

namespace imp::scripting {

using google::protobuf::imp_proto::Any;
using scripting::BaseMessageHandler;
using scripting::MessageToNative;
using scripting::MessageToScript;

namespace {
// Returns true if the given message type is in the set of messages that are
// handled inline by the scripting system instead of delegated to a handler.
bool IsInlineHandledMessage(absl::string_view message_type) {
  static const absl::flat_hash_set<absl::string_view> kInlineHandledMessages = {
      EventListenerAddRequest::kTypeUrl,
      EventListenerRemoveRequest::kTypeUrl,
      NodeEvent::kTypeUrl,
  };

  return kInlineHandledMessages.contains(message_type);
}
}  // namespace

ScriptingSystem::ScriptingSystem(const Context& context,
                                 ScriptMessageHandlerProvider* provider,
                                 const WebViewParams& params,
                                 BufferAccess script)
    : ScriptingSystem(*provider) {
  web_view_ = WebView::Create(*this, context, params, std::move(script));
}

ScriptingSystem::ScriptingSystem(
    const Context& context, scripting::ScriptMessageHandlerProvider* provider,
    void* external_web_view, BufferAccess script)
    : ScriptingSystem(*provider) {
  web_view_ =
      WebView::Create(*this, context, external_web_view, std::move(script));
}

ScriptingSystem::ScriptingSystem(ScriptMessageHandlerProvider& provider)
    : provider_(provider) {
  provider_.SetScriptMessageHandler(this);
}

ScriptingSystem::~ScriptingSystem() {
  ClearRemembered();
  output::RemoveExternalLogHandler(web_view_.get());
  if (provider_.GetScriptMessageHandler() == this) {
    provider_.SetScriptMessageHandler(nullptr);
  }
}

void ScriptingSystem::AddHandler(
    std::unique_ptr<scripting::BaseMessageHandler> handler) {
  std::vector<absl::string_view> type_urls =
      handler->GetSupportedRequestTypeUrls();

  if (type_urls.empty()) {
    // This is a programmer error, so Fatal in this case.
    // If this occurs, it's likely from a MultiMessageHandler where
    // MultiMessageHandler::AddHandler was not called before this method was
    // called. Otherwise, it's probably from a custom subclass of
    // BaseMessageHandler that hasn't been implemented correctly.
    IMP_LOG(imp::FATAL)
        << "Attempting to add handler that doesn't support any request types. "
           "Skipping adding the handler.";
    return;
  }

  // Warn if a handler for the same request type is set twice.
  // It will override the previously set handler, which arguably has a use
  // case but is more likely a bug.
  for (const auto& type_url : type_urls) {
    if (IsInlineHandledMessage(type_url)) {
      IMP_LOG(imp::FATAL)
          << "Attempting to add a handler for a message type that is already "
             "being handled natively by the ScriptingSystem.";
    }

    if (registry_.count(type_url) != 0) {
      IMP_LOG(imp::WARNING) << "A MessageHandler for the request type " << type_url
                   << " has already been set. Overriding with the new handler.";
    }

    registry_[type_url] = handler.get();
  }

  handlers_.insert(std::move(handler));
}

absl::Status ScriptingSystem::SendRegisteredEvent(const Any& event_any,
                                                  NodeHandle target) const {
  auto event_iter = events_.find(event_any.type_url);
  if (event_iter == events_.end()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "No registered event found for type: ", event_any.type_url));
  }

  return event_iter->second(provider_.GetDispatcher(), event_any, target);
}

template <typename ResponseHandlerT>
void SendResponse(int32_t message_id, ResponseHandlerT& response_handler,
                  absl::Status status = absl::OkStatus()) {
  MessageToScript message_to_script;
  message_to_script.message_id = message_id;
  if (!status.ok()) {
    message_to_script.error = status.ToString();
  }
  response_handler(message_to_script, nullptr);
}

template <typename Response, typename ResponseHandlerT>
void SendResponse(int32_t message_id, ResponseHandlerT& response_handler,
                  absl::StatusOr<Response> response) {
  MessageToScript message_to_script;
  message_to_script.message_id = message_id;
  if (!response.ok()) {
    message_to_script.error = response.status().ToString();
  } else {
    absl::StatusOr<Any> packed_response = proto::PackAny(*response);
    if (packed_response.ok()) {
      message_to_script.content = *packed_response;
    } else {
      message_to_script.error = packed_response.status().ToString();
    }
  }
  response_handler(message_to_script, nullptr);
}

void ScriptingSystem::HandleMessage(const MessageToNative& message,
                                    const scripting::PlatformArgs& args,
                                    ResponseHandler response_handler) {
  if (!Executor::ForegroundExecutor() ||
      Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "Impress scripting APIs can only be called on the foreground "
                  "thread.";
  }
  if (!message.message_id) {
    IMP_LOG(imp::FATAL) << "Received scripting message with no message_id!";
  }
  const Any& content = message.content;
  absl::string_view key = content.type_url;

  // First, check if the message contains a registered event type.
  if (events_.contains(key)) {
    SendResponse(message.message_id, response_handler,
                 SendRegisteredEvent(content, NodeHandle()));
    return;
  }

  // If this is a NodeEvent, unpack it and send the event on the target node.
  if (key == NodeEvent::kTypeUrl) {
    
    absl::StatusOr<NodeEvent> unpacked_message =
        proto::UnpackAny<NodeEvent>(content);
    if (!unpacked_message.ok()) {
      SendResponse(message.message_id, response_handler,
                   unpacked_message.status());
      return;
    }
    if (!events_.contains(unpacked_message->event.type_url)) {
      SendResponse(message.message_id, response_handler,
                   absl::FailedPreconditionError(
                       absl::StrCat("No registered event found for type: ",
                                    unpacked_message->event.type_url)));
      return;
    }
    SendResponse(
        message.message_id, response_handler,
        SendRegisteredEvent(unpacked_message->event, unpacked_message->target));
    return;
  }

  // Event listener requests are handled inline by the scripting system instead
  // of being delegated to a handler. This is because the event listener code
  // relies on storing the response_handler, which is not something that
  // is exposed by the BaseMessageHandler interface. The response handler is
  // also a move-only type so there are challenges with adding it to the
  // MessageHandler interface.
  if (key == EventListenerAddRequest::kTypeUrl) {
    
    absl::StatusOr<EventListenerAddRequest> unpacked_message =
        proto::UnpackAny<EventListenerAddRequest>(message.content);
    if (!unpacked_message.ok()) {
      SendResponse(message.message_id, response_handler,
                   unpacked_message.status());
      return;
    }
    HandleEventListenerAddRequest(*unpacked_message, message.message_id,
                                  std::move(response_handler));
    return;
  }

  if (key == EventListenerRemoveRequest::kTypeUrl) {
    
    absl::StatusOr<EventListenerRemoveRequest> unpacked_message =
        proto::UnpackAny<EventListenerRemoveRequest>(message.content);
    if (!unpacked_message.ok()) {
      SendResponse(message.message_id, response_handler,
                   unpacked_message.status());
      return;
    }
    HandleEventListenerRemoveRequest(*unpacked_message, message.message_id,
                                     std::move(response_handler));
    return;
  }

  // Next, check if this is a type with a registered message handler.
  auto iter = registry_.find(key);
  if (iter == registry_.end()) {
    SendResponse(
        message.message_id, response_handler,
        absl::NotFoundError("Unhandled message type!" + std::string(key)));
    return;
  }

  iter->second->HandleAnyMessage(content, args)
      .Then([message_id = message.message_id,
             response_handler = std::move(response_handler)](
                absl::StatusOr<BaseMessageHandler::Response> response) mutable {
        if (!response.ok()) {
          SendResponse(message_id, response_handler, response.status());
          return;
        }
        MessageToScript message_to_script;
        message_to_script.message_id = message_id;
        void* out = nullptr;
        // Handle the response value, if any (an empty response is valid).
        if (response->HasProtoValue()) {
          // The response is an Any proto for a proto-based response.
          message_to_script.content = response->ValueAsProto();
        } else if (response->HasPlatformObjectValue()) {
          // The response is a void* for a platform-specific response object.
          out = response->ValueAsPlatformObject();
        }
        response_handler(message_to_script, out);
      })
      .KeptBy(this);
}

void ScriptingSystem::HandleEventListenerAddRequest(
    const EventListenerAddRequest& request, int32_t message_id,
    ResponseHandler response_handler) {
  if (!request.target.IsValid() && !request.target.IsDefaultValue()) {
    SendResponse(
        message_id, response_handler,
        absl::InvalidArgumentError("Invalid Node - must be either the default "
                                   "(empty) NodeHandle() or a valid node."));
    return;
  }

  // This class allows us to move the response handler into the event connection
  // below and still call it at the end of the function to resolve the
  // original request. The response to the request requires the connection ID,
  // which is only available after calling Connect() and moving the handler.
  class ResponseHandlerWrapper {
   public:
    explicit ResponseHandlerWrapper(ResponseHandler response_handler)
        : response_handler_(std::move(response_handler)) {}

    void operator()(const MessageToScript& message_to_script, void* out) {
      response_handler_(message_to_script, out);
    }

   private:
    ResponseHandler response_handler_;
  };

  std::unique_ptr<ResponseHandlerWrapper> response_handler_wrapper =
      std::make_unique<ResponseHandlerWrapper>(std::move(response_handler));
  ResponseHandlerWrapper* response_handler_ptr = response_handler_wrapper.get();

  // Register the event connection. Use this to manage the lifetime of the
  // connection, since an explicit RemoveEventListenerRequest is required to
  // disconnect the event.
  auto& event_type_url = request.event_type_url;
  Dispatcher::Connection connection = provider_.GetDispatcher().Connect(
      request.target, imp::Hash(request.event_type_url),
      [response_handler = std::move(response_handler_wrapper),
       event_type_url](const Event& event) -> Dispatcher::PropagationResult {
        google::protobuf::imp_proto::Any any;
        if (event.ToAny(&any)) {
          EventListenerMessage event_listener_message;
          event_listener_message.listener_id = event.GetConnectionId();
          event_listener_message.event = any;
          MessageToScript message_to_script;
          if (proto::PackAny(event_listener_message, &message_to_script.content)
                  .ok()) {
            (*response_handler)(message_to_script, nullptr);
          } else {
            IMP_LOG(imp::ERROR) << "Unable to pack event to send to script.";
          }
        } else {
          // This could happen if the event is not a proto-based event type.
          IMP_LOG(imp::ERROR) << "Runtime events are not supported. Event type: "
                     << event_type_url;
        }
        // TODO: Allow script to control event propagation.
        return imp::Dispatcher::PropagationResult::kAccept;
      },
      this);

  // Create the response including the connection ID.
  absl::StatusOr<EventListenerAddResponse> response =
      EventListenerAddResponse();
  response->listener_id = connection.GetId();
  SendResponse(message_id, *response_handler_ptr, response);
}
void ScriptingSystem::HandleEventListenerRemoveRequest(
    const EventListenerRemoveRequest& request, int32_t message_id,
    ResponseHandler response_handler) const {
  Dispatcher& d = provider_.GetDispatcher();
  d.Disconnect(request.listener_id);
  MessageToScript message_to_script;
  message_to_script.message_id = message_id;
  response_handler(message_to_script, nullptr);
}

inline scripting::LogLevel OutputKindToLogLevel(imp::output::OutputKind kind) {
  return static_cast<scripting::LogLevel>(static_cast<size_t>(kind));
}

// A set of asserts that the OutputKind and LogLevel (proto) enums match.
// We cannot share an enum with imp::output because that is in the core
// imp code that disallows RTTI, which is required for the proto library.
static_assert(static_cast<size_t>(imp::output::OutputKind::kInfo) ==
                  static_cast<size_t>(LogLevel::INFO),
              "Enum mismatch");

static_assert(static_cast<size_t>(imp::output::OutputKind::kWarning) ==
                  static_cast<size_t>(LogLevel::WARNING),
              "Enum mismatch");

static_assert(static_cast<size_t>(imp::output::OutputKind::kError) ==
                  static_cast<size_t>(LogLevel::ERROR),
              "Enum mismatch");

static_assert(static_cast<size_t>(imp::output::OutputKind::kFatal) ==
                  static_cast<size_t>(LogLevel::FATAL),
              "Enum mismatch");

static_assert(
    imp::output::OutputKind::kMax == imp::output::OutputKind::kFatal,
    "New fields added but assert and imp::web::LogLevel enum not updated");

}  // namespace imp::scripting
