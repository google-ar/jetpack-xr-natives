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
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/common/platform_helpers.h"
#include "core/config.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_writer.h"
#include "core/scripting/base_message_handler.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/scripting/web/web_view.h"
#include "core/view/scripting/script_message_handler.h"
#include "core/view/scripting/script_message_handler_provider.h"

namespace imp::scripting {

using google::protobuf::imp_proto::Any;
using scripting::BaseMessageHandler;
using scripting::MessageToNative;
using scripting::MessageToScript;

ScriptingSystem::ScriptingSystem(const Context& context,
                                 ScriptMessageHandlerProvider* provider,
                                 const WebViewParams& params,
                                 BufferAccess script)
    : ScriptingSystem(context, provider,
                      WebView::Create(context, params, std::move(script))) {}

ScriptingSystem::ScriptingSystem(
    const Context& context, scripting::ScriptMessageHandlerProvider* provider,
    void* external_web_view, BufferAccess script)
    : ScriptingSystem(
          context, provider,
          WebView::Create(external_web_view, context, std::move(script))) {}

ScriptingSystem::ScriptingSystem(const Context& context,
                                 ScriptMessageHandlerProvider* provider,
                                 std::unique_ptr<WebView> web_view)
    : web_view_(std::move(web_view)), provider_(provider) {
  if (provider_) {
    provider_->SetScriptMessageHandler(this);
  }

  web_view_->SetScriptMessageHandler(this);

#if IMP_RUNTIME(DEV)
  output::AddExternalLogHandler(web_view_.get(), HandleLog);
#endif  // IMP_RUNTIME(DEV)
}

ScriptingSystem::~ScriptingSystem() {
  ClearRemembered();
  output::RemoveExternalLogHandler(web_view_.get());
  if (provider_ != nullptr) {
    if (provider_->GetScriptMessageHandler() == this) {
      provider_->SetScriptMessageHandler(nullptr);
    }
  }
  if (web_view_) {
    web_view_->SetScriptMessageHandler(nullptr);
  }
}

void ScriptingSystem::LoadAPI() const { web_view_->LoadInjectionScript(); }

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
    if (registry_.count(type_url) != 0) {
      IMP_LOG(imp::WARNING) << "A MessageHandler for the request type " << type_url
                   << " has already been set. Overriding with the new handler.";
    }

    registry_[type_url] = handler.get();
  }

  handlers_.insert(std::move(handler));
}

absl::Status ScriptingSystem::SendRegisteredEvent(const Any& event_any,
                                                  NodeHandle target) {
  auto event_iter = events_.find(event_any.type_url);
  if (event_iter == events_.end()) {
    return absl::FailedPreconditionError(absl::StrCat(
        "No registered event found for type: ", event_any.type_url));
  }

  if (!provider_) {
    return absl::FailedPreconditionError(
        "No ScriptMessageHandlerProvider available.");
  }

  return event_iter->second(provider_->GetDispatcher(), event_any, target);
}

// TODO: All platforms should call this method rather than doing
// their own decoding.
void ScriptingSystem::HandleMessage(const std::string& message,
                                    const scripting::PlatformArgs& args,
                                    scripting::PlatformArgs& out) {
  MessageToNative message_proto;
  std::string decoded;
  if (!DeserializeBase64(message, &decoded)) {
    IMP_LOG(imp::ERROR) << "Failed to decode base64 string received from JS: "
               << message;
    return;
  }

  if (!ParseFromArray(decoded.c_str(), decoded.length(), &message_proto)) {
    IMP_LOG(imp::ERROR) << "Failed to parse message from Javascript!";
    return;
  }

  // It is safe to ignore the returned future, since it is .KeptBy() this.
  auto unused = HandleMessage(message_proto, args, out);
}

Future<MessageToScript> ScriptingSystem::HandleMessage(
    const MessageToNative& message, const scripting::PlatformArgs& args,
    scripting::PlatformArgs& out) {
  if (!Executor::ForegroundExecutor() ||
      Executor::CurrentExecutor() != Executor::ForegroundExecutor()) {
    IMP_LOG(imp::FATAL) << "Impress scripting APIs can only be called on the foreground "
                  "thread.";
  }
  // TODO: Allow messages without id if they are synchronous?
  if (!message.message_id) {
    IMP_LOG(imp::FATAL) << "Received scripting message with no message_id!";
  }
  const Any& content = message.content;
  auto key = content.type_url;

  MessageToScript message_to_script;
  message_to_script.message_id = message.message_id;

  // First, check if this is a registered event type.
  if (events_.contains(key)) {
    absl::Status event_status = SendRegisteredEvent(content);
    if (!event_status.ok()) {
      message_to_script.error = std::string(event_status.ToString());
    }
    // TODO: Do not call web_view_->PostMessage() here.
    if (web_view_) {
      web_view_->PostMessage(message_to_script);
    }
    return Future<MessageToScript>(message_to_script);
  }

  // Next, check if this is a type with a registered message handler.
  auto iter = registry_.find(key);
  if (iter == registry_.end()) {
    message_to_script.error = "Unhandled message type!" + std::string(key);
    // TODO: Do not call web_view_->PostMessage() here.
    if (web_view_) {
      web_view_->PostMessage(message_to_script);
    }
    return Future<MessageToScript>(message_to_script);
  }

  Future<MessageToScript> result_future =
      iter->second->HandleAnyMessage(content, args, out)
          .Then([this, message, message_to_script](
                    absl::StatusOr<BaseMessageHandler::OptionalResponse>
                        optional_response_or) mutable {
            // The response is either a Status, indicating an error, or
            // an optional protocol buffer response.
            if (optional_response_or.ok()) {
              // If this handler returns a response, add that to the message.
              if (optional_response_or->has_value()) {
                message_to_script.content = optional_response_or->value();
              }
            } else {
              message_to_script.error =
                  std::string(optional_response_or.status().message());
            }
            if (web_view_) {
              // TODO: Do not call web_view_->PostMessage() here.
              // Instead, each platform should wire up the Future<> response
              // of HandleMessage as they see fit (sync, async, etc). and call
              // PostMessage themselves. This will help with the refactor to
              // make WebView no longer needed for Java scripting.
              web_view_->PostMessage(message_to_script);
            }
            return Future<MessageToScript>(message_to_script);
          });
  result_future.KeptBy(this);
  return result_future;
}

inline scripting::LogLevel OutputKindToLogLevel(imp::output::OutputKind kind) {
  return static_cast<scripting::LogLevel>(static_cast<size_t>(kind));
}

void ScriptingSystem::HandleLog(void* context, output::OutputKind kind,
                                absl::string_view log) {
  auto log_fn = [context, kind, log_str = std::string(log)]() {
    scripting::WebView* web_view = reinterpret_cast<WebView*>(context);
    scripting::LogMessage log_message;
    log_message.level = OutputKindToLogLevel(kind);
    log_message.log = std::move(log_str);
    scripting::MessageToScript message_to_script;
    if (proto::PackAny(log_message, &message_to_script.content).ok()) {
      web_view->PostMessage(message_to_script);
    }
  };

  if (Executor::CurrentExecutor() == Executor::ForegroundExecutor()) {
    log_fn();
  } else {
    Executor::ForegroundExecutor()->Schedule(log_fn);
  }
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
