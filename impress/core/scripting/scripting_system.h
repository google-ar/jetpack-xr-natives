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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_SCRIPTING_SYSTEM_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_SCRIPTING_SYSTEM_H_

#include <cstdint>
#include <memory>
#include <set>

#include "absl/base/attributes.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/string_view.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/common/invocable.h"
#include "core/common/platform_helpers.h"
#include "core/common/rememberer.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/node_handle.h"
#include "core/proto/any.proto.imp.h"
#include "core/scripting/base_message_handler.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/scripting/proto/events.proto.imp.h"
#include "core/scripting/web/web_view.h"
#include "core/view/scripting/script_message_handler.h"
#include "core/view/scripting/script_message_handler_provider.h"
#include "core/view/utils/string_map.h"
#include "mediapipe/framework/port/status_macros.h"

namespace imp::scripting {

using google::protobuf::imp_proto::Any;

// The main container of web functionality for the Imp engine.
// This class constructs a WebView wrapper and registers handlers for proto
// buffers that represent the JS API for Imp.
class ScriptingSystem : public Rememberer, public ScriptMessageHandler {
 public:
  // Creates and initializes a WebView with the provided parameters.
  // The script parameter is the script code to inject into javascript.
  // TODO: (broken link) - remove this constructor.
  ABSL_DEPRECATED("Create a WebView and ScriptingSystem separately.")
  ScriptingSystem(const Context& context,
                  ScriptMessageHandlerProvider* provider,
                  const WebViewParams& params,
                  BufferAccess script = BufferAccess());

  // Connects the Imp scripting interface with an already-initialized WebView.
  // Note that to enable the scripting interface, LoadAPI needs to be called,
  // and is recommended to be done so on every page load.
  // The script parameter is the script code to inject into javascript.
  // TODO: (broken link) - remove this constructor.
  ABSL_DEPRECATED("Create a WebView and ScriptingSystem separately.")
  ScriptingSystem(const Context& context,
                  ScriptMessageHandlerProvider* provider,
                  void* external_web_view,
                  BufferAccess script = BufferAccess());

  explicit ScriptingSystem(ScriptMessageHandlerProvider& provider);

  ~ScriptingSystem() override;

  // Adds a MessageHandler that will be invoked to handle its specified types.
  void AddHandler(std::unique_ptr<scripting::BaseMessageHandler> handler);

  // Registers an event type (must be a proto message of type imp::Event) so
  // it can be sent from script to native and sent via the dispatcher.
  template <typename TEvent>
  void RegisterEventType();

  // Sends an event of a registered type to the dispatcher with optional target.
  absl::Status SendRegisteredEvent(const Any& any,
                                   NodeHandle target = NodeHandle()) const;

  // Expose the HandleMessage(message) variants to clients.
  using ScriptMessageHandler::HandleMessage;

  // Handles an incoming message from script by delegating to handlers.
  void HandleMessage(const MessageToNative& message, const PlatformArgs& args,
                     ResponseHandler response_handler) override;

  // Returns the WebView owned by this ScriptingSystem (which may be null).
  // TODO: (broken link) - remove this method.
  ABSL_DEPRECATED("Construct a WebView and ScriptingSystem separately.")
  WebView* GetWebView() { return web_view_.get(); }

 private:
  // Handles forwarding a log from imp output to the script console.
  static void HandleLog(void* context, output::OutputKind kind,
                        absl::string_view log);

  void HandleEventListenerAddRequest(const EventListenerAddRequest& request,
                                     int32_t message_id,
                                     ResponseHandler response_handler);
  void HandleEventListenerRemoveRequest(
      const EventListenerRemoveRequest& request, int32_t message_id,
      ResponseHandler response_handler) const;

  // TODO: (broken link) - remove this field.
  std::unique_ptr<WebView> web_view_;

  std::set<std::unique_ptr<BaseMessageHandler>> handlers_;
  ScriptMessageHandlerProvider& provider_;

  // Stores a map of proto type urls to the message handler that handles that
  // type of proto.
  //
  // The key is a string_view, which doesn't own it's data. This means that the
  // string_view's data must outlive the lifetime of the registry. This is
  // guaranteed because the string_view for type_urls for generated protos
  // is a static constexpr that lives for the lifetime of the application.
  // The advantage of this is that we don't need pay any extra memory or cpu
  // cost for storing or copying duplicate strings.
  StringViewMap<BaseMessageHandler*> registry_;

  // Stores a map from proto type url to a function to deserialize an Event from
  // an Any. This allows script to send events of registered types to native.
  StringViewMap<
      Invocable<absl::Status(Dispatcher& dispatcher, const Any&, NodeHandle)>>
      events_;
};

template <typename TEvent>
void ScriptingSystem::RegisterEventType() {
  events_[TEvent::kTypeUrl] = [](Dispatcher& dispatcher, const Any& any,
                                 NodeHandle target) -> absl::Status {
    absl::StatusOr<TEvent> event = proto::UnpackAny<TEvent>(any);
    MP_RETURN_IF_ERROR(event.status());
    dispatcher.Send(target, *event);
    return absl::OkStatus();
  };
}

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_SCRIPTING_SYSTEM_H_
