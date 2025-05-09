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

#include "core/scripting/message_handlers/event_handler.h"

#include "core/common/log.h"
#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/common/hash.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/ncsb/dispatcher/event.h"
#include "core/proto/any.proto.imp.h"
#include "core/proto/proto_writer.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/scripting/proto/events.proto.imp.h"
#include "core/scripting/scripting_system.h"
#include "core/view/base_view.h"

namespace imp::scripting {

EventHandler::EventHandler(BaseView& base_view,
                           ScriptingSystem& scripting_system)
    : view_(base_view), scripting_system_(scripting_system) {
  AddHandler<EventListenerAddRequest, EventListenerAddResponse>(this);
  AddHandler<EventListenerRemoveRequest, absl::Status>(this);
  AddHandler<NodeEvent, absl::Status>(this);
}

Future<EventListenerAddResponse> EventHandler::HandleMessage(
    const EventListenerAddRequest& message) {
  if (!message.target.IsValid() && !message.target.IsDefaultValue()) {
    return Future<EventListenerAddResponse>(
        absl::InvalidArgumentError("Invalid Node - must be either the default "
                                   "(empty) NodeHandle() or a valid node."));
  }

  auto& event_type_url = message.event_type_url;
  // TODO: optimization so we only send one message per unique
  // event (ex: two listeners on the same global event type should only
  // generate one MessageToScript, not two).
  auto connection = view_.GetDispatcher().Connect(
      message.target, imp::Hash(event_type_url),
      [this,
       event_type_url](const Event& event) -> Dispatcher::PropagationResult {
        google::protobuf::imp_proto::Any any;
        if (event.ToAny(&any)) {
          EventListenerMessage event_listener_message;
          event_listener_message.listener_id = event.GetConnectionId();
          event_listener_message.event = any;
          MessageToScript message_to_script;
          if (proto::PackAny(event_listener_message, &message_to_script.content)
                  .ok()) {
            scripting_system_.GetWebView()->PostMessage(message_to_script);
          } else {
            IMP_LOG(imp::ERROR) << "Unable to pack event msg to send to js.";
          }
        } else {
          // This could happen if the event is not a proto-based event type.
          IMP_LOG(imp::ERROR) << "Runtime events are not supported. Event type: "
                     << event_type_url;
        }
        // TODO: Allow JS to control event propagation.
        return imp::Dispatcher::PropagationResult::kAccept;
      },
      &rememberer_);
  EventListenerAddResponse response;
  response.listener_id = connection.GetId();
  return Future<EventListenerAddResponse>(response);
}

Future<absl::Status> EventHandler::HandleMessage(
    const EventListenerRemoveRequest& message) {
  Dispatcher& d = view_.GetDispatcher();
  d.Disconnect(message.listener_id);
  return Future<absl::Status>(absl::OkStatus());
}

Future<absl::Status> EventHandler::HandleMessage(const NodeEvent& message) {
  NodeHandle target = message.target;
  if (!target) {
    return Future<absl::Status>(
        absl::InvalidArgumentError("Invalid Node target for NodeEvent."));
  }

  return Future<absl::Status>(
      scripting_system_.SendRegisteredEvent(message.event, message.target));
}

}  // namespace imp::scripting
