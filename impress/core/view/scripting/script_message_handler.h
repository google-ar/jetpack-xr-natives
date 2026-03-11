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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_SCRIPTING_SCRIPT_MESSAGE_HANDLER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_SCRIPTING_SCRIPT_MESSAGE_HANDLER_H_

#include <utility>
#include <vector>

#include "core/common/invocable.h"
#include "core/scripting/proto/bridge.proto.imp.h"

namespace imp {
namespace scripting {

using PlatformArgs = std::vector<void*>;
using ResponseHandler = imp::Invocable<void(const MessageToScript&, void*)>;

// An interface for a class that handles messages from script (i.e. Javascript).
class ScriptMessageHandler {
 public:
  virtual ~ScriptMessageHandler() {}

  // Handles an incoming message from script by delegating to handlers.
  // Platform args is an optional vector of void* arguments such as a jobject.
  // The response handler is called with the MessageToScript response and an
  // optional platform-specific void* output parameter (e.g. a jobject).
  virtual void HandleMessage(const MessageToNative& message,
                             const PlatformArgs& args,
                             ResponseHandler response_handler) = 0;

  // Handles an incoming message from script by delegating to handlers.
  void HandleMessage(const MessageToNative& message,
                     ResponseHandler response_handler) {
    HandleMessage(message, PlatformArgs(), std::move(response_handler));
  }
};

}  // namespace scripting
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_SCRIPTING_SCRIPT_MESSAGE_HANDLER_H_
