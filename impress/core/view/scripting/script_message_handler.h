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

#include <string>
#include <vector>

#include "core/async/future.h"
#include "core/scripting/proto/bridge.proto.imp.h"

namespace imp {
namespace scripting {

using PlatformArgs = std::vector<void*>;

// An interface for a class that handles messages from script (i.e. Javascript).
struct ScriptMessageHandler {
  virtual ~ScriptMessageHandler() {}

  // Handles a string message from script. Message format based on contract.
  // Platform args is an optional vector of void* arguments such as a jobject.
  virtual void HandleMessage(const std::string& message,
                             const PlatformArgs& args, PlatformArgs& out) = 0;

  // Handles a string message from script. Message format based on contract.
  void HandleMessage(const std::string& message) {
    PlatformArgs out;
    HandleMessage(message, PlatformArgs(), out);
  }

  // Handles an incoming message from script by delegating to handlers.
  // Platform args is an optional vector of void* arguments such as a jobject.
  virtual Future<MessageToScript> HandleMessage(const MessageToNative& message,
                                                const PlatformArgs& args,
                                                PlatformArgs& out) = 0;

  // Handles an incoming message from script by delegating to handlers.
  Future<MessageToScript> HandleMessage(const MessageToNative& message) {
    PlatformArgs out;
    return HandleMessage(message, PlatformArgs(), out);
  }
};

}  // namespace scripting
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_SCRIPTING_SCRIPT_MESSAGE_HANDLER_H_
