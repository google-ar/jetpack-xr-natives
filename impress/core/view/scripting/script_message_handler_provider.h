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

#ifndef THIRD_PARTY_IMPRESS_CORE_WEB_SCRIPT_MESSAGE_HANDLER_PROVIDER_H_
#define THIRD_PARTY_IMPRESS_CORE_WEB_SCRIPT_MESSAGE_HANDLER_PROVIDER_H_

#include "absl/base/attributes.h"
#include "core/view/scripting/script_message_handler.h"

namespace imp {

class Dispatcher;

namespace scripting {

// An interface for a provider of ScriptMessageHandler objects.
class ScriptMessageHandlerProvider {
 public:
  virtual ~ScriptMessageHandlerProvider() {}

  // Sets the ScriptMessageHandler that will be returned by Get().
  virtual void SetScriptMessageHandler(ScriptMessageHandler* handler) = 0;

  // Returns the ScriptMessageHandler associated with this provider.
  virtual ScriptMessageHandler* GetScriptMessageHandler() const = 0;

  // Returns the Dispatcher to use for sending events.
  virtual Dispatcher& GetDispatcher() noexcept = 0;

  // Sets the bridge to use for posting scripting responses and events.
  // Assumed to be of type c/g/ar/imp/core/scripting/ScriptEndpoint.java.
  ABSL_DEPRECATED(
      "Create the scripting system with ScriptingSystem(*this) in Setup() "
      "instead. See //third_party/impress/samples/scripting for an example.")
  virtual void SetScriptEndpoint(void* script_endpoint) = 0;
};

}  // namespace scripting
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_WEB_SCRIPT_MESSAGE_HANDLER_PROVIDER_H_
