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

#include <memory>

#include "core/view/scripting/script_message_handler.h"

namespace imp {

class Dispatcher;

namespace scripting {

// An interface for a provider of ScriptMessageHandler objects.
struct ScriptMessageHandlerProvider {
  virtual ~ScriptMessageHandlerProvider() {}

  // Sets the ScriptMessageHandler that will be returned by Get().
  virtual void SetScriptMessageHandler(ScriptMessageHandler* handler) = 0;

  // Returns the ScriptMessageHandler associated with this provider.
  virtual ScriptMessageHandler* GetScriptMessageHandler() const = 0;

  // Returns the Dispatcher to use for sending events.
  virtual Dispatcher& GetDispatcher() noexcept = 0;

  // Sets the bridge to use for posting scripting responses and events.
  // Assumed to be of type c/g/ar/imp/core/scripting/ScriptEndpoint.java.
  // TODO: it would probably take more refactoring but it would be
  // nice if SetScriptEndpoint could take a ScriptEndpoint interface or
  // something, and the JniWrapper for the jobject is an implementation of the
  // interface that gets created in view_jni instead of web_view.cc.
  virtual void SetScriptEndpoint(void* script_endpoint) = 0;
};

}  // namespace scripting
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_WEB_SCRIPT_MESSAGE_HANDLER_PROVIDER_H_
