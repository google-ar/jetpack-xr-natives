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

#include "core/scripting/web/web_view.h"

#include "absl/strings/string_view.h"
#include "core/scripting/proto/bridge.proto.imp.h"

namespace imp::scripting {

WebView::WebView()
    : state_(State::kAvailable), script_message_handler_(nullptr) {}

WebView::State WebView::GetState() const { return state_; }

void WebView::OnWebViewDestroyed() { state_ = State::kUnavailable; }

void WebView::SetScriptMessageHandler(
    ScriptMessageHandler* script_message_handler) {
  script_message_handler_ = script_message_handler;
}

void WebView::HandleMessage(const MessageToNative& message) {
  if (!script_message_handler_) return;
  // Future is .KeptBy(script_message_handler_) internally, so we can ignore the
  // return value.
  PlatformArgs out;
  auto unused = script_message_handler_->HandleMessage(message, {}, out);
}

}  // namespace imp::scripting
