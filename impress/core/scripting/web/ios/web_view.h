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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_IOS_WEB_VIEW_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_IOS_WEB_VIEW_H_

#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/scripting/web/ios/IMPWebView.h"
#include "core/scripting/web/web_view.h"

namespace imp::scripting {

// The native implementation of Imp WebView on iOS.
class IosWebView : public WebView {
 public:
  // Construct a new IosWebView given an Imp BaseView and params.
  IosWebView(ScriptMessageHandler& script_message_handler,
             const Context& context, const WebViewParams& params,
             BufferAccess injection_script);
  IosWebView(ScriptMessageHandler& script_message_handler,
             void* external_web_view, BufferAccess injection_script);

  IosWebView(const IosWebView&) = delete;
  IosWebView& operator=(const IosWebView&) = delete;

  ~IosWebView();

  void LoadInjectionScript() override;

 private:
  IMPWebView* web_view_;
};

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_IOS_WEB_VIEW_H_
