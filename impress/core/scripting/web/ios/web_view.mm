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

#include "core/scripting/web/ios/web_view.h"

#include "core/scripting/message_helpers.h"
#include "core/view/framework/view.h"

namespace imp::scripting {

IosWebView::IosWebView(ScriptMessageHandler& script_message_handler, const Context& context,
                       const WebViewParams& params, BufferAccess injection_script)
    : WebView() {
  CGRect frame;
  frame.origin.x = params.location_px.x;
  frame.origin.y = params.location_px.y;
  frame.size.width = params.dimensions_px.x;
  frame.size.height = params.dimensions_px.y;
  NSString* url = [NSString stringWithCString:params.url.c_str()
                                     encoding:[NSString defaultCStringEncoding]];
  NSString* script = [[NSString alloc] initWithBytes:injection_script.StringView().data()
                                              length:injection_script.StringView().size()
                                            encoding:[NSString defaultCStringEncoding]];
  web_view_ = [[IMPWebView alloc] initWithWebView:this
                             scriptMessageHandler:script_message_handler
                                          context:context
                                            frame:frame
                                              url:url
                                  injectionScript:script];
}

IosWebView::IosWebView(ScriptMessageHandler& script_message_handler, void* external_web_view,
                       BufferAccess injection_script)
    : WebView() {
  NSString* script = [[NSString alloc] initWithBytes:injection_script.StringView().data()
                                              length:injection_script.StringView().size()
                                            encoding:[NSString defaultCStringEncoding]];
  web_view_ = [[IMPWebView alloc] initWithWebView:this
                             scriptMessageHandler:script_message_handler
                                  injectionScript:script
                                  externalWebView:(__bridge WKWebView*)external_web_view];
}

IosWebView::~IosWebView() { [web_view_ onWebViewDestruction]; }

void IosWebView::LoadInjectionScript() { [web_view_ injectScript]; }

std::unique_ptr<WebView> WebView::Create(ScriptMessageHandler& script_message_handler,
                                         const Context& context, const WebViewParams& params,
                                         BufferAccess injection_script) {
  return absl::make_unique<IosWebView>(script_message_handler, context, params,
                                       std::move(injection_script));
}

std::unique_ptr<WebView> WebView::Create(ScriptMessageHandler& script_message_handler,
                                         const Context& context, void* web_view,
                                         BufferAccess injection_script) {
  return absl::make_unique<IosWebView>(script_message_handler, web_view,
                                       std::move(injection_script));
}

}  // namespace imp::scripting
