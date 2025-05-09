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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WASM_WEB_VIEW_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WASM_WEB_VIEW_H_

#include "core/common/rememberer.h"
#include "core/scripting/web/web_view.h"
#include "core/view/framework/view.h"

namespace imp::scripting {

// The native implementation of Imp WebView on Web-Assembly (WASM).
class WasmWebView : public WebView, public Rememberer {
 public:
  // Construct a new WasmWebView given an Imp BaseView and params.
  WasmWebView(const Context& context, const WebViewParams& params,
              BufferAccess injection_script);
  WasmWebView(void* external_web_view, BufferAccess injection_script);

  WasmWebView(const WasmWebView&) = delete;
  WasmWebView& operator=(const WasmWebView&) = delete;

  void PostMessage(const MessageToScript& message) override;

  void LoadInjectionScript() override;

 private:
  BufferAccess injection_script_;
  InjectionTarget injection_target_;
};

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_WEB_WASM_WEB_VIEW_H_
