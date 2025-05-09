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

#include "core/scripting/web/wasm/web_view.h"

#include <emscripten/emscripten.h>
#include <emscripten/threading.h>

#include <memory>
#include <string>
#include <utility>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/strings/str_format.h"
#include "core/common/buffer_access.h"
#include "core/common/platform_helpers.h"
#include "core/common/resource_helpers.h"
#include "core/common/string_helpers.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/web/wasm/wasm_helpers.h"
#include "javascript/core/imp_web_bridge_js_embed.h"

namespace imp::scripting {

inline constexpr absl::string_view kBridgeJsFilename = "bridge.js";
inline constexpr absl::string_view kConsoleLog = "console.log(`%s`);";

WasmWebView::WasmWebView(const Context& context, const WebViewParams& params,
                         BufferAccess injection_script)
    : injection_script_(std::move(injection_script)),
      injection_target_(params.injection_target) {
  // In WASM, by the time this line is reached, the page is already loading
  // as this code is being loaded by the page. This means it is the right time
  // to inject the script.
  LoadInjectionScript();
}

WasmWebView::WasmWebView(void* external_web_view,
                         BufferAccess injection_script) {
  IMP_LOG(imp::FATAL) << "Cannot instantiate WasmWebView with external web view.";
}

// A helper function to eval the given script in the javascript environment.
// This is used to asynchronously inject the injection_script_.
void EvalInjectionScript(void* script) {
  emscripten_run_script(static_cast<char*>(script));
}

void WasmWebView::LoadInjectionScript() {
  switch (injection_target_) {
    case InjectionTarget::kMainPage:
      // In the case of injecting to the main page, it would happen
      // "too synchronously" - ScriptBridge is still in the process of
      // initializing when this injection is happening, and, if the API gets
      // injected synchronously, the API script loads before ImpWeb is fully
      // hooked up on the native side. We need at least a frame delay for the
      // set of calls related to setting up ImpWeb to complete before the script
      // is injected.
      Future<absl::Status>::Schedule(
          [this]() {
      // If threads are enabled, we have to ensure the script is injected
      // back on the main thread, not the background thread. If threads
      // are not enabled, the background thread is the same thread so we
      // can just inject (and the async function doesn't exist).
#if defined(__EMSCRIPTEN_PTHREADS__)
            emscripten_async_run_in_main_runtime_thread(
                EM_FUNC_SIG_VI, EvalInjectionScript, injection_script_.Data());
#else
            emscripten_run_script(
                reinterpret_cast<const char*>(injection_script_.Data()));
#endif
            return absl::OkStatus();
          },
          Executor::Type::kBackground)
          .KeptBy(this);
      break;
    case InjectionTarget::kIFrame:
      // Load the bridge script.
      RegisterPackagedResources(imp_web_bridge_js_embed_create());
      BufferAccess bridge_script;
      if (auto status = LoadPackagedFile(kBridgeJsFilename, &bridge_script);
          !status.ok()) {
        IMP_LOG(imp::FATAL) << status;
      }

      // Inject the bridge script into the javascript environment.
      emscripten_run_script(std::string(bridge_script.StringView()).c_str());

      // Finally, inject the ImpWeb API script into the iframe using the
      // bridge's injection feature.
      InjectScriptToBridge(injection_script_.StringView());
      break;
  }
}

void WasmWebView::PostMessage(const MessageToScript& message) {
  // Post message to the main thread so the script has DOM access.
  MAIN_THREAD_EM_ASM(
      {eval(UTF8ToString($0))},
      absl::StrFormat(kPostMessage, SerializeToBase64(message)).c_str());
}

std::unique_ptr<WebView> WebView::Create(const Context& context,
                                         const WebViewParams& params,
                                         BufferAccess injection_script) {
  return std::make_unique<WasmWebView>(context, params,
                                       std::move(injection_script));
}

std::unique_ptr<WebView> WebView::Create(void* web_view, const Context& context,
                                         BufferAccess injection_script) {
  return std::make_unique<WasmWebView>(web_view, std::move(injection_script));
}

}  // namespace imp::scripting
