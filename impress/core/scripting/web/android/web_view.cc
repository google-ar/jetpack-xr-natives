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

#include <jni.h>

#include <memory>
#include <string>
#include <utility>

#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "core/scripting/web/android/web_view.h"
#include "core/view/scripting/script_message_handler.h"

namespace imp::scripting {

#ifdef IMP_WEB_ENABLE_CHROME_DEBUG
static void EnableWebContentsDebugging(const Context& context) {
  JNIEnv* env = context.GetJniEnv();
  jclass web_view_class = env->FindClass("android/webkit/WebView");
  jmethodID set_web_contents_debugging_enabled = env->GetStaticMethodID(
      web_view_class, "setWebContentsDebuggingEnabled", "(Z)V");
  env->CallStaticVoidMethod(web_view_class, set_web_contents_debugging_enabled,
                            JNI_TRUE);
}
#endif

std::unique_ptr<WebView> WebView::Create(
    ScriptMessageHandler& script_message_handler, const Context& context,
    const WebViewParams& params, BufferAccess injection_script) {
  bool clear_cache = false;
#ifdef IMP_WEB_ENABLE_CHROME_DEBUG
  EnableWebContentsDebugging(context);
  clear_cache = true;
#endif
  return std::make_unique<AndroidWebView>(script_message_handler, context,
                                          params, std::move(injection_script),
                                          clear_cache);
}

std::unique_ptr<WebView> WebView::Create(
    ScriptMessageHandler& script_message_handler, const Context& context,
    void* web_view, BufferAccess injection_script) {
  // TODO: branch on something other than the presence of script?
  if (injection_script) {
    bool clear_cache = false;
#ifdef IMP_WEB_ENABLE_CHROME_DEBUG
    EnableWebContentsDebugging(context);
    clear_cache = true;
#endif
    return std::make_unique<AndroidWebView>(
        script_message_handler, context, static_cast<jobject>(web_view),
        std::move(injection_script), clear_cache);
  } else {
    // TODO: in this case, the "web_view" is actually the
    // ScriptingBridge.java jobject.
    return std::make_unique<AndroidScriptingInterface>(context, web_view);
  }
}

}  // namespace imp::scripting
