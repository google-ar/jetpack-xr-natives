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

#include <memory>
#include <string>
#include <utility>

#include "core/common/jni_helpers.h"
#include "core/scripting/message_helpers.h"

namespace imp::scripting {

template <class T, class... Allowlist>
using JniAllowlist = JniAllowlist<T, Allowlist...>;

template <class T>
inline jlong ToJava(T* p) {
  return JniAllowlist<T, WebView>::ToJava(p);
}

/**  Wraps the ImpWebViewFragment Java class for native calls. */
class AndroidImpWebViewFragment : public JavaWrapper {
 public:
  explicit AndroidImpWebViewFragment(const Context& context, WebView* web_view,
                                     const WebViewParams& params,
                                     BufferAccess injection_script,
                                     bool clear_cache)
      : JavaWrapper(context, "com/google/ar/imp/core/web/ImpWebViewFragment",
                    "()V"),
        context_(context),
        web_view_(web_view) {
    JniHandle inflate =
        GetMethodHandle("inflate",
                        "(Lcom/google/ar/imp/core/web/FragmentHost;"
                        "JFFIILjava/lang/String;Ljava/lang/String;Z)V");
    CallVoidMethod(
        inflate, context_.GetFragmentHost(), web_view_, params.location_px.x,
        params.location_px.y, params.dimensions_px.x, params.dimensions_px.y,
        ToString(context_.GetJniEnv(), params.url),
        ToString(context_.GetJniEnv(), injection_script.StringView().empty()
                                           ? ""
                                           : injection_script.StringView()),
        clear_cache);
  }

  ~AndroidImpWebViewFragment() override {
    JniHandle deflate = GetStaticMethodHandle(
        "deflate", "(Lcom/google/ar/imp/core/web/FragmentHost;J)V");
    CallStaticVoidMethod(deflate, context_.GetFragmentHost(), web_view_);
  }

  void PostMessage(const MessageToScript& message) {
    JniHandle post_message_handle =
        GetMethodHandle("postMessage", "(Ljava/lang/String;)V");
    assert(post_message_handle);
    CallVoidMethod(post_message_handle,
                   ToString(context_.GetJniEnv(), SerializeToBase64(message)));
  }

 private:
  const Context& context_;
  WebView* web_view_;
};

/** Wraps the ImpWebViewBridge Java class for native calls. */
class AndroidImpWebViewBridge : public JavaWrapper {
 public:
  explicit AndroidImpWebViewBridge(const Context& context,
                                   WebView* native_web_view, jobject web_view,
                                   BufferAccess injection_script,
                                   bool clear_cache)
      : JavaWrapper(
            context, "com/google/ar/imp/core/web/ImpWebViewBridge",
            "(JLandroid/webkit/WebView;Ljava/lang/String;Z)V",
            ToJava<WebView>(native_web_view), web_view,
            ToString(context.GetJniEnv(), injection_script.StringView().empty()
                                              ? ""
                                              : injection_script.StringView()),
            static_cast<jboolean>(clear_cache)),
        context_(context) {
    post_message_ = GetMethodHandle("postMessage", "(Ljava/lang/String;)V");
    inject_script_ = GetMethodHandle("injectScript", "()V");
    set_injection_script_ =
        GetMethodHandle("setInjectionScript", "(Ljava/lang/String;)V");
    assert(post_message_);
    assert(inject_script_);
    assert(set_injection_script_);
  }

  void PostMessage(const MessageToScript& message) {
    CallVoidMethod(post_message_,
                   ToString(context_.GetJniEnv(),
                            SerializeToBase64<MessageToScript>(message)));
  }

  void InjectScript() { CallVoidMethod(inject_script_); }

  void SetInjectionScript(BufferAccess injection_script) {
    CallVoidMethod(
        set_injection_script_,
        ToString(context_.GetJniEnv(), injection_script.StringView().empty()
                                           ? ""
                                           : injection_script.StringView()));
  }

 private:
  const Context& context_;
  JniHandle post_message_;
  JniHandle inject_script_;
  JniHandle set_injection_script_;
};

// Android WebView native implementation.
class AndroidWebView : public WebView {
 public:
  // Creates a WebView by nesting it in an Android Fragment
  AndroidWebView(const Context& context, const WebViewParams& params,
                 BufferAccess injection_script, bool clear_cache)
      : WebView(),
        android_web_view_fragment_(std::make_unique<AndroidImpWebViewFragment>(
            context, this, params, std::move(injection_script), clear_cache)) {}

  // Creates a wrapper over an existing Android WebView
  AndroidWebView(const Context& context, jobject external_web_view,
                 BufferAccess injection_script, bool clear_cache)
      : WebView(),
        android_web_view_bridge_(std::make_unique<AndroidImpWebViewBridge>(
            context, this, external_web_view, std::move(injection_script),
            clear_cache)) {}

  AndroidWebView(const AndroidWebView&) = delete;
  AndroidWebView& operator=(const AndroidWebView&) = delete;

  void LoadInjectionScript() override {
    android_web_view_bridge_->InjectScript();
  }

  void PostMessage(const MessageToScript& message) override {
    if (this->GetState() == WebView::State::kUnavailable) {
      return;
    }
    // TODO Separate the PostMessage functionality from the
    // fragment
    if (android_web_view_fragment_) {
      android_web_view_fragment_->PostMessage(message);
    } else if (android_web_view_bridge_) {
      android_web_view_bridge_->PostMessage(message);
    }
  }

 private:
  std::unique_ptr<AndroidImpWebViewFragment> android_web_view_fragment_;
  std::unique_ptr<AndroidImpWebViewBridge> android_web_view_bridge_;
};

// Android Java<->C++ scripting interface.
// TODO: refactor/rename WebView to make this cleaner.
// Also, This is technically against the g3 C++ style guide (multiple
// implementation inheritance is strongly discouraged)
class AndroidScriptingInterface : public WebView, public JavaWrapper {
 public:
  AndroidScriptingInterface(const Context& context, void* scripting_bridge)
      : JavaWrapper(context.GetJniEnv(),
                    static_cast<jobject>(scripting_bridge)),
        context_(context) {
    post_message_ = GetMethodHandle("postMessage", "([B)V");
    assert(post_message_);
  }

  AndroidScriptingInterface(const AndroidWebView&) = delete;
  AndroidScriptingInterface& operator=(const AndroidWebView&) = delete;

  void LoadInjectionScript() override {
    // Do nothing. There is no javascript needed for pure-Java scripting.
  }

  void PostMessage(const MessageToScript& message) override {
    std::string serialized;
    imp::proto::SerializeTo(&message, &serialized);

    CallVoidMethod(post_message_,
                   ToByteArray(context_.GetJniEnv(), serialized));
  }

 private:
  const Context& context_;
  JniHandle post_message_;
};

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

std::unique_ptr<WebView> WebView::Create(const Context& context,
                                         const WebViewParams& params,
                                         BufferAccess injection_script) {
  bool clear_cache = false;
#ifdef IMP_WEB_ENABLE_CHROME_DEBUG
  EnableWebContentsDebugging(context);
  clear_cache = true;
#endif
  return std::make_unique<AndroidWebView>(
      context, params, std::move(injection_script), clear_cache);
}

std::unique_ptr<WebView> WebView::Create(void* web_view, const Context& context,
                                         BufferAccess injection_script) {
  // TODO: branch on something other than the presence of script?
  if (injection_script) {
    bool clear_cache = false;
#ifdef IMP_WEB_ENABLE_CHROME_DEBUG
    EnableWebContentsDebugging(context);
    clear_cache = true;
#endif
    return std::make_unique<AndroidWebView>(
        context, static_cast<jobject>(web_view), std::move(injection_script),
        clear_cache);
  } else {
    // TODO: in this case, the "web_view" is actually the
    // ScriptingBridge.java jobject.
    return std::make_unique<AndroidScriptingInterface>(context, web_view);
  }
}

}  // namespace imp::scripting
