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

#include "core/scripting/testing/web_view_test_runner.h"

#include <memory>

#include "absl/memory/memory.h"
#include "core/common/jni_helpers.h"

namespace imp::scripting {

/** Wraps the TestWebView Java class. */
class TestWebView : public JavaWrapper {
 public:
  explicit TestWebView(const Context& context, WebViewTestRunner* handle)
      : JavaWrapper(context, "com/google/ar/imp/core/web/testing/TestWebView",
                    "()V") {
    JniHandle init = GetMethodHandle("init",
                                     "(Landroid/app/Activity;"
                                     "J)Landroid/webkit/WebView;");
    web_view_ = Env()->NewGlobalRef(CallObjectMethod(
        init, context.GetActivityContext(),
        (jlong)handle));  // NOLINT define for instrumented testing
    evaluate_ = GetMethodHandle("evaluateJavaScript", "(Ljava/lang/String;)V");
    run_all_tests_ = GetMethodHandle("runAllTests", "(Ljava/lang/String;)Z");
  }

  jobject GetWebView() { return web_view_; }

  bool RunAllTests(BufferAccess script) {
    return CallBooleanMethod(run_all_tests_,
                             ToString(Env(), script.StringView()));
  }

  void EvaluateJavaScript(absl::string_view script) {
    CallVoidMethod(evaluate_, ToString(Env(), script));
  }

  TestWebView(const TestWebView&) = delete;
  TestWebView& operator=(const TestWebView&) = delete;

  ~TestWebView() override { Env()->DeleteGlobalRef(web_view_); }

 private:
  jobject web_view_;
  JniHandle evaluate_;
  JniHandle run_all_tests_;
};

/** Android WebView Test Runner. */
class AndroidWebViewTestRunner : public WebViewTestRunner {
 public:
  explicit AndroidWebViewTestRunner(testing::TestView* test_view,
                                    const Context& context)
      : WebViewTestRunner(test_view),
        test_web_view_(std::make_unique<TestWebView>(context, this)) {}

  AndroidWebViewTestRunner(const AndroidWebViewTestRunner&) = delete;
  AndroidWebViewTestRunner& operator=(const AndroidWebViewTestRunner&) = delete;

  void* GetWebView() override { return test_web_view_->GetWebView(); }

  void EvaluateJavaScript(absl::string_view script) override {
    test_web_view_->EvaluateJavaScript(script);
  }

 protected:
  bool Run(BufferAccess script) override {
    return test_web_view_->RunAllTests(std::move(script));
  }

 private:
  std::unique_ptr<TestWebView> test_web_view_;
};

std::unique_ptr<WebViewTestRunner> WebViewTestRunner::Create(
    testing::TestView* test, const Context& context) {
  return std::make_unique<AndroidWebViewTestRunner>(test, context);
}

}  // namespace imp::scripting
