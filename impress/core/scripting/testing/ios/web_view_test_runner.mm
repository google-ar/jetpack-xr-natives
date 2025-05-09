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
#include "third_party/absl/memory/memory.h"
#include "core/scripting/testing/ios/IMPTestWebView.h"

namespace imp::scripting {
class IosWebViewTestRunner : public WebViewTestRunner {
 public:
  explicit IosWebViewTestRunner(testing::TestView* test_view) : WebViewTestRunner(test_view) {
    test_web_view_ = [[IMPTestWebView alloc] initWithRunner:this];
  }

  IosWebViewTestRunner(const IosWebViewTestRunner&) = delete;
  IosWebViewTestRunner& operator=(const IosWebViewTestRunner&) = delete;

  void* GetWebView() override { return (__bridge void*)(test_web_view_.wkWebView); }

  void EvaluateJavaScript(absl::string_view message) override {
    NSString* str = [[NSString alloc] initWithBytes:message.data()
                                             length:message.size()
                                           encoding:[NSString defaultCStringEncoding]];
    [test_web_view_ evaluateJavaScript:str];
  }

 protected:
  bool Run(BufferAccess script) override {
    NSString* run_script = [[NSString alloc] initWithBytes:script.StringView().data()
                                                    length:script.StringView().size()
                                                  encoding:[NSString defaultCStringEncoding]];
    return [test_web_view_ runAllTests:run_script];
  }

 private:
  IMPTestWebView* test_web_view_;
};

std::unique_ptr<WebViewTestRunner> WebViewTestRunner::Create(testing::TestView* test_view,
                                                             const Context& context) {
  return absl::make_unique<IosWebViewTestRunner>(test_view);
}

}  // namespace imp::scripting
