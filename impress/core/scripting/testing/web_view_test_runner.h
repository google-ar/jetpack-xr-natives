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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_TESTING_WEB_VIEW_TEST_RUNNER_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_TESTING_WEB_VIEW_TEST_RUNNER_H_

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/common/context.h"
#include "testing/view_fixture.h"

namespace imp::scripting {

using ::imp::testing::TestView;

/** Initializes a platform test WebView and runs a JavaScript test script. */
class WebViewTestRunner {
 public:
  static std::unique_ptr<WebViewTestRunner> Create(TestView* test_view,
                                                   const Context& context);

  virtual ~WebViewTestRunner() {}

  // Returns pointer to the test WebView.
  virtual void* GetWebView() = 0;
  // Wraps the script as an executable function and then evaluates the provided
  // script in the WebView.
  virtual void EvaluateJavaScript(absl::string_view script) = 0;

  // Evaluates the JS test script in the test WebView and returns OK when the
  // test suite completes running.
  Future<absl::Status> RunAllTests(BufferAccess script);
  // Receives console messages that report JS test results from the WebView.
  void OnConsoleMessage(absl::string_view type, absl::string_view message);

  void DrainAllExecutors();

  static constexpr const absl::string_view kLogType = "LOG";
  static constexpr const absl::string_view kTestStatusCompleted =
      "Testing complete";
  static constexpr const absl::string_view kTestStatusPassed = "passed";
  static constexpr const absl::string_view kTestStatusFailed = "failed";

 protected:
  explicit WebViewTestRunner(TestView* test_view);
  virtual bool Run(BufferAccess script) = 0;

  TestView* test_view_;
  Future<absl::Status> response_;
};

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_TESTING_WEB_VIEW_TEST_RUNNER_H_
