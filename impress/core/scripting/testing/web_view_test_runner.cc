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

#include "absl/strings/match.h"

namespace imp::scripting {

WebViewTestRunner::WebViewTestRunner(testing::TestView* test_view)
    : test_view_(test_view), response_() {}

void WebViewTestRunner::DrainAllExecutors() {
  if (test_view_) {
    test_view_->DrainAllExecutors();
  }
}
Future<absl::Status> WebViewTestRunner::RunAllTests(BufferAccess script) {
  assert(Run(std::move(script)));
  return response_;
}

void WebViewTestRunner::OnConsoleMessage(const absl::string_view type,
                                         const absl::string_view message) {
  IMP_LOG(imp::INFO) << type << ":" << message;
  if (type == kLogType && message.rfind(kTestStatusCompleted, 0) == 0) {
    if (absl::StrContains(message, kTestStatusFailed)) {
      response_.Return(absl::InternalError("Test failure"));
    } else if (absl::StrContains(message, kTestStatusPassed)) {
      response_.Return(absl::OkStatus());
    } else {
      response_.Return(absl::InternalError(
          "Something went wrong, did not receive status passed or failed"));
    }
  }
}

}  // namespace imp::scripting
