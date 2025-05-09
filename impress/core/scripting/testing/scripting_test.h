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

#ifndef THIRD_PARTY_IMPRESS_CORE_SCRIPTING_TESTING_SCRIPTING_TEST_H_
#define THIRD_PARTY_IMPRESS_CORE_SCRIPTING_TESTING_SCRIPTING_TEST_H_

#include <memory>
#include <utility>

#include "core/common/context.h"
#include "core/scripting/testing/web_view_test_runner.h"
#include "testing/test_view.h"

#if IMP_PLATFORM(ANDROID)
#include "core/common/jni_helpers.h"
#endif

namespace imp::scripting {

template <typename T>
class GenericScriptingTest {
 public:
  explicit GenericScriptingTest(std::unique_ptr<Context> context);
  ~GenericScriptingTest();

  T* GetTestView();
  void* GetWebView();

  // Wraps the script as an executable function and then evaluates the provided
  // script in the WebView.
  void EvaluateJavaScript(absl::string_view script);

  // Evaluates the JS test script in the test WebView and returns OK when the
  // test suite completes running.
  Future<absl::Status> RunAllTests(BufferAccess script);

  // Callback for platform code.
  void InitTestView(std::unique_ptr<Context> context);

  // Callback for platform code.
  void TearDownView();

 private:
  Future<std::unique_ptr<T>> init_view_future_;
  Future<absl::Status> tear_down_completion_future_;
  std::unique_ptr<T> test_view_;
  std::unique_ptr<WebViewTestRunner> runner_;

  std::unique_ptr<T> Init(std::unique_ptr<Context> context);
  void TearDown();
};

template <typename T>
GenericScriptingTest<T>::GenericScriptingTest(std::unique_ptr<Context> context)
    : init_view_future_(),
      tear_down_completion_future_(),
      test_view_(std::move(Init(std::move(context)))),
      runner_(WebViewTestRunner::Create(
          reinterpret_cast<imp::testing::TestView*>(test_view_.get()),
          test_view_->GetView()->GetContext())) {}

template <typename T>
GenericScriptingTest<T>::~GenericScriptingTest() {
  TearDown();
}

template <typename T>
T* GenericScriptingTest<T>::GetTestView() {
  return test_view_.get();
}

template <typename T>
void* GenericScriptingTest<T>::GetWebView() {
  return runner_->GetWebView();
}

template <typename T>
void GenericScriptingTest<T>::EvaluateJavaScript(absl::string_view script) {
  runner_->EvaluateJavaScript(script);
}

template <typename T>
Future<absl::Status> GenericScriptingTest<T>::RunAllTests(BufferAccess script) {
  // Sandboxing glTF asset loading doesn't work within the Android Emulator so
  // ensure it is disabled.
  // This must be done here so it's done on the main thread and happens after
  // the View is setup, which is where the sandboxed gltf loader is usually set.
  test_view_->GetView()->GetAssetManager().SetSandboxedGltfLoader({});
  return runner_->RunAllTests(std::move(script));
}

template <typename T>
void GenericScriptingTest<T>::InitTestView(std::unique_ptr<Context> context) {
  // This must be done here so it's done on the main thread.
  init_view_future_.Return(std::make_unique<T>(std::move(context)));
}

template <typename T>
std::unique_ptr<T> GenericScriptingTest<T>::Init(
    std::unique_ptr<Context> context) {
#if IMP_PLATFORM(ANDROID)
  JNIEnv* env = context->GetJniEnv();
  jclass scripting_test_class =
      env->FindClass("com/google/ar/imp/core/web/testing/ScriptingTest");
  jmethodID init =
      env->GetStaticMethodID(scripting_test_class, "initTestView", "(JJ)V");
  env->CallStaticVoidMethod(scripting_test_class, init, (jlong)this,
                            (jlong)context.release());
#elif IMP_PLATFORM(IOS)
  InitTestView(std::move(context));
#endif
  typename Future<std::unique_ptr<T>>::Result view_future =
      testing::BaseExecutorTestHelper::MoveFuture<std::unique_ptr<T>>(
          init_view_future_);
  
  return *std::move(view_future);
}

template <typename T>
void GenericScriptingTest<T>::TearDown() {
#if IMP_PLATFORM(ANDROID)
  JNIEnv* env = test_view_->GetView()->GetContext().GetJniEnv();
  jclass scripting_test_class =
      env->FindClass("com/google/ar/imp/core/web/testing/ScriptingTest");
  jmethodID tear_down =
      env->GetStaticMethodID(scripting_test_class, "tearDownView", "(J)V");
  env->CallStaticVoidMethod(scripting_test_class, tear_down, (jlong)this);
#elif IMP_PLATFORM(IOS)
  TearDownView();
#endif
  // Wait for async completion of TearDownView().
  MP_ASSERT_OK(testing::BaseExecutorTestHelper::GetFuture<absl::Status>(
      tear_down_completion_future_));
}

template <typename T>
void GenericScriptingTest<T>::TearDownView() {
  runner_.reset();
  test_view_.reset();
  tear_down_completion_future_.Return(absl::OkStatus());
}

using ScriptingTest = GenericScriptingTest<imp::testing::TestView>;

}  // namespace imp::scripting

#endif  // THIRD_PARTY_IMPRESS_CORE_SCRIPTING_TESTING_SCRIPTING_TEST_H_
