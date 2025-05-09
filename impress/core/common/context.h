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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_CONTEXT_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_CONTEXT_H_

#include <jni.h>

#include <memory>
#include <string>
#include <vector>

#include "absl/types/span.h"
#include "core/common/jni_context.h"
#include "core/config.h"

namespace imp {

// Provides access to platform-specific application context data for use in
// imp components.
// Not thread-safe, but a copy can be made for each thread.

// TODO: guard the platform-specific code with #ifdefs.
class Context {
 public:
  Context(JavaVM* jvm, jobject activity_context,
          jobject fragment_host = nullptr);
  // The default constructor is included for Android temporarily to allow
  // incremental development of components depending on Context.
  Context();

  // Create a desktop context holding the commandline arguments.
  Context(int argc, char* argv[]);

  // Create an iOS context with commandline arguments and a bridged pointer to
  // the UIView that created it. The view is not retained by this pointer, so
  // the Context must not outlive its owner.
  Context(std::vector<std::string>&& arguments, void* owning_ui_view);

  // Context can be copied but not moved.
  Context(const Context& other);
  Context& operator=(const Context& other);
  Context(Context&& other) = delete;
  Context& operator=(Context&& other) = delete;

  ~Context();

  imp::JniContext* GetJniContext() { return &jni_; }

  // If GetJniEnv is called on a non-const Context, a new JNIEnv will be created
  // the first time it is called in a thread.
  JNIEnv* GetJniEnv() const { return jni_.GetJniEnv(); }

  // If GetJniEnv is called on a const Context, it will not create a new JNIEnv.
  // If a JNIEnv has never been used on this thread, it will return nullptr.
  JNIEnv* TryGetJniEnv() const { return jni_.TryGetJniEnv(); }
  jobject GetActivityContext() const { return activity_context_; }
  jobject GetFragmentHost() const { return fragment_host_; }
  const std::vector<std::string>& GetArguments() const { return arguments_; }
  void* GetOwningUIView() const { return owning_ui_view_; }

 private:
  imp::JniContext jni_;
  jobject activity_context_;
  jobject fragment_host_;
  std::vector<std::string> arguments_;
  void* owning_ui_view_;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_CONTEXT_H_
