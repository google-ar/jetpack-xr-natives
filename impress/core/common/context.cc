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

#include "core/common/context.h"

#include <cassert>
#include <string>
#include <utility>
#include <vector>

namespace imp {

Context::Context(JavaVM* jvm, jobject activity_context, jobject fragment_host)
    : jni_(jvm) {
  if (auto* jni_env = jni_.GetJniEnv()) {
    activity_context_ = jni_env->NewGlobalRef(activity_context);
    fragment_host_ = jni_env->NewGlobalRef(fragment_host);
  }
}

Context::Context()
    : activity_context_(nullptr),
      fragment_host_(nullptr),
      owning_ui_view_(nullptr) {}

Context::Context(int argc, char* argv[])
    : activity_context_(nullptr), fragment_host_(nullptr), arguments_(argc) {
  for (int i = 0; i < argc; i++) {
    arguments_[i] = argv[i];
  }
}

Context::Context(std::vector<std::string>&& arguments, void* owning_ui_view)
    : activity_context_(nullptr),
      fragment_host_(nullptr),
      arguments_(std::move(arguments)),
      owning_ui_view_(owning_ui_view) {}

Context::Context(const Context& other)
    : activity_context_(nullptr),
      fragment_host_(nullptr),
      owning_ui_view_(nullptr) {
  *this = other;
}

Context& Context::operator=(const Context& other) {
  if (this != &other) {
    if (activity_context_ != nullptr || fragment_host_ != nullptr) {
      // Release refs if Context is in use.
      if (auto* jni_env = jni_.TryGetJniEnv()) {
        jni_env->DeleteGlobalRef(activity_context_);
        jni_env->DeleteGlobalRef(fragment_host_);
      }
    }
    jni_ = other.jni_;
    if (auto* jni_env = jni_.GetJniEnv()) {
      activity_context_ = other.activity_context_
                              ? jni_env->NewGlobalRef(other.activity_context_)
                              : nullptr;
      fragment_host_ = other.fragment_host_
                           ? jni_env->NewGlobalRef(other.fragment_host_)
                           : nullptr;
    }
    arguments_ = other.arguments_;
    owning_ui_view_ = other.owning_ui_view_;
  }
  return *this;
}

Context::~Context() {
  if (auto* jni_env = jni_.TryGetJniEnv()) {
    jni_env->DeleteGlobalRef(activity_context_);
    jni_env->DeleteGlobalRef(fragment_host_);
  }
  // Reset ptrs to ensure misuse causes a segfault.
  activity_context_ = nullptr;
  fragment_host_ = nullptr;
}

}  // namespace imp
