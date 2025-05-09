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

#include "core/view/platforms/android/wrappers/file.h"

#include <jni.h>

#include <string>

#include "core/common/jni_helpers.h"

namespace imp::android {

File::File(JNIEnv* env, jobject file) : JavaWrapper(env, file) {
  get_path_ = GetMethodHandle("getPath", "()Ljava/lang/String;");
  get_name_ = GetMethodHandle("getName", "()Ljava/lang/String;");
}

std::string File::GetPath() {
  jobject path = CallObjectMethod(get_path_);
  return GetString(Env(), static_cast<jstring>(path));
}

std::string File::GetName() {
  jobject name = CallObjectMethod(get_name_);
  return GetString(Env(), static_cast<jstring>(name));
}

}  // namespace imp::android
