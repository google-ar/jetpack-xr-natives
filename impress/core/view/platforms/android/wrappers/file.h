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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_FILE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_FILE_H_

#include <jni.h>

#include <string>

#include "core/common/jni_helpers.h"

namespace imp::android {

// JNI wrapper for the Java IO File class.
class File : public JavaWrapper {
 public:
  File(JNIEnv* env, JniUniquePtr<jobject> file);

  std::string GetPath();
  std::string GetName();

 private:
  JniHandle get_path_;
  JniHandle get_name_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_FILE_H_
