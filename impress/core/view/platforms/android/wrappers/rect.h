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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_RECT_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_RECT_H_

#include "core/common/context.h"
#include "core/common/jni_helpers.h"
#include "core/math/vec.h"

namespace imp::android {

// JNI wrapper for the Android Rect class.
class Rect : public JavaWrapper {
 public:
  explicit Rect(JNIEnv* env);

  int GetBottom();
  int GetLeft();
  int GetRight();
  int GetTop();

  int GetWidth();
  int GetHeight();

 private:
  JniHandle bottom_;
  JniHandle left_;
  JniHandle right_;
  JniHandle top_;

  JniHandle width_;
  JniHandle height_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_RECT_H_
