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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_TYPEFACE_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_TYPEFACE_H_

#include <jni.h>

#include <string>

#include "absl/strings/string_view.h"
#include "core/common/jni_helpers.h"

namespace imp::android {

// JNI wrapper for the Android Typeface class.
class Typeface : public JavaWrapper {
 public:
  // Constructs a Typeface by calling the equivalent Typeface.create() method.
  Typeface(JNIEnv* env, absl::string_view family_name, jint style);

  // Constructs a Typeface by calling the equivalent Typeface.create() method.
  Typeface(JNIEnv* env, Typeface& family, jint weight, jboolean italic);

  // Constructs a Typeface by calling the method Typeface.create() with
  // a base template of Typeface.DEFAULT and the given style.
  Typeface(JNIEnv* env, jint style);

  // Constructs a Typeface by wrapping an existing Typeface jobject.
  Typeface(JNIEnv* env, jobject j_typeface);

  absl::string_view GetFontFamilyName() const;

 private:
  std::string font_family_name_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_TYPEFACE_H_
