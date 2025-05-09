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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_BITMAP_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_BITMAP_H_

#include "absl/status/statusor.h"
#include "core/common/jni_helpers.h"

namespace imp::android {

// https://developer.android.com/reference/android/graphics/Bitmap.Config
enum class BitmapConfig {
  ALPHA_8,
  ARGB_4444,
  ARGB_8888,
  RGBA_F16,
  RGB_565,
};

class Bitmap : public JavaWrapper {
 public:
  Bitmap(JNIEnv* env, jobject jbitmap);

  void Recycle();

  absl::StatusOr<BitmapConfig> GetBitmapConfig();

 private:
  JniHandle recycle_;
  JniHandle get_config_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_BITMAP_H_
