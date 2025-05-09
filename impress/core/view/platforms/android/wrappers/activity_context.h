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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_ACTIVITY_CONTEXT_H_
#define THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_ACTIVITY_CONTEXT_H_

#include <jni.h>

#include "absl/strings/string_view.h"
#include "core/common/jni_helpers.h"
#include "core/config.h"
#include "core/view/platforms/android/wrappers/file.h"

#if IMP_PLATFORM(ANDROID)
#include <android/asset_manager.h>
#endif  // IMP_PLATFORM(ANDROID)

namespace imp::android {

// JNI wrapper for the android.content.Context class.
class ActivityContext : public JavaWrapper {
 public:
  ActivityContext(JNIEnv* env, jobject context);

  File GetFilesDir();
  File GetExternalFilesDir(absl::string_view type);
  File GetCacheDir();
#if IMP_PLATFORM(ANDROID)
  AAssetManager* GetAssets();
#endif  // IMP_PLATFORM(ANDROID)

#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
  std::string GetPackageName();
  jobject GetContentResolver();
#endif  // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)

 private:
  JniHandle get_files_dir_;
  JniHandle get_external_files_dir_;
  JniHandle get_cache_dir_;
  JniHandle get_assets_;
  JniHandle get_package_name_;
  JniHandle get_content_resolver_;
};

}  // namespace imp::android

#endif  // THIRD_PARTY_IMPRESS_CORE_VIEW_PLATFORMS_ANDROID_WRAPPERS_ACTIVITY_CONTEXT_H_
