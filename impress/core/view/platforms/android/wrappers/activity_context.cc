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

#include "core/view/platforms/android/wrappers/activity_context.h"

#include <jni.h>

#include "absl/strings/string_view.h"
#include "core/common/jni_helpers.h"
#include "core/config.h"
#include "core/view/platforms/android/wrappers/file.h"

#if IMP_PLATFORM(ANDROID)
#include <android/asset_manager_jni.h>
#endif  // IMP_PLATFORM(ANDROID)

namespace imp::android {

ActivityContext::ActivityContext(JNIEnv* env, jobject context)
    : JavaWrapper(env, context) {
  get_files_dir_ = GetMethodHandle("getFilesDir", "()Ljava/io/File;");
  get_external_files_dir_ = GetMethodHandle(
      "getExternalFilesDir", "(Ljava/lang/String;)Ljava/io/File;");
  get_cache_dir_ = GetMethodHandle("getCacheDir", "()Ljava/io/File;");
  get_assets_ =
      GetMethodHandle("getAssets", "()Landroid/content/res/AssetManager;");
  get_package_name_ = GetMethodHandle("getPackageName", "()Ljava/lang/String;");
  get_content_resolver_ = GetMethodHandle(
      "getContentResolver", "()Landroid/content/ContentResolver;");
}

File ActivityContext::GetFilesDir() {
  jobject files_dir = CallObjectMethod(get_files_dir_);
  return File(Env(), files_dir);
}

File ActivityContext::GetExternalFilesDir(absl::string_view type) {
  jobject files_dir =
      CallObjectMethod(get_external_files_dir_, ToString(Env(), type));
  return File(Env(), files_dir);
}

File ActivityContext::GetCacheDir() {
  jobject cache_dir = CallObjectMethod(get_cache_dir_);
  return File(Env(), cache_dir);
}

#if IMP_PLATFORM(ANDROID)
AAssetManager* ActivityContext::GetAssets() {
  jobject asset_manager = CallObjectMethod(get_assets_);
  return AAssetManager_fromJava(Env(), asset_manager);
}
#endif  // IMP_PLATFORM(ANDROID)

#if IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)
std::string ActivityContext::GetPackageName() {
  return CallStringMethod(get_package_name_);
}

jobject ActivityContext::GetContentResolver() {
  return CallObjectMethod(get_content_resolver_);
}
#endif  // IMP_PLATFORM(ANDROID) || IMP_PLATFORM(ROBOLECTRIC)

}  // namespace imp::android
