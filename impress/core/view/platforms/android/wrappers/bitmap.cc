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

#include "core/view/platforms/android/wrappers/bitmap.h"

namespace imp::android {

namespace {

absl::StatusOr<BitmapConfig> GetBitmapConfigFromJava(JNIEnv* env,
                                                     jobject config) {
  JniUniquePtr<jclass> bitmap_config_class =
      FindClass(env, "android/graphics/Bitmap$Config");
  auto compare_config = [env, bitmap_config_class = bitmap_config_class.get(),
                         config](const char* name) {
    auto reference_config_id = env->GetStaticFieldID(
        bitmap_config_class, name, "Landroid/graphics/Bitmap$Config;");
    auto reference_config =
        env->GetStaticObjectField(bitmap_config_class, reference_config_id);
    return env->IsSameObject(config, reference_config);
  };

  if (compare_config("ARGB_8888")) {
    return BitmapConfig::ARGB_8888;
  } else if (compare_config("RGBA_F16")) {
    return BitmapConfig::RGBA_F16;
  } else if (compare_config("RGB_565")) {
    return BitmapConfig::RGB_565;
  } else if (compare_config("ARGB_4444")) {
    return BitmapConfig::ARGB_4444;
  } else if (compare_config("ALPHA_8")) {
    return BitmapConfig::ALPHA_8;
  } else {
    return absl::InternalError("Unable to determine bitmap config");
  }
}

}  // namespace

Bitmap::Bitmap(JNIEnv* env, jobject jbitmap) : JavaWrapper(env, jbitmap) {
  recycle_ = GetMethodHandle("recycle", "()V");
  get_config_ =
      GetMethodHandle("getConfig", "()Landroid/graphics/Bitmap$Config;");
  assert(recycle_);
}

void Bitmap::Recycle() { CallVoidMethod(recycle_); }

absl::StatusOr<BitmapConfig> Bitmap::GetBitmapConfig() {
  return GetBitmapConfigFromJava(Env(), CallObjectMethod(get_config_));
}

}  // namespace imp::android
