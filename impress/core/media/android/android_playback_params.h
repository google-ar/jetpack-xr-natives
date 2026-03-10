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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_PLAYBACK_PARAMS_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_PLAYBACK_PARAMS_H_

#include <jni.h>

#include <utility>

#include "core/common/jni_helpers.h"

namespace imp::media {

class AndroidPlaybackParams : public JavaWrapper {
 public:
  explicit AndroidPlaybackParams(JNIEnv* env,
                                 JniUniquePtr<jobject> playback_params)
      : JavaWrapper(env, std::move(playback_params),
                    "android/media/PlaybackParams") {
    set_speed_ =
        GetMethodHandle("setSpeed", "(F)Landroid/media/PlaybackParams;");
  }

  bool SetSpeed(float speed) {
    CallVoidMethod(set_speed_, speed);
    return !CheckIfException(Env());
  }

 private:
  // TODO: Properly log the exceptions thrown
  bool CheckIfException(JNIEnv* env) {
    if (env->ExceptionCheck()) {
      env->ExceptionDescribe();
      env->ExceptionClear();
      return true;
    }
    return false;
  }

  imp::JniHandle set_speed_;
};

}  // namespace imp::media

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_PLAYBACK_PARAMS_H_
