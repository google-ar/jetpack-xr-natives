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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_EXOPLAYER_LISTENER_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_EXOPLAYER_LISTENER_H_

#include <jni.h>

#include "core/common/context.h"
#include "core/common/jni_helpers.h"

namespace imp::media {

class AndroidExoPlayerListener {
 public:
  virtual void OnReady() = 0;
  virtual void OnPlaybackComplete() = 0;
  virtual void OnSeekComplete() = 0;
  virtual void OnBuffering(int buffering_state) = 0;
  virtual ~AndroidExoPlayerListener() = default;
};

template <class T, class... Allowlist>
using JniAllowlist = JniAllowlist<T, Allowlist...>;

template <class T>
inline jlong ToJava(T* p) {
  return JniAllowlist<T, AndroidExoPlayerListener>::ToJava(p);
}

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, AndroidExoPlayerListener>::FromJava(n);
}

// LINT.IfChange(ImpExoPlayerListener)
class ImpExoPlayerListener : public JavaWrapper {
 public:
  ImpExoPlayerListener(const Context& context,
                       AndroidExoPlayerListener* listener)
      : JavaWrapper(context,
                    "com/google/ar/imp/core/media/ImpExoPlayerListener", "(J)V",
                    ToJava<AndroidExoPlayerListener>(listener)) {}
};
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/ImpExoPlayerListener.java:ImpExoPlayerListener
// )

}  // namespace imp::media

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_EXOPLAYER_LISTENER_H_
