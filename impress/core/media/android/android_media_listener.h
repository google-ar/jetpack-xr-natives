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

#ifndef THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_LISTENER_H_
#define THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_LISTENER_H_

#include <jni.h>

#include "core/common/jni_helpers.h"

namespace imp::media {

class AndroidMediaListener {
 public:
  virtual void OnPlaybackComplete() = 0;
  virtual void OnSeekComplete() = 0;
  virtual void OnReady() = 0;
  virtual bool OnError(int what, int extra) = 0;
  virtual bool OnInfo(int what, int extra) = 0;
  virtual ~AndroidMediaListener() = default;
};

template <class T, class... Allowlist>
using JniAllowlist = JniAllowlist<T, Allowlist...>;

template <class T>
inline jlong ToJava(T* p) {
  return JniAllowlist<T, AndroidMediaListener>::ToJava(p);
}

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, AndroidMediaListener>::FromJava(n);
}

// LINT.IfChange(OnCompletionListener)
class OnCompletionListener : public JavaWrapper {
 public:
  OnCompletionListener(const Context& context, AndroidMediaListener* listener)
      : JavaWrapper(context,
                    "com/google/ar/imp/core/media/OnCompletionListener", "(J)V",
                    ToJava<AndroidMediaListener>(listener)) {}
};
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/OnCompletionListener.java:OnCompletionListener
// )

// LINT.IfChange(OnSeekCompleteListener)
class OnSeekCompleteListener : public JavaWrapper {
 public:
  OnSeekCompleteListener(const Context& context, AndroidMediaListener* listener)
      : JavaWrapper(context,
                    "com/google/ar/imp/core/media/OnSeekCompleteListener",
                    "(J)V", ToJava<AndroidMediaListener>(listener)) {}
};
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/OnSeekCompleteListener.java:OnSeekCompleteListener
// )

// LINT.IfChange(OnPreparedListener)
class OnPreparedListener : public JavaWrapper {
 public:
  OnPreparedListener(const Context& context, AndroidMediaListener* listener)
      : JavaWrapper(context, "com/google/ar/imp/core/media/OnPreparedListener",
                    "(J)V", ToJava<AndroidMediaListener>(listener)) {}
};
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/OnPreparedListener.java:OnPreparedListener
// )

// LINT.IfChange(OnErrorListener)
class OnErrorListener : public JavaWrapper {
 public:
  OnErrorListener(const Context& context, AndroidMediaListener* listener)
      : JavaWrapper(context, "com/google/ar/imp/core/media/OnErrorListener",
                    "(J)V", ToJava<AndroidMediaListener>(listener)) {
    ToJava<AndroidMediaListener>(listener);
  }
};
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/OnErrorListener.java:OnErrorListener
// )

// LINT.IfChange(OnInfoListener)
class OnInfoListener : public JavaWrapper {
 public:
  OnInfoListener(const Context& context, AndroidMediaListener* listener)
      : JavaWrapper(context, "com/google/ar/imp/core/media/OnInfoListener",
                    "(J)V", ToJava<AndroidMediaListener>(listener)) {
    ToJava<AndroidMediaListener>(listener);
  }
};
// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/media/OnInfoListener.java:OnInfoListener
// )

}  // namespace imp::media

#endif  // THIRD_PARTY_IMPRESS_CORE_MEDIA_ANDROID_ANDROID_MEDIA_LISTENER_H_
