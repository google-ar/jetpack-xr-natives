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

#ifndef THIRD_PARTY_IMPRESS_VIEW_PLATFORMS_ANDROID_UTIL_JNI_CONTEXT_H_
#define THIRD_PARTY_IMPRESS_VIEW_PLATFORMS_ANDROID_UTIL_JNI_CONTEXT_H_

#include <jni.h>

namespace imp {

// Provides Imp with access to Java-specific classes such as JNIEnv, and
// convenience methods for calling JNI methods or creating objects, using types
// defined in jni_signature.
class JniContext {
 public:
  // Initializes some global state with |env| if not already initialized.
  // We assume there is only one global JVM.
  explicit JniContext(JNIEnv* env);

  explicit JniContext(JavaVM* vm);
  JniContext() {}

  // Preferably, call SetJniEnv() with the current thread's |env| so we don't
  // need to AttachCurrentThread on the JVM.  All JNI calls from Java into C++
  // will provide a valid JNIEnv.
  void SetJniEnv(JNIEnv* env);

  // Returns the last JNIEnv set on this thread, or Attaches a new JNIEnv if
  // none available.  The attached JNIEnv will be automatically Detached when
  // the thread terminates.  Do not access this returned value from different
  // threads, get a new one for each thread.
  JNIEnv* GetJniEnv() const;

  // As above, but will not create a JNIEnv if one is not already setup on the
  // thread.
  JNIEnv* TryGetJniEnv() const;

  // Nulls out the static JVM pointer.  Should only be calling in Unit Tests.
  static void ResetJVM();
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_VIEW_PLATFORMS_ANDROID_UTIL_JNI_CONTEXT_H_
