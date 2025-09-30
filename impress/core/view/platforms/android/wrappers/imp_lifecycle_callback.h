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

#ifndef THIRD_PARTY_ARCORE_AR_IMP_CORE_VIEW_PLATFORMS_ANDROID_MESSAGE_HANDLERS_ANDROID_IMP_LIFECYCLE_CALLBACK_H
#define THIRD_PARTY_ARCORE_AR_IMP_CORE_VIEW_PLATFORMS_ANDROID_MESSAGE_HANDLERS_ANDROID_IMP_LIFECYCLE_CALLBACK_H

#include <jni.h>

#include "core/common/jni_helpers.h"
#include "core/view/base_view.h"

namespace imp {

// JNI wrapper for lifecycle callbacks interface.
class ImpLifeCycleCallback : public JavaWrapper {
 public:
  // Creates a new callback wrapper. The `callback` object _must_ implement the
  // ImpLifeCycleCallback interface:
  //     (broken link)
  //
  // This constructor will perform an IsInstanceOf check at runtime, and will
  // intentionally crash if the callback does not implement that interface, or
  // if the Java interface was accidentally removed/renamed by ProGuard.
  explicit ImpLifeCycleCallback(BaseView& view, JniUniquePtr<jobject> callback);

  void OnEditorEnabled(bool enabled);

 private:
  JniHandle on_editor_enabled_;
};

}  // namespace imp

#endif  // THIRD_PARTY_ARCORE_AR_IMP_CORE_VIEW_PLATFORMS_ANDROID_MESSAGE_HANDLERS_ANDROID_IMP_LIFECYCLE_CALLBACK_H
