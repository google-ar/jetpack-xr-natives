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

#include "core/common/jni_helpers.h"
#include "core/view/base_view.h"

namespace imp {

// JNI wrapper for lifecycle callbacks interface.
class ImpLifeCycleCallback : public JavaWrapper {
 public:
  explicit ImpLifeCycleCallback(BaseView& view, jobject callback);

  void OnEditorEnabled(bool enabled);

 private:
  JniHandle on_editor_enabled_;
};

}  // namespace imp

#endif  // THIRD_PARTY_ARCORE_AR_IMP_CORE_VIEW_PLATFORMS_ANDROID_MESSAGE_HANDLERS_ANDROID_IMP_LIFECYCLE_CALLBACK_H
