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

#include "core/view/platforms/android/wrappers/imp_lifecycle_callback.h"

#include <jni.h>

#include <utility>

#include "core/common/jni_helpers.h"
#include "core/view/base_view.h"

namespace imp {

ImpLifeCycleCallback::ImpLifeCycleCallback(BaseView& view,
                                           JniUniquePtr<jobject> callback)
    : JavaWrapper(view.GetContext().GetJniEnv(), std::move(callback),
                  "com/google/ar/imp/view/ImpApi$ImpLifeCycleCallback") {
  on_editor_enabled_ = GetMethodHandle("onEditorEnabled", "(Z)V");
}

void ImpLifeCycleCallback::OnEditorEnabled(bool enabled) {
  CallVoidMethod(on_editor_enabled_, enabled);
}

}  // namespace imp
