/*
 * Copyright 2026 Google LLC
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

#include "core/editor/remote_editor/android_remote_editor_script_api_bridge_wrapper.h"
#include "core/config.h"

#if IMP_PLATFORM(ANDROID)

#include <string>
#include <utility>

#include "core/common/jni_helpers.h"
#include "core/editor/remote_editor/android_remote_editor_wrapper.h"

namespace imp {

AndroidRemoteEditorScriptApiBridgeWrapper::
    AndroidRemoteEditorScriptApiBridgeWrapper(
        JNIEnv* env, JniUniquePtr<jobject> java_script_api_bridge,
        AndroidRemoteEditorWrapper::Callback* callback)
    : JavaWrapper(env, std::move(java_script_api_bridge),
                  "com/google/ar/imp/core/editor/RemoteEditorScriptApiBridge"),
      callback_(callback) {
  // LINT.IfChange(postMessageToScript)
  post_message_to_script_method_ =
      GetMethodHandle("postMessageToScript", "(Ljava/lang/String;)V");
  // LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/core/editor/RemoteEditorScriptApiBridge.java:postMessageToScript)
}

void AndroidRemoteEditorScriptApiBridgeWrapper::PostMessageToScript(
    const std::string& message) {
  JNIEnv* env = Env();
  JniUniquePtr<jstring> serialized_jstring = ToJniString(env, message);
  CallVoidMethod(post_message_to_script_method_, serialized_jstring.get());
  JavaExceptionPrintClear(env);
}

}  // namespace imp

#endif  // IMP_PLATFORM(ANDROID)
