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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_REMOTE_EDITOR_SCRIPT_API_BRIDGE_WRAPPER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_REMOTE_EDITOR_SCRIPT_API_BRIDGE_WRAPPER_H_

#include "core/config.h"

#if IMP_PLATFORM(ANDROID)

#include <string>

#include "core/common/jni_helpers.h"
#include "core/editor/remote_editor/android_remote_editor_wrapper.h"

namespace imp {

// Android implementation of a C++ wrapper for the Java
// RemoteEditorScriptApiBridge class.
class AndroidRemoteEditorScriptApiBridgeWrapper : public JavaWrapper {
 public:
  AndroidRemoteEditorScriptApiBridgeWrapper(
      JNIEnv* env, JniUniquePtr<jobject> java_script_api_bridge,
      AndroidRemoteEditorWrapper::Callback* callback);

  // Sends a message to the script via the Java ScriptApiBridge server.
  void PostMessageToScript(const std::string& message);

  AndroidRemoteEditorWrapper::Callback* GetCallback() const {
    return callback_;
  }

 private:
  JniHandle post_message_to_script_method_;
  AndroidRemoteEditorWrapper::Callback* callback_ = nullptr;
};

}  // namespace imp

#endif  // IMP_PLATFORM(ANDROID)

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_ANDROID_REMOTE_EDITOR_SCRIPT_API_BRIDGE_WRAPPER_H_
