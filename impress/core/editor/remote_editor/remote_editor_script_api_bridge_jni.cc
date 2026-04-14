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

#include "core/config.h"

#if IMP_PLATFORM(ANDROID)

#include <string>
#include <utility>

#include "absl/base/call_once.h"
#include "core/common/log.h"
#include "core/async/executor.h"
#include "core/common/buffer_access.h"
#include "core/common/jni_helpers.h"
#include "core/common/registry.h"
#include "core/editor/remote_editor/android_remote_editor_script_api_bridge_wrapper.h"
#include "core/proto/proto_reader.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/scripting/script_message_handler.h"

#define JNI_SCRIPT_API_BRIDGE_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                                  \
      Java_com_google_ar_imp_core_editor_RemoteEditorScriptApiBridge_##method_name  // NOLINT

namespace {

using ::imp::JniAllowlist;

template <class T>
T* FromJava(jlong n) {
  return JniAllowlist<
      T, imp::BaseView, imp::Executor,
      imp::android::AndroidRemoteEditorScriptApiBridgeWrapper>::FromJava(n);
}

}  // namespace

extern "C" {

// LINT.IfChange(nativePostMessageToNative)
JNI_SCRIPT_API_BRIDGE_METHOD(void, nativePostMessageToNative)
(JNIEnv* env, jobject thiz, jlong script_api_bridge_wrapper_ptr,
 jlong view_handle, jlong executor_handle, jbyteArray request_bytes) {
  // Parse the incoming byte array into a MessageToNative proto.
  if (!script_api_bridge_wrapper_ptr) {
    IMP_LOG(imp::ERROR)
        << "script_api_bridge_wrapper_ptr is null in postMessageToNative";
    return;
  }
  if (!request_bytes) {
    IMP_LOG(imp::ERROR) << "request_bytes is null in postMessageToNative";
    return;
  }
  imp::BufferAccess byte_buffer = imp::FromByteArray(env, request_bytes);
  imp::scripting::MessageToNative request;
  if (!imp::proto::ParseMessage(byte_buffer.StringView(), &request)) {
    IMP_LOG(imp::ERROR) << "Failed to parse MessageToNative in postMessageToNative";
    return;
  }

  // Get access to the ScriptMessageHandler.
  imp::BaseView* view = FromJava<imp::BaseView>(view_handle);
  if (!view) {
    IMP_LOG(imp::ERROR) << "View is null in postMessageToNative";
    return;
  }

  imp::scripting::ScriptMessageHandler* script_message_handler =
      view->GetScriptMessageHandler();
  if (!script_message_handler) {
    IMP_LOG(imp::FATAL) << "Unable to post script message with no bridge!";
  }

  imp::android::AndroidRemoteEditorScriptApiBridgeWrapper* wrapper =
      FromJava<imp::android::AndroidRemoteEditorScriptApiBridgeWrapper>(
          script_api_bridge_wrapper_ptr);

  // This call is coming from a websocket server thread, so we need to schedule
  // the message handling to be executed on the foreground thread.
  imp::Executor* executor = FromJava<imp::Executor>(executor_handle);
  if (!executor) {
    IMP_LOG(imp::ERROR) << "Executor is null in postMessageToNative";
    return;
  }

  // Note: it is critical to move the byte_buffer into the lambda. The message
  // proto may have references directly to the byte buffer data, so its lifetime
  // must extend through the call to HandleMessage.
  // The inner lambda captures only `wrapper` (a single pointer) which easily
  // qualifies for std::function's small-object-optimization (SOO).
  executor->ScheduleInvocable([script_message_handler,
                               byte_buffer = std::move(byte_buffer),
                               request = std::move(request),
                               wrapper]() mutable {
    script_message_handler->HandleMessage(
        request,
        [wrapper](const imp::scripting::MessageToScript& response, void* out) {
          // Serialize the MessageToScript proto to a Base64 string, which
          // is the expected format for Javascript responses. It will be
          // converted back to a byte array in websocket_entry_point.ts.
          std::string serialized = imp::scripting::SerializeToBase64(response);

          wrapper->PostMessageToScript(serialized);
        });
  });
}
// LINT.ThenChange(//depot/google3/third_party/impress/java/com/google/ar/imp/core/editor/RemoteEditorScriptApiBridge.java:nativePostMessageToNative)

}  // extern "C"

#endif  // IMP_PLATFORM(ANDROID)
