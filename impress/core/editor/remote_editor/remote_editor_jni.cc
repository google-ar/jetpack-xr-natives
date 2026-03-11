/*
 * Copyright 2025 Google LLC
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

#include <jni.h>

#include <string>
#include <utility>

#include "core/common/log.h"
#include "core/async/executor.h"
#include "core/common/buffer_access.h"
#include "core/common/jni_helpers.h"
#include "core/proto/proto_reader.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/view/base_view.h"
#include "core/view/scripting/script_message_handler.h"

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_core_editor_RemoteEditorWebSocketServer_##method_name  // NOLINT

namespace {

using ::imp::JniAllowlist;

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, imp::BaseView, imp::Executor>::FromJava(n);
}

}  // namespace

extern "C" {

JNI_METHOD(void, nPostMessageToNative)
(JNIEnv* env, jclass /*clazz*/, jobject websocket_server, jlong view_handle,
 jlong executor_handle, jbyteArray request_bytes) {
  // Parse the incoming byte array into a MessageToNative proto.
  imp::BufferAccess byte_buffer = imp::FromByteArray(env, request_bytes);
  imp::scripting::MessageToNative request;
  imp::proto::ParseMessage(byte_buffer.StringView(), &request);

  // Get access to the ScriptingSystem.
  auto* view = FromJava<imp::BaseView>(view_handle);
  imp::scripting::ScriptMessageHandler* script_message_handler =
      view->GetScriptMessageHandler();
  if (!script_message_handler) {
    IMP_LOG(imp::FATAL) << "Unable to post script message with no bridge!";
  }

  // Create a weak global reference to the websocket server object. This
  // prevents the Java object from being garbage collected while it's still
  // needed in C++.
  imp::JniUniquePtr<jobject> websocket_server_global =
      imp::WrapJni(env, env->NewWeakGlobalRef(websocket_server));

  // This call is coming from a websocket server thread, so we need to schedule
  // the message handling to be executed on the foreground thread.
  auto* executor = FromJava<imp::Executor>(executor_handle);

  // Note: it is critical to move the byte_buffer into the lambda. The message
  // proto may have references directly to the byte buffer data, so its lifetime
  // must extend through the call to HandleMessage.
  executor->ScheduleInvocable([script_message_handler,
                               byte_buffer = std::move(byte_buffer), request,
                               websocket_server_global = std::move(
                                   websocket_server_global)]() mutable {
    script_message_handler->HandleMessage(
        request,
        [script_bridge_global = std::move(websocket_server_global)](
            const imp::scripting::MessageToScript& response, void* out) {
          // Note: using the JniUniquePtr deleter is the only way to get the
          // JNI Env* for this object without capturing in the lambda, which
          // would result in the lambda being 24 bytes and unable to use
          // small-object-optimization.
          auto& custom_deleter = script_bridge_global.get_deleter();
          JNIEnv* env = custom_deleter.env();
          if (env->IsSameObject(script_bridge_global.get(), nullptr)) {
            IMP_LOG(imp::ERROR) << "Script bridge is null in response_handler!";
            return;
          }

          // Serialize the MessageToScript proto to a Base64 string, which
          // is the expected format for Javascript responses. It will be
          // converted back to a byte array in websocket_entry_point.cc.
          std::string serialized = imp::scripting::SerializeToBase64(response);
          imp::JniUniquePtr<jstring> serialized_jstring =
              imp::ToJniString(env, serialized);
          // Find the Java class for the WebSocket server.
          jclass script_api_class = env->FindClass(
              "com/google/ar/imp/core/editor/RemoteEditorWebSocketServer");
          // Get the method ID for the Java postMessageToScript method.
          jmethodID post_message_method = env->GetMethodID(
              script_api_class, "postMessageToScript", "(Ljava/lang/String;)V");
          env->CallVoidMethod(script_bridge_global.get(), post_message_method,
                              serialized_jstring.get());
        });
  });
}

}  // extern "C"
