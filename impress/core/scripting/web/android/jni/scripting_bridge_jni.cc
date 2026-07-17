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

#include <jni.h>

#include <string>
#include <utility>
#include <vector>

#include "core/common/log.h"
#include "core/common/buffer_access.h"
#include "core/common/jni_helpers.h"
#include "core/proto/proto_reader.h"
#include "core/proto/proto_writer.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/view/scripting/script_message_handler.h"
#include "core/view/scripting/script_message_handler_provider.h"

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_core_scripting_ScriptBridge_##method_name

namespace {

using ::imp::JniAllowlist;
using ::imp::scripting::MessageToNative;

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<
      T, imp::scripting::ScriptMessageHandlerProvider>::FromJava(n);
}

}  // namespace

extern "C" {
// LINT.IfChange(scripting)

JNI_METHOD(void, nPostMessage)
(JNIEnv* env, jclass /*clazz*/, jobject script_bridge,
 jlong script_message_handler_provider_handle, jbyteArray request_bytes,
 jobject jargs) {
  imp::BufferAccess byte_buffer = imp::FromByteArray(env, request_bytes);
  imp::scripting::MessageToNative request;
  imp::proto::ParseMessage(byte_buffer.StringView(), &request);

  // Convert jargs into a std::vector<void*>.
  std::vector<void*> args;
  if (jargs) {
    jclass cls_List = env->GetObjectClass(jargs);
    jmethodID mid_size = env->GetMethodID(cls_List, "size", "()I");
    int jargs_size = env->CallIntMethod(jargs, mid_size);
    args.reserve(jargs_size);
    jmethodID mid_iterator =
        env->GetMethodID(cls_List, "iterator", "()Ljava/util/Iterator;");
    jobject iterator = env->CallObjectMethod(jargs, mid_iterator);
    jclass cls_Iterator = env->GetObjectClass(iterator);
    jmethodID mid_hasNext = env->GetMethodID(cls_Iterator, "hasNext", "()Z");
    jmethodID mid_next =
        env->GetMethodID(cls_Iterator, "next", "()Ljava/lang/Object;");
    while (env->CallBooleanMethod(iterator, mid_hasNext)) {
      jobject arg = env->CallObjectMethod(iterator, mid_next);
      args.push_back(arg);
    }
  }

  imp::scripting::ScriptMessageHandlerProvider*
      script_message_handler_provider =
          FromJava<imp::scripting::ScriptMessageHandlerProvider>(
              script_message_handler_provider_handle);
  if (!script_message_handler_provider) {
    IMP_LOG(imp::ERROR)
        << "ScriptMessageHandlerProvider is null, unable to post message.";
    return;
  }
  imp::scripting::ScriptMessageHandler* script_message_handler =
      script_message_handler_provider->GetScriptMessageHandler();
  if (!script_message_handler) {
    IMP_LOG(imp::FATAL) << "Unable to post script message with no bridge!";
  }

  imp::JniUniquePtr<jobject> script_bridge_global =
      imp::WrapJni(env, env->NewWeakGlobalRef(script_bridge));

  script_message_handler->HandleMessage(
      request, args,
      [script_bridge_global = std::move(script_bridge_global)](
          const imp::scripting::MessageToScript& response, void* out) {
        // Note: using the JniUniquePtr deleter is the only way to get the JNI
        // Env* for this object without capturing in the lambda, which would
        // result in the lambda being 24 bytes and unable to use
        // small-object-optimization.
        const imp::details::JniDeleter<jobject>& custom_deleter =
            script_bridge_global.get_deleter();
        JNIEnv* env = custom_deleter.env();
        if (env->IsSameObject(script_bridge_global.get(), nullptr)) {
          IMP_LOG(imp::ERROR) << "Script bridge is null in response_handler!";
          return;
        }

        std::string serialized;
        imp::proto::SerializeTo(&response, &serialized);
        jclass script_api_class =
            env->FindClass("com/google/ar/imp/core/scripting/ScriptBridge");
        jmethodID post_message_method = env->GetMethodID(
            script_api_class, "postMessage", "([BLjava/lang/Object;)V");
        env->CallVoidMethod(script_bridge_global.get(), post_message_method,
                            imp::ToByteArray(env, serialized),
                            static_cast<jobject>(out));
      });
}

// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/scripting/ScriptBridge.java:scripting
// )
}  // extern "C"
