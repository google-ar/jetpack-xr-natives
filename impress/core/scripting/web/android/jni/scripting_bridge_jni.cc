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

#include <string>
#include <vector>

#include "core/common/log.h"
#include "core/common/jni_helpers.h"
#include "core/proto/proto_reader.h"
#include "core/proto/proto_writer.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/view/scripting/script_message_handler.h"
#include "core/view/view_host.h"

#define JNI_METHOD(return_type, method_name) \
  IMP_JNI return_type JNICALL                \
      Java_com_google_ar_imp_core_scripting_ScriptBridge_##method_name

namespace {

using ::imp::JniAllowlist;
using ::imp::scripting::MessageToNative;

// template <class T>
// inline jlong ToJava(T* p) {
//   return JniAllowlist<T, imp::ViewHost>::ToJava(p);
// }

template <class T>
inline T* FromJava(jlong n) {
  return JniAllowlist<T, imp::ViewHost>::FromJava(n);
}

}  // namespace

extern "C" {
// LINT.IfChange(scripting)

JNI_METHOD(jbyteArray, nPostMessage)
(JNIEnv* env, jclass /*clazz*/, jlong view_host_handle,
 jbyteArray request_bytes, jobject jargs, jobject jout) {
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

  auto* view_host = FromJava<imp::ViewHost>(view_host_handle);
  imp::scripting::ScriptMessageHandler* script_message_handler =
      view_host->GetView()->GetScriptMessageHandler();
  if (!script_message_handler) {
    IMP_LOG(imp::FATAL) << "Unable to post script message with no bridge!";
  }

  // Create a vector for the out arguments.
  std::vector<void*> out;

  imp::Future<imp::scripting::MessageToScript> future =
      script_message_handler->HandleMessage(request, args, out);

  // Handle out params.
  if (jout) {
    if (out.empty()) {
      IMP_LOG(imp::FATAL) << "Expected out params from message handler but got none.";
    }
    // Get the size of the out params list and verify it is initially empty.
    jclass cls_List = env->GetObjectClass(jout);
    jmethodID mid_size = env->GetMethodID(cls_List, "size", "()I");
    int jout_size = env->CallIntMethod(jout, mid_size);
    if (jout_size != 0) {
      IMP_LOG(imp::FATAL) << "Expected empty out list to be filled by C++.";
    }
    // Add each item from the vector to the out list as jobject.
    jmethodID mid_add =
        env->GetMethodID(cls_List, "add", "(Ljava/lang/Object;)Z");
    for (int i = 0; i < out.size(); i++) {
      env->CallBooleanMethod(jout, mid_add, static_cast<jobject>(out[i]));
    }
  } else if (!out.empty()) {
    IMP_LOG(imp::FATAL)
        << "Message handler filled out params but JNI received null out list.";
  }

  if (future.Ready()) {
    absl::StatusOr<imp::scripting::MessageToScript> response = future.Get();

    // TODO: handle error status.
    std::string serialized;
    imp::proto::SerializeTo(&response.value(), &serialized);

    return imp::ToByteArray(env, serialized);
  }
  return env->NewByteArray(0);
}

// LINT.ThenChange(
//     //depot/google3/third_party/impress/java/com/google/ar/imp/core/scripting/ScriptBridge.java:scripting
// )
}  // extern "C"
