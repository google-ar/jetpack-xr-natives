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

#include "core/editor/remote_editor/remote_editor_server.h"

#include "absl/status/status.h"
#include "core/async/executor.h"
#include "core/common/jni_helpers.h"
#include "core/config.h"
#include "core/ncsb/node.h"
#include "core/view/base_view.h"

#if IMP_PLATFORM(ANDROID)
#include <jni.h>

#include <memory>
#endif

namespace imp {
namespace android {

namespace {
template <class T>
inline jlong ToJava(T* p) {
  return JniAllowlist<T, BaseView, Executor>::ToJava(p);
}

}  // namespace

RemoteEditorHttpServerWrapper::RemoteEditorHttpServerWrapper(BaseView& view,
                                                             int port)
    : JavaWrapper(view.GetContext().GetJniEnv(),
                  "com/google/ar/imp/core/editor/"
                  "RemoteEditorHttpServer",
                  "(Landroid/content/Context;I)V",
                  view.GetContext().GetActivityContext(), port) {
  CallVoidMethod(GetMethodHandle("startServer", "()V"));
}

RemoteEditorWebSocketServerWrapper::RemoteEditorWebSocketServerWrapper(
    BaseView& view, int port)
    : JavaWrapper(view.GetContext().GetJniEnv(),
                  "com/google/ar/imp/core/editor/"
                  "RemoteEditorWebSocketServer",
                  "(JJI)V", ToJava(&view),
                  ToJava(Executor::ForegroundExecutor()), port) {
  CallVoidMethod(GetMethodHandle("startServer", "()V"));
}

}  // namespace android

absl::Status RemoteEditorServer::Setup(int http_port, int web_socket_port) {
#if IMP_PLATFORM(ANDROID)
  http_server_wrapper_ =
      std::make_unique<android::RemoteEditorHttpServerWrapper>(GetView(),
                                                               http_port);

  web_socket_server_wrapper_ =
      std::make_unique<android::RemoteEditorWebSocketServerWrapper>(
          GetView(), web_socket_port);
  return absl::OkStatus();
#endif
  return absl::UnimplementedError(
      "Remote editor is not implemented for this platform.");
}

}  // namespace imp
