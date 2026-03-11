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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_WEB_SOCKET_SERVER_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_WEB_SOCKET_SERVER_H_

#include "absl/status/status.h"
#include "core/common/jni_helpers.h"
#include "core/config.h"
#include "core/ncsb/component.h"

#if IMP_PLATFORM(ANDROID)
#include <memory>
#endif

namespace imp {
namespace android {

// JNI wrapper for the RemoteEditorHttpServer class.
class RemoteEditorHttpServerWrapper : public JavaWrapper {
 public:
  RemoteEditorHttpServerWrapper(BaseView& view, int port);
};

// JNI wrapper for the RemoteEditorWebSocketServer class.
class RemoteEditorWebSocketServerWrapper : public JavaWrapper {
 public:
  RemoteEditorWebSocketServerWrapper(BaseView& view, int port);
};
}  // namespace android

// Create this component to start the remote editor server.
// This component will start HTTP and WebSocket servers on the given ports.
// It will serve web files from the android assets (it assumes an index.html
// file is present in the assets). The WebSocket server will communicate with
// the remote editor client, and relay messages back and forth between the
// client and the Impress scripting API.
//
// This assumes you have created an imp::scripting::ScriptingSystem separately.
class RemoteEditorServer : public Component {
 public:
  // The default port for the HTTP server.
  static constexpr int kDefaultHttpServerPort = 1234;
  // The default port for the WebSocket server.
  static constexpr int kDefaultWebSocketServerPort = 6277;

  absl::Status Setup(int http_port = kDefaultHttpServerPort,
                     int web_socket_port = kDefaultWebSocketServerPort);

 private:
#if IMP_PLATFORM(ANDROID)
  std::unique_ptr<android::RemoteEditorHttpServerWrapper> http_server_wrapper_;
  std::unique_ptr<android::RemoteEditorWebSocketServerWrapper>
      web_socket_server_wrapper_;
#endif
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_WEB_SOCKET_SERVER_H_
