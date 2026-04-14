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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_CONFIG_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_CONFIG_H_

namespace imp {

// Specifies the operational mode for the remote editor session.
// This determines how the user interface is presented and handled
// between the host application and the remote client.
enum class RemoteEditorMode {
  // Streams the editor UI to the remote client.
  kUiStreaming,
  // Uses a custom web UI.
  kCustomUi,
};

// Configuration for the RemoteEditorServer.
struct RemoteEditorConfig {
  // Default port for the HTTP server.
  static constexpr int kDefaultHttpServerPort = 1234;
  // Default port for the ScriptAPI bridge WebSocket server.
  static constexpr int kDefaultScriptApiBridgePort = 6277;
  // Default port for the UI streaming WebSocket server.
  static constexpr int kDefaultUiStreamingPort = 6278;

  // The port the HTTP server is listening on.
  int http_port = kDefaultHttpServerPort;

  // The port the ScriptAPI bridge WebSocket server is listening on.
  int script_api_bridge_port = kDefaultScriptApiBridgePort;

  // The port the UI streaming WebSocket server is listening on.
  int ui_streaming_port = kDefaultUiStreamingPort;

  // The current operation mode.
  RemoteEditorMode mode = RemoteEditorMode::kUiStreaming;
};

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_CONFIG_H_
