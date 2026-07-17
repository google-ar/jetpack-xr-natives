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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_INFO_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_INFO_H_

#include <variant>

namespace imp::editor {

// Container for Remote Editor configuration types.
//
// The Remote Editor allows you to host a remote UI directly from your device,
// enabling remote editing and modification of the application state. This
// decoupled approach provides a more flexible way to inspect state and modify
// your scene, typically leveraging a larger screen on a remote client.
//
// For more details, see:
// google3/third_party/impress/prototypes/samples/remote_editor/g3doc/README.md
struct RemoteEditorInfo {
  // A configuration that explicitly disables remote UI, stopping it if it's
  // running.
  struct RemoteUiDisabledConfig {
    bool operator==(const RemoteUiDisabledConfig&) const = default;
  };

  // Configuration for a bespoke Web-based UI.
  //
  // This mode allows you to host a tailored HTML/JS interface that interacts
  // with your application via the Impress Scripting API.
  //
  // This is useful for creating custom "tweak" dashboards or specialized
  // workflows that leverage a larger browser screen without the layout or
  // overhead of the full in-app editor.
  struct RemoteCustomUiConfig {
    // Default port for the HTTP server.
    static constexpr int kDefaultHttpServerPort = 1234;

    // Default port for the ScriptAPI bridge server.
    static constexpr int kDefaultScriptApiBridgePort = 6277;

    // The port the HTTP server is listening on.
    int http_port = kDefaultHttpServerPort;

    // The port the ScriptAPI bridge server is listening on.
    int script_api_bridge_port = kDefaultScriptApiBridgePort;

    bool operator==(const RemoteCustomUiConfig&) const = default;
  };

  // Configuration for streaming the in-app Impress editor UI to a web-based
  // client.
  //
  // In this mode, the built-in Impress editor UI is rendered offscreen on the
  // device and streamed directly to a web browser as a video feed. This allows
  // for full, remote control of all built-in editor tools.
  //
  // When using this configuration, the remote editor mirrors the enabled state
  // of the in-app editor, starting and stopping automatically as the in-app
  // editor is toggled.
  struct RemoteEditorStreamingConfig : public RemoteCustomUiConfig {
    // Default port for the UI streaming server.
    static constexpr int kDefaultUiStreamingPort = 6278;

    // The port the UI streaming server is listening on.
    int ui_streaming_port = kDefaultUiStreamingPort;

    bool operator==(const RemoteEditorStreamingConfig&) const = default;
  };

  using RemoteUiConfig =
      std::variant<RemoteUiDisabledConfig, RemoteCustomUiConfig,
                   RemoteEditorStreamingConfig>;
};

}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_INFO_H_
