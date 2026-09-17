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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_H_

#include <memory>

#include "filament/filament/include/filament/RenderTarget.h"
#include "filament/filament/include/filament/Renderer.h"
#include "core/common/rememberer.h"
#include "core/editor/editor_info.h"
#include "core/editor/layout/layout_composer.h"
#include "core/editor/remote_editor/remote_editor_info.h"
#include "core/ncsb/node_handle.h"
#include "core/view/base_view.h"

namespace imp::editor {
// Interface for managing Impress remote editing, allowing a remote client
// to connect to the application.
class RemoteEditor : public Rememberer {
 public:
  // Factory function to create the appropriate RemoteEditor implementation
  // for the current platform.
  static std::unique_ptr<RemoteEditor> Create(
      const RemoteEditorInfo::RemoteUiConfig& config, BaseView& view);

  virtual ~RemoteEditor() = default;

  // Returns true if the remote editor is ready to receive input and stream
  // frames.
  virtual bool IsReadyToSendDataToRemote() const { return false; }

  // Invoked after the offscreen render pass to allow the remote editor to
  // capture or blit UI from the provided render target. Used to support
  // offscreen texture streaming (e.g. for OpenXR video streaming).
  virtual void BlitFromRenderTarget(filament::Renderer* renderer,
                                    filament::RenderTarget* render_target) {}

  // Starts the remote editor with the provided configuration.
  void Start(const RemoteEditorInfo::RemoteUiConfig& config);

  // Stops the remote editor.
  void Stop();

  // True after Start() and before Stop(), independent of active connections.
  bool IsRunning() const { return running_; }

  // Returns true if the provided config differs from the current
  // configuration.
  virtual bool HasConfigChanged(
      const RemoteEditorInfo::RemoteUiConfig& config) const = 0;

  // True if any WebSocket client is connected (can linger briefly after
  // Stop()).
  bool HasActiveConnections() const { return connection_count_ > 0; }

  // Returns the current number of active client connections.
  int ConnectionCount() const { return connection_count_; }

 protected:
  explicit RemoteEditor(BaseView& view);

  // Hook for platform-specific startup logic.
  virtual void OnStart(const RemoteEditorInfo::RemoteUiConfig& config) = 0;

  // Hook for platform-specific shutdown logic.
  virtual void OnStop() = 0;

  // Returns whether the remote editor configuration allows switching the
  // display mode when clients connect or disconnect.
  virtual bool IsDisplayModeSwitchingAllowed() const { return false; }

  // Called by derived classes when a remote editor client connects.
  void OnClientConnected();

  // Called by derived classes when a remote editor client disconnects.
  void OnClientDisconnected();

  // Called to forcefully reset all connections and revert the display mode.
  void ClearConnections();

  void SetDisplayMode(editor::EditorInfo::DisplayMode mode);

  BaseView& GetView() const { return view_; }

  BaseView& view_;

  // Indicates whether the underlying services have been started.
  bool running_ = false;

 private:
  bool ShouldSwitchToRemoteMode() const;
  bool ShouldSwitchToNativeMode() const;

  int connection_count_ = 0;
  std::unique_ptr<editor::LayoutComposer> native_layout_composer_;
};
}  // namespace imp::editor

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_REMOTE_EDITOR_H_
