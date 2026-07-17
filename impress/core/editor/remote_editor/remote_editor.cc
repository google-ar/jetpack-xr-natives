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

#include "core/editor/remote_editor/remote_editor.h"

#include <functional>
#include <memory>
#include <utility>
#include <variant>

#include "core/common/log.h"
#include "absl/status/statusor.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/editor_constants.h"
#include "core/editor/editor_info.h"
#include "core/editor/layout/layout_composer.h"
#include "core/editor/remote_editor/noop_remote_editor.h"
#include "core/editor/remote_editor/remote_editor_info.h"
#include "core/editor/widget_ui_system.h"
#include "core/view/base_view.h"

#if IMP_PLATFORM(ANDROID)
#include "core/common/registry.h"
#include "core/editor/remote_editor/android_web_ui_remote_editor.h"
#include "core/scripting/message_handlers/forward_input_handler.h"
#include "core/scripting/scripting_system.h"
#endif

namespace imp::editor {

std::unique_ptr<RemoteEditor> RemoteEditor::Create(
    const RemoteEditorInfo::RemoteUiConfig& config, BaseView& view) {
  if (std::holds_alternative<RemoteEditorInfo::RemoteUiDisabledConfig>(
          config)) {
    IMP_LOG(imp::INFO) << "Remote Editor: Disabled";
    return std::make_unique<NoopRemoteEditor>(view);
  }

  // TODO: Remove this check when RemoteEditor is supported on
  // SplitEngineSerializer.
  if (view.GetSplitEngineSerializer() != nullptr &&
      std::holds_alternative<RemoteEditorInfo::RemoteEditorStreamingConfig>(
          config)) {
    IMP_LOG(imp::INFO)
        << "Remote Editor: Streaming disabled due to SplitEngineSerializer.";
    return std::make_unique<NoopRemoteEditor>(view);
  }

#if IMP_PLATFORM(ANDROID)
  bool is_streaming =
      std::holds_alternative<RemoteEditorInfo::RemoteEditorStreamingConfig>(
          config);
  if (is_streaming ||
      std::holds_alternative<RemoteEditorInfo::RemoteCustomUiConfig>(config)) {
    imp::scripting::ScriptingSystem& scripting_system =
        view.GetRegistry().GetOrRegister<imp::scripting::ScriptingSystem>(
            [&view]() {
              return std::make_unique<imp::scripting::ScriptingSystem>(view);
            });
    if (is_streaming) {
      // UI streaming mode requires the forward input handler to forward input
      // events from the remote client to the Impress engine.
      scripting_system.AddHandler(
          std::make_unique<scripting::ForwardInputHandler>(view));
    }
    return std::make_unique<AndroidWebUiRemoteEditor>(view);
  }
#endif

  return std::make_unique<NoopRemoteEditor>(view);
}

RemoteEditor::RemoteEditor(BaseView& view) : view_(view) {}

void RemoteEditor::Start(const RemoteEditorInfo::RemoteUiConfig& config) {
  if (running_ && !HasConfigChanged(config)) {
    return;
  }

  if (running_) {
    Stop();
  }

  running_ = true;
  OnStart(config);
}

void RemoteEditor::Stop() {
  if (!running_) {
    return;
  }

  running_ = false;
  OnStop();
}

void RemoteEditor::SetDisplayMode(editor::EditorInfo::DisplayMode mode) {
  absl::StatusOr<std::reference_wrapper<editor::Editor>> editor =
      view_.GetRegistry().Get<editor::Editor>();
  if (!editor.ok()) {
    return;
  }
  editor->get().SetDisplayMode(mode);

  editor::WidgetUiSystem& widget_ui_system = editor->get().GetWidgetUiSystem();
  if (mode == editor::EditorInfo::DisplayMode::kRemoteScreen) {
    // Extract the active layout to safely store the app's native layout
    // (e.g., mobile or XR) so it can be restored when the client disconnects.
    // We only transition to kRemoteScreen on the first client connection, so
    // we should never already have a cached native layout here.
    native_layout_composer_ = widget_ui_system.ReleaseLayoutComposer();

    // Apply the default desktop layout for the remote web UI.
    widget_ui_system.SetLayoutComposer(std::make_unique<editor::LayoutComposer>(
        editor::kDefaultDesktopLayoutConfig));
  } else if (mode == editor::EditorInfo::DisplayMode::kNativeScreen) {
    // Restore the original native layout. Note that std::move automatically
    // nullifies native_layout_composer_ after transferring ownership,
    // readying it for the next remote connection.
    widget_ui_system.SetLayoutComposer(std::move(native_layout_composer_));
  }
}

void RemoteEditor::OnClientConnected() {
  connection_count_++;
  if (ShouldSwitchToRemoteMode()) {
    SetDisplayMode(editor::EditorInfo::DisplayMode::kRemoteScreen);
  }
}

void RemoteEditor::OnClientDisconnected() {
  if (connection_count_ > 0) {
    connection_count_--;
    if (ShouldSwitchToNativeMode()) {
      SetDisplayMode(editor::EditorInfo::DisplayMode::kNativeScreen);
    }
  } else {
    IMP_LOG(imp::WARNING) << "Received a client disconnect event, but the connection "
                    "count is already 0. "
                 << "This may indicate duplicate disconnect events or a "
                    "connection tracking bug.";
  }
}

void RemoteEditor::ClearConnections() {
  if (connection_count_ > 0) {
    connection_count_ = 0;
    if (ShouldSwitchToNativeMode()) {
      SetDisplayMode(editor::EditorInfo::DisplayMode::kNativeScreen);
    }
  }
}

bool RemoteEditor::ShouldSwitchToRemoteMode() const {
  return connection_count_ == 1 && IsDisplayModeSwitchingAllowed();
}

bool RemoteEditor::ShouldSwitchToNativeMode() const {
  return connection_count_ == 0 && IsDisplayModeSwitchingAllowed();
}

}  // namespace imp::editor
