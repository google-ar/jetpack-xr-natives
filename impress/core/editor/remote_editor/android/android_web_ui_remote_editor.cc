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

#include "core/editor/remote_editor/android/android_web_ui_remote_editor.h"

#include <android/native_window.h>

#include <memory>
#include <variant>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/remote_editor/android/android_remote_editor_wrapper.h"
#include "core/editor/remote_editor/remote_editor_info.h"
#include "core/editor/remote_editor/remote_editor_renderer.h"
#include "core/editor/remote_editor/remote_editor_xr_renderer.h"
#include "core/editor/remote_editor/web_ui_remote_editor.h"
#include "core/view/base_view.h"

namespace imp::editor {

namespace {
// Helper template to allow overloading lambdas for std::visit
template <class... Ts>
struct LambdaVisitor : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
LambdaVisitor(Ts...) -> LambdaVisitor<Ts...>;

// Web UI streaming mode strictly expects exactly two active WebSockets:
// 1. Script API control bridge socket.
// 2. Video streamer socket.
constexpr int kRequiredStreamingConnectionCount = 2;
}  // namespace

AndroidWebUiRemoteEditor::AndroidWebUiRemoteEditor(BaseView& view)
    : WebUiRemoteEditor(view) {
  executor_ = Executor::ForegroundExecutor();
  android_remote_editor_wrapper_ =
      std::make_unique<AndroidRemoteEditorWrapper>(view_);
}

AndroidWebUiRemoteEditor::~AndroidWebUiRemoteEditor() {
  ClearRemembered();
  // Suppress false alarms during graceful application shutdown, where the
  // foreground executor may have already been destroyed by the executor system.
  if (Executor::CurrentExecutor() && executor_ != Executor::CurrentExecutor()) {
    IMP_LOG(imp::ERROR) << "AndroidWebUiRemoteEditor must be destroyed on the "
                  "foreground thread.";
  }
  Stop();
  ReleaseNativeWindow();
  if (active_window_info_) {
    active_window_info_->RevokeWindowAndWait();
  }
}

void AndroidWebUiRemoteEditor::OnWebUiStart() {
  int http_port = 0;
  int script_api_bridge_port = 0;
  int ui_streaming_port = 0;

  std::visit(
      LambdaVisitor{
          [&](const imp::editor::RemoteEditorInfo::RemoteUiDisabledConfig&
                  config) {
            // Ports are 0, nothing to do.
          },
          [&](const imp::editor::RemoteEditorInfo::RemoteCustomUiConfig&
                  config) {
            http_port = config.http_port;
            script_api_bridge_port = config.script_api_bridge_port;
          },
          [&](const imp::editor::RemoteEditorInfo::RemoteEditorStreamingConfig&
                  config) {
            http_port = config.http_port;
            script_api_bridge_port = config.script_api_bridge_port;
            ui_streaming_port = config.ui_streaming_port;
          }},
      config_);

  if (android_remote_editor_wrapper_) {
    android_remote_editor_wrapper_->Start(
        {.http_port = http_port,
         .script_api_bridge_port = script_api_bridge_port,
         .ui_streaming_port = ui_streaming_port},
        *this);
  }

  UpdateRemoteRenderingState();
}

void AndroidWebUiRemoteEditor::OnWebUiStop() {
  if (android_remote_editor_wrapper_) {
    android_remote_editor_wrapper_->Stop();
  }

  ClearConnections();

  UpdateRemoteRenderingState();
}

bool AndroidWebUiRemoteEditor::IsReadyToSendDataToRemote() const {
  // During stop/restart, running_ clears before JNI connection_count_ reaches
  // 0.
  if (!IsRunning() || !HasActiveConnections()) {
    return false;
  }
  if (std::holds_alternative<
          imp::editor::RemoteEditorInfo::RemoteEditorStreamingConfig>(
          config_)) {
    // Inspect active_window_ptr_ under mutex lock to avoid data races against
    // concurrent disconnect/teardown assignments on background JNI threads.
    bool has_window = false;
    {
      absl::MutexLock lock(active_window_info_->lock);
      has_window = (active_window_info_->active_window_ptr_ != nullptr);
    }
    // Require surface and renderer readiness before streaming or layout switch.
    return remote_editor_renderer_ != nullptr &&
           remote_editor_renderer_->IsReady() && has_window;
  }
  return true;
}

void AndroidWebUiRemoteEditor::NotifyClientConnected() {
  // This method is called via JNI from Java WebSocket callbacks.
  // RemoteEditorWebSocketServer marshals these callbacks to the Android UI
  // thread, which corresponds to Impress's foreground thread, so no further
  // thread-switching is needed here.
  OnClientConnected();
  UpdateRemoteRenderingState();
}

void AndroidWebUiRemoteEditor::NotifyClientDisconnected() {
  // This method is called via JNI from Java WebSocket callbacks.
  // RemoteEditorWebSocketServer marshals these callbacks to the Android UI
  // thread, which corresponds to Impress's foreground thread, so no further
  // thread-switching is needed here.
  OnClientDisconnected();
  UpdateRemoteRenderingState();
}

absl::Status AndroidWebUiRemoteEditor::SetRenderTargetWindow(
    void* native_window, int width, int height) {
  if (!running_) {
    return absl::FailedPreconditionError(
        "SetRenderTargetWindow cannot be called when the remote editor is not "
        "running.");
  }
  if (!native_window) {
    return absl::InvalidArgumentError(
        "native_window cannot be null in SetRenderTargetWindow. "
        "Use ClearRenderTargetWindow instead.");
  }

  // This function may be called from background threads. We must route it to
  // the foreground thread because AndroidWebUiRemoteEditor is not thread-safe
  // and assumes all state mutations occur on the main thread.
  if (executor_ != Executor::CurrentExecutor()) {
    Future<absl::Status>::Schedule(
        [this, native_window, width, height]() -> absl::Status {
          absl::Status status =
              SetRenderTargetWindow(native_window, width, height);
          if (!status.ok()) {
            IMP_LOG(imp::ERROR) << "Failed to set editor UI render surface: " << status;
            ReleaseNativeWindow();
          }
          return status;
        },
        Executor::Type::kForeground)
        .KeptBy(this);
    return absl::OkStatus();
  }

  if (!std::holds_alternative<
          imp::editor::RemoteEditorInfo::RemoteEditorStreamingConfig>(
          config_)) {
    return absl::FailedPreconditionError(
        "SetRenderTargetWindow can only be called in kUiStreaming mode.");
  }

  filament::Engine* engine = view_.GetHost()->GetEngine();
  if (engine) {
    engine->flushAndWait();
  }
  if (remote_editor_renderer_) {
    active_window_info_->RevokeWindowAndWait();
    remote_editor_renderer_.reset();
    if (engine) {
      engine->flushAndWait();
    }
  }
  ReleaseNativeWindow();
  native_window_ = native_window;
  if (native_window_) {
    ANativeWindow_setBuffersGeometry(
        static_cast<ANativeWindow*>(native_window_), width, height,
        WINDOW_FORMAT_RGBA_8888);
  }
  {
    absl::MutexLock lock(active_window_info_->lock);
    active_window_info_->active_window_ptr_ = native_window_;
  }
  width_ = width;
  height_ = height;
  UpdateRemoteRenderingState();
  return absl::OkStatus();
}

void AndroidWebUiRemoteEditor::ClearRenderTargetWindow() {
  if (!running_) {
    return;
  }
  // This function may be called from background threads. We must route it to
  // the foreground thread because AndroidWebUiRemoteEditor is not thread-safe
  // and assumes all state mutations occur on the main thread.
  if (executor_ != Executor::CurrentExecutor()) {
    Future<absl::Status>::Schedule(
        [this]() {
          ClearRenderTargetWindow();
          return absl::OkStatus();
        },
        Executor::Type::kForeground)
        .KeptBy(this);
    return;
  }

  // Protect against dangling pointers during asynchronous application teardown,
  // where FilamentHost or Engine may already be destroyed.
  filament::Engine* engine = view_.GetHost()->GetEngine();
  if (engine) {
    engine->flushAndWait();
  }
  if (remote_editor_renderer_) {
    active_window_info_->RevokeWindowAndWait();
    remote_editor_renderer_.reset();
    if (engine) {
      engine->flushAndWait();
    }
  }
  ReleaseNativeWindow();
  width_ = 0;
  height_ = 0;
  UpdateRemoteRenderingState();
}

void AndroidWebUiRemoteEditor::ReleaseNativeWindow() {
  active_window_info_->RevokeWindowAndWait();
  if (android_remote_editor_wrapper_) {
    android_remote_editor_wrapper_->ReleaseNativeWindow(native_window_);
  }
  native_window_ = nullptr;
}

void AndroidWebUiRemoteEditor::UpdateRemoteRenderingState() {
  filament::Engine* engine = view_.GetHost()->GetEngine();
  if (engine) {
    engine->flushAndWait();
  }
  // Web UI streaming mode strictly requires both the Script API bridge and
  // Video Streamer WebSockets to be fully established before initiating the
  // rendering pipeline.
  bool should_stream =
      running_ && ConnectionCount() == kRequiredStreamingConnectionCount &&
      std::holds_alternative<
          imp::editor::RemoteEditorInfo::RemoteEditorStreamingConfig>(
          config_) &&
      native_window_;

  if (should_stream) {
    if (absl::StatusOr<std::reference_wrapper<editor::Editor>> editor_or =
            view_.GetRegistry().Get<editor::Editor>();
        editor_or.ok()) {
      if (!remote_editor_renderer_) {
        if (view_.GetHost()->IsInXr()) {
          remote_editor_renderer_ =
              std::make_unique<editor::RemoteEditorXrRenderer>(
                  view_, *active_window_info_);
        } else {
          remote_editor_renderer_ =
              std::make_unique<editor::RemoteEditorRenderer>(view_);
        }
      }
      SetDisplayMode(editor::EditorInfo::DisplayMode::kRemoteScreen);
      remote_editor_renderer_->SetRenderTargetWindow(native_window_, width_,
                                                     height_);
    }
  } else {
    if (remote_editor_renderer_) {
      active_window_info_->RevokeWindowAndWait();
      remote_editor_renderer_.reset();
      if (engine) {
        engine->flushAndWait();
      }
    }
  }
}

void AndroidWebUiRemoteEditor::BlitFromRenderTarget(
    filament::Renderer* renderer, filament::RenderTarget* render_target) {
  if (remote_editor_renderer_) {
    remote_editor_renderer_->ReadPixelsIfNeeded(renderer, render_target);
  }
}

}  // namespace imp::editor
