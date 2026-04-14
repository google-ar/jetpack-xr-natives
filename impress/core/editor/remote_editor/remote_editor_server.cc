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

#include "core/editor/remote_editor/remote_editor_server.h"

#include <functional>
#include <memory>

#include "core/common/log.h"
#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "core/async/executor.h"
#include "core/async/future.h"
#include "core/config.h"
#include "core/editor/editor.h"
#include "core/editor/editor_constants.h"
#include "core/editor/layout/layout_composer.h"
#include "core/editor/remote_editor/remote_editor_config.h"
#include "core/editor/remote_editor/remote_editor_renderer.h"
#include "core/editor/widget_ui_system.h"
#include "core/view/base_view.h"

#if IMP_PLATFORM(ANDROID)
#include "core/editor/remote_editor/android_remote_editor_server_wrapper.h"
#endif

namespace imp {
namespace {
const char* ModeToString(RemoteEditorServer::Mode mode) {
  switch (mode) {
#define CASE(m)                     \
  case RemoteEditorServer::Mode::m: \
    return #m
    CASE(kUiStreaming);
    CASE(kCustomUi);
#undef CASE
  }
  return "Unknown";
}
}  // namespace

absl::StatusOr<std::unique_ptr<RemoteEditorServer>> RemoteEditorServer::Create(
    BaseView& view, const RemoteEditorConfig& config) {
#if IMP_PLATFORM(ANDROID)
  if (!view.GetScriptMessageHandler()) {
    return absl::FailedPreconditionError(
        "RemoteEditorServer requires a ScriptMessageHandler to be attached to "
        "the BaseView.");
  }

  std::unique_ptr<RemoteEditorServer> server(
      new RemoteEditorServer(view, config));

  if (absl::StatusOr<std::reference_wrapper<editor::Editor>> editor_or =
          view.GetRegistry().Get<editor::Editor>();
      editor_or.ok()) {
    editor_or->get().SetEnabled(true);
  }

  server->Start();
  return server;
#else   // IMP_PLATFORM(ANDROID)
  return absl::UnimplementedError(
      "RemoteEditorServer is not implemented for this platform.");
#endif  // IMP_PLATFORM(ANDROID)
}

RemoteEditorServer::RemoteEditorServer(BaseView& view,
                                       const RemoteEditorConfig& config)
    : view_(view) {
  executor_ = Executor::ForegroundExecutor();
  config_ = config;
#if IMP_PLATFORM(ANDROID)
  remote_editor_server_wrapper_ =
      std::make_unique<android::AndroidRemoteEditorServerWrapper>(view_);
#endif  // IMP_PLATFORM(ANDROID)
}

RemoteEditorServer::~RemoteEditorServer() {
  ClearRemembered();
  if (executor_ != Executor::CurrentExecutor()) {
    IMP_LOG(imp::DFATAL)
        << "RemoteEditorServer must be destroyed on the foreground thread.";
  }
  Stop();
  ReleaseNativeWindow();
}

void RemoteEditorServer::Start() { Start(config_); }

void RemoteEditorServer::Start(const RemoteEditorConfig& config) {
  if (running_) {
    // Already running the same config - do nothing.
    if (config_.http_port == config.http_port &&
        config_.script_api_bridge_port == config.script_api_bridge_port &&
        config_.ui_streaming_port == config.ui_streaming_port &&
        config_.mode == config.mode) {
      return;
    }
  }

  Stop();
  config_ = config;
  running_ = true;

  if (remote_editor_server_wrapper_) {
    remote_editor_server_wrapper_->StartServer(config_, *this);
  }

  UpdateRemoteRenderingState();
}

void RemoteEditorServer::Stop() {
  if (!running_) {
    return;
  }

  IMP_LOG(imp::INFO) << "Stopping Remote Editor Server.";
  running_ = false;
  if (remote_editor_server_wrapper_) {
    remote_editor_server_wrapper_->StopServer();
  }

  UpdateRemoteRenderingState();
}

absl::Status RemoteEditorServer::SetRenderTargetWindow(void* native_window,
                                                       int width, int height) {
  if (!running_) {
    return absl::FailedPreconditionError(
        "SetRenderTargetWindow cannot be called when server is not "
        "running.");
  }
  if (!native_window) {
    return absl::InvalidArgumentError(
        "native_window cannot be null in SetRenderTargetWindow. "
        "Use ClearRenderTargetWindow instead.");
  }

  // This function may be called from background threads. We must route it to
  // the foreground thread because RemoteEditorServer is not thread-safe and
  // assumes all state mutations occur on the main thread.
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

  if (config_.mode != Mode::kUiStreaming) {
    return absl::FailedPreconditionError(
        absl::StrCat("SetRenderTargetWindow can only be called in kUiStreaming "
                     "mode. Current mode is ",
                     ModeToString(config_.mode), "."));
  }

  ReleaseNativeWindow();
  native_window_ = native_window;
  width_ = width;
  height_ = height;
  UpdateRemoteRenderingState();
  return absl::OkStatus();
}

void RemoteEditorServer::ClearRenderTargetWindow() {
  if (!running_) {
    return;
  }
  // This function may be called from background threads. We must route it to
  // the foreground thread because RemoteEditorServer is not thread-safe and
  // assumes all state mutations occur on the main thread.
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

  ReleaseNativeWindow();
  width_ = 0;
  height_ = 0;
  UpdateRemoteRenderingState();
}

void RemoteEditorServer::ReleaseNativeWindow() {
  if (remote_editor_server_wrapper_) {
    remote_editor_server_wrapper_->ReleaseNativeWindow(native_window_);
  }
  native_window_ = nullptr;
}

void RemoteEditorServer::UpdateRemoteRenderingState() {
  bool should_stream =
      running_ && config_.mode == Mode::kUiStreaming && native_window_;

  if (should_stream && !remote_editor_renderer_) {
    // START STREAMING: Create renderer
    remote_editor_renderer_ =
        std::make_unique<editor::RemoteEditorRenderer>(view_);
    remote_editor_renderer_->SetRenderTargetWindow(native_window_, width_,
                                                   height_);
    if (absl::StatusOr<std::reference_wrapper<editor::Editor>> editor_or =
            view_.GetRegistry().Get<editor::Editor>();
        editor_or.ok()) {
      editor::WidgetUiSystem& widget_ui_system =
          editor_or->get().GetWidgetUiSystem();
      widget_ui_system.SetLayoutComposer(
          std::make_unique<editor::LayoutComposer>(
              editor::kDefaultDesktopLayoutConfig));
    }
  } else if (!should_stream && remote_editor_renderer_) {
    // STOP STREAMING: Destroy renderer
    remote_editor_renderer_.reset();
  } else if (should_stream && remote_editor_renderer_) {
    // If streaming is already active, ensure window is set (e.g. after resize)
    remote_editor_renderer_->SetRenderTargetWindow(native_window_, width_,
                                                   height_);
  }
}

}  // namespace imp
