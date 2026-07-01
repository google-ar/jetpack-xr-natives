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

#include "core/editor/remote_editor/ios/ios_web_ui_remote_editor.h"

#include <memory>
#include <utility>

#include "core/common/log.h"
#include "third_party/absl/status/statusor.h"
#include "core/config.h"

#include "core/math/vec.h"
#include "core/scripting/message_helpers.h"
#include "core/scripting/proto/bridge.proto.imp.h"
#include "core/video/video_writer.h"
#include "core/view/base_view.h"
#include "core/view/scripting/script_message_handler.h"
#include "core/view/view_events.h"
#include "core/window/filament_host.h"


namespace imp::window::ios {
namespace {
// Helper template to allow overloading lambdas for std::visit.
template <class... Ts>
struct LambdaVisitor : Ts... {
  using Ts::operator()...;
};
template <class... Ts>
LambdaVisitor(Ts...) -> LambdaVisitor<Ts...>;

constexpr uint2 kDefaultDimensions = {1920, 1080};
}  // namespace

IosWebUiRemoteEditor::IosWebUiRemoteEditor(BaseView& view) : editor::WebUiRemoteEditor(view) {
  executor_ = Executor::ForegroundExecutor();
  video_writer_ = video::CreateVideoStreamWriterIos();
}

IosWebUiRemoteEditor::~IosWebUiRemoteEditor() {
  ClearRemembered();
  if (executor_ != Executor::CurrentExecutor()) {
    IMP_LOG(imp::ERROR) << "IosWebUiRemoteEditor must be destroyed on the "
                  "foreground thread.";
  }
  Stop();
}

bool IosWebUiRemoteEditor::IsReadyToSendDataToRemote() const {
  return video_writer_ && video_writer_->IsReady();
}

void IosWebUiRemoteEditor::OnWebUiStart() {
  if (std::holds_alternative<imp::editor::RemoteEditorInfo::RemoteUiDisabledConfig>(config_)) {
    return;
  }

  int http_port = 0;
  int script_api_bridge_port = 0;
  int ui_streaming_port = 0;

  std::visit(
      LambdaVisitor{[&](const imp::editor::RemoteEditorInfo::RemoteUiDisabledConfig& config) {
                      // Already handled by the early return above, but needed for exhaustiveness.
                    },
                    [&](const imp::editor::RemoteEditorInfo::RemoteCustomUiConfig& config) {
                      http_port = config.http_port;
                      script_api_bridge_port = config.script_api_bridge_port;
                    },
                    [&](const imp::editor::RemoteEditorInfo::RemoteEditorStreamingConfig& config) {
                      http_port = config.http_port;
                      script_api_bridge_port = config.script_api_bridge_port;
                      ui_streaming_port = config.ui_streaming_port;
                    }},
      config_);
  if (video_writer_) {
    video_writer_->SetPorts(http_port, ui_streaming_port, script_api_bridge_port);
    video_writer_->SetOnClientConnectedCallback([this](bool connected) {
      if (executor_) {
        executor_->Schedule([this, connected]() {
          if (connected) {
            OnClientConnected();
          } else {
            OnClientDisconnected();
            if (view_.GetHost()) {
              view_.GetHost()->EnsureNextRenderCompletes();
            }
          }
        });
      }
    });

    if (auto status = video_writer_->Open(kDefaultDimensions); !status.ok()) {
      IMP_LOG(imp::ERROR) << "Failed to open video writer: " << status;
    }

    video_writer_->SetOnScriptMessageCallback([this](const void* data, size_t len) {
      if (!executor_) return;

      scripting::ScriptMessageHandler* script_message_handler = view_.GetScriptMessageHandler();
      if (!script_message_handler) {
        IMP_LOG(imp::WARNING) << "IosWebUiRemoteEditor: No script message handler available";
        return;
      }

      std::string binary_data(static_cast<const char*>(data), len);
      executor_->Schedule([this, script_message_handler, binary_data]() {
        scripting::MessageToNative request;
        if (!scripting::ParseFromArray(binary_data.data(), binary_data.size(), &request)) {
          IMP_LOG(imp::ERROR) << "IosWebUiRemoteEditor: Failed to parse MessageToNative";
          return;
        }

        script_message_handler->HandleMessage(
            request, [this](const scripting::MessageToScript& response, void*) {
              std::string serialized = scripting::SerializeToBase64(response);
              video_writer_->BroadcastScriptMessage(serialized);
            });
      });
    });

    processing_input_connection_ =
        view_.GetDispatcher().Connect([this](const imp::ViewPreFrameUpdateEvent& event) {
          if (video_writer_) {
            video_writer_->ProcessInput(view_.GetHost());
          }
        });

    write_frame_connection_ =
        view_.GetDispatcher().Connect([this](const ViewSecondaryRenderEvent& event) {
          bool ready = video_writer_ && video_writer_->IsReady();
          if (ready) {
            video_writer_->CaptureFrame(view_.GetHost());
            video_writer_->WriteFrame();
          }
        });

    view_paused_connection_ =
        view_.GetDispatcher().Connect([this](const imp::ViewPausedEvent& event) {
          if (video_writer_) {
            static_cast<void>(video_writer_->Close());
          }
          ClearConnections();
        });

    view_resumed_connection_ =
        view_.GetDispatcher().Connect([this](const imp::ViewResumedEvent& event) {
          if (video_writer_) {
            if (auto status = video_writer_->Open(kDefaultDimensions); !status.ok()) {
              IMP_LOG(imp::ERROR) << "Failed to reopen video writer: " << status;
            }
          }
        });
  }
}

void IosWebUiRemoteEditor::OnWebUiStop() {
  processing_input_connection_.Disconnect();
  write_frame_connection_.Disconnect();
  view_paused_connection_.Disconnect();
  view_resumed_connection_.Disconnect();
  if (video_writer_) {
    static_cast<void>(video_writer_->Close());
  }
}

}  // namespace imp::window::ios
