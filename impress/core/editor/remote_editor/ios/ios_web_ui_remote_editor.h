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

#ifndef THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_IOS_IOS_WEB_UI_REMOTE_EDITOR_H_
#define THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_IOS_IOS_WEB_UI_REMOTE_EDITOR_H_

#include <functional>
#include <memory>

#include "core/editor/remote_editor/web_ui_remote_editor.h"
#include "core/math/vec.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/video/video_writer.h"
#include "core/video/web_server_video_stream_writer_ios.h"
#include "core/view/base_view.h"
#include "core/window/filament_host.h"

namespace imp::window::ios {

// Abstracts away the video_writer and manages the remote editor lifecycle on
// iOS.
class IosWebUiRemoteEditor : public editor::WebUiRemoteEditor {
 public:
  // Constructor is private to enforce creation through the factory method.
  explicit IosWebUiRemoteEditor(BaseView& view);
  ~IosWebUiRemoteEditor() override;

  // --- RemoteEditor overrides ---
  bool IsReadyToSendDataToRemote() const override;

  // --- video::VideoWriter implementation ---
 protected:
  void OnWebUiStart() override;
  void OnWebUiStop() override;

 private:
  // --- Core dependencies ---
  // Executor for scheduling asynchronous tasks, if applicable.
  Executor* executor_ = nullptr;

  std::unique_ptr<video::WebServerVideoStreamWriterIos> video_writer_;
  Dispatcher::ScopedConnection write_frame_connection_;
  Dispatcher::ScopedConnection processing_input_connection_;
  Dispatcher::ScopedConnection view_paused_connection_;
  Dispatcher::ScopedConnection view_resumed_connection_;
};

}  // namespace imp::window::ios

#endif  // THIRD_PARTY_IMPRESS_CORE_EDITOR_REMOTE_EDITOR_IOS_IOS_WEB_UI_REMOTE_EDITOR_H_
