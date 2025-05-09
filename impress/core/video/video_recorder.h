/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_RECORDER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_RECORDER_H_
#include "absl/status/status.h"
#include "core/ncsb/dispatcher/dispatcher.h"
#include "core/video/video_writer.h"
#include "core/view/base_view.h"
#include "core/view/view_events.h"
#include "core/window/filament_host.h"
namespace imp {
namespace video {

// TODO: Add unit tests.
// Records video files of the app's video/audio output.
class VideoRecorder {
 public:
  // Start recording a video of the app.
  static Future<std::unique_ptr<VideoRecorder>> Open(
      BaseView* view, absl::string_view filename,
      bool record_microphone_audio = true);

  // Stops an active video recording.
  Future<absl::Status> Close();

 private:
  VideoRecorder(bool record_microphone_audio);

  Dispatcher::ScopedConnection connection_capture_frame_;
  Dispatcher::ScopedConnection connection_write_frame_;
  std::unique_ptr<VideoWriter> video_writer_;
};

}  // namespace video
}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_RECORDER_H_
