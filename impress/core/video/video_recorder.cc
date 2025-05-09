// Copyright 2024 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "core/video/video_recorder.h"

#include "core/math/vec.h"
namespace imp {
namespace video {
VideoRecorder::VideoRecorder(bool record_microphone_audio)
    : video_writer_(VideoWriter::CreateVideoWriter(record_microphone_audio)) {}

Future<std::unique_ptr<VideoRecorder>> VideoRecorder::Open(
    BaseView* view, absl::string_view filename, bool record_microphone_audio) {
  uint2 dimensions = view->GetHost()->GetPixelDimensions();
  return Future<std::unique_ptr<VideoRecorder>>::Schedule(
             [dimensions, filename = std::string(filename),
              record_microphone_audio]() mutable
             -> absl::StatusOr<std::unique_ptr<VideoRecorder>> {
               auto video_recorder =
                   absl::WrapUnique(new VideoRecorder(record_microphone_audio));
               MP_RETURN_IF_ERROR(
                   video_recorder->video_writer_->Open(dimensions, filename));

               return std::move(video_recorder);
             },
             Executor::Type::kBackground)
      .Then([view](std::unique_ptr<VideoRecorder> video_recorder) {
        video_recorder->connection_capture_frame_ =
            view->GetDispatcher().Connect(
                [view, video_writer = video_recorder->video_writer_.get()](
                    const ViewPostRenderEvent& ev) {
                  video_writer->CaptureFrame(view->GetHost());
                });
        video_recorder->connection_write_frame_ = view->GetDispatcher().Connect(
            [video_writer = video_recorder->video_writer_.get()](
                const ViewPostFrameEvent& ev) { video_writer->WriteFrame(); });
        return video_recorder;
      });
}

Future<absl::Status> VideoRecorder::Close() {
  if (!video_writer_) {
    return Future<absl::Status>(
        absl::UnavailableError("Recording is not active!"));
  }
  connection_capture_frame_.Disconnect();
  connection_write_frame_.Disconnect();
  Future<absl::Status> result = video_writer_->Close();
  video_writer_.reset();
  return result;
}
}  // namespace video
}  // namespace imp
