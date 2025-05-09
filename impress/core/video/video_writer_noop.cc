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

#import "core/video/video_writer.h"

namespace imp::video {

// Stub implementation of VideoWriter for unsupported platforms.
class VideoWriterNoop : public VideoWriter {
 public:
  ~VideoWriterNoop() override = default;
  absl::Status Open(uint2 dimensions, absl::string_view filename) override {
    return absl::UnimplementedError(
        "Video Writer not supported on this platform");
  }
  void CaptureFrame(window::FilamentHost* filament_host) override {}
  void WriteFrame() override {}
  Future<absl::Status> Close() override {
    return Future<absl::Status>(absl::UnimplementedError(
        "Video Writer not supported on this platform"));
  }
};

std::unique_ptr<VideoWriter> VideoWriter::CreateVideoWriter(
    bool record_microphone_audio) {
  return std::make_unique<VideoWriterNoop>();
}

}  // namespace imp::video
