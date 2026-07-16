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

#include "third_party/absl/memory/memory.h"
#import "core/video/video_writer.h"

namespace imp {
namespace video {

// Forward declarations of factory functions.
std::unique_ptr<VideoWriter> CreateVideoFileWriterIos(bool record_microphone_audio);
std::unique_ptr<VideoWriter> CreateVideoStreamWriterIos();

namespace {

// A proxy VideoWriter that delegates to either VideoFileWriterIos or VideoStreamWriterIos
// based on whether a filename is provided in Open().
class VideoWriterIos : public VideoWriter {
 public:
  explicit VideoWriterIos(bool record_microphone_audio)
      : record_microphone_audio_(record_microphone_audio) {}

  absl::Status Open(uint2 dimensions, absl::string_view filename) override {
    if (filename.empty()) {
      impl_ = CreateVideoStreamWriterIos();
    } else {
      impl_ = CreateVideoFileWriterIos(record_microphone_audio_);
    }

    return impl_->Open(dimensions, filename);
  }

  void CaptureFrame(window::FilamentHost* filament_host) override {
    if (impl_) {
      impl_->CaptureFrame(filament_host);
    }
  }

  void WriteFrame() override {
    if (impl_) {
      impl_->WriteFrame();
    }
  }

  Future<absl::Status> Close() override {
    if (impl_) {
      return impl_->Close();
    }
    return Future<absl::Status>(absl::OkStatus());
  }

  bool IsReady() const override {
    if (impl_) {
      return impl_->IsReady();
    }
    return false;
  }

  void ProcessInput(window::FilamentHost* host) override {
    if (impl_) {
      impl_->ProcessInput(host);
    }
  }

 private:
  std::unique_ptr<VideoWriter> impl_;
  bool record_microphone_audio_;
};

}  // namespace

std::unique_ptr<VideoWriter> VideoWriter::CreateVideoWriter(bool record_microphone_audio) {
  return std::make_unique<VideoWriterIos>(record_microphone_audio);
}

}  // namespace video
}  // namespace imp
