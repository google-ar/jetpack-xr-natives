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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_WRITER_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_WRITER_H_

#include "absl/status/status.h"
#include "core/async/future.h"
#include "core/common/buffer_access.h"
#include "core/math/vec.h"
#include "core/window/filament_host.h"

namespace imp {
namespace video {

// A base class for platform-specific video writers.
class VideoWriter {
 public:
  virtual ~VideoWriter() {}
  // Opens a new video for writing with the given filename and dimensions.
  virtual absl::Status Open(uint2 dimensions, absl::string_view filename) = 0;
  // Captures the current frame. Must be called between render() and endFrame().
  virtual void CaptureFrame(window::FilamentHost* filament_host) = 0;
  // Writes a frame to the video file. Must be called after endFrame().
  virtual void WriteFrame() = 0;
  // Closes an active video, finishing the writing process.
  virtual Future<absl::Status> Close() = 0;
  // Creates a new video writer. Must be implemented for each platform.
  // record_microphone_audio maybe false if e.g. microphone access has not been
  // granted to the application.
  static std::unique_ptr<VideoWriter> CreateVideoWriter(
      bool record_microphone_audio);
};

}  // namespace video
}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_WRITER_H_
