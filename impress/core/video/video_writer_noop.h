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

#ifndef THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_WRITER_NOOP_H_
#define THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_WRITER_NOOP_H_

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/math/math.h"
#include "core/math/vec.h"
#include "core/video/video_writer.h"
#include "core/window/filament_host.h"

namespace imp::video {

// Stub implementation of VideoWriter for unsupported platforms.
class VideoWriterNoop : public VideoWriter {
 public:
  ~VideoWriterNoop() override = default;
  absl::Status Open(uint2 dimensions, absl::string_view filename) override;
  void CaptureFrame(window::FilamentHost* filament_host) override;
  void WriteFrame() override;
  Future<absl::Status> Close() override;
  bool IsReady() const override { return false; }
  uint2 GetDimensions() const override { return {0, 0}; }
};

}  // namespace imp::video

#endif  // THIRD_PARTY_IMPRESS_CORE_VIDEO_VIDEO_WRITER_NOOP_H_
