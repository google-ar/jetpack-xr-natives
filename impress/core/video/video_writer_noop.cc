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

#include "core/video/video_writer_noop.h"

#include "absl/status/status.h"
#include "absl/strings/string_view.h"
#include "core/async/future.h"
#include "core/math/vec.h"
#include "core/window/filament_host.h"

namespace imp::video {

absl::Status VideoWriterNoop::Open(uint2 dimensions,
                                   absl::string_view filename) {
  return absl::OkStatus();
}

void VideoWriterNoop::CaptureFrame(window::FilamentHost* filament_host) {}

void VideoWriterNoop::WriteFrame() {}

Future<absl::Status> VideoWriterNoop::Close() { return absl::OkStatus(); }

}  // namespace imp::video
