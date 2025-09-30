// Copyright 2025 Google LLC
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

#include "core/split_engine/desktop/utils/buffer_factory.h"

#include <cstdint>
#include <cstring>

#include "absl/numeric/int128.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "absl/types/span.h"

namespace imp::split_engine {

absl::Status Buffer::Write(absl::Span<const uint8_t> data) {
  const absl::uint128 offset_bytes = offset_bytes_;
  const absl::uint128 data_size_bytes = data.size();

  if (offset_bytes + data_size_bytes > Size()) {
    return absl::OutOfRangeError(
        absl::StrCat("Offset(", offset_bytes, ") + data(", data_size_bytes,
                     ") size exceeds buffer size(", Size(), ")."));
  }
  memcpy(Data() + offset_bytes_, data.data(), data.size());
  offset_bytes_ += data.size();
  return absl::OkStatus();
}

}  // namespace imp::split_engine
