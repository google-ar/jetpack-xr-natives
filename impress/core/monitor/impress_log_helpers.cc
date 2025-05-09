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

#include "core/monitor/impress_log_helpers.h"

#include <stdint.h>

#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "absl/time/time.h"
#include "core/monitor/simple_histogram.h"

namespace imp {

absl::Status ValidateDuration(int64_t sec, int64_t ns) {
  if (sec < -315576000000 || sec > 315576000000) {
    return ::absl::InvalidArgumentError(absl::StrCat("seconds=", sec));
  }
  if (ns < -999999999 || ns > 999999999) {
    return ::absl::InvalidArgumentError(absl::StrCat("nanos=", ns));
  }
  if ((sec < 0 && ns > 0) || (sec > 0 && ns < 0)) {
    return ::absl::InvalidArgumentError("sign mismatch");
  }
  return absl::OkStatus();
}

absl::Status ValidateTimestamp(int64_t sec, int64_t ns) {
  // sec must be [0001-01-01T00:00:00Z, 9999-12-31T23:59:59.999999999Z]
  if (sec < -62135596800 || sec > 253402300799) {
    return ::absl::InvalidArgumentError(absl::StrCat("seconds=", sec));
  }
  if (ns < 0 || ns > 999999999) {
    return ::absl::InvalidArgumentError(absl::StrCat("nanos=", ns));
  }
  return absl::OkStatus();
}

}  // namespace imp
