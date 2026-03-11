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

#include "apibindings/testing/marshalling/jni_test_utils.h"

#include <cstddef>
#include <string>

#include "absl/status/status.h"
#include "absl/strings/cord.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"

namespace imp {

absl::Status VerifyTestPattern(const absl::Cord& data, size_t expected_size) {
  if (data.size() != expected_size) {
    return absl::InternalError("Size of data " + std::to_string(data.size()) +
                               " does not match expected size " +
                               std::to_string(expected_size));
  }
  size_t index = 0;
  for (absl::string_view chunk : data.Chunks()) {
    for (char c : chunk) {
      std::byte b = static_cast<std::byte>(c);
      if (index == expected_size - 1) {
        if (b != kTestPatternSentinel) {
          return absl::InternalError(absl::StrFormat(
              "Sentinel corrupted at index %d. Expected: %d, Actual: %d", index,
              kTestPatternSentinel, b));
        }
      } else {
        if (b != kTestPatternByte) {
          return absl::InternalError(absl::StrFormat(
              "Test pattern corrupted at index %d. Expected: %d, Actual: %d",
              index, kTestPatternByte, b));
        }
      }
      index++;
    }
  }
  return absl::OkStatus();
}

}  // namespace imp
