/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_JNI_TEST_UTILS_H_
#define THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_JNI_TEST_UTILS_H_

#include <cstddef>

#include "absl/status/status.h"
#include "absl/strings/cord.h"

namespace imp {

// Test pattern bytes and sentinel used for verifying the marshalling of byte
// arrays across the JNI boundary.
constexpr std::byte kTestPatternByte = std::byte{0xcd};
constexpr std::byte kTestPatternSentinel = std::byte{0xee};

// Helper to verify the pattern over the JNI boundary.
absl::Status VerifyTestPattern(const absl::Cord& data, size_t expected_size);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_APIBINDINGS_TESTING_MARSHALLING_JNI_TEST_UTILS_H_
