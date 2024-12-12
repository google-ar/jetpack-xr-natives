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

#include "openxr/openxr_manager_utils.h"

#include <openxr/openxr.h>

#include <ctime>

#include "absl/numeric/int128.h"

namespace androidx::xr::openxr {

XrPosef MultiplyPoses(const XrPosef &pose1, const XrPosef &pose2) {
  XrPosef result;
  result.position.x = pose1.position.x + pose2.position.x;
  result.position.y = pose1.position.y + pose2.position.y;
  result.position.z = pose1.position.z + pose2.position.z;

  result.orientation.x = pose1.orientation.w * pose2.orientation.x +
                         pose1.orientation.x * pose2.orientation.w +
                         pose1.orientation.y * pose2.orientation.z -
                         pose1.orientation.z * pose2.orientation.y;
  result.orientation.y = pose1.orientation.w * pose2.orientation.y +
                         pose1.orientation.y * pose2.orientation.w +
                         pose1.orientation.x * pose2.orientation.z -
                         pose1.orientation.z * pose2.orientation.x;
  result.orientation.z = pose1.orientation.w * pose2.orientation.z +
                         pose1.orientation.z * pose2.orientation.w +
                         pose1.orientation.x * pose2.orientation.y -
                         pose1.orientation.y * pose2.orientation.x;
  result.orientation.w = pose1.orientation.w * pose2.orientation.w -
                         pose1.orientation.x * pose2.orientation.x -
                         pose1.orientation.y * pose2.orientation.y -
                         pose1.orientation.z * pose2.orientation.z;
  return result;
}

absl::uint128 UuidToUint128(const XrUuidEXT &uuid) {
  absl::uint128 result = 0;
  for (int i = 0; i < 16; ++i) {
    result = (result << 8) | uuid.data[i];
  }
  return result;
}

timespec AddTimespecs(const timespec &timespec1, const timespec &timespec2) {
  constexpr int kNanosPerSecond = 1000000000;
  timespec result;
  result.tv_sec = timespec1.tv_sec + timespec2.tv_sec;
  result.tv_nsec = timespec1.tv_nsec + timespec2.tv_nsec;
  if (result.tv_nsec >= kNanosPerSecond) {
    result.tv_sec += 1;
    result.tv_nsec -= kNanosPerSecond;
  }
  return result;
}

}  // namespace androidx::xr::openxr
