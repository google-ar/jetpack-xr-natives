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

#ifndef JETPACK_XR_NATIVES_OPENXR_OPENXR_MANAGER_UTILS_H_
#define JETPACK_XR_NATIVES_OPENXR_OPENXR_MANAGER_UTILS_H_

#include <openxr/openxr.h>

#include <ctime>

#include "absl/numeric/int128.h"

namespace androidx::xr::openxr {

// Multiplies two poses together. The first pose is the base and the second pose
// is the offset from the base. This is used to get a pose in the reference
// space of the first pose. This does not normalize the quaternion.
XrPosef MultiplyPoses(const XrPosef& pose1, const XrPosef& pose2);

// Converts a XrUuidEXT to a absl::uint128.
absl::uint128 UuidToUint128(const XrUuidEXT& uuid);

// Adds two timespecs.
timespec AddTimespecs(const timespec& timespec1, const timespec& timespec2);

}  // namespace androidx::xr::openxr

#endif  // JETPACK_XR_NATIVES_OPENXR_OPENXR_MANAGER_UTILS_H_
