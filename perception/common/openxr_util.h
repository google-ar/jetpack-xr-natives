// Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_OPENXR_LOGGING_H_
#define THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_OPENXR_LOGGING_H_

#include <openxr/openxr.h>
#include <openxr/openxr_reflection.h>

#include "absl/log/log.h"

namespace androidx::xr::openxr {

#define XR_ENUM_CASE_STR(name, val) \
  case name:                        \
    return #name;

// Returns a string representation of an XrResult.
constexpr const char* XrEnumStr(XrResult e) {
  switch (e) {
    XR_LIST_ENUM_XrResult(XR_ENUM_CASE_STR) default : return "Unknown";
  }
}

// Returns false and logs an error if the expression fails.
#define XR_RETURN_IF_FAILED(expr)                                     \
  do {                                                                \
    const XrResult xr_result = (expr);                                \
    if (XR_FAILED(xr_result)) {                                       \
      LOG(ERROR) << #expr << " failed with " << XrEnumStr(xr_result); \
      return false;                                                   \
    } else {                                                          \
      VLOG(3) << #expr << " succeeded!";                              \
    }                                                                 \
  } while (false)

// Returns XrResult and logs an error if the expression fails.
#define XR_RETURN_RESULT_IF_FAILED(expr)                              \
  do {                                                                \
    const XrResult xr_result = (expr);                                \
    if (XR_FAILED(xr_result)) {                                       \
      LOG(ERROR) << #expr << " failed with " << XrEnumStr(xr_result); \
      return xr_result;                                               \
    } else {                                                          \
      VLOG(3) << #expr << " succeeded!";                              \
    }                                                                 \
  } while (false)

}  // namespace androidx::xr::openxr

#endif  // THIRD_PARTY_JETPACK_XR_NATIVES_LOGGING_OPENXR_LOGGING_H_
