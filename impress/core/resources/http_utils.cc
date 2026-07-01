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

#include "core/resources/http_utils.h"

#include "absl/status/status.h"
#include "absl/strings/str_cat.h"

namespace imp {
namespace resources {

// Lightweight mapping of HTTP codes to absl::StatusCode.
absl::Status HttpCodeToStatus(int code) {
  absl::StatusCode status_code;
  switch (code) {
    case 200:  // 200 OK
      return absl::OkStatus();
    case 400:  // 400 Bad Request
      status_code = absl::StatusCode::kInvalidArgument;
      break;
    case 401:  // 401 Unauthorized
      status_code = absl::StatusCode::kUnauthenticated;
      break;
    case 403:  // 403 Forbidden
      status_code = absl::StatusCode::kPermissionDenied;
      break;
    case 404:  // 404 Not Found
      status_code = absl::StatusCode::kNotFound;
      break;
    case 409:  // 409 Conflict
      status_code = absl::StatusCode::kAborted;
      break;
    case 416:  // 416 Range Not Satisfiable
      status_code = absl::StatusCode::kOutOfRange;
      break;
    case 429:  // 429 Too Many Requests
      status_code = absl::StatusCode::kResourceExhausted;
      break;
    case 499:  // 499 Client Closed Request
      status_code = absl::StatusCode::kCancelled;
      break;
    case 504:  // 504 Gateway Timeout
      status_code = absl::StatusCode::kDeadlineExceeded;
      break;
    case 501:  // 501 Not Implemented
      status_code = absl::StatusCode::kUnimplemented;
      break;
    case 503:  // 503 Service Unavailable
      status_code = absl::StatusCode::kUnavailable;
      break;
    default: {
      if (code >= 200 && code < 300) return absl::OkStatus();
      if (code >= 400 && code < 500) {
        status_code = absl::StatusCode::kFailedPrecondition;
      } else if (code >= 500 && code < 600) {
        status_code = absl::StatusCode::kInternal;
      } else {
        status_code = absl::StatusCode::kUnknown;
      }
    }
  }
  return absl::Status(status_code, absl::StrCat("HTTP Error: ", code));
}

}  // namespace resources
}  // namespace imp
