/*
 * Copyright 2025 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "core/common/filament_status_helpers.h"

#include "absl/status/status.h"
#include "filament/libs/utils/include/utils/Status.h"

namespace imp {

absl::Status FilamentStatusToAbslStatus(const utils::Status& status) {
  switch (status.getCode()) {
    case utils::StatusCode::OK:
      return absl::OkStatus();
    case utils::StatusCode::INVALID_ARGUMENT:
      return absl::InvalidArgumentError(status.getMessage());
    case utils::StatusCode::INTERNAL:
      return absl::InternalError(status.getMessage());
    case utils::StatusCode::UNSUPPORTED:
      return absl::UnimplementedError(status.getMessage());
      // Note: Omit default so that the compiler can catch any missing
      // utils::StatusCode if it's extended.
  }
  return absl::UnknownError(status.getMessage());
}
}  // namespace imp
