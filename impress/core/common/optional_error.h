/*
 * Copyright 2024 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_OPTIONAL_ERROR_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_OPTIONAL_ERROR_H_

#include <string>

#include "absl/status/status.h"
#include "absl/types/optional.h"
#include "core/common/string_helpers.h"

namespace imp {

// TODO: Deprecate OptionalError
using OptionalError = absl::Status;

inline OptionalError Error(const char* error_message) {
  return absl::InternalError(error_message);
}

template <class... Args>
OptionalError Error(const absl::FormatSpec<Args...>& format, Args&&... args) {
  return absl::InternalError(FormatString(format, std::forward<Args>(args)...));
}

const OptionalError& NoError();

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_OPTIONAL_ERROR_H_
