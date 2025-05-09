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

#include "core/common/owned_ptr_utilities.h"

#include "absl/base/attributes.h"
#include "absl/base/const_init.h"
#include "absl/base/log_severity.h"
#include "absl/synchronization/mutex.h"

namespace imp {

namespace {

ABSL_CONST_INIT absl::Mutex owned_ptr_log_severity_mutex(absl::kConstInit);
ABSL_CONST_INIT absl::LogSeverity owned_ptr_log_severity =
    absl::LogSeverity::kFatal;

}  // namespace

void SetOwnedPtrLogSeverity(absl::LogSeverity log_severity) {
  absl::MutexLock lock(&owned_ptr_log_severity_mutex);
  owned_ptr_log_severity = log_severity;
}

absl::LogSeverity GetOwnedPtrLogSeverity() {
  absl::MutexLock lock(&owned_ptr_log_severity_mutex);
  return owned_ptr_log_severity;
}

}  // namespace imp
