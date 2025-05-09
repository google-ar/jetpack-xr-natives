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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_PTR_UTILITIES_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_PTR_UTILITIES_H_

#include "absl/base/log_severity.h"

namespace imp {

// Overrides the default log severity for all OwnedPtr objects.
//
// By default, OwnedPtr will Fatal if it is destroyed while any
// BorrowedPtr objects are still outstanding. This method can be used to
// override the default behavior to instead log a warning or error.
//
// In general, fataling is preferred because it will help catch bugs more
// quickly, make them easier to debug, be included in production crash
// reports, and prevent memory corruption.
//
// However, in some cases, it may be helpful to override the default
// behavior. For example, when migrating from existing code to use
// OwnedPtr, fataling may not be appropriate if the existing code is
// already leaking memory. In this case, the severity can be downgraded to a
// warning or error while the migration is ongoing.
//
// This method is thread safe and can be called at any time.
//
// Note: This is a free function instead of being part of OwnedPtr because
// OwnedPtr is templated, and this allows the severity to be set without
// specifying the type of OwnedPtr.
void SetOwnedPtrLogSeverity(absl::LogSeverity log_severity);

// Returns the current log severity for all OwnedPtr objects.
//
// This is thread safe and can be called at any time.
absl::LogSeverity GetOwnedPtrLogSeverity();

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_OWNED_PTR_UTILITIES_H_
