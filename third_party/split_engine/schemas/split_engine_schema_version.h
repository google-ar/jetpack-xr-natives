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

#ifndef THIRD_PARTY_SPLIT_ENGINE_SCHEMAS_SPLIT_ENGINE_SCHEMA_VERSION_H_
#define THIRD_PARTY_SPLIT_ENGINE_SCHEMAS_SPLIT_ENGINE_SCHEMA_VERSION_H_

#include <cstdint>
#include <limits>

namespace android_xr {

// Summary of API Level Logic:
//
// The `XrExtensions` library returns two values: the system API level and the
// preview API level.
//
// * System API level: An integer [1..N] that increments with new API releases.
//   By default, this is the production API level.
// * Preview API level: `0` if preview APIs are disabled, or [1..N] for
//   revisions to the preview API. By default, preview APIs are enabled.
//
// For Split Engine, the preview API level only matters on the app side; we
// don't currently expose it. On the system renderer side, the split engine
// code is built-in and naturally associated with the current preview API level.
//
// System Property Overrides:
// We support setting a system property to override the API level for testing.
// This allows intentionally downgrading the API level to emulate previous
// system versions, disabling preview APIs, or enabling experimental APIs.
//
// Valid Tag Combinations:
// * (requires_api: N)               - Allowed if system API level >= N.
// * (preview_api)                   - Allowed if preview APIs are enabled.
// * (requires_api: N, unstable_api) - Allowed if system API level >= N AND
//                                     the app's Java package is in the
//                                     allowlist.
// * (preview_api, unstable_api)     - Allowed if preview APIs are enabled
//                                     AND the app's Java package is in the
//                                     allowlist.
// * (no tag)                        - Allowed if system API level is
//                                     experimental.
//
// Notable Invalid Combinations:
// * (requires_api: N, preview_api)  - An API cannot be both released and
//                                     preview.
// * (unstable_api)                  - `unstable_api` is a modifier. It must be
//                                     used alongside a released or preview API.
//
// API Level Setting Rules on Split Engine:
//
// 1. `kSplitEngineExperimentalApiLevel`:
//    - `required_api`: Allowed for all values of N.
//    - `preview_api`: Allowed.
//    - `unstable_api`: Allowed for any app.
//    - Unmarked (experimental): Allowed.
//
// 2. `kSplitEnginePreviewApiLevel`:
//    - `required_api`: Allowed for all values of N.
//    - `preview_api`: Allowed.
//    - `unstable_api`: Only allowed for apps with permission (allowlist).
//    - Unmarked (experimental): Disallowed.
//
// 3. Production (any integer value N):
//    - `required_api`: Allowed only if API level >= N.
//    - `preview_api`: Disallowed.
//    - `unstable_api`: Only allowed for apps with permission (allowlist).
//    - Unmarked (experimental): Disallowed.

// This is the API level that is currently in production.
// This value is used by the split engine renderer as the default value for its
// api level validation.
inline constexpr int32_t kSplitEngineProductionApiLevel = 1;

// The split engine preview API level.
// This value is used by the split engine renderer to allow the preview
// part of the schema to be used.
// This means the current production API level will be allowed in addition to
// any fields/tables marked with `preview_api`.
// Fields/tables marked with `unstable_api` will still only be allowed for apps
// in the allow list.
inline constexpr int32_t kSplitEnginePreviewApiLevel =
    std::numeric_limits<int32_t>::max() - 1;

// The split engine experimental API level.
// This value is used by the split engine renderer to allow the experimental
// part of the schema to be used.
// This means all fields/tables are allowed, including fields/tables marked with
// `unstable_api` and `preview_api`.
inline constexpr int32_t kSplitEngineExperimentalApiLevel =
    std::numeric_limits<int32_t>::max();

}  // namespace android_xr

// This macro is used to check if a feature that is currently experimental
// should be promoted to production after the api level is bumped.
//
// If the production API level is bumped, this macro will cause a compile-time
// error, prompting the developer to evaluate the guarded feature for promotion.
//
// Usage:
//   SPLIT_ENGINE_MAYBE_PROMOTE_FEATURE_TO_PRODUCTION(production_api_level)
//
//   production_api_level: The current production API level.
#define SPLIT_ENGINE_MAYBE_PROMOTE_FEATURE_AFTER_PRODUCTION_API_LEVEL_BUMP(   \
    production_api_level)                                                     \
  static_assert(                                                              \
      (production_api_level) == ::android_xr::kSplitEngineProductionApiLevel, \
      "Re-evaluate the guarded feature for promotion to the production API "  \
      "and bump the macro parameter to the new production API level.");

#endif  // THIRD_PARTY_SPLIT_ENGINE_SCHEMAS_SPLIT_ENGINE_SCHEMA_VERSION_H_
