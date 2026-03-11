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

// This is the API level that is currently in production.
// This value is used by the split engine renderer as the default value for its
// api level validation.
inline constexpr int32_t kSplitEngineProductionApiLevel = 1;

// The split engine experimental API level.
// This value is used by the split engine renderer to allow the experimental
// part of the schema to be used.
inline constexpr int32_t kSplitEngineExperimentalApiLevel =
    std::numeric_limits<int32_t>::max();

}  // namespace android_xr

#endif  // THIRD_PARTY_SPLIT_ENGINE_SCHEMAS_SPLIT_ENGINE_SCHEMA_VERSION_H_
