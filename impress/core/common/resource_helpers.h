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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_RESOURCE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_RESOURCE_HELPERS_H_

#include "absl/status/statusor.h"
#include "absl/strings/cord.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"

struct FileToc;

namespace imp {

// TODO: Port from sceneform
void RegisterPackagedResources(const FileToc* resources);

// Returns true if `filename` matches a path to a file packaged into the binary.
bool PackagedFileExists(absl::string_view filename);

// Returns the contents of a file packaged into the binary.  Returns an error
// if the packaged file is not found.
OptionalError LoadPackagedFile(absl::string_view filename,
                               BufferAccess* access);

// Returns the contents of a file packaged into the binary as an absl::Cord.
// Returns an error if the packaged file is not found.
absl::StatusOr<absl::Cord> PackagedFileToCord(absl::string_view filename);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_RESOURCE_HELPERS_H_
