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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_FILE_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_FILE_HELPERS_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "absl/strings/string_view.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"

namespace imp {

// Saves the string from `data` into `filename`.
// Returns an error if `filename` cannot be saved.
OptionalError SaveFile(absl::string_view filename, const uint8_t* data,
                       size_t size);

// Load the binary payload from file `filename` into `access`. Returns an error
// if `filename` does not exist or cannot be read.
OptionalError LoadBinary(absl::string_view filename, BufferAccess* access);

// Saves the binary payload from `access` into file `filename`.
// Returns an  error if `filename` cannot be saved.
OptionalError SaveBinary(absl::string_view filename,
                         const BufferAccess& access);
// Saves the binary payload from `data` w/ `size` into file `filename`.
// Returns an  error if `filename` cannot be saved.
OptionalError SaveBinary(absl::string_view filename, const uint8_t* data,
                         size_t size);

// Strip leading folders/separators from a filename, leaving just the file.
// e.g. "/folder/file.ext" --> "file.ext"
absl::string_view GetLocalFilenameFromFilename(absl::string_view filename);

// Strip the base name, leaving the path to the directory containing filename.
// Will not contain a trailing separator.
// e.g. "/folder/file.ext" --> "/folder"
absl::string_view GetDirectoryFromFilename(absl::string_view filename);

// Like GetBasenameFromFilaname, but also removes the extension.
// e.g. "/folder/file.ext" --> "file"
absl::string_view RemoveDirectoryAndExtensionFromFilename(
    absl::string_view filename);

// Return the extension (including the leading period) of a filename.
// e.g. "/folder/file.ext" --> ".ext"
absl::string_view GetExtensionFromFilename(absl::string_view filename);

// Return the full filename minus the extension.
// e.g. "/folder/file.ext" --> "/folder/file"
absl::string_view RemoveExtensionFromFilename(absl::string_view filename);

// Return all extensions (including the leading period) of a filename.
// e.g. "/folder/file.foo.bar" --> ".foo.bar"
absl::string_view GetAllExtensionsFromFilename(absl::string_view filename);

// Joins a Directory and Basename into a Filename.  For example:
//   ("folder/foo", "bar.png") returns "folder/foo/bar.png".
std::string JoinPath(absl::string_view directory, absl::string_view basename);

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_FILE_HELPERS_H_
