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

#include "core/common/file_helpers.h"

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>

#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"
#include "core/common/buffer_access.h"
#include "core/common/optional_error.h"

namespace imp {

namespace {

OptionalError LoadAccess(absl::string_view filename, bool binary,
                         BufferAccess* access) {
  std::string filename_string(filename);
  FILE* file = fopen(filename_string.c_str(), binary ? "rb" : "r");
  if (!file) {
    return Error("Failed to open '%s': %s", filename_string.c_str(),
                 strerror(errno));
  }

  fseek(file, 0L, SEEK_END);
  size_t length = ftell(file);
  if (length >= std::numeric_limits<ptrdiff_t>::max()) {
    fclose(file);
    return Error("Failed to read size from '%s' (is it a file?)",
                 filename_string.c_str());
  }
  fseek(file, 0L, SEEK_SET);
  uint8_t* write_ptr = BufferAccess::Create(length, access);
  if (!write_ptr) {
    fclose(file);
    return Error("Failed to allocate %lu bytes to load '%s'", length,
                 filename_string.c_str());
  }

  fread(write_ptr, sizeof(uint8_t), length, file);
  fclose(file);
  return NoError();
}

OptionalError SaveString(absl::string_view filename, const uint8_t* data,
                         size_t size, bool binary) {
  std::string filename_string(filename);
  FILE* file = fopen(filename_string.c_str(), binary ? "wb" : "w");
  if (!file) return Error("Failed to open '%s'", filename_string.c_str());

  size_t written = fwrite(data, sizeof(uint8_t), size, file);
  fclose(file);

  if (written < size)
    return Error("Failed to write all %lu bytes to '%s', only wrote %lu", size,
                 filename_string.c_str(), written);
  return NoError();
}

// Everything after the last instance of delim, including the delim.
template <typename DelimType>
absl::string_view RemainderAtLastOf(absl::string_view contents,
                                    DelimType delim) {
  const size_t index = contents.find_last_of(delim);
  return (index == absl::string_view::npos) ? absl::string_view{}
                                            : contents.substr(index);
}

// Everything after the last instance of delim, including the delim.
template <typename DelimType>
absl::string_view RemainderAtFirstOf(absl::string_view contents,
                                     DelimType delim) {
  const size_t index = contents.find_first_of(delim);
  return (index == absl::string_view::npos) ? absl::string_view{}
                                            : contents.substr(index);
}

// Everything after (but not including) the last instance of delim.
template <typename DelimType>
absl::string_view RemainderAfterLastOf(absl::string_view contents,
                                       DelimType delim) {
  const size_t index = contents.find_last_of(delim);
  return (index == absl::string_view::npos)
             ? contents
             : ((index > contents.size()) ? absl::string_view{}
                                          : contents.substr(index + 1));
}

// Everything up to (but not including) the last instance of delim.
template <typename DelimType>
absl::string_view ContentsBeforeLastOf(absl::string_view contents,
                                       DelimType delim) {
  const size_t index = contents.find_last_of(delim);
  return (index == absl::string_view::npos) ? absl::string_view{}
                                            : contents.substr(0, index);
}

template <typename DelimType>
absl::string_view ContentsOrContentsBeforeLastOf(absl::string_view contents,
                                                 DelimType delim) {
  const size_t index = contents.find_last_of(delim);
  return (index == absl::string_view::npos) ? contents
                                            : contents.substr(0, index);
}

}  // namespace

OptionalError SaveFile(absl::string_view filename, const uint8_t* data,
                       size_t size) {
  return SaveString(filename, data, size, /*binary=*/false);
}

// Load the binary payload from file `filename` into the string `binary`.
// Returns an error if `filename` does not exist or cannot be read.
OptionalError LoadBinary(absl::string_view filename, BufferAccess* access) {
  return LoadAccess(filename, /*binary=*/true, access);
}

// Saves the binary payload from `binary` into file `filename`.
// Returns an  error if `filename` cannot be saved.
OptionalError SaveBinary(absl::string_view filename,
                         const BufferAccess& access) {
  return SaveBinary(filename, access.Data(), access.Size());
}
OptionalError SaveBinary(absl::string_view filename, const uint8_t* data,
                         size_t size) {
  return SaveString(filename, data, size, /*binary=*/true);
}

absl::string_view GetLocalFilenameFromFilename(absl::string_view filename) {
  return RemainderAfterLastOf(filename, "/\\");
}

absl::string_view GetDirectoryFromFilename(absl::string_view filename) {
  return ContentsBeforeLastOf(filename, "/\\");
}

absl::string_view RemoveDirectoryAndExtensionFromFilename(
    absl::string_view filename) {
  return ContentsOrContentsBeforeLastOf(GetLocalFilenameFromFilename(filename),
                                        '.');
}

absl::string_view GetExtensionFromFilename(absl::string_view filename) {
  return RemainderAtLastOf(filename, '.');
}

absl::string_view RemoveExtensionFromFilename(absl::string_view filename) {
  return ContentsOrContentsBeforeLastOf(filename, '.');
}

absl::string_view GetAllExtensionsFromFilename(absl::string_view filename) {
  return RemainderAtFirstOf(filename, '.');
}

std::string JoinPath(absl::string_view directory, absl::string_view basename) {
  // Ensure directory does not have a trailing slash.
  if (directory.find_first_of("/\\", directory.length() - 1) !=
      absl::string_view::npos)
    directory = directory.substr(0, directory.length() - 1);

  // Ensure basename does not have a leading slash (unless dirname is empty,
  // in which case we treat basename as a full path).
  if (!basename.empty() && !directory.empty() &&
      basename.find_last_of("/\\", 0) != absl::string_view::npos)
    basename = basename.substr(1);

  // For consistency, emit local paths (e.g. './f.txt') without a leading './'.
  if (directory.empty() || directory == ".") return std::string(basename);

  // Combine the cleaned directory and base names.
  return absl::StrFormat("%.*s/%.*s", static_cast<int>(directory.size()),
                         directory.data(), static_cast<int>(basename.size()),
                         basename.data());
}

}  // namespace imp
