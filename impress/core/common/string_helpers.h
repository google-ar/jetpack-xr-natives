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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_STRING_HELPERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_STRING_HELPERS_H_

#include <cstddef>
#include <cstdint>
#include <string>

#include "absl/base/attributes.h"
#include "absl/strings/str_format.h"
#include "absl/strings/string_view.h"

namespace imp {

template <typename... Args>
ABSL_DEPRECATED("Use `absl::StrFormat()` instead.")
std::string
    FormatString(const absl::FormatSpec<Args...>& format, Args&&... args) {
  return absl::StrFormat(format, std::forward<Args>(args)...);
}

// Take a string_view pointing to a compound string and pass the contained
// sub-strings (as string-views) through a provided output iterator (e.g.
// std::back_inserter).
template <class OutputIt>
OutputIt Split(absl::string_view contents, char sep, OutputIt output_it) {
  size_t cursor = contents.find_first_not_of(sep, 0);
  while (cursor != absl::string_view::npos) {
    size_t next_sep = contents.find_first_of(sep, cursor);
    *output_it++ = contents.substr(cursor, next_sep - cursor);
    // Move cursor to the first non-separator.
    cursor = contents.find_first_not_of(sep, next_sep);
  }
  return output_it;
}

// Return a copy of a string with all letters converted to lower case.
std::string ToLower(absl::string_view view);

// Returns buffer as a base-64 encoded string.
std::string SerializeToBase64(const uint8_t* data, uint32_t size);

// Converts string_b64 to its binary equivalent and writes it into dest.
bool DeserializeBase64(absl::string_view string_b64, std::string* dest);

// Extracts the leading comments and whitespace from a string.
//
// Note: This only works for line comments starting with a # character.
absl::string_view ExtractLeadingCommentsAndWhitespace(
    absl::string_view content);

}  // namespace imp
#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_STRING_HELPERS_H_
