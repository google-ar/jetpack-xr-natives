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

#include "core/common/string_helpers.h"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <string>

#include "absl/strings/ascii.h"
#include "absl/strings/escaping.h"
#include "absl/strings/string_view.h"

namespace imp {

std::string ToLower(absl::string_view view) {
  std::string result(view.data(), view.size());
  std::transform(result.begin(), result.end(), result.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  return result;
}

// Returns buffer as a base-64 encoded string.
std::string SerializeToBase64(const uint8_t* data, uint32_t size) {
  std::string str(data, data + size);
  std::string encoded;
  absl::Base64Escape(str, &encoded);
  return encoded;
}

bool DeserializeBase64(absl::string_view string_b64, std::string* dest) {
  return absl::Base64Unescape(string_b64, dest);
}

absl::string_view ExtractLeadingCommentsAndWhitespace(
    absl::string_view content) {
  size_t end_of_header = 0;
  while (end_of_header < content.length()) {
    const size_t line_end = content.find('\n', end_of_header);
    const absl::string_view line =
        line_end == absl::string_view::npos
            ? content.substr(end_of_header)
            : content.substr(end_of_header, line_end - end_of_header);
    const absl::string_view trimmed_line =
        absl::StripLeadingAsciiWhitespace(line);

    if (!trimmed_line.empty() && trimmed_line[0] != '#') {
      // This line is not a comment or a whitespace-only line, so we're done.
      break;
    }

    if (line_end == absl::string_view::npos) {
      end_of_header = content.length();
      break;
    }
    end_of_header = line_end + 1;
  }
  return content.substr(0, end_of_header);
}

}  // namespace imp
