/*
 * Copyright 2026 Google LLC
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

#ifndef THIRD_PARTY_IMPRESS_CORE_COMMON_STRING_NUMBERS_H_
#define THIRD_PARTY_IMPRESS_CORE_COMMON_STRING_NUMBERS_H_

#include <string>
#include <type_traits>

#include "absl/strings/ascii.h"
#include "absl/strings/str_cat.h"
#include "absl/strings/string_view.h"

// Copied from internal Google library. We should replace this with the
// open-sourced library version once it's open-sourced.

namespace imp {

// Converts an integer to a string.  Commas are inserted if the result would
// have more than three consecutive digits, where every comma is followed
// by exactly 3 digits.
template <typename IntType>
inline std::string SimpleItoaWithCommas(IntType ii) {
  static_assert(std::is_integral_v<IntType>);
  std::string s1 = absl::StrCat(ii);
  absl::string_view sp1(s1);
  std::string output;
  // Copy leading non-digit characters unconditionally.
  // This picks up the leading sign.
  while (!sp1.empty() && !absl::ascii_isdigit(sp1[0])) {
    output.push_back(sp1[0]);
    sp1.remove_prefix(1);
  }
  // Copy rest of input characters.
  for (absl::string_view::size_type i = 0; i < sp1.size(); ++i) {
    if (i > 0 && (sp1.size() - i) % 3 == 0) {
      output.push_back(',');
    }
    output.push_back(sp1[i]);
  }
  return output;
}

}  // namespace imp

#endif  // THIRD_PARTY_IMPRESS_CORE_COMMON_STRING_NUMBERS_H_
